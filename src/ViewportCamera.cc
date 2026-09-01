/// ViewportCamera exposes the gz GUI user-camera over ROS 2 so external scripts
/// can frame and record scenes without touching gz directly. Each render frame the
/// camera pose is composed as reference * local: the reference is the world origin,
/// a constant pose, or a tracked entity (sampled from the ECM in Update), and the
/// local pose comes from the one-shot set_view service or the streamed cmd_view
/// topic. set_view snaps, cmd_view is a buffer of timestamped keyframes interpolated
/// by time, so publish jitter is invisible as long as a keyframe leads the clock.
/// ROS lives on its own spin thread, state crosses to the render thread under a mutex.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <variant>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/plugin/Register.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/Image.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <viewport_control_msgs/msg/viewport_view.hpp>
#include <viewport_control_msgs/srv/viewport_capture.hpp>
#include <viewport_control_msgs/srv/viewport_set_projection.hpp>
#include <viewport_control_msgs/srv/viewport_set_reference_frame.hpp>
#include <viewport_control_msgs/srv/viewport_set_view.hpp>

#include "arena_gz_plugins/ViewportCamera.hh"

namespace arena_gz_plugins
{

using namespace std::chrono_literals;

using SetView = viewport_control_msgs::srv::ViewportSetView;
using SetReferenceFrame = viewport_control_msgs::srv::ViewportSetReferenceFrame;
using SetProjection = viewport_control_msgs::srv::ViewportSetProjection;
using Capture = viewport_control_msgs::srv::ViewportCapture;
using ViewportView = viewport_control_msgs::msg::ViewportView;

namespace
{
// Arena pins the gz world origin to map (static map->odom at the origin), so the
// user-camera world pose is map-frame.
constexpr const char * kPoseFrame = "map";
// Publish the camera pose every Nth render frame, ~10 Hz at 60 fps.
constexpr unsigned int kPublishEveryNFrames = 6;
// Release the camera to manual control this long after the last cmd_view. Larger
// than the helper's keyframe lead so the interpolation buffer plays out first.
constexpr auto kStreamTimeout = 400ms;
// A camera move beyond these thresholds while only tracking (no live stream) is
// read as the user grabbing the view, which releases the track.
constexpr double kManualPosEps = 0.01;  // m
constexpr double kManualAngEps = 0.01;  // rad

// Zero-roll look-at: orient a gz camera (forward +X, up +Z) from eye to target.
gz::math::Quaterniond LookAt(
  const gz::math::Vector3d & _eye, const gz::math::Vector3d & _target)
{
  gz::math::Vector3d dir = _target - _eye;
  if (dir.Length() < 1e-9)
    return gz::math::Quaterniond::Identity;
  dir.Normalize();
  const double yaw = std::atan2(dir.Y(), dir.X());
  const double pitch = -std::asin(std::clamp(dir.Z(), -1.0, 1.0));
  return gz::math::Quaterniond(0.0, pitch, yaw);
}

// Drop the reference-frame rotation channels the caller chose not to inherit.
gz::math::Pose3d ReduceReference(const gz::math::Pose3d & _pose, uint8_t _mode)
{
  switch (_mode)
  {
    case SetReferenceFrame::Request::YAW_ONLY:
      return gz::math::Pose3d(
        _pose.Pos(), gz::math::Quaterniond(0.0, 0.0, _pose.Rot().Yaw()));
    case SetReferenceFrame::Request::POSITION_ONLY:
      return gz::math::Pose3d(_pose.Pos(), gz::math::Quaterniond::Identity);
    default:  // FULL
      return _pose;
  }
}

gz::math::Pose3d ToPose(const geometry_msgs::msg::Pose & _p)
{
  return gz::math::Pose3d(
    gz::math::Vector3d(_p.position.x, _p.position.y, _p.position.z),
    gz::math::Quaterniond(
      _p.orientation.w, _p.orientation.x, _p.orientation.y, _p.orientation.z));
}

std::chrono::steady_clock::duration ToDuration(const builtin_interfaces::msg::Time & _t)
{
  return std::chrono::seconds(_t.sec) + std::chrono::nanoseconds(_t.nanosec);
}

// True if pose _a differs meaningfully from _b, used to spot a manual camera grab.
bool Moved(const gz::math::Pose3d & _a, const gz::math::Pose3d & _b)
{
  if ((_a.Pos() - _b.Pos()).Length() > kManualPosEps)
    return true;
  const gz::math::Quaterniond dq = _a.Rot() * _b.Rot().Inverse();
  return 2.0 * std::acos(std::min(1.0, std::abs(dq.W()))) > kManualAngEps;
}

// The reference world pose: a tracked entity (once sampled), else the constant
// refPose, else the world origin for a tracked-but-unresolved entity.
gz::math::Pose3d ResolveReference(const std::string & _refEntity,
  const gz::math::Pose3d & _refPose, uint8_t _refMode,
  const std::optional<gz::math::Pose3d> & _refTargetPose)
{
  if (_refEntity.empty())
    return ReduceReference(_refPose, _refMode);
  if (_refTargetPose)
    return ReduceReference(*_refTargetPose, _refMode);
  return gz::math::Pose3d::Zero;
}

// Compose the camera world pose: reference * local, keeping the aim world-stable
// when requested. The single source the live drive and capture both go through.
gz::math::Pose3d ComposeWorld(const gz::math::Pose3d & _ref,
  const gz::math::Pose3d & _local, bool _worldOrientation)
{
  return gz::math::Pose3d(
    _ref.Pos() + _ref.Rot() * _local.Pos(),
    _worldOrientation ? _local.Rot() : _ref.Rot() * _local.Rot());
}

// One streamed keyframe: a local-frame pose tagged with the wall time it is due.
struct Keyframe
{
  rclcpp::Time time;
  gz::math::Pose3d local;
  bool worldOrientation;
  double fov;  // <= 0 leaves fov unchanged
};

// Sim-agnostic interpolation core: sample the buffered keyframes at _now, lerping
// position and slerping orientation between the two bracketing the time, clamping
// to the ends outside the buffered span. Leaves the outputs untouched if empty.
// A jitter buffer like this is why publish rate stops mattering: as long as a
// keyframe leads _now, a late or missing publish is invisible.
void SampleBuffer(const std::deque<Keyframe> & _buf, const rclcpp::Time & _now,
  gz::math::Pose3d & _local, bool & _worldOrientation, double & _fov)
{
  if (_buf.empty())
    return;
  if (_now <= _buf.front().time)
  {
    _local = _buf.front().local;
    _worldOrientation = _buf.front().worldOrientation;
    _fov = _buf.front().fov;
    return;
  }
  if (_now >= _buf.back().time)
  {
    _local = _buf.back().local;
    _worldOrientation = _buf.back().worldOrientation;
    _fov = _buf.back().fov;
    return;
  }
  std::size_t i = 1;
  while (i < _buf.size() && _buf[i].time < _now)
    ++i;
  const Keyframe & a = _buf[i - 1];
  const Keyframe & b = _buf[i];
  const double span = (b.time - a.time).seconds();
  const double alpha = span > 1e-9 ? (_now - a.time).seconds() / span : 1.0;
  _local = gz::math::Pose3d(
    a.local.Pos() + (b.local.Pos() - a.local.Pos()) * alpha,
    gz::math::Quaterniond::Slerp(alpha, a.local.Rot(), b.local.Rot(), true));
  _worldOrientation = b.worldOrientation;
  _fov = b.fov > 0.0 ? b.fov : a.fov;
}
}  // namespace

class ViewportCameraPrivate
{
public:
  void SpinLoop()
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(this->ros);
    // spin_once blocks up to the timeout for work, so it stays responsive without
    // busy-waiting and re-checks the stop flag each wake for a clean teardown.
    while (rclcpp::ok() && !this->stopSpin)
      exec.spin_once(100ms);
  }

  // ROS, owned by the spin thread.
  rclcpp::Node::SharedPtr ros;
  rclcpp::Service<SetView>::SharedPtr setViewSrv;
  rclcpp::Service<SetReferenceFrame>::SharedPtr setReferenceFrameSrv;
  rclcpp::Service<SetProjection>::SharedPtr setProjectionSrv;
  rclcpp::Service<Capture>::SharedPtr captureSrv;
  rclcpp::Subscription<ViewportView>::SharedPtr viewSub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub;
  std::thread spinThread;
  std::atomic<bool> stopSpin{false};

  // User-camera handle, resolved lazily on the render thread.
  gz::rendering::CameraPtr camera;

  // Desired view and reference frame, guarded by mutex.
  std::mutex mutex;
  bool oneShot{false};                 // a set_view placement to apply once, then release
  bool streaming{false};               // cmd_view is driving the camera continuously
  bool localSet{false};                // a local pose has been provided at least once
  std::chrono::steady_clock::time_point lastView;  // arrival time of the last cmd_view
  gz::math::Pose3d localPose;          // set_view's one-shot local pose
  bool worldOrientation{false};        // keep the aim world-stable (one-shot path)
  std::deque<Keyframe> viewBuffer;     // streamed keyframes, interpolated by time
  std::optional<double> pendingFov;
  std::optional<gz::rendering::CameraProjectionType> pendingProjection;

  std::string refEntity;               // tracked entity, empty -> constant refPose
  std::string warnedEntity;            // last entity logged as missing (warn once)
  gz::math::Pose3d refPose;            // constant reference (origin by default)
  uint8_t refMode{SetReferenceFrame::Request::FULL};
  std::optional<gz::math::Pose3d> refTargetPose;  // entity world pose, sampled in Update

  // Latest UpdateInfo.simTime, sampled every Update. Gates capture on min_sim_time.
  std::chrono::steady_clock::duration latestSimTime{0};

  // Capture handshake: the service fills the request and waits on captureCv for
  // the render thread to render the pose and hand back the pixels.
  std::condition_variable captureCv;
  bool captureRequested{false};
  bool captureDone{false};
  gz::math::Pose3d captureLocal;       // local pose to snap to for the grab
  bool captureWorldOrientation{false};
  double captureFov{0.0};
  std::chrono::steady_clock::duration captureMinSimTime{0};  // defer until latestSimTime reaches this
  bool captureOk{false};
  std::string captureMsg;
  sensor_msgs::msg::Image captureImage;

  // Render-thread only.
  std::optional<gz::math::Pose3d> appliedPose;
  unsigned int frameCount{0};
};

ViewportCamera::ViewportCamera()
  : dataPtr(std::make_unique<ViewportCameraPrivate>())
{
}

ViewportCamera::~ViewportCamera()
{
  this->dataPtr->stopSpin = true;
  if (this->dataPtr->spinThread.joinable())
    this->dataPtr->spinThread.join();
}

void ViewportCamera::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Arena viewport camera";

  // Render events arrive through the main-window event stream.
  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);
  this->dataPtr->ros = std::make_shared<rclcpp::Node>("arena_viewport_camera");

  this->dataPtr->setViewSrv = this->dataPtr->ros->create_service<SetView>(
    "/arena/viewport/set_view",
    [this](const std::shared_ptr<SetView::Request> _req,
        std::shared_ptr<SetView::Response> _res)
    {
      const gz::math::Vector3d eye(_req->eye.x, _req->eye.y, _req->eye.z);
      const gz::math::Vector3d target(_req->target.x, _req->target.y, _req->target.z);
      {
        std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
        this->dataPtr->localPose = gz::math::Pose3d(eye, LookAt(eye, target));
        this->dataPtr->worldOrientation = false;
        this->dataPtr->oneShot = true;
        this->dataPtr->streaming = false;
        this->dataPtr->localSet = true;
        if (_req->fov > 0.0)
          this->dataPtr->pendingFov = _req->fov;
      }
      _res->success = true;
      _res->message = "ok";
    });

  this->dataPtr->setReferenceFrameSrv =
    this->dataPtr->ros->create_service<SetReferenceFrame>(
      "/arena/viewport/set_reference_frame",
      [this](const std::shared_ptr<SetReferenceFrame::Request> _req,
          std::shared_ptr<SetReferenceFrame::Response> _res)
      {
        std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
        if (!_req->entity.empty())
        {
          this->dataPtr->refEntity = _req->entity;
          this->dataPtr->refMode = _req->mode;
          this->dataPtr->refTargetPose.reset();
          _res->message = "tracking " + _req->entity;
        }
        else if (_req->has_pose)
        {
          this->dataPtr->refEntity.clear();
          this->dataPtr->refPose = ReduceReference(ToPose(_req->pose), _req->mode);
          this->dataPtr->refMode = SetReferenceFrame::Request::FULL;
          this->dataPtr->refTargetPose.reset();
          _res->message = "constant reference set";
        }
        else
        {
          // Latch: freeze the reference at its current world pose so the camera
          // stays put, leaving the local offset untouched.
          const gz::math::Pose3d current = this->dataPtr->refEntity.empty()
            ? this->dataPtr->refPose
            : ReduceReference(
                this->dataPtr->refTargetPose.value_or(this->dataPtr->refPose),
                this->dataPtr->refMode);
          this->dataPtr->refEntity.clear();
          this->dataPtr->refPose = current;
          this->dataPtr->refMode = SetReferenceFrame::Request::FULL;
          this->dataPtr->refTargetPose.reset();
          _res->message = "latched current pose";
        }
        _res->success = true;
      });

  this->dataPtr->setProjectionSrv = this->dataPtr->ros->create_service<SetProjection>(
    "/arena/viewport/set_projection",
    [this](const std::shared_ptr<SetProjection::Request> _req,
        std::shared_ptr<SetProjection::Response> _res)
    {
      gz::rendering::CameraProjectionType type;
      if (_req->projection == "perspective")
        type = gz::rendering::CPT_PERSPECTIVE;
      else if (_req->projection == "orthographic")
        type = gz::rendering::CPT_ORTHOGRAPHIC;
      else
      {
        _res->success = false;
        _res->message = "projection must be 'perspective' or 'orthographic'";
        return;
      }
      {
        std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
        this->dataPtr->pendingProjection = type;
      }
      _res->success = true;
      _res->message = "ok";
    });

  // Deep queue: every keyframe must reach the buffer, not just the latest one.
  this->dataPtr->viewSub = this->dataPtr->ros->create_subscription<ViewportView>(
    "/arena/viewport/cmd_view",
    rclcpp::QoS(rclcpp::KeepLast(64)).best_effort(),
    [this](const ViewportView::SharedPtr _msg)
    {
      Keyframe kf;
      kf.time = rclcpp::Time(_msg->target_time);
      kf.local = ToPose(_msg->pose);
      kf.worldOrientation = _msg->world_orientation;
      kf.fov = _msg->fov;
      std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
      this->dataPtr->viewBuffer.push_back(kf);
      this->dataPtr->streaming = true;
      this->dataPtr->lastView = std::chrono::steady_clock::now();
      this->dataPtr->localSet = true;
    });

  this->dataPtr->captureSrv = this->dataPtr->ros->create_service<Capture>(
    "/arena/viewport/capture",
    [this](const std::shared_ptr<Capture::Request> _req,
        std::shared_ptr<Capture::Response> _res)
    {
      std::unique_lock<std::mutex> lock(this->dataPtr->mutex);
      this->dataPtr->captureLocal = ToPose(_req->pose);
      this->dataPtr->captureWorldOrientation = _req->world_orientation;
      this->dataPtr->captureFov = _req->fov;
      this->dataPtr->captureMinSimTime = ToDuration(_req->min_sim_time);
      this->dataPtr->captureDone = false;
      this->dataPtr->captureRequested = true;
      // Block this service call until the render thread renders the pose and
      // fills the pixels (or give up if the GUI never renders). Longer than the
      // 5s render-only wait to also cover a step + state-propagation delay when
      // min_sim_time holds the capture back.
      const bool ready = this->dataPtr->captureCv.wait_for(
        lock, 15s, [this] { return this->dataPtr->captureDone; });
      if (!ready)
      {
        this->dataPtr->captureRequested = false;
        _res->success = false;
        _res->message = "capture timed out (is the GUI rendering, or is min_sim_time not yet reached?)";
        return;
      }
      _res->success = this->dataPtr->captureOk;
      _res->message = this->dataPtr->captureMsg;
      _res->image = this->dataPtr->captureImage;
    });

  this->dataPtr->posePub =
    this->dataPtr->ros->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/arena/viewport/camera_pose", rclcpp::QoS(10));

  this->dataPtr->spinThread =
    std::thread(&ViewportCameraPrivate::SpinLoop, this->dataPtr.get());
}

void ViewportCamera::Update(
  const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm)
{
  std::string entity;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->latestSimTime = _info.simTime;
    entity = this->dataPtr->refEntity;
  }
  if (entity.empty())
    return;

  const gz::sim::Entity e =
    _ecm.EntityByComponents(gz::sim::components::Name(entity));
  if (e == gz::sim::kNullEntity)
  {
    if (this->dataPtr->warnedEntity != entity)
    {
      this->dataPtr->warnedEntity = entity;
      RCLCPP_WARN(
        this->dataPtr->ros->get_logger(),
        "tracked entity '%s' not found; camera holds the world frame", entity.c_str());
    }
    return;
  }
  this->dataPtr->warnedEntity.clear();

  const gz::math::Pose3d pose = gz::sim::worldPose(e, _ecm);
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->refTargetPose = pose;
}

bool ViewportCamera::eventFilter(QObject * _obj, QEvent * _event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->RefreshCamera();
    if (this->dataPtr->camera)
    {
      this->ApplyToCamera();
      this->MaybeCapture();
    }
  }
  return QObject::eventFilter(_obj, _event);
}

void ViewportCamera::RefreshCamera()
{
  if (this->dataPtr->camera)
    return;
  gz::rendering::ScenePtr scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!scene)
    return;
  for (unsigned int i = 0; i < scene->NodeCount(); ++i)
  {
    auto cam = std::dynamic_pointer_cast<gz::rendering::Camera>(
      scene->NodeByIndex(i));
    if (!cam)
      continue;
    gz::rendering::Variant flag = cam->UserData("user-camera");
    const bool * isUser = std::get_if<bool>(&flag);
    if (isUser && *isUser)
    {
      this->dataPtr->camera = cam;
      return;
    }
  }
}

void ViewportCamera::ApplyToCamera()
{
  const rclcpp::Time now = this->dataPtr->ros->now();
  bool oneShot = false;
  bool streaming = false;
  bool localSet = false;
  gz::math::Pose3d localPose;
  bool worldOrientation = false;
  double sampledFov = 0.0;
  std::optional<double> pendingFov;
  std::optional<gz::rendering::CameraProjectionType> pendingProjection;
  std::string refEntity;
  gz::math::Pose3d refPose;
  uint8_t refMode = SetReferenceFrame::Request::FULL;
  std::optional<gz::math::Pose3d> refTargetPose;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    oneShot = this->dataPtr->oneShot;
    this->dataPtr->oneShot = false;
    // Drop keyframes fully in the past, keeping the one bracketing `now`.
    while (this->dataPtr->viewBuffer.size() > 1 &&
      this->dataPtr->viewBuffer[1].time <= now)
      this->dataPtr->viewBuffer.pop_front();
    streaming = this->dataPtr->streaming;
    if (streaming &&
      std::chrono::steady_clock::now() - this->dataPtr->lastView > kStreamTimeout)
    {
      this->dataPtr->streaming = false;  // stream went quiet: release to manual
      streaming = false;
    }
    localSet = this->dataPtr->localSet;
    if (oneShot)
    {
      localPose = this->dataPtr->localPose;
      worldOrientation = this->dataPtr->worldOrientation;
    }
    else
    {
      SampleBuffer(this->dataPtr->viewBuffer, now, localPose, worldOrientation, sampledFov);
    }
    std::swap(pendingFov, this->dataPtr->pendingFov);
    std::swap(pendingProjection, this->dataPtr->pendingProjection);
    refEntity = this->dataPtr->refEntity;
    refPose = this->dataPtr->refPose;
    refMode = this->dataPtr->refMode;
    refTargetPose = this->dataPtr->refTargetPose;
  }

  const gz::rendering::CameraPtr & camera = this->dataPtr->camera;
  if (pendingProjection)
    camera->SetProjectionType(*pendingProjection);
  if (pendingFov)
    camera->SetHFOV(gz::math::Angle(*pendingFov));
  else if (sampledFov > 0.0)
    camera->SetHFOV(gz::math::Angle(sampledFov));

  // Resolve the reference world pose. A tracked entity that isn't sampled yet
  // (still spawning, or a wrong name) falls back to the world origin so the shot
  // plays in world frame instead of freezing.
  const gz::math::Pose3d ref =
    ResolveReference(refEntity, refPose, refMode, refTargetPose);

  // Persist-follow state: tracking a resolved entity with no live stream. If the
  // user grabs the camera (its pose drifts from what we last set), drop the track
  // so they keep control until they start a new shot.
  const bool followOnly = refTargetPose && localSet && !streaming && !oneShot;
  if (followOnly && this->dataPtr->appliedPose &&
    Moved(camera->WorldPose(), *this->dataPtr->appliedPose))
  {
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
      this->dataPtr->refEntity.clear();
      this->dataPtr->refPose = gz::math::Pose3d::Zero;
      this->dataPtr->refMode = SetReferenceFrame::Request::FULL;
      this->dataPtr->refTargetPose.reset();
    }
    refTargetPose.reset();
  }

  // Drive while a one-shot placement is pending, the stream is live, or a tracked
  // entity is resolved, a one-shot set_view applies once and then releases. An
  // unresolved track (missing or late entity) does not drive on its own, so a
  // finished stream still releases the camera to manual control.
  const bool drive = oneShot || streaming || (refTargetPose && localSet);
  if (drive)
  {
    // The buffer already interpolates, so apply the composed pose directly.
    this->dataPtr->appliedPose = ComposeWorld(ref, localPose, worldOrientation);
    camera->SetWorldPose(*this->dataPtr->appliedPose);
  }
  else
  {
    // not driving (released to manual): forget the applied pose so the next
    // drive snaps from the camera's current pose instead of a stale one.
    this->dataPtr->appliedPose.reset();
  }

  if (++this->dataPtr->frameCount % kPublishEveryNFrames == 0)
  {
    const gz::math::Pose3d pose = camera->WorldPose();
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->dataPtr->ros->now();
    msg.header.frame_id = kPoseFrame;
    msg.pose.position.x = pose.Pos().X();
    msg.pose.position.y = pose.Pos().Y();
    msg.pose.position.z = pose.Pos().Z();
    msg.pose.orientation.w = pose.Rot().W();
    msg.pose.orientation.x = pose.Rot().X();
    msg.pose.orientation.y = pose.Rot().Y();
    msg.pose.orientation.z = pose.Rot().Z();
    this->dataPtr->posePub->publish(msg);
  }
}

void ViewportCamera::MaybeCapture()
{
  gz::math::Pose3d local;
  bool worldOrientation = false;
  double fov = 0.0;
  std::string refEntity;
  gz::math::Pose3d refPose;
  uint8_t refMode = SetReferenceFrame::Request::FULL;
  std::optional<gz::math::Pose3d> refTargetPose;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (!this->dataPtr->captureRequested)
      return;
    // Defer while the scene hasn't caught up to the requested sim time yet; the
    // request stays pending and a later render (after state propagation) retries.
    if (this->dataPtr->latestSimTime < this->dataPtr->captureMinSimTime)
      return;
    local = this->dataPtr->captureLocal;
    worldOrientation = this->dataPtr->captureWorldOrientation;
    fov = this->dataPtr->captureFov;
    refEntity = this->dataPtr->refEntity;
    refPose = this->dataPtr->refPose;
    refMode = this->dataPtr->refMode;
    refTargetPose = this->dataPtr->refTargetPose;
  }

  // Snap to the requested pose in the active reference frame, exactly as the live
  // drive composes it, then render that frame and read the pixels back.
  const gz::math::Pose3d world = ComposeWorld(
    ResolveReference(refEntity, refPose, refMode, refTargetPose), local, worldOrientation);
  const gz::rendering::CameraPtr & camera = this->dataPtr->camera;
  camera->SetWorldPose(world);
  if (fov > 0.0)
    camera->SetHFOV(gz::math::Angle(fov));

  sensor_msgs::msg::Image image;
  bool ok = false;
  std::string message = "camera produced no readable frame";
  gz::rendering::Image frame = camera->CreateImage();
  camera->Capture(frame);
  const unsigned int w = camera->ImageWidth();
  const unsigned int h = camera->ImageHeight();
  const unsigned int pixels = w * h;
  const unsigned char * src = frame.Data<unsigned char>();
  const unsigned int channels = pixels ? frame.MemorySize() / pixels : 0;
  if (src && channels >= 3)
  {
    image.width = w;
    image.height = h;
    image.encoding = "rgb8";
    image.step = w * 3;
    image.data.resize(static_cast<std::size_t>(pixels) * 3);
    for (unsigned int p = 0; p < pixels; ++p)  // copy RGB, dropping any alpha
    {
      image.data[p * 3 + 0] = src[p * channels + 0];
      image.data[p * 3 + 1] = src[p * channels + 1];
      image.data[p * 3 + 2] = src[p * channels + 2];
    }
    image.header.stamp = this->dataPtr->ros->now();
    image.header.frame_id = kPoseFrame;
    ok = true;
    message = "ok";
  }

  // Keep appliedPose consistent so a later live drive resumes without a jump.
  this->dataPtr->appliedPose = world;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->captureImage = std::move(image);
    this->dataPtr->captureOk = ok;
    this->dataPtr->captureMsg = std::move(message);
    this->dataPtr->captureRequested = false;
    this->dataPtr->captureDone = true;
  }
  this->dataPtr->captureCv.notify_all();
}

}  // namespace arena_gz_plugins

GZ_ADD_PLUGIN(
  arena_gz_plugins::ViewportCamera,
  gz::gui::Plugin)
