#pragma once

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>

namespace arena_gz_plugins
{

class ViewportCameraPrivate;

/// GUI plugin that exposes the viewport user-camera over ROS 2, so external
/// scripts (e.g. scene recording) can drive the view without touching gz.
///
/// Services (advertised under the global /arena/viewport namespace):
///   set_view             ViewportSetView             one-shot look from eye toward target
///   set_reference_frame  ViewportSetReferenceFrame   frame the view is expressed in
///   set_projection       ViewportSetProjection       perspective | orthographic
/// Subscribes to /arena/viewport/cmd_view (ViewportView) for streamed poses and
/// publishes the live camera pose on /arena/viewport/camera_pose (PoseStamped).
/// The capture service (ViewportCapture) snaps to an exact pose, renders, and
/// returns the frame, so an external recorder can dump a deterministic sequence.
/// A request may set min_sim_time to defer the render until the scene has caught
/// up to that sim time (tracked from UpdateInfo.simTime each Update).
///
/// The camera pose is composed every render frame as reference * local: the
/// reference is the world origin, a constant pose, or a tracked entity (sampled
/// from the ECM in Update); the local pose comes from set_view or the stream.
/// Threading: ROS runs on its own spin thread; the user-camera is only touched
/// on the render thread (the gz::gui Render event), state crosses under a mutex.
class ViewportCamera : public gz::sim::GuiSystem
{
  Q_OBJECT

public:
  ViewportCamera();
  ~ViewportCamera() override;

  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  void Update(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

protected:
  bool eventFilter(QObject * _obj, QEvent * _event) override;

private:
  /// Resolve the user-camera from the render scene, once it exists. Render thread.
  void RefreshCamera();

  /// Drain queued commands onto the camera and publish its pose. Render thread.
  void ApplyToCamera();

  /// Fulfil a pending capture request: snap to the exact pose, render, read the
  /// pixels back, and wake the waiting service. No-op if none pending. Render thread.
  void MaybeCapture();

  std::unique_ptr<ViewportCameraPrivate> dataPtr;
};

}  // namespace arena_gz_plugins
