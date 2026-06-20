import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: viewportCamera
  color: "transparent"
  Layout.minimumWidth: 280
  Layout.minimumHeight: 60
  anchors.fill: parent

  Label {
    anchors.fill: parent
    anchors.margins: 10
    wrapMode: Text.WordWrap
    verticalAlignment: Text.AlignVCenter
    text: "Viewport camera control on /arena/viewport/* " +
          "(set_view, set_reference_frame, set_projection, cmd_view, camera_pose)."
  }
}
