import QtQuick 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    id: root
    visible: true
    width: 800; height: 600
    title: "Panel de Navegación Autónoma"

    // ... tu Canvas y el resto ...

    Rectangle {
        anchors.fill: parent
        color: "black"

        Column {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top; anchors.topMargin: 20
            spacing: 10

            Text {
                text: "Estado del USV"
                font.pixelSize: 30; color: "white"
            }
            Text {
                id: alertaText
                text: estadoUSV.estadoTexto
                font.pixelSize: 20; color: "orange"
            }


            
        }
    }

    Connections {
        target: simuladorAPF
        // ... el resto de tus conexiones ...
    }
}
