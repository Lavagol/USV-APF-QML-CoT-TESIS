import sys
##from socket_handler import SocketHandler
from utils.servidor_ip import start_server
import os

# Configurar las variables de entorno para Qt solo en macOS
if sys.platform == "darwin":
    os.environ["QML2_IMPORT_PATH"] = "/Users/anibalcaceresguajardo/Qt/6.7.3/macos/qml"
    os.environ["QT_PLUGIN_PATH"] = "/Users/anibalcaceresguajardo/Qt/6.7.3/macos/plugins"
    
    
if __name__ == "__main__":
        start_server()
