from pathlib import Path
from PySide6.QtCore    import QObject, Signal, Property, QUrl
from PySide6.QtWidgets import QApplication
from PySide6.QtQml     import QQmlApplicationEngine

from client.simulador.simulador             import SimuladorAPF
from client.handlers.socket_handlercorreo  import SocketHandler

import sys

# ───────── Modelo que expone el texto al QML ─────────
class ModeloEstado(QObject):
    estadoTextoChanged = Signal()
    tiempoTextoChanged =Signal()
    def __init__(self):
        super().__init__()
        self._estado = "Sin alertas"
        self._tiempo= "0.0 s" #valor inicial
    def getEstado(self):
        return self._estado

    def setEstado(self, valor):
        if self._estado != valor:
            self._estado = valor
            self.estadoTextoChanged.emit()

    estadoTexto = Property(str, getEstado, setEstado, notify=estadoTextoChanged)

    def getTiempo(self):
        return self._tiempo
    
    def setTiempo(self, valor):
        if self._tiempo != valor:
            self._tiempo = valor
            self.tiempoTextoChanged.emit()

    tiempoTexto = Property (str,getTiempo,setTiempo, notify=tiempoTextoChanged)

# ───────────────────── MAIN ──────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    engine = QQmlApplicationEngine()
    
    # — 1) instancio primero modelo, socket y simulador —
    modelo = ModeloEstado()
    socket_thread = SocketHandler(host="127.0.0.1", port=65432)
    simulador     = SimuladorAPF()

    # — 2) Luego las registro en el contexto QM —
    ctx = engine.rootContext()
    ctx.setContextProperty("estadoUSV",     modelo)
    ctx.setContextProperty("socketHandler", socket_thread)
    ctx.setContextProperty("simuladorAPF",  simulador)

    # 3) Cargo la interfaz
    qml_path = Path(__file__).parent / "ui" / "interface.qml"
    engine.load(QUrl.fromLocalFile(qml_path.as_posix()))
    if not engine.rootObjects():
        sys.exit("❌ No se pudo cargar interface.qml")

    # ─── lógica de red y planificador ───
    #socket_thread = SocketHandler(host="127.0.0.1", port=65432)
    #simulador     = SimuladorAPF()

    #  ─── Conexiones entre socket y simulador ───
    socket_thread.posicionUSV_real.connect(simulador.fijar_origin)
    socket_thread.metaActualizada_real.connect(simulador.fijar_meta)
    socket_thread.obstaculosActualizados.connect(simulador.actualizarObstaculos)

    # conectar alertas del simulador a QML
    simulador.alertaActualizada.connect(modelo.setEstado)
    #para mostrar el Qtimer
    simulador.tiempoActualizado.connect(
    lambda s: modelo.setTiempo(f"{s:,.1f} s")  

)
    # (Opcionales, para debug en consola:)
    socket_thread.rumboGeodesico.connect(
        lambda r: print(f"[GEO] Rumbo geodésico → {r:.1f}°")
    )

    simulador.posicionInterna.connect(
        lambda x, y: print(f"[GUI] Pos interna → X={x:.1f}, Y={y:.1f}")
    )
    
    simulador.metaInterna.connect(
        lambda xm, ym: print(f"[GUI] Meta interna → X={xm:.1f}, Y={ym:.1f}")
    )

    socket_thread.obstaculosActualizados_real.connect(
        lambda obs: print(
            "[OBS REAL] " +
            "; ".join(f"({x:.1f},{y:.1f})" for x, y, _ in obs)
        )
    )

    # 4) Arranco el hilo de socket y la aplicación
    socket_thread.start()
    app.aboutToQuit.connect(socket_thread.stop)

    sys.exit(app.exec())