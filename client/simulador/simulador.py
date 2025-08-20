"""
- Este simulador escucha señales Qt (posiciónUSV_real, metaActualizada_real, obstaculosActualizados)
  que llegan desde el SocketHandler.
- Convierte todo a un sistema interno XY relativo al origen (0,0).
- Cada iteración del QTimer ejecuta avanzar():
    * Llama a calcular_recomendacion(), algoritmo de PLANIFICADOR LOCAL Y AVANCE LIBRE.
    * Mueve el USV.
    * Actualiza GUI, métricas y logs.
"""

from PySide6.QtCore import QObject, QTimer, Signal, QPointF, QDateTime # Importa clases Qt necesarias
import numpy as np # Librería para operaciones matemáticas y vectoriales
import csv, json  # Para guardar datos en archivos CSV y JSON
from pyproj import Transformer  # Para transformar coordenadas UTM <-> WGS84

#  IMPORTO el módulo para poder sincronizar dt si quiero
from client.APF import recomendacion as reco_mod # Importa el módulo completo para modificar dt_step
from client.APF.recomendacion import calcular_recomendacion # Importa directamente la función de recomendación (importante)


class SimuladorAPF(QObject):   # Clase de QObject para usar señales QT
    # Definición de señales Qt para comunicar con la interfaz QML
    alertaActualizada    = Signal(str)             # Señal que envía mensajes de estado
    actualizarObstaculos = Signal(list)            # Señal para recibir obstáculos desde fuera
    posicionInterna      = Signal(float, float)    # Señal para reportar posición interna (x,y)
    metaInterna          = Signal(float, float)    # Señal para reportar la meta interna
    tiempoActualizado    = Signal(float)           # Señal para reportar tiempo transcurrido

    def __init__(self, parent=None):  #Inicia  el objeto Qt.
        super().__init__(parent)

        # ---- Config inicial ---- Velocidad del USV, v_max se le pasa al planner como límite de avance por iteración.
        self.v_knots       = 15.0
        self.v_max         = self.v_knots * 0.514444

        # ---- Estado general ----
        self._origin_raw   = None      # Guarda origen UTM real (x0,y0)
        self._origin_fijado = False    # Bandera o flag para saber si origen ya fue fijado
        self._meta_fijada   = False    # Bandera o flag para saber si la meta ya fue fijada
        self._start_time    = None     # Hora de inicio de la simulación
        self._dist_inicial  = None     # Distancia inicial entre origen y meta

        # Posiciones y estado dinámico
        self.robot_pos   = QPointF(0.0, 0.0)   # Posición actual del USV en coordenadas internas
        self.goal_pos    = None                # Posición de la meta (interna)
        self.obstaculos  = []                  # Lista de obstáculos [(QPointF, tipo), ...]
        self.hist_pos    = []      # historial corto para suavizar

        # ---- Trayectoria y logs ----
        self.trayectoria = []   # Guarda todos los puntos de la trayectoria interna
        self.log         = []   # Logs adicionales opcionales

        # ---- Timer ----
        self.timer = QTimer(self)                # Temporizador Qt
        self.timer.timeout.connect(self.avanzar) # Llama a avanzar() en cada tick
        self.actualizarObstaculos.connect(self._on_obstaculos_actualizados)  
        # Conecta la señal de obstáculos al método interno que actualiza la lista

        # ---- Métricas para análisis ----
        self.run_id      = QDateTime.currentDateTime().toString("yyyyMMdd_HHmmss") #como quedará guardado el archivo .csv
        self.dist_total  = 0.0      # Distancia total recorrida
        self.dist_apf    = 0.0      # Distancia recorrida en modo APF
        self.dist_pre    = 0.0      # Distancia recorrida en modo preventivo
        self.dist_escape = 0.0      # Distancia recorrida en escape lateral
        self.n_escape    = 0        # Número de escapes laterales realizados
        self.max_dpsi    = 0.0      # Máximo cambio de rumbo registrado
        self._rumbo_prev = None     # Rumbo anterior (para calcular variaciones)
        self._pos_prev   = None     # Posición anterior (para calcular distancias paso a paso)

        self.PMA_min    = float("inf")  # Clearance mínimo (distancia mínima a obstáculo)
        self.step_log   = []            # Registro de datos por cada paso del simulador
        self.last_summary = None        # Resumen final de la corrida
        self.min_pos    = []   # [(id_obs, x_min, y_min, d_min), ...]; guarda posiciones de mínima distancia a obstáculos

    # ==========================================================
    #  Origen y Meta
    # ==========================================================

    def fijar_origin(self, x0, y0): 
        """Fija el origen UTM real → (0,0) interno"""
        self._origin_raw = (x0, y0) # Guarda coordenadas reales como origen base
        if not self._origin_fijado: # Solo se fija la primera vez
            self.robot_pos      = QPointF(0.0, 0.0) #se pone el usv en el 0,0
            self._origin_fijado = True #marcar que ya está definido
            print(f"🌐 Origin={self._origin_raw} → interna (0,0)")
            self._maybe_start()   #si hay meta se inicia el simulador

    def fijar_meta(self, x_meta, y_meta):
        """Convierte meta a coordenadas internas y arranca si corresponde."""
        if self._meta_fijada:   # Si ya estaba fijada, no hace nada, true si la meta ya está definida
            return
        self._meta_fijada = True

        ox, oy = self._origin_raw # Origen real en UTM, convierte coordenada X em meta internay al igual que el Y
        xi = (x_meta - ox)
        yi = (y_meta - oy)

        self.goal_pos     = QPointF(xi, yi)   #GUardar meta coord. internas
        self._start_time  = QDateTime.currentDateTime() #hora inicio simulación
        self._dist_inicial = np.hypot(xi, yi) #distnacia inicial origen - meta 

        print(f"🏁 Meta interna → X={xi:.1f}, Y={yi:.1f}")
        self.metaInterna.emit(xi, yi)
        self._maybe_start()

    def finalizar__llegada(self):
        # Tiempo transcurrido desde inicio
        elapsed_ms  = self._start_time.msecsTo(QDateTime.currentDateTime())
        elapsed_min = elapsed_ms / 1000.0 / 60.0 # conversión de ms a min
        teoric_min  = self._dist_inicial / 1852.0 / self.v_knots * 60 # Tiempo teórico (en min) a la velocidad establecida

        self.robot_pos = QPointF(self.goal_pos) #USV en la meta
        self.alertaActualizada.emit(
        f"✅ Llegó en {elapsed_min:.1f} min (teórico {teoric_min:.1f})" #para cuando termine la simulación
    )
        self.timer.stop()   #deteneoms el simuladora y guardamos resultados
        self._guardar_logs() 
    
    
    
    def _maybe_start(self):
        """Inicia el timer sólo si hay origen y meta definidos."""
        if self._origin_fijado and self.goal_pos and not self.timer.isActive():
            print(f"▶️ Iniciando APF @ {self.v_knots} kn")  # Mensaje de inicio, en el temrinal
            self.timer.start(41)  # ms  (41 aproximado a 24 hz, para buscar una simulación más continua.

    # ==========================================================
    #   Obstáculos
    # ==========================================================

    def _on_obstaculos_actualizados(self, lista_xy_tipo):
        """
        Recibe [(x_int, y_int, tipo), ...] y guarda como [(QPointF, tipo), ...]
        """
        self.obstaculos = [(QPointF(x, y), tipo) for x, y, tipo in lista_xy_tipo]

        for pt, tipo in self.obstaculos:
            print(f"🛑 Obstáculo de tipo '{tipo}' en interna → ({pt.x():.1f},{pt.y():.1f})")

    # ==========================================================
    #   cada iteración del simulador
    # ==========================================================

    def avanzar(self):
        # 1) dt del simulador (seg por iteración)
        dt = self.timer.interval() / 1000.0
        # sincronizar el planificador APF
        reco_mod.dt_step = dt   # sincronización del dt del planificador con el simulador

        # 2) Distancia a meta ANTES de mover
        dx = self.goal_pos.x() - self.robot_pos.x()
        dy = self.goal_pos.y() - self.robot_pos.y()
        dist = np.hypot(dx, dy) #dist. actual a la meta

        # Emito posición actual al QML o al GUI(para no perder el print externo)
        self.posicionInterna.emit(self.robot_pos.x(), self.robot_pos.y())

        # 3) Chequeo de llegada (snap final si ya está encima)
        if dist < 1.0:
            self.finalizar__llegada()
            return

        # 4) Planificador
        reco = calcular_recomendacion(
            (self.robot_pos.x(), self.robot_pos.y()),       # Posición actual
            (self.goal_pos.x(),  self.goal_pos.y()),        # Meta
            [(pt.x(), pt.y(), tipo) for pt, tipo in self.obstaculos],  # Obstáculos
            historial=self.hist_pos,                        # Historial de posiciones
            v_max=self.v_max                                # Velocidad máxima permitida
        )

        # Guardar punto mínimo si el planner lo entrega, ojo que se guarda solo en el .json el punto mínimo al obstáculo más cercano, si es que hay más obstáculos
        if reco.get("pos_min_iter") is not None:
            self.min_pos.append((
                reco["obst_min_id"],
                *reco["pos_min_iter"],
                reco["distancia_minima"]
            ))

        # 5) Movimiento según recomendación, importante
        rumbo_apf = reco["rumbo"]               # Rumbo en grados
        ang       = np.radians(rumbo_apf)       # Convierte a radianes
        vx        = np.cos(ang) * reco["velocidad"]   # Velocidad en X
        vy        = np.sin(ang) * reco["velocidad"]   # Velocidad en Y

        # para NO pasar la meta
        step_dx  = vx * dt
        step_dy  = vy * dt
        step_len = np.hypot(step_dx, step_dy) #longitud del paso que daría el USV
        if step_len > dist:                     #si el paso es mayor quer la dist. a la meta, se recalcula un paso más corto
            scale   = dist / step_len
            step_dx *= scale    #se reduce proporcional el paso en X e Y
            step_dy *= scale

        self.robot_pos.setX(self.robot_pos.x() + step_dx)  #actualizamos la posición
        self.robot_pos.setY(self.robot_pos.y() + step_dy)
        self.trayectoria.append((self.robot_pos.x(), self.robot_pos.y()))  #guardar la trayectoria

        # 6) Chequeo de llegada después de mover
        dist2 = np.hypot(self.goal_pos.x() - self.robot_pos.x(),
                         self.goal_pos.y() - self.robot_pos.y())
        if dist2 < 1.0:
            self.finalizar__llegada()
            return

        # 7) Historial corto 
        self.hist_pos.append((self.robot_pos.x(), self.robot_pos.y()))
        if len(self.hist_pos) > 5:
            self.hist_pos.pop(0)

        # 8) Métricas y log por paso
        secs = 0.0
        if self._start_time is not None:
            secs = self._start_time.msecsTo(QDateTime.currentDateTime()) / 1000.0
            self.tiempoActualizado.emit(secs)

        mm = int(secs // 60)
        ss = secs % 60

        # distancia del paso actual
        if self._pos_prev is None:
            d_step = 0.0
        else:
            d_step = np.hypot(self.robot_pos.x() - self._pos_prev.x(),
                              self.robot_pos.y() - self._pos_prev.y())

        self.dist_total += d_step  # Suma a distancia total
        maniobra = reco["maniobra"] # Tipo de maniobra usada
        if maniobra == "AVANCE APF":
            self.dist_apf += d_step
        elif maniobra == "AVANCE PREVENTIVO":
            self.dist_pre += d_step
        elif maniobra == "ESCAPE LATERAL":
            self.dist_escape += d_step
            self.n_escape    += 1

        if reco["PMA"] is not None: #PMA mínimo alcanzado
            self.PMA_min = min(self.PMA_min, reco["PMA"])

        if self._rumbo_prev is not None:  # Variación de rumbo
            dpsi = abs(((rumbo_apf - self._rumbo_prev + 180) % 360) - 180)
        else:
            dpsi = 0.0
        if dpsi > self.max_dpsi:
            self.max_dpsi = dpsi
        self._rumbo_prev = rumbo_apf
        self._pos_prev   = QPointF(self.robot_pos)

        # log por iteración
        self.step_log.append({
            "t_s"          : secs,
            "x"            : self.robot_pos.x(),
            "y"            : self.robot_pos.y(),
            "rumbo_deg"    : rumbo_apf,
            "vel"          : reco["velocidad"],
            "maniobra"     : maniobra,
            "PMA"          : reco["PMA"],
            "sector_min"   : reco["sector_min"],
            "ang_min"      : reco["ang_min"],
            "R_geo_min"    : reco["R_geo_min"],
            "dist_step"    : d_step,
            "dist_minima"  : reco["distancia_minima"],
            "F_rep"        : reco.get("F_rep"),
            "F_Att"        : reco.get("F_Att")
        })

        # Mensaje a GUI
        grados_gui = (90 - rumbo_apf + 1.4) % 360
        msg  = (reco["alerta"] + "\n") if reco["alerta"] else ""
        msg += (f"📍 ({self.robot_pos.x():.1f},{self.robot_pos.y():.1f})\n"
                f"🧭 Rumbo: {grados_gui:.1f}°\n"
                f"🚀 Vel.: {reco['velocidad']} m/s\n"
                f"🛑 Maniobra: {maniobra}\n"
                f"⏱️ Tiempo: {mm:02d}:{ss:04.1f} s")
        self.alertaActualizada.emit(msg)

    # ==========================================================
    #   Guardado de logs
    # ==========================================================

    def _guardar_logs(self):
        # 1) Trayectoria interna + a GPS
        np.save("trayectoria.npy", np.array(self.trayectoria))

        if self._origin_raw is not None:
            transformer = Transformer.from_crs("EPSG:32719", "EPSG:4326", always_xy=True)
            tray_json = []
            for x, y in self.trayectoria:
                x_utm = x + self._origin_raw[0]
                y_utm = y + self._origin_raw[1]
                lon, lat = transformer.transform(x_utm, y_utm)
                tray_json.append({"lat": lat, "lon": lon})
            with open("trayectoria_gps.json", "w", encoding="utf-8") as f:
                json.dump(tray_json, f, indent=2, ensure_ascii=False)
            print("[✔] Exportado → trayectoria_gps.json")
        else:
            print("[⚠] No se pudo exportar trayectoria GPS: origen UTM no fijado.")

        # 2) Obstáculos internos + GPS
        obst_data = []
        for pt, tipo in self.obstaculos:
            obst_data.append([pt.x(), pt.y(), tipo])
        np.save("obstaculos.npy", np.array(obst_data, dtype=object))

        if self._origin_raw is not None:
            transformer = Transformer.from_crs("EPSG:32719", "EPSG:4326", always_xy=True)
            obst_json = []
            for pt, tipo in self.obstaculos:
                x_utm = pt.x() + self._origin_raw[0]
                y_utm = pt.y() + self._origin_raw[1]
                lon, lat = transformer.transform(x_utm, y_utm)
                obst_json.append({"lat": lat, "lon": lon, "tipo": tipo})
            with open("obstaculos_gps.json", "w", encoding="utf-8") as f:
                json.dump(obst_json, f, indent=2, ensure_ascii=False)
            print("[✔] Exportado → obstaculos_gps.json")
        else:
            print("[⚠] No se pudo exportar obstáculos: origen UTM no fijado.")

        # 3) Posiciones mínimas por obstáculo
        if self.min_pos:
            best = {}
            for idx, xm, ym, d in self.min_pos:
                if idx not in best or d < best[idx][2]:
                    best[idx] = (xm, ym, d)
            min_pos_arr  = np.array([[x, y] for _, (x, y, _) in sorted(best.items())],
                                    dtype=float)
            min_dist_arr = np.array([d for _, (_, _, d) in sorted(best.items())],
                                    dtype=float)
            np.save("min_pos.npy",  min_pos_arr)
            np.save("min_dist.npy", min_dist_arr)
            print("[✔] Exportado → min_pos.npy  /  min_dist.npy")

        # 4) CSV paso a paso
        if self.step_log:
            fname_steps = f"run_{self.run_id}_steps.csv"
            with open(fname_steps, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=self.step_log[0].keys())
                w.writeheader()
                w.writerows(self.step_log)
            print(f"[✔] Exportado → {fname_steps}")

        # 5) Resumen final
        summary = {
            "run_id"       : self.run_id,
            "dist_total_m" : round(self.dist_total, 2),
            "dist_apf_m"   : round(self.dist_apf, 2),
            "dist_pre_m"   : round(self.dist_pre, 2),
            "dist_escape_m": round(self.dist_escape, 2),
            "n_escape"     : self.n_escape,
            "PMA_min_m"    : None if self.PMA_min == float("inf") else round(self.PMA_min, 2),
            "max_dpsi_deg" : round(self.max_dpsi, 1),
            "v_knots"      : self.v_knots,
            "dt_s"         : self.timer.interval()/1000.0,
        }
        self.last_summary = summary

        fname_sum = f"run_{self.run_id}_summary.json"
        with open(fname_sum, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        print(f"[✔] Exportado → {fname_sum}")

        # 6) log_usv.csv (si lo usas)
        if self.log:
            with open("log_usv.csv", "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=self.log[0].keys())
                w.writeheader()
                w.writerows(self.log)
