import socket
import time
import xml.etree.ElementTree as ET
from PySide6.QtCore import QThread, Signal
from pyproj import Transformer, Geod # (WGS 84 → UTM 19 S) (rumbo geodésico).
import numpy as _np
from client.models.parametros_obstaculos import PARAMS, clasificar_obstaculo

class SocketHandler(QThread):
    mensajeRecibido = Signal(str)
    conexionCaida = Signal()
    coordenadasRecibidas = Signal(float, float, str, float, float, float, float, int, int)
    conexionExitosa = Signal()
    solicitarCOT = Signal()
    motorEstadoCambiado = Signal(bool)  # True = ON , False = OFF
    #nueva señal para los OBTACULOS
    #obstaculosActualizados=Signal(list)
    obstaculosActualizados       = Signal(list) #x,y,tipo) y _real (UTM)
    obstaculosActualizados_real  = Signal(list)
    posicionUSV = Signal(float, float)
    metaActualizada = Signal(float, float)     # lat, lon
    # señales para la simulación en metros reales (sin escala)
    posicionUSV_real       = Signal(float, float) # UTM reales en m
    metaActualizada_real   = Signal(float, float) #análogas.
    rumboGeodesico = Signal(float)
# ------------------------------------------------------------------
    # Parámetros de proyección y filtrado (añadidos para AEQD y cross-track)
    Escala=1.0

    def __init__(self, host="127.0.0.1", port=65432):  #self, host="10.3.141.201", port=65432, Crea socket
        super().__init__()
        self.host = host
        self.port = port
        self.is_running = True
        self.conectado = False
        self.socket = None
        self.cr_enviado = False
        self.ultimo_mensaje = None

        self.connected_since = None  # <--- Nuevo atributo
        self.ultimo_envio = 0  # Timestamp del último envío
        self.intervalo_minimo = 1  # segundos entre comandos (1000ms)
        self.tr_utm = Transformer.from_crs("EPSG:4326", "EPSG:32719", always_xy=True) #proyección fija
        self._origin_utm=None #se fijará al primer punto recibido
        #self.Escala=0.2
        self.geod = Geod(ellps="WGS84") 
    def estaConectado(self):
        """Devuelve True si el socket está conectado, de lo contrario False."""
        return self.conectado and self.socket is not None

    def procesarMensaje(self, response: str): # núcleo de conversión
        """Procesar el mensaje recibido desde el servidor"""
        print(f"Respuesta del servidor: {response}")
        self.ultimo_mensaje = response

        # Validación rápida de XML
        if not (response.startswith("<?xml") and "<event" in response and "</event>" in response):
            print("El mensaje recibido no es un XML válido.")
            self.conexionExitosa.emit()
            return

        self.mensajeRecibido.emit(response)

        try: #Extrae lat/lon
            root = ET.fromstring(response)
            point = root.find("point")
            detail = root.find("detail")

            # --- 1) Extraer coordenadas GPS ---
            lat0 = float(point.get('lat', '0'))
            lon0 = float(point.get('lon', '0'))

            meta_txt = detail.get('meta', '0,0')
            lat_meta, lon_meta = map(float, meta_txt.split(','))

             # ——— 1.5) PROYECCIÓN UTM reales (metros) ———
            x0_utm, y0_utm = self.tr_utm.transform(lon0,   lat0)
            xm_utm, ym_utm = self.tr_utm.transform(lon_meta, lat_meta)
            # emito al simulador sin escala:
            #self._origin_utm = (x0_utm, y0_utm)
            #fijar la referencia UTM solo una vez
            if self._origin_utm is None:
                self._origin_utm=(x0_utm, y0_utm)
            self.posicionUSV_real.emit(x0_utm, y0_utm)
            self.metaActualizada_real.emit(xm_utm, ym_utm)

            # TEST de tiempo a 5 nudos (opcional)
            d_meta_m  = _np.hypot(xm_utm - x0_utm, ym_utm - y0_utm)
            d_meta_nm = d_meta_m / 1852.0
            t_min     = (d_meta_nm / 5.0) * 60.0
            print(f"[TEST] Dist real: {d_meta_m:.1f} m → {t_min:.1f} min @5 kn")
            
            # Cálculo del rumbo geodésico real con pyproj.Geod
            rumbo_geo, _, _ = self.geod.inv(lon0, lat0, lon_meta, lat_meta)
            rumbo_geo = (rumbo_geo + 360) % 360
            print(f"[GEO] Rumbo geodésico WGS84: {rumbo_geo:.1f}°")
            self.rumboGeodesico.emit(rumbo_geo)
            # --- 2) Proyección UTM 19S ---
            def geo_to_xy(lat, lon): # Convierte lat/lon → UTM
                # primero a UTM
                x, y = self.tr_utm.transform(lon, lat)
                ox, oy = self._origin_utm
                return (x - ox), (y - oy) #* self.Escala Resta el origen UTM
                # lo pasamos a interno restando el origen
                #xi = (x - self._origin_utm[0]) * self.Escala
                #yi = (y - self._origin_utm[1]) * self.Escala
                #return xi, yi

            # Emitir USV y meta en metros   #Emite posicionUSV(x,y) y metaActualizada(x,y) para la simulación en plano XY.
            x0, y0 = geo_to_xy(lat0, lon0)
            self.posicionUSV.emit(x0, y0)

            xm, ym = geo_to_xy(lat_meta, lon_meta)
            self.metaActualizada.emit(xm, ym)

            # Obstáculos: igual, restamos origen
            #obs = []
            #for k, v in detail.attrib.items():
            #    if k.startswith('obstaculo'):
            #        try:
            #           lo, ln = map(float, v.split(','))
            #           obs.append((lo, ln))
            #        except ValueError:
            #           pass


  
            #obs_xy = [geo_to_xy(lo, ln) for lo, ln in obs]
            #self.obstaculosActualizados.emit(obs_xy)

            # ——— Obstáculos + clasificación ———
            # esperamos que el XML envíe también algo como tipo_obstaculo1="Piedra", tipo_obstaculo2="Barco",….
            obst_raw = []
            for clave, valor in detail.attrib.items():
                # buscamos claves 'obstaculoN'
                if clave.startswith('obstaculo') and not clave.startswith('tipo_'):
                    idx = clave.replace('obstaculo','')                     # e.g. '1'
                    lat_o, lon_o = map(float, valor.split(','))
                # leemos su tipo bruto (o cadena vacía si no viene)
                    tipo_bruto = detail.attrib.get(f'tipo_obstaculo{idx}', '')
                # normalizamos con tu función de clasificación
                    tipo = clasificar_obstaculo(tipo_bruto)
                    obst_raw.append((lat_o, lon_o, tipo))

        # proyectamos y emitimos con (x, y, tipo)
            obs_xy_tipo = [
                (*geo_to_xy(lat_o, lon_o), tipo)
                for lat_o, lon_o, tipo in obst_raw
            ]
            self.obstaculosActualizados.emit(obs_xy_tipo)            


            #Obtáculos en UTM reales(sin escala)
            #obs_real=[
             #   (
             #       self.tr_utm.transform(ln, lo)[0] - self._origin_utm[0],
             #       self.tr_utm.transform(ln, lo)[1] - self._origin_utm[1],
             #   )
             #   for lo, ln in obs
            #]
            obs_real_tipo = [] #	Emite obstaculosActualizados (x,y,tipo) y obstaculosActualizados_real (UTM, tipo).
            for lat_o, lon_o, tipo in obst_raw:
                ux, uy = self.tr_utm.transform(lon_o, lat_o)
                obs_real_tipo.append((ux - self._origin_utm[0],
                                    uy - self._origin_utm[1],
                                    tipo))

            self.obstaculosActualizados_real.emit(obs_real_tipo)

                #    calculas distancias USV→Meta y USV→cada obstáculo Debug: distancias USV→Meta y USV→Obs-i en metros.	
            def _dist(a, b):
                return _np.hypot(a[0]-b[0], a[1]-b[1])
    
                # distancia USV→Meta
            d_meta = _dist((x0, y0), (xm, ym))
            print(f"[VALID] Dist USV→Meta: {d_meta:.1f} m")

                # distancias USV→Obstáculo
            for i, (x_o, y_o, _) in enumerate(obs_xy_tipo, start=1):
                d_o = _dist((x0, y0), (x_o, y_o))
                print(f"[VALID] Dist USV→Obs{i}: {d_o:.1f} m")
          
            # --- 3) Estado del motor ---
            estado_motor = detail.attrib.get('estado_motor', '0')
            self.motorEstadoCambiado.emit(estado_motor in ('1', '2'))

            # --- 4) Datos numéricos crudos ---
            lat_raw = lat0
            lon_raw = lon0
            ce    = point.get('ce', '0')
            temp  = float(detail.attrib.get('temp', '0'))
            speed = float(detail.attrib.get('speed', '0'))
            fuel  = float(detail.attrib.get('fuel_level', '0'))
            vbat  = float(detail.attrib.get('vbat', '0'))
            rpm   = int(detail.attrib.get('rpm', '0'))

            self.coordenadasRecibidas.emit(
                lat_raw, lon_raw, ce, temp, speed, fuel, vbat,
                int(estado_motor), rpm
            )

            # --- 5) Señal de conexión confirmada ---
            if detail.attrib.get('contact_callsign') == 'USV-DIPRIDA':
                self.conexionExitosa.emit()

        except ET.ParseError as e:
            print(f"Error al parsear el XML: {e}")


    def run(self):
        print("Ejecutando hilo del socket.")

        while self.is_running:
            # Conexión inicial
            if not self.socket:
                try:
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.socket.settimeout(5.0)
                    print(f"Conectando a {self.host}:{self.port}...")
                    self.socket.connect((self.host, self.port))
                    self.conectado = True
                    self.connected_since = time.time()
                    self.cr_enviado = False
                    print("Conexión establecida con el servidor.")
                    self.solicitarCOT.emit()
                except socket.error as e:
                    print(f"Error al conectar al servidor: {e}")
                    self.conexionCaida.emit()
                    if self.socket:
                        self.socket.close()
                    self.socket = None
                    time.sleep(5)
                    continue

            # Enviar CR si no se ha enviado aún
            if self.conectado and not self.cr_enviado:
                try:
                    print("Enviando mensaje CR...")
                    self.socket.sendall(b"CR")
                    response = self.socket.recv(4096)
                    if not response:
                        raise socket.error("Sin respuesta al CR")
                    self.procesarMensaje(response.decode().strip())
                    self.cr_enviado = True
                    time.sleep(10)
                except (socket.timeout, socket.error) as e:
                    print(f"Error durante CR: {e}")
                    self.conexionCaida.emit()
                    self._handle_disconnect()
                    time.sleep(5)
                    continue

            # Polling GET_COT
            if self.conectado:
                try:
                    time.sleep(3)
                    self.enviar_comando("GET_COT")
                    response = self.socket.recv(4096)
                    if not response:
                        raise socket.error("Sin respuesta a GET_COT")
                    self.procesarMensaje(response.decode().strip())
                except (socket.timeout, socket.error) as e:
                    print(f"Error recibiendo COT: {e}")
                    self.conexionCaida.emit()
                    self._handle_disconnect()
                    time.sleep(5)
                    continue

            time.sleep(6)

    def enviar_comando(self, comando: str):
        if not self.estaConectado():
            print("Socket desconectado; comando descartado")
            return

        ahora = time.time()
        if ahora - self.ultimo_envio < self.intervalo_minimo:
            print(f"Ignorado (≤{self.intervalo_minimo}s): {comando}")
            return

        try:
            self.socket.sendall(comando.encode())
            self.ultimo_envio = ahora
            print(f"Comando enviado: {comando}")
        except socket.error as e:
            print(f"Error al enviar comando: {e}")
            self._handle_disconnect()

    def _handle_disconnect(self):
        self.conectado = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.socket = None
        self.cr_enviado = False

    def stop(self):
        self.is_running = False
        if self.socket:
            self.socket.close()
        self.quit()
        self.wait()