import numpy as np #para operaciones vectoriales
from ..models.parametros_obstaculos import PARAMS  # k_repulsión y d0 de parametros_obtaculos
#import client.APF.recomendacion as R  no se usa
# ---------------------------------------------------
#  PARÁMETROS GLOBALES (solo valores por defecto)
# ---------------------------------------------------
Escala_sim = 1

k_att      = 1



#NO UTILIZADOS, YA QUE CADA OBJETO TIENE SU K_REP Y D0 DEL parametros_obstaculos
k_rep_base = 500
d0_base    = 15

v_max_def  = 5  # valor por defecto, (si no se pasa v_max como argumento), pero no se utilizará

# --- Parámetros fuerza tangencial (preventivo) -----------
k_tan      = 300.0 # cte. para la fuerza tangencial
d_pre      = 150.0 # distancia (si d_eff < d_pre)
angle_pre  = 10.0  # Umbral angular [grados]
cos_pre    = np.cos(np.deg2rad(angle_pre)) # para pasar a radianes

# --- Escape lateral  -----------------------   la idea es dejar el D_lat respecto a tipo de obstáculo, es solo escape de emergencia
D_lat       = 90   # [m] distancia lateral total a recorrer en el escape
safety_dist = 5    # [m] distancia de seguridad
dt_step     = 0.5   # valor por defecto, el simulador lo sobreescribe    # [s] (tu simulador corre a 500 ms)

# --- radios para activar recomendacion vs alerta  solo avida sector donde esta el obstáculo---
radio_alerta        = 400
radio_recomendacion = 350

# --------------------------
#  sector según ángulo relativo obstáculo–meta
# --------------------------
def _sector_from_angle(ang_deg):
    """ang_deg en [0,360). Devuelve 'frontal', 'izquierda', 'trasera' o 'derecha'."""
    if   45 <= ang_deg < 135:  return "izquierda"
    elif 135 <= ang_deg < 225: return "trasera"
    elif 225 <= ang_deg < 315: return "derecha"
    else:                      return "frontal"



# -------------------------
#  PLANIFICADOR PRINCIPAL
# -------------------------
def calcular_recomendacion(
    pos_usv,        # (x, y) posición actual del USV en coordenadas internas
    pos_objetivo,   # (x, y) posición de la meta en coordenadas internas
    obstaculos,     # lista de (x_obs, y_obs, tipo) en coordenadas internas
    *,
    v_max=v_max_def,  # velocidad máxima permitida (si no pasa, usa v_max_def)
    historial=None    # no se usa aquí, pero queda para extender
):
    """
    Salida: dict con 'rumbo', 'velocidad', 'maniobra' y diagnosticos.
    Maniobras posibles:
      - AVANCE LIBRE        (sin alerta)
      - AVANCE ALERTA       (hay obstáculo en radio de alerta)
      - AVANCE APF          (fuerzas atractiva+repulsiva)
      - AVANCE PREVENTIVO   (tangencial activada)
      - ESCAPE LATERAL      (emergencia ±90°)
    """
    # Magnitudes escalares, 
    norm_F_Att = 0.0
    norm_F_rep = 0.0

    # -- persistente: índices de obstáculos ya pasados (PMA completado) --
    #handled = getattr(calcular_recomendacion, "_handled", set())


    # SOLO POR SI PARAMS NO ENTREGA INFORMACIÓN PERO NO DEBERÍA PASAR.
    k_rep_global = k_rep_base
    d0_global    = d0_base * Escala_sim

    # --------- Estado del ESCAPE lateral (para mantenerlo activo en los pasos correspondientes)  ----------
    lat_counter = getattr(calcular_recomendacion, "_lat_counter", 0) # pasos restantes
    lat_dir     = getattr(calcular_recomendacion, "_lat_dir",     None) # dirección lateral (unitaria)

    #  contadores de distancia por modo
    avance_apf        = getattr(calcular_recomendacion, "_avance_apf",        0.0)
    avance_preventivo = getattr(calcular_recomendacion, "_avance_preventivo", 0.0)
    avance_lat        = getattr(calcular_recomendacion, "_avance_lat",        0.0)

    # Posiciones a np.array para operar vectorialmente
    robot = np.array(pos_usv,      dtype=float)
    goal  = np.array(pos_objetivo, dtype=float)

    # --------------------------------------------------
    # 0) ESCAPE lateral activo → mantengo rumbo ±90° y salgo del peligro
    # ---------------------------------------------------
    if lat_counter > 0 and lat_dir is not None:
        rumbo = np.degrees(np.arctan2(lat_dir[1], lat_dir[0])) % 360

        # es como el temporizador del avance lateral para poder llegar a 0.
        lat_counter -= 1
        calcular_recomendacion._lat_counter = lat_counter

        # acumular distancia lateral estimada
        avance_lat += v_max * dt_step
        calcular_recomendacion._avance_lat = avance_lat
         # En ESCAPE no se recalculan fuerzas; solo se mantiene el rumbo lateral
        return {
            "rumbo"           : round(rumbo, 1),
            "velocidad"       : round(v_max, 2),
            "maniobra"        : "ESCAPE LATERAL",
            "alerta"          : None,
            "force_total"     : np.array([0.0, 0.0]),
            "norm_F"          : 0.0,
            "distancia_minima": None,
            "pos_min_iter"    : None,
            "obst_min_id"     : None,
            # (no se recalcula aquí), porque en escape lateral solo se estan consumiendo los pasos
            "sector_min"    : None,
            "ang_min"       : None,
            "PMA"           : None,
            "R_geo_min"     : None,
        }

    # --------------------------------------------------------
    # 1) Escaneo de obstáculos → decidir si activamos planner
    # --------------------------------------------------------
    alerta, recomendar = None, False  # 'alerta' = solo aviso; 'recomendar' = activar recomendacion
    distancia_min = float("inf")      # mínima distancia 
    obst_min_id   = None              # índice del obstáculo más cercano
    pos_min_iter  = None              # posición del robot al evaluar esa mínima

    # Para registrar ángulo/sector y PMA del obstáculo más cercano
    ang_min    = None  # ángulo relativo obstáculo–meta (grados)
    sector_min = None  # frontal/izquierda/trasera/derecha
    R_geo_min  = 0.0  # radio físico del obstáculo más cercano 

    # Bucle para encontrar el obstáculo más cercano y decidir si se activa el APF
    for idx, (x_obs, y_obs, tipo) in enumerate(obstaculos):
        #if idx in handled:
        #    continue     # Distancia al centro
        # Distancia  al centro del obstáculo
        d = np.linalg.norm(robot - (x_obs, y_obs))

        # Vectores hacia obs y hacia la meta (para ángulo relativo)
        vec_obs  = np.array([x_obs, y_obs]) - robot
        vec_goal = goal - robot

        # Ángulo relativo (obs vs goal), en grados [0, 360)
        ang_tmp = (np.degrees(np.arctan2(vec_obs[1], vec_obs[0])) -
                   np.degrees(np.arctan2(vec_goal[1], vec_goal[0]))) % 360
        
        # Sector según el ángulo relativo
        sector_tmp = _sector_from_angle(ang_tmp)

        # Si es el más cercano, guardamos información
        if d < distancia_min:
            distancia_min = d
            obst_min_id   = idx
            pos_min_iter  = tuple(robot)
            ang_min       = ang_tmp
            sector_min    = sector_tmp
            R_geo_min     = PARAMS.get(tipo, {}).get("radio_geo", 0.0)

        #  Si está en el radio de alerta → generar texto de advertencia
        if radio_recomendacion <= d < radio_alerta:
            alerta = f"⚠️ Obstáculo a {d:.1f} m, sector {sector_tmp}"
        # Si entra en el radio de recomendación → activar recomendación
        if d < radio_recomendacion:
            recomendar = True

    # PMA (clearance real al borde)
    PMA = distancia_min - R_geo_min if np.isfinite(distancia_min) else None # np.isfinite(x), filtro de seguridad para no calcular distancias con valores infinitos o inválidos. , devuelve True si x es un número finito y Devuelve False si x = np.inf, -np.inf o NaN

    # ------------------------------------------------------
    # 2) MODO LIBRE / ALERTA (no se usa APF), directo a la meta
    # ------------------------------------------------------
    # Si no hay obstáculo, vuelas directo a la meta
    if not recomendar:
        dir_unit = (goal - robot) / np.linalg.norm(goal - robot) # Dirección unitaria hacia la meta
        rumbo    = np.degrees(np.arctan2(dir_unit[1], dir_unit[0])) % 360 # Rumbo en grados [0, 360)
        maniobra = "AVANCE ALERTA" if alerta else "AVANCE LIBRE" # Etiqueta según si había alerta en el radio 
        return {
            "rumbo"           : round(rumbo, 1),
            "velocidad"       : round(v_max, 2),
            "distancia_minima": round(distancia_min, 2) if np.isfinite(distancia_min) else None,
            "maniobra"        : maniobra,
            "alerta"          : alerta,
            "force_total"     : np.array([0.0, 0.0]),
            "norm_F"          : 0.0,
            "sector_min"    : sector_min,
            "ang_min"       : None if ang_min is None else round(ang_min, 1),
            "PMA"           : None if PMA is None else round(PMA, 2),
            "R_geo_min"     : R_geo_min,
            "F_rep"         : round(norm_F_rep, 3),
            "obst_min_id"   : obst_min_id,
            "pos_min_iter"  : pos_min_iter,
            "F_rep"         : 0.0,
            "F_Att"         : round(norm_F_Att,3)
        }

    # ======================================================
    # 3) PLANIFICADOR LOCAL, se busca (Debilitar(modular) la Fatt si el USV esta muy cerca del obs, mejor evasion
    # ======================================================

        
    # 3‑a) Atractiva (Ahora escalada)
    F_Att = k_att * (goal - robot)
    #F_att = escala_att * k_att * (goal - robot) 
    norm_F_Att = float(np.linalg.norm(F_Att)) #magnitud escalar
    print(f"‖F_Att‖ = {np.linalg.norm(F_Att):8.3f}")

    # 3‑b) Repulsiva por tipo 
    F_rep = np.zeros(2)
    norm_F_rep = 0.0

    for x_obs, y_obs, tipo in obstaculos:
        params_i = PARAMS.get(tipo, {})     # fila de parámetros por tipo
        Rg     = params_i.get('radio_geo', 0.0) # radio físico (borde real)
        d0_i   = params_i.get('radio',     d0_global)  # Radio de influencia si no hay del parametros
        krep_i = params_i.get('repulsion', k_rep_global) # constante de repulsión si nohay de parametros
        
        vec = robot - np.array((x_obs, y_obs))
        d   = np.linalg.norm(vec)

        # Solo calcular fuerza si el robot está dentro de la zona de influencia
        
        if d <= d0_i:
            d_eff = max(d - Rg, 0.1)  # distancia al borde físico
            # Prevenir división por cero si está justo en el borde
 

        # Vector unitario de repulsión (desde el centro del obstáculo)
            gradiente_unitario = vec / d
            magnitud_fuerza = krep_i * ((d0_i - d_eff) / (d0_i * d_eff))**2# Magnitud de la fuerza. Crece exponencialmente al acercarse al borde.
            F_rep += magnitud_fuerza * gradiente_unitario
    
    norm_F_rep = float(np.linalg.norm(F_rep)) #magnitud escalar
           # F_rep += krep_i * ((1/d_eff) - (1/d0_i)) / d_eff**2 * (vec / d)
    print(f"‖F_rep‖ = {np.linalg.norm(F_rep):8.3f}")

    # 3‑c) Tangente preventiva (usa d_eff)
    F_tan = np.zeros(2)
    if np.linalg.norm(F_Att) > 1e-6:
        hdir = F_Att / np.linalg.norm(F_Att)  # heading hacia meta (unitario)
        for x_obs, y_obs, tipo in obstaculos:
            vec = robot - np.array((x_obs, y_obs))
            d   = np.linalg.norm(vec)
            Rg  = PARAMS.get(tipo, {}).get('radio_geo', 0.0)
            d_eff = max(1e-3, d - Rg)

            if d_eff < d_pre:  # sólo si estoy bastante cerca
                u_r    = vec / d    # radial (unitario) desde obs → robot
                cos_th = np.dot(hdir, -u_r)  # coseno del ángulo entre "hacia meta" y "hacia obs"
                if cos_th > cos_pre:         # "obstáculo por delante": está alineado con rumbo a meta
                    gamma = (cos_th - cos_pre) / (1 - cos_pre)  # alineamiento
                    beta  = (d_pre - d_eff) / d_pre             # cercanía
                    k_eff = k_tan * beta * gamma                # cte efectiva
                    # vector tangencial (perp. a u_r); sentido inicial
                    u_t   = np.array([-u_r[1], u_r[0]])      
                    # Elegir el sentido que "favorezca" avance a la meta
                    if np.dot(u_t, goal - robot) < 0:
                        u_t = -u_t
                    F_tan = k_eff * u_t
                    break   # con el primer obstáculo más cercano al peligro actuará

    # 3‑d) Fuerza total  y su calculo de magnitud
    F_tot  = F_Att + F_rep + F_tan
    norm_F = np.linalg.norm(F_tot)

    # --------------------------------------------------
    # 4) Predicción → activar escape lateral (solo modo APF)
    # ---------------------------------------------------
    if norm_F > 1e-6:
        direction_pred = F_tot / norm_F       # rumbo previsto
        speed_pred     = min(v_max, norm_F)   # velocidad prevista
    else:
        direction_pred = np.zeros(2)
        speed_pred     = 0.0

    next_pos = robot + direction_pred * speed_pred * dt_step  # posición prevista a dt_step

    # Distancia efectiva al obstáculo = distancia al centro - radio_geo
    def dist_eff(p, x, y, t):
        Rg = PARAMS.get(t, {}).get('radio_geo', 0.0)
        return np.linalg.norm(p - np.array((x, y))) - Rg

    activar_escape = (
        (not np.any(F_tan)) and
        any(dist_eff(next_pos, x_obs, y_obs, tipo) < safety_dist
            for x_obs, y_obs, tipo in obstaculos)
    )

    if activar_escape:
        # ±90° respecto a la dirección a la meta
        goal_dir = goal - robot    
        goal_dir /= np.linalg.norm(goal_dir)

        u_up = np.array([-goal_dir[1],  goal_dir[0]])  # +90°
        u_dn = -u_up                                   # -90°
        # Score: penaliza (como un costo artifical) direcciones cuya next_pos queden muy cerca del borde (1/dist)
        def score(u):
            p = robot + u * v_max * dt_step
            return sum(1 / max(1e-6, dist_eff(p, x, y, t)) for x, y, t in obstaculos)
        
        # se Elige la lateralidad con menor "riesgo"
        lat_dir = u_up if score(u_up) < score(u_dn) else u_dn

        #cuántos pasos mantener el lateral para cubrir D_lat
        lat_steps = int(D_lat / (v_max * dt_step))
        calcular_recomendacion._lat_counter = max(lat_steps - 1, 0)  # 1er paso lo da el sim
        calcular_recomendacion._lat_dir     = lat_dir
        
        # Rumbo de ese lateral
        rumbo = np.degrees(np.arctan2(lat_dir[1], lat_dir[0])) % 360

        avance_lat += v_max * dt_step
        calcular_recomendacion._avance_lat = avance_lat

        return {
            "rumbo"           : round(rumbo, 1),
            "velocidad"       : round(v_max, 2),
            "maniobra"        : "ESCAPE LATERAL",
            "alerta"          : None,
            "force_total"     : F_tot,
            "norm_F"          : norm_F,
            "distancia_minima": round(distancia_min, 2) if np.isfinite(distancia_min) else None,
            "pos_min_iter"    : pos_min_iter,
            "obst_min_id"     : obst_min_id,
            "sector_min"      : sector_min,
            "ang_min"         : None if ang_min is None else round(ang_min, 1),
            "PMA"             : None if PMA is None else round(PMA, 2),
            "R_geo_min"       : R_geo_min,
            "F_rep"           : round(norm_F_rep,3),
            "F_Att"           : round(norm_F_Att,3)

        }

    # -------------------------------------------------
    # 5) Maniobra normal (APF o PREVENTIVO)
    # -------------------------------------------------
    if np.any(F_tan):
        direction = F_tot / norm_F
        maniobra  = "AVANCE PREVENTIVO"
    else:
        direction = F_tot / norm_F
        maniobra  = "AVANCE APF"

    # Inercia, en esta ocación no se utiliza por temas de trabajos se buscaba suavizar el rumbo pero se mantendrá desactivado
    alpha = 0
    prev = getattr(calcular_recomendacion, "_dir_prev", None)
    if prev is not None and alpha > 0 and np.linalg.norm(prev) > 1e-6:
        direction = (1 - alpha) * direction + alpha * prev
        direction /= np.linalg.norm(direction)
    calcular_recomendacion._dir_prev = direction
    # ───────────────────────────
    # Rumbo final y distancia mínima en metros (si aplicara escala)
    rumbo = np.degrees(np.arctan2(direction[1], direction[0])) % 360
    distancia_min_m = distancia_min * Escala_sim if np.isfinite(distancia_min) else None

    # (opcional) acumular distancias estimadas
    paso_est = v_max * dt_step
    if maniobra == "AVANCE APF":
        avance_apf += paso_est
        calcular_recomendacion._avance_apf = avance_apf
    elif maniobra == "AVANCE PREVENTIVO":
        avance_preventivo += paso_est
        calcular_recomendacion._avance_preventivo = avance_preventivo

    return {
        "rumbo"           : round(rumbo, 1),
        "velocidad"       : round(v_max, 2),
        "maniobra"        : maniobra,
        "alerta"          : alerta,
        "force_total"     : F_tot,
        "norm_F"          : norm_F,
        "distancia_minima": distancia_min_m,
        "pos_min_iter"    : pos_min_iter,
        "obst_min_id"     : obst_min_id,
        "sector_min"      : sector_min,
        "ang_min"         : None if ang_min is None else round(ang_min, 1),
        "PMA"             : None if PMA is None else round(PMA, 2),
        "R_geo_min"       : R_geo_min,
        "F_rep"           : round(norm_F_rep,3),
        "F_Att"           : round(norm_F_Att,3)
    }
