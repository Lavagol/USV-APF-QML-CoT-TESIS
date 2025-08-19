import numpy as np
from ..models.parametros_obstaculos import PARAMS  # radio / repulsión por tipo
import client.APF.recomendacion as R
# ───────────────────────────────────────────────────────────
#  PARÁMETROS GLOBALES (solo valores por defecto)
# ───────────────────────────────────────────────────────────
Escala_sim = 1
k_att      = 0.5
# 💡 NUEVO: Modulación de la fuerza de atracción
# Vamos a reducir la fuerza de atracción si estamos cerca de un obstáculo.


k_rep_base = 500
d0_base    = 15

v_max_def  = 5  # valor por defecto

# --- Parámetros fuerza tangencial (preventivo) ---
k_tan      = 300.0
d_pre      = 150.0
angle_pre  = 10.0
cos_pre    = np.cos(np.deg2rad(angle_pre))

# --- Escape lateral (día 5) -----------------------
D_lat       = 90   # [m] desplazamiento lateral total
safety_dist = 5    # [m] distancia de seguridad
dt_step     = 0.5   # valor por defecto, el simulador lo sobreescribe    # [s] (tu simulador corre a 500 ms)

# --- radios para activar planner vs alerta ---
radio_alerta        = 400
radio_recomendacion = 350
# ───────────────────────────────────────────────────────────

# ─── Helper: sector según ángulo relativo obstáculo–meta ───
def _sector_from_angle(ang_deg):
    """ang_deg en [0,360). Devuelve 'frontal', 'izquierda', 'trasera' o 'derecha'."""
    if   45 <= ang_deg < 135:  return "izquierda"
    elif 135 <= ang_deg < 225: return "trasera"
    elif 225 <= ang_deg < 315: return "derecha"
    else:                      return "frontal"


def calcular_recomendacion(
    pos_usv,
    pos_objetivo,
    obstaculos,
    *,
    v_max=v_max_def,
    historial=None
):
    """
    Planificador local con 5 etiquetas posibles:
        AVANCE LIBRE / AVANCE ALERTA / AVANCE APF /
        AVANCE PREVENTIVO / ESCAPE 
    """
     # --- estado persistente para PMA ---
    #obst_prev     = getattr(calcular_recomendacion, "_obst_prev",    None)
    #min_dist_prev = getattr(calcular_recomendacion, "_min_dist_prev", float("inf"))
    #passed_pma    = getattr(calcular_recomendacion, "_passed_pma",   False)

    norm_F_Att = 0.0
    norm_F_rep = 0.0

    # ── persistente: índices de obstáculos ya pasados (PMA completado) ──
    handled = getattr(calcular_recomendacion, "_handled", set())
    # Dependen de la escala
    k_rep_global = k_rep_base
    d0_global    = d0_base * Escala_sim

    # --------- Estado persistente del HOLD lateral ----------
    lat_counter = getattr(calcular_recomendacion, "_lat_counter", 0)
    lat_dir     = getattr(calcular_recomendacion, "_lat_dir",     None)

    # (opcionales) contadores de distancia por modo
    avance_apf        = getattr(calcular_recomendacion, "_avance_apf",        0.0)
    avance_preventivo = getattr(calcular_recomendacion, "_avance_preventivo", 0.0)
    avance_lat        = getattr(calcular_recomendacion, "_avance_lat",        0.0)

    # Posiciones
    robot = np.array(pos_usv,      dtype=float)
    goal  = np.array(pos_objetivo, dtype=float)

    # ======================================================
    # 0) HOLD lateral activo → mantengo rumbo ±90° y salgo
    # ======================================================
    if lat_counter > 0 and lat_dir is not None:
        rumbo = np.degrees(np.arctan2(lat_dir[1], lat_dir[0])) % 360

        lat_counter -= 1
        calcular_recomendacion._lat_counter = lat_counter

        # acumular distancia lateral estimada
        avance_lat += v_max * dt_step
        calcular_recomendacion._avance_lat = avance_lat

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
            # nuevos campos (None porque aquí no recalculamos)
            "sector_min"    : None,
            "ang_min"       : None,
            "PMA"           : None,
            "R_geo_min"     : None,
        }

    # ======================================================
    # 1) Escaneo de obstáculos → decidir si activamos planner
    # ======================================================
    alerta, recomendar = None, False
    distancia_min = float("inf")
    obst_min_id   = None
    pos_min_iter  = None

    # Para registrar ángulo/sector y PMA del obstáculo más cercano
    ang_min    = None
    sector_min = None
    R_geo_min  = 0.0

    # Bucle para encontrar el obstáculo más cercano y decidir si se activa el APF
    for idx, (x_obs, y_obs, tipo) in enumerate(obstaculos):
        if idx in handled:
            continue     # Distancia al centro
        d = np.linalg.norm(robot - (x_obs, y_obs))

        # Cálculo único de ángulo y sector para ESTE obstáculo
        vec_obs  = np.array([x_obs, y_obs]) - robot
        vec_goal = goal - robot
        ang_tmp = (np.degrees(np.arctan2(vec_obs[1], vec_obs[0])) -
                   np.degrees(np.arctan2(vec_goal[1], vec_goal[0]))) % 360
        sector_tmp = _sector_from_angle(ang_tmp)

        # Si es el más cercano, guardamos info
        if d < distancia_min:
            distancia_min = d
            obst_min_id   = idx
            pos_min_iter  = tuple(robot)
            ang_min       = ang_tmp
            sector_min    = sector_tmp
            R_geo_min     = PARAMS.get(tipo, {}).get("radio_geo", 0.0)

        # Bloque alerta original (usamos sector_tmp)
        if radio_recomendacion <= d < radio_alerta:
            alerta = f"⚠️ Obstáculo a {d:.1f} m, sector {sector_tmp}"

        if d < radio_recomendacion:
            recomendar = True

    # PMA (clearance real al borde)
    PMA = distancia_min - R_geo_min if np.isfinite(distancia_min) else None
    """
    # ── Detectar cambio de obstáculo o paso del PMA ──
    if obst_min_id != obst_prev:
        # Nuevo obstáculo más cercano → reinicio
        min_dist_prev = distancia_min
        passed_pma    = False
    else:
        if distancia_min <= min_dist_prev:
            # seguimos acercándonos
            min_dist_prev = distancia_min
        else:
            # la distancia empieza a aumentar → hemos pasado el PMA
            passed_pma = True

                # ── marcamos este obstáculo como “evasión completada” ──
            if obst_min_id not in handled:
                handled.add(obst_min_id)
                calcular_recomendacion._handled = handled

                # Guardar estados para la próxima llamada
    calcular_recomendacion._obst_prev      = obst_min_id
    calcular_recomendacion._min_dist_prev  = min_dist_prev
    calcular_recomendacion._passed_pma     = passed_pma
    """
    # ======================================================
    # 2) MODO LIBRE / ALERTA (no se usa APF), directo a la meta
    # ======================================================
    #si la variable es false, ningún obtáculo esta cerca, USV directo a la meta
        # Si ya pasamos el PMA, anulamos la recomendación para volver directo
    #if passed_pma:
    #    recomendar = False
    
    if not recomendar:
        dir_unit = (goal - robot) / np.linalg.norm(goal - robot)
        rumbo    = np.degrees(np.arctan2(dir_unit[1], dir_unit[0])) % 360
        maniobra = "AVANCE ALERTA" if alerta else "AVANCE LIBRE"
        return {
            "rumbo"           : round(rumbo, 1),
            "velocidad"       : round(v_max, 2),
            "distancia_minima": round(distancia_min, 2) if np.isfinite(distancia_min) else None,
            "maniobra"        : maniobra,
            "alerta"          : alerta,
            "force_total"     : np.array([0.0, 0.0]),
            "norm_F"          : 0.0,
            # nuevos
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
    #dist_modulacion = 60.0 # Umbral para empezar a reducir la fuerza (ajustable)
    #escala_att = 1.0  # Por defecto, la fuerza de atracción es completa.
    #if distancia_min < dist_modulacion:
        # Escala la fuerza linealmente: 0 si estás encima del obstáculo, 1 si estás en el límite.
    #    escala_att = (distancia_min / dist_modulacion)
        
    # 3‑a) Atractiva (Ahora escalada)
    F_Att = k_att * (goal - robot)
    #F_att = escala_att * k_att * (goal - robot) 
    norm_F_Att = float(np.linalg.norm(F_Att)) #magnitud escalar
    print(f"‖F_Att‖ = {np.linalg.norm(F_Att):8.3f}")
    # 3‑b) Repulsiva por tipo (con radio inflado)
    F_rep = np.zeros(2)
    norm_F_rep = 0.0
    for x_obs, y_obs, tipo in obstaculos:

        params_i = PARAMS.get(tipo, {})
        Rg     = params_i.get('radio_geo', 0.0)
        d0_i   = params_i.get('radio',     d0_global)  # Radio de influencia
        krep_i = params_i.get('repulsion', k_rep_global)
        
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
        hdir = F_Att / np.linalg.norm(F_Att)
        for x_obs, y_obs, tipo in obstaculos:
            vec = robot - np.array((x_obs, y_obs))
            d   = np.linalg.norm(vec)
            Rg  = PARAMS.get(tipo, {}).get('radio_geo', 0.0)
            d_eff = max(1e-3, d - Rg)

            if d_eff < d_pre:
                u_r    = vec / d
                cos_th = np.dot(hdir, -u_r)
                if cos_th > cos_pre:
                    gamma = (cos_th - cos_pre) / (1 - cos_pre)
                    beta  = (d_pre - d_eff) / d_pre
                    k_eff = k_tan * beta * gamma
                    u_t   = np.array([-u_r[1], u_r[0]])
                    if np.dot(u_t, goal - robot) < 0:
                        u_t = -u_t
                    F_tan = k_eff * u_t
                    break

    # 3‑d) Fuerza total
    F_tot  = F_Att + F_rep + F_tan
    norm_F = np.linalg.norm(F_tot)

    # ======================================================
    # 4) Predicción → activar escape lateral (solo modo APF)
    # ======================================================
    if norm_F > 1e-6:
        direction_pred = F_tot / norm_F
        speed_pred     = min(v_max, norm_F)
    else:
        direction_pred = np.zeros(2)
        speed_pred     = 0.0

    next_pos = robot + direction_pred * speed_pred * dt_step

    # Distancia efectiva al obstáculo (restando radio_geo)
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

        u_up = np.array([-goal_dir[1],  goal_dir[0]])
        u_dn = -u_up

        def score(u):
            p = robot + u * v_max * dt_step
            return sum(1 / max(1e-6, dist_eff(p, x, y, t)) for x, y, t in obstaculos)

        lat_dir = u_up if score(u_up) < score(u_dn) else u_dn

        lat_steps = int(D_lat / (v_max * dt_step))
        calcular_recomendacion._lat_counter = max(lat_steps - 1, 0)  # 1er paso lo da el sim
        calcular_recomendacion._lat_dir     = lat_dir

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

    # ======================================================
    # 5) Maniobra normal (APF o PREVENTIVO)
    # ======================================================
    if np.any(F_tan):
        direction = F_tot / norm_F
        maniobra  = "AVANCE PREVENTIVO"
    else:
        direction = F_tot / norm_F
        maniobra  = "AVANCE APF"

    # ───────── Inercia ─────────
    alpha = 0
    prev = getattr(calcular_recomendacion, "_dir_prev", None)
    if prev is not None and alpha > 0 and np.linalg.norm(prev) > 1e-6:
        direction = (1 - alpha) * direction + alpha * prev
        direction /= np.linalg.norm(direction)
    calcular_recomendacion._dir_prev = direction
    # ───────────────────────────

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