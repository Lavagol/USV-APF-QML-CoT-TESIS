import numpy as np
import matplotlib.pyplot as plt
from client.models.parametros_obstaculos import PARAMS
# —————————— PARÁMETROS ——————————
Escala = 1.0                               # 1 unidad interna = 0.1 m
distancias_marcadas = [2, 4, 6, 8, 10, 12, 14,20, 50, 100,150, 200]   # radios (unid. internas)

# —————————— TRAYECTORIA ——————————
tray = np.load("trayectoria.npy", allow_pickle=True)
xs, ys = tray[:, 0] * Escala, tray[:, 1] * Escala

# —————————— OBSTÁCULOS ——————————
obs = np.load("obstaculos.npy", allow_pickle=True)
if obs.ndim == 1:
    obs = np.array([o for o in obs if o is not None], dtype=object)
    if obs.size:
        obs = np.vstack(obs)

if obs.ndim != 2 or obs.shape[1] < 2:
    ox = oy = ot = np.empty(0)
else:
    ox = obs[:, 0].astype(float) * Escala
    oy = obs[:, 1].astype(float) * Escala
    ot = obs[:, 2] if obs.shape[1] >= 3 else np.array(['desconocido'] * len(ox))

# —————————— FIGURA ——————————
fig, ax = plt.subplots(figsize=(8, 8))

# —————————— PUNTOS DE DISTANCIA MÍNIMA (si existen) ——————————
try:
    min_pos  = np.load("min_pos.npy") * Escala   # (N,2)
    min_dist = np.load("min_dist.npy")           # (N,)
    for (xm, ym), dmin in zip(min_pos, min_dist):
        ax.scatter(xm, ym, marker='o', s=60,
                   facecolors='orange', edgecolors='black', zorder=4)
        ax.text(xm, ym, f"{dmin:.1f} m",
                fontsize=8, color='black',
                ha='left', va='bottom',
                bbox=dict(boxstyle='round,pad=0.15',
                          fc='white', ec='orange', lw=0.8, alpha=0.7))
except FileNotFoundError:
    pass

# —————————— ESTILOS DE OBSTÁCULO ——————————
estilos = {
    'piedra'    : dict(marker='o', s=150, facecolor='lightgray', edgecolor='black'),
    'barco'     : dict(marker='s', s=150, facecolor='skyblue',   edgecolor='black'),
    'boya'      : dict(marker='^', s=150, facecolor='orange',    edgecolor='black'),
    'desconocido': dict(marker='o', s=150, facecolor='none',     edgecolor='black'),
}

# —————————— TRAZADO PRINCIPAL ——————————
ax.plot(xs, ys, 'b.-', lw=1, ms=4, label="Trayectoria")
ax.scatter(xs[0],  ys[0],  c='red',   s=120, label="Inicio", zorder=4)
ax.scatter(xs[-1], ys[-1], c='green', s=120, label="Meta",   zorder=4)

# Círculos de influencia
for x0, y0 in zip(ox, oy):
    for r in distancias_marcadas:
        ax.add_patch(plt.Circle((x0, y0),
                                r * Escala,          # ← misma escala
                                edgecolor='gray',
                                facecolor='none',
                                linestyle='--',
                                linewidth=0.8,
                                alpha=0.30,
                                zorder=1))
# --- Radio físico (radio_geo) en rojo (o el color que quieras) ---
for x0, y0, tipo in zip(ox, oy, ot):
    Rg = PARAMS.get(tipo, {}).get("radio_geo", 0.0) * Escala
    if Rg > 0:
        ax.add_patch(plt.Circle((x0, y0), Rg,
                                edgecolor='red', facecolor='none',
                                linestyle='-', linewidth=1.2,
                                alpha=0.7, zorder=2))
# Obstáculos
vistos = set()
for x0, y0, tipo in zip(ox, oy, ot):
    cfg   = estilos.get(tipo, estilos['desconocido'])
    label = tipo.capitalize() if tipo not in vistos else None
    ax.scatter(x0, y0, marker=cfg['marker'], s=cfg['s'],
               facecolors=cfg['facecolor'], edgecolors=cfg['edgecolor'],
               label=label, zorder=3)
    vistos.add(tipo)

# —————————— AJUSTES FINALES ——————————
ax.set_title("Trayectoria USV (orientación interna)")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_aspect('equal', 'box')
ax.set_xlim(xs.min() - 20, xs.max() + 20)
ax.set_ylim(ys.min() - 20, ys.max() + 20)
ax.grid(alpha=0.3)
ax.legend(loc='best', fontsize=9)
plt.tight_layout()
plt.show()
