import json
import folium

# Estilos de mapa y sus atributos (solo OpenStreetMap final con mejoras)
map_styles = {
    "OpenStreetMap_final": {
        "tiles": "OpenStreetMap",
        "custom": False,
        "attr": None
    }
}

# Cargar archivos GPS (rutas locales)
with open("trayectoria_gps.json", "r", encoding="utf-8") as f:
    trayectoria = json.load(f)

with open("obstaculos_gps.json", "r", encoding="utf-8") as f:
    obstaculos = json.load(f)

# Coordenadas de centro
centro_lat = trayectoria[0]["lat"]
centro_lon = trayectoria[0]["lon"]
trayecto_coords = [(p["lat"], p["lon"]) for p in trayectoria]

# Crear mapa final con mejoras
for nombre, cfg in map_styles.items():
    print(f"🗺 Generando mapa: {nombre}...")

    if cfg["custom"]:
        m = folium.Map(location=[centro_lat, centro_lon], zoom_start=16, tiles=None)
        folium.TileLayer(
            tiles=cfg["tiles"],
            attr=cfg["attr"],
            name=nombre,
            control=False
        ).add_to(m)
    else:
        m = folium.Map(
            location=[centro_lat, centro_lon],
            zoom_start=16,
            tiles=cfg["tiles"]
        )

    # Punto inicial
    folium.Marker(
        location=trayecto_coords[0],
        icon=folium.Icon(color="green", icon="play"),
        tooltip="Inicio"
    ).add_to(m)

    # Punto final
    folium.Marker(
        location=trayecto_coords[-1],
        icon=folium.Icon(color="blue", icon="flag"),
        tooltip="Destino"
    ).add_to(m)

    # Trayectoria
    folium.PolyLine(trayecto_coords, color="blue", weight=3, tooltip="Trayectoria").add_to(m)

    # Obstáculos
    for obs in obstaculos:
        folium.Marker(
            location=(obs["lat"], obs["lon"]),
            icon=folium.Icon(color="red", icon="exclamation-sign"),
            tooltip=f"Obstáculo: {obs['tipo']}"
        ).add_to(m)

    # Guardar
    filename = f"mapa_{nombre.lower()}.html"
    m.save(filename)
    print(f"[✔] Mapa guardado: {filename}")
