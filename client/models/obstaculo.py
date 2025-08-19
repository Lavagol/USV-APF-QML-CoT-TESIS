# client/models/obstaculo.py
class Obstacle:
    """
    Representación de un obstáculo con tipo y radio de influencia.

    Atributos:
        lat (float): Latitud en grados.
        lon (float): Longitud en grados.
        tipo (str): "piedra" | "barco" | "boya".
        radio (float): Radio de seguridad en metros.
    """
    def __init__(self, lat: float, lon: float, tipo: str, radio: float):
        self.lat = lat
        self.lon = lon
        self.tipo = tipo
        self.radio = radio



