Escala=1
PARAMS = {
    'piedra':  { 'repulsion': 50000*Escala**3,  'radio': 200 *Escala, "radio_geo":20.0 },# 
    'barco':   { 'repulsion': 300000*Escala**3,  'radio': 250*Escala, "radio_geo":90.0 },# 
    'boya':    { 'repulsion': 15500*Escala**3,  'radio': 100 *Escala, "radio_geo":10.0},# 
    'default': { 'repulsion': 150*Escala**3,  'radio': 150 *Escala, "radio_geo":30.0},
    #'piedra':  { 'repulsion': 500*Escala**3,  'radio': 10.0 *Escala },# 100 m
    #'barco':   { 'repulsion': 800*Escala**3,  'radio': 30.0*Escala },# 300 m
    #'boya':    { 'repulsion': 250*Escala**3,  'radio': 5.0  *Escala},# 50 m
    #'default': { 'repulsion': 400*Escala**3,  'radio': 15.0 *Escala},
}

def clasificar_obstaculo(raw_tag: str) -> str:
    """
    Dada la etiqueta recibida en el CoT (por ejemplo 'Rocas', 'Vessel', 'Buoy'),
    normaliza y devuelve uno de los tipos en PARAMS.
    """
    tag = raw_tag.strip().lower()
    if 'roc' in tag or 'rock' in tag:
        return 'piedra'
    if 'buoy' in tag or 'boya' in tag:
        return 'boya'
    if 'vessel' in tag or 'ship' in tag or 'boat' in tag:
        return 'barco'
    return 'default'