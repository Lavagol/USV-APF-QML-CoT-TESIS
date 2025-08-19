from datetime import datetime, timezone #Importa datetime y timezone, pero no se utilizan en el resto de la función. Normalmente se usan para los campos time, start, stale del CoT.

nombre_emisor = "Proveedor de obstaculos" #Variable global con el callsign que aparecerá en el atributo contact_callsign

def generar_mensaje_cot(nombre_emisor): #Función que recibe el nombre del emisor y devuelve el XML.
    uuid = "DIPRIDA-01"

    cot_xml = f"""<?xml version="1.0" standalone="yes"?>   
<event version="2.0">
  <detail contact_callsign="{nombre_emisor}"
            obstaculo1="-33.025874, -71.610697"
            tipo_obstaculo1="Boya"
            obstaculo2="-33.028270, -71.612834"
            tipo_obstaculo2="Buque"
            obstaculo3="-33.030139, -71.616662"
            tipo_obstaculo3="Roca"
          
          meta="-33.033287, -71.620677" />
  <point lat="-33.024760" lon="-71.608800" ce="0.0" hae="13" le="0.0" />
</event>"""
    return cot_xml #Entrega la cadena XML.



"""
  meta="-33.032296, -71.614664" #prueba de fase A
  meta="-33.032300, -71.609303 1km
  meta="-33.032296, -71.613593 600mts
           
           
           obstaculo1="-33.032023, -71.617363"
            tipo_obstaculo1="Vessel" CENTRO!!

                     obstaculo1="-33.032287, -71.617258"
            tipo_obstaculo1="Roca"y boya CENTRO !!

          LATERAL
          lat="-33.032296" lon="-71.620022" INCIO
          "-33.032296, -71.613593" META
          obstaculo1="-33.025806, -71.610909"
          tipo_obstaculo1="Boya"
          obstaculo1="-33.028270, -71.612834"
          tipo_obstaculo1="Vessel"
          obstaculo3="-33.030389, -71.616563"
          tipo_obstaculo3="Roca"

          
          obstaculo1="-33.032160, -71.617367"
          tipo_obstaculo1="Boya" centro
          


          obstaculo1="-33.030864, -71.624098"
          tipo_obstaculo1="Boya"
    cot_xml = f<?xml version="1.0" standalone="yes"?>   
<event version="2.0">
  <detail contact_callsign="{nombre_emisor}"
          obstaculo1="-33.027319, -71.614251"
          tipo_obstaculo1="Vessel"
          obstaculo2="-33.027561, -71.614829"
          tipo_obstaculo2="Roca"
          obstaculo3="-33.027113, -71.614009"
          tipo_obstaculo3="Boya"
          obstaculo4="-33.027667, -71.614518"
          tipo_obstaculo4="Vessel"
          obstaculo5="-33.027495, -71.614354"
          tipo_obstaculo5="Roca"
          obstaculo6="-33.027886, -71.614760"
          tipo_obstaculo6="Roca"
          obstaculo7="-33.027946, -71.614694"
          tipo_obstaculo7="Roca"
          obstaculo8="-33.027916, -71.615040"
          tipo_obstaculo8="Roca"
          obstaculo9="-33.028204, -71.615427"
          tipo_obstaculo9="Roca"
          obstaculo10="-33.028458, -71.615115"
          tipo_obstaculo10="Roca"
          obstaculo11="-33.030698, -71.623891"
          tipo_obstaculo11="Roca"

          meta="-33.031654, -71.620245" />
  <point lat="-33.026181" lon="-71.612689" ce="0.0" hae="13" le="0.0" />
</event>
"""