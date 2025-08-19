import serial
import socket
import threading
import random
from time import sleep

from utils.generar_cot import generar_mensaje_cot

import time

ser = serial.Serial()
ser.port = "/dev/ttyUSB0"
ser.baudrate = 9600
ser.bytesize = serial.EIGHTBITS #number of bits per bytes
ser.parity = serial.PARITY_NONE #set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
#ser.timeout = None          #block read
ser.timeout = 1            #non-block read
#ser.timeout = 2              #timeout block read
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 2     #timeout for write



# Diccionario global para almacenar las conexiones y nombres de los clientes
clients = {}
id_clients = {}
client_id = ""
max_clients = 3
index = 1

# ParÃ¡metros de entrada al archivo COT
tipo_objetivo = "vehÃ­culo"
latitud = 30.0090027
longitud = -85.9578735                         
altura = 10.0
nombre_emisor = "USV-DIPRIDA"
estado_motor = 0
marcha_actual = 'N'
acel_actual = 0
timon_actual = 50
rpm=800
temp=35
vbat=round(11 + random.random(),2)
fuel_level=85
speed=60
valor_pwm1=0
valor_pwm2=0
mensaje_cot = ""




# Funcion para manejar cada cliente
def handle_client(client_socket, client_address):
    try:
        global index
        client_name = "Cliente" + str(index)
        index += 1
        clients[client_socket] = client_name

        # Recibir ID del cliente solo una vez al conectar
        client_id = client_socket.recv(10).decode('utf-8')
        print(f"ID de {client_name}: {client_id}")
        id_clients[client_socket] = client_id

        print(f"{client_name} se ha conectado desde {client_address}")

        # Generar y enviar mensaje COT al conectar
        mensaje_cot = generar_mensaje_cot(
            nombre_emisor
        )
        print("Mensaje COT generado para " + id_clients[client_socket])
        print("mensaje COT: " + mensaje_cot)
        client_socket.sendall(mensaje_cot.encode('utf-8'))

        old_message = "NO_MESSAGE"

        while True:
            message = client_socket.recv(1024).decode('utf-8').upper()
            if not message:
                break

            print(f"Mensaje de {id_clients[client_socket]}: {message}")

            match message:
                case "GET_COT":
                    mensaje_cot = generar_mensaje_cot(
                        nombre_emisor
                    )
                    print("Mensaje COT enviado a " + id_clients[client_socket])
                    client_socket.sendall(mensaje_cot.encode('utf-8'))

                case "ENGINE":
                    aux = message[6:]
                    estado_motor = int(aux)
                    mensaje_cot = generar_mensaje_cot(
                        tipo_objetivo, latitud, longitud, altura, nombre_emisor, message,
                        estado_motor, marcha_actual, acel_actual, timon_actual, rpm, temp, vbat,
                        fuel_level, speed, valor_pwm1, valor_pwm2, tipo_objetivo="a-f-S", how="m-g"
                    )
                    print("Mensaje COT enviado a " + id_clients[client_socket])
                    client_socket.sendall(mensaje_cot.encode('utf-8'))

            if message != "GET_COT":
                old_message = message

    except Exception as e:
        print(f"Error con {client_address}: {e}")
    finally:
        print(f"{clients[client_socket]} se ha desconectado.")
        client_socket.close()
        del clients[client_socket]

"""
def update_cot(msg_cot):
    global tipo_objetivo
    global latitud
    global longitud                       
    global altura
    global nombre_emisor
    global estado_motor
    global marcha_actual
    global acel_actual
    global timon_actual
    global rpm
    global temp
    global vbat
    global fuel_level
    global speed
    global valor_pwm1
    global valor_pwm2
    global mensaje_cot
    msg_cot = generar_mensaje_cot(tipo_objetivo, latitud, longitud, altura, nombre_emisor, message, estado_motor, marcha_actual, acel_actual, timon_actual, rpm, temp, vbat, fuel_level, speed, valor_pwm1, valor_pwm2, tipo_objetivo="a-f-S", how="m-g")    
"""

# FunciÃ³n para reenviar el mensaje a otros clientes conectados
def broadcast(message, sender_socket):
    for client_socket in clients:
        if client_socket != sender_socket:  # No enviar el mensaje de vuelta al remitente
            try:
                client_socket.sendall(message.encode('utf-8'))
            except Exception as e:
                print(f"Error enviando mensaje: {e}")
                client_socket.close()
                

# FunciÃ³n para responder el mensaje al cliente que lo mandÃ³
def send_message(message, sender_socket):
    for client_socket in clients:
        if client_socket == sender_socket:  # Enviar el mensaje de vuelta sÃ³lo al remitente
            try:
                client_socket.sendall(message.encode('utf-8'))
            except Exception as e:
                print(f"Error enviando mensaje: {e}")
                client_socket.close()
                
                
# FunciÃ³n para enviar el mensaje a un cliente especÃ­fico
def send_msg(message, receiver_id):
    for s in id_clients:
        if id_clients[s] == receiver_id:
            ua_socket = s        
            try:
                ua_socket.sendall(message.encode('utf-8'))
            except Exception as e:
                print(f"Error enviando mensaje: {e}")
                ua_socket.close()


# Configuracion del servidor

# Nota: Se requiere la instalacion de FindMyIP para poder 
def start_server():
    #import FindMyIP as ip
    #server_ip = ip.internal()
    
    server_ip =  '0.0.0.0'     #'10.3.141.250'
    print("server_ip: ")
    print(server_ip)
    
    server_port = 65432
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((server_ip, server_port))
    server.listen(max_clients)  # Limitar a 2 clientes
    print(f"Servidor escuchando en {server_ip}:{server_port}")

    client_threads = []

    while len(clients) < max_clients:
        # Aceptar nuevas conexiones de clientes
        client_socket, client_address = server.accept()
        client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address))
        client_thread.start()
        client_threads.append(client_thread)

    # Esperar a que los hilos de los clientes terminen
    for thread in client_threads:
        thread.join()

    print("Servidor cerrado.")

