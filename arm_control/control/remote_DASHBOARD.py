import socket
import time
import sys	

# HOST='127.0.0.1'
HOST='192.168.1.102'    # '172.16.103.101'
PORT = 29999

try: 
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    print("Prees RETURN for next command")


#######    EJERCICIO:
#Enviar comandos DASHBOARD al robot para:

#DAR POTENCIA AL  BRAZO
#QUITAR FRENOS
#CARGAR PROGRAMA   a.urp
#EJECUTARLO
#PAUSARLO
#EJECUTARLO
#PARARLO
#PONER FRENOS

#######    COMANDOS  ########  ########  

#para python3 debe cambiarse el comando SEND de este modo:
    s.send(str.encode('popup Hola\n'))
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('close popup\n'))
    print('close popup')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('power on\n'))
    print('power on')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('brake release\n'))
    print('brake release')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('load Test_external_control.urp\n'))
    print('load program')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('play\n'))
    print('play program')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('pause\n'))
    print('pause program')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('play\n'))
    print('play program')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('stop\n'))
    print('stop program')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('power off\n'))
    print('power off')
    c = sys.stdin.read(1) #press Return

########  ########  ########  ########  
    print("END")
    s.close()
except:
    print("Problem with Host")
