import socket
import time
import sys	

# HOST='127.0.0.1'
HOST='192.168.1.102'    # '172.16.103.101'
PORT = 30002

try: 
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    print("Prees RETURN for next command")


#######    EJERCICIO:
#Enviar comandos SCRIPT al robot para:

#VENTANA POPUP  "INICIANDO"
#SALIDA DIGITAL 7 a nivel alto
#SALIDA DIGITAL 7 a nivel bajo
#SALIDA DIGITAL HERRAMIENTA a nivel alto
#SALIDA DIGITAL HERRAMIENTA a nivel bajo
#MOVEJ a la posicion de inicio totalmente vertical 
#MOVEJ girando base y codo      -90 grados

#######    COMANDOS  ########  ########  

    s.send(str.encode('popup("INICIANDO")\n'))
    print('popup')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('close popup\n'))
    print('close popup')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('set_digital_out(0, True)\n'))
    print('Digital output 0 HIGH')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('set_digital_out(0, False)\n'))
    print('Digital output 0 LOW')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('set_tool_digital_out(1, True)\n'))
    print('Digital tool output 1 HIGH')
    c = sys.stdin.read(1) #press Return

    s.send(str.encode('set_tool_digital_out(1, False)\n'))
    print('Digital tool output 1 LOW')
    c = sys.stdin.read(1) #press Return
    
    # s.send(str.encode('movej([d2r(124.47),d2r(-71.06),d2r(-100.79),d2r(-76),d2r(86.2),d2r(356.61)], a=1.4, v=1.05, t=5, r =0)\n'))
    # print('Pose 1')
    # c = sys.stdin.read(1) #press Return
    
    # s.send(str.encode('movej([d2r(115.23),d2r(-66.63),d2r(-44.36),d2r(-128.46),d2r(17.30),d2r(257.48)], a=1.4, v=1.05, t=5, r =0)\n'))
    # print('Pose 2')
    # c = sys.stdin.read(1) #press Return

    


########  ########  ########  ########  
    print("END")
    s.close()
except:
    print("Problem with Host")
