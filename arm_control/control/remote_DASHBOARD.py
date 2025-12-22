import socket
import time
import sys	

# HOST='127.0.0.1'
HOST='192.168.1.102'    # '172.16.103.101'
PORT = 29999

def send_command(sock, command, wait_for_input=True):
    """Send a command and read the response"""
    sock.send(str.encode(command + '\n'))
    print(f'\n>>> Sent: {command}')
    data = sock.recv(1024)
    response = data.decode('utf-8').strip()
    print(f'<<< Robot response: {response}')
    if wait_for_input:
        input('Press RETURN to continue...')
    return response

def check_robot_status(sock):
    """Check and display current robot status"""
    print("\n" + "="*60)
    print("ROBOT STATUS CHECK:")
    print("="*60)
    
    # Check robot mode
    sock.send(str.encode('robotmode\n'))
    data = sock.recv(1024)
    print(f'Robot Mode: {data.decode("utf-8").strip()}')
    
    # Check safety status
    sock.send(str.encode('safetystatus\n'))
    data = sock.recv(1024)
    print(f'Safety Status: {data.decode("utf-8").strip()}')
    
    # Check program state
    sock.send(str.encode('programState\n'))
    data = sock.recv(1024)
    print(f'Program State: {data.decode("utf-8").strip()}')
    
    # Check if program is running
    sock.send(str.encode('running\n'))
    data = sock.recv(1024)
    print(f'Running: {data.decode("utf-8").strip()}')
    
    # Check loaded program
    sock.send(str.encode('get loaded program\n'))
    data = sock.recv(1024)
    print(f'Loaded Program: {data.decode("utf-8").strip()}')
    
    # Check if in remote control
    sock.send(str.encode('is in remote control\n'))
    data = sock.recv(1024)
    print(f'Remote Control: {data.decode("utf-8").strip()}')
    
    print("="*60 + "\n")
    input('Press RETURN to continue...')

try: 
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    # Read initial connection message
    data = s.recv(1024)
    print(f"Connected to robot. Response: {data.decode('utf-8').strip()}")
    print("\nPress RETURN for next command")


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

    # Initial status check
    check_robot_status(s)

#para python3 debe cambiarse el comando SEND de este modo:
    # s.send(str.encode('get loaded program\n'))
    # print('get loaded program')
    # c = sys.stdin.read(1) #press Return

    # Restart safety if needed
    send_command(s, 'restart safety')
    time.sleep(2)  # Wait for safety to restart
    check_robot_status(s)

    # s.send(str.encode('popup Hola\n'))
    # print('open popup')
    # c = sys.stdin.read(1) #press Return

    # s.send(str.encode('close popup\n'))
    # print('close popup')
    # c = sys.stdin.read(1) #press Return

    # Close any existing popups before powering on
    send_command(s, 'close popup', wait_for_input=False)
    send_command(s, 'close safety popup', wait_for_input=False)
    
    # Power on the robot
    send_command(s, 'power on')
    time.sleep(3)  # Wait for power on to complete
    check_robot_status(s)

    # Release brakes
    send_command(s, 'brake release')
    time.sleep(2)  # Wait for brakes to release
    check_robot_status(s)

    # Load the program
    send_command(s, 'load Test_external_control.urp')
    time.sleep(2)  # Wait for program to load
    check_robot_status(s)

    # s.send(str.encode('close popup\n'))
    # print('close popup')
    # c = sys.stdin.read(1) #press Return

    # Close any popups that might block program start
    send_command(s, 'close popup', wait_for_input=False)
    send_command(s, 'close safety popup', wait_for_input=False)
    
    # Start the program
    send_command(s, 'play')
    time.sleep(2)
    check_robot_status(s)

    # Close popup again if needed
    send_command(s, 'close popup')
    
    # Pause the program
    send_command(s, 'pause')
    check_robot_status(s)

    # Resume the program
    send_command(s, 'play')
    check_robot_status(s)

    # Stop the program
    send_command(s, 'stop')
    check_robot_status(s)

    # Power off
    send_command(s, 'power off')

########  ########  ########  ########  
    print("\n" + "="*60)
    print("PROGRAM COMPLETED SUCCESSFULLY")
    print("="*60)
    s.close()
except Exception as e:
    print(f"\nERROR: {e}")
    print("Problem with Host or command execution")
    import traceback
    traceback.print_exc()
