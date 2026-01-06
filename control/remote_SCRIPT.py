#!/usr/bin/env python3

import socket
import time
import sys	

# HOST='127.0.0.1'
HOST='192.168.1.102'    # '172.16.103.101'
SCRIPT_PORT = 30002
DASHBOARD_PORT = 29999

def check_robot_ready_for_script():
    """Check if robot is in correct state for script execution"""
    try:
        dash = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        dash.settimeout(5)
        dash.connect((HOST, DASHBOARD_PORT))
        dash.recv(1024)  # Read welcome message
        
        # Check robot mode
        dash.send(str.encode('robotmode\n'))
        mode = dash.recv(1024).decode('utf-8').strip()
        print(f"Current Robot Mode: {mode}")
        
        # Check safety status
        dash.send(str.encode('safetystatus\n'))
        safety = dash.recv(1024).decode('utf-8').strip()
        print(f"Current Safety Status: {safety}")
        
        # Check if program is running
        dash.send(str.encode('running\n'))
        running = dash.recv(1024).decode('utf-8').strip()
        print(f"Program Running: {running}")
        
        dash.close()
        
        # Determine if ready
        if "POWER_OFF" in mode:
            print("\n⚠️  WARNING: Robot is POWERED OFF. Please power on first!")
            return False
        elif "Program running: true" in running:
            print("\n⚠️  WARNING: A program is running. Stop it before using freedrive!")
            return False
        elif "NORMAL" not in safety and "REDUCED" not in safety:
            print(f"\n⚠️  WARNING: Safety status is {safety}. Resolve safety issues first!")
            return False
        else:
            print("\n✓ Robot is ready for script commands")
            return True
            
    except Exception as e:
        print(f"⚠️  Could not check robot status: {e}")
        return False

def send_script_command(sock, command, description):
    """Send a script command (no response expected from Script server)"""
    sock.send(str.encode(command + '\n'))
    print(f'\n>>> Sent: {description}')
    print(f'    Command: {command}')
    input('Press RETURN to continue...')

try:
    print("="*70)
    print("UR10e SCRIPT SERVER CONNECTION")
    print("="*70)
    
    # First, check robot state via Dashboard
    print("\nChecking robot status...")
    if not check_robot_ready_for_script():
        print("\n" + "="*70)
        print("ROBOT NOT READY - Please resolve issues above")
        print("="*70)
        print("\nTo prepare robot for script commands:")
        print("1. Use remote_DASHBOARD.py to power on and release brakes")
        print("2. Make sure no program is running")
        print("3. Make sure safety status is NORMAL or REDUCED")
        sys.exit(1)
    
    # Connect to Script server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10)
    s.connect((HOST, SCRIPT_PORT))
    print(f"\n✓ Connected to Script server at {HOST}:{SCRIPT_PORT}")
    print("\nNote: Script server doesn't send responses for commands")
    print("Check robot behavior to verify commands are executing\n")
    input("Press RETURN to start sending commands...")


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

    # FREEDRIVE MODE
    print("\n" + "="*70)
    print("ENABLING FREEDRIVE MODE")
    print("="*70)
    print("Requirements:")
    print("  - Robot must be powered on")
    print("  - Brakes must be released")
    print("  - No program should be running")
    print("  - Safety status must be NORMAL or REDUCED")
    
    send_script_command(s, 'freedrive_mode()', 'Enable Freedrive Mode')
    print("⚠️  Robot should now be in freedrive - you can move it manually")
    print("⚠️  Press the button on the teach pendant to exit freedrive")
    time.sleep(5)  # Give time for freedrive to activate

    send_script_command(s, 'end_freedrive_mode()', 'Disable Freedrive Mode')
    print("✓ Freedrive mode ended")
    time.sleep(1)

    # POPUP EXAMPLE
    # send_script_command(s, 'popup("INICIANDO")', 'Show popup message')
    # time.sleep(2)
    
    # DIGITAL OUTPUT EXAMPLES
    # send_script_command(s, 'set_digital_out(7, True)', 'Digital output 7 HIGH')
    # time.sleep(1)
    
    # send_script_command(s, 'set_digital_out(7, False)', 'Digital output 7 LOW')
    # time.sleep(1)
    
    # TOOL DIGITAL OUTPUT EXAMPLES
    # send_script_command(s, 'set_tool_digital_out(0, True)', 'Tool digital output 0 HIGH')
    # time.sleep(1)
    
    # send_script_command(s, 'set_tool_digital_out(0, False)', 'Tool digital output 0 LOW')
    # time.sleep(1)
    
    # MOVE EXAMPLES
    # Note: d2r() converts degrees to radians
    # send_script_command(s, 
    #     'movej([d2r(124.47),d2r(-71.06),d2r(-100.79),d2r(-76),d2r(86.2),d2r(356.61)], a=1.4, v=1.05, t=5, r=0)',
    #     'Move to Pose 1')
    # time.sleep(6)  # Wait for move to complete
    
    # send_script_command(s, 
    #     'movej([d2r(115.23),d2r(-66.63),d2r(-44.36),d2r(-128.46),d2r(17.30),d2r(257.48)], a=1.4, v=1.05, t=5, r=0)',
    #     'Move to Pose 2')
    # time.sleep(6)  # Wait for move to complete
    
    


########  ########  ########  ########  
    print("\n" + "="*70)
    print("SCRIPT COMMANDS COMPLETED")
    print("="*70)
    s.close()
    
except KeyboardInterrupt:
    print("\n\nScript interrupted by user")
    try:
        s.close()
    except:
        pass
except socket.timeout:
    print("\n✗ ERROR: Connection timeout")
    print(f"Could not connect to robot at {HOST}")
except ConnectionRefusedError:
    print("\n✗ ERROR: Connection refused")
    print(f"Check that robot is accessible at {HOST}:{SCRIPT_PORT}")
except Exception as e:
    print(f"\n✗ ERROR: {e}")
    import traceback
    traceback.print_exc()
