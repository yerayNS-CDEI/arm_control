#!/usr/bin/env python3
"""
Robot Status Checker - Quick diagnostic tool for UR10e
Use this to check the current state of the robot without sending commands
"""
import socket
import sys

HOST = '192.168.1.102'  # Change to your robot IP
PORT = 29999

def query_robot(sock, command):
    """Send a query command and return the response"""
    sock.send(str.encode(command + '\n'))
    data = sock.recv(1024)
    return data.decode('utf-8').strip()

def main():
    try:
        print("="*70)
        print("UR10e ROBOT STATUS DIAGNOSTIC TOOL")
        print("="*70)
        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)  # 5 second timeout
        s.connect((HOST, PORT))
        
        # Read initial connection message
        data = s.recv(1024)
        print(f"\n✓ Connected: {data.decode('utf-8').strip()}\n")
        
        # Get all status information
        print("ROBOT INFORMATION:")
        print("-" * 70)
        print(f"Serial Number:     {query_robot(s, 'get serial number')}")
        print(f"Robot Model:       {query_robot(s, 'get robot model')}")
        print(f"Software Version:  {query_robot(s, 'PolyscopeVersion')}")
        
        print("\nROBOT STATE:")
        print("-" * 70)
        print(f"Robot Mode:        {query_robot(s, 'robotmode')}")
        print(f"Safety Status:     {query_robot(s, 'safetystatus')}")
        print(f"Remote Control:    {query_robot(s, 'is in remote control')}")
        print(f"Operational Mode:  {query_robot(s, 'get operational mode')}")
        
        print("\nPROGRAM STATE:")
        print("-" * 70)
        print(f"Loaded Program:    {query_robot(s, 'get loaded program')}")
        print(f"Program State:     {query_robot(s, 'programState')}")
        print(f"Is Running:        {query_robot(s, 'running')}")
        print(f"Is Saved:          {query_robot(s, 'isProgramSaved')}")
        
        print("\n" + "="*70)
        print("INTERPRETATION GUIDE:")
        print("="*70)
        print("""
Robot Modes:
  - POWER_OFF: Robot needs to be powered on
  - IDLE: Robot is ready but not running a program
  - RUNNING: Robot is executing a program
  - BOOTING: Robot is starting up
  
Safety Status:
  - NORMAL: Everything is OK
  - PROTECTIVE_STOP: Robot stopped due to collision/force
  - SAFEGUARD_STOP: Emergency stop or safety I/O triggered
  - FAULT: Safety system has a fault
  
Common Issues:
  1. If Robot Mode = POWER_OFF → Send 'power on' and 'brake release'
  2. If Safety Status != NORMAL → Check teach pendant or use 'unlock protective stop'
  3. If Remote Control = false → Enable remote control on teach pendant
  4. If Program fails to load → Check program name and path
  5. If Program won't start → Check for safety popups or protective stops
        """)
        
        s.close()
        print("\n✓ Status check completed successfully")
        
    except socket.timeout:
        print(f"\n✗ ERROR: Connection timeout. Is the robot at {HOST}:{PORT}?")
        sys.exit(1)
    except ConnectionRefusedError:
        print(f"\n✗ ERROR: Connection refused. Check IP address and robot network settings.")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
