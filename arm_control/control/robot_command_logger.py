#!/usr/bin/env python3
"""
Interactive Robot Command Logger
Allows you to send commands manually and see responses in real-time
Logs everything to a file for debugging
"""
import socket
import sys
from datetime import datetime

HOST = '192.168.1.102'  # Change to your robot IP
PORT = 29999
LOG_FILE = 'robot_communication.log'

def log_message(message, to_file=True, to_screen=True):
    """Log message with timestamp"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    formatted = f"[{timestamp}] {message}"
    
    if to_screen:
        print(formatted)
    
    if to_file:
        with open(LOG_FILE, 'a') as f:
            f.write(formatted + '\n')

def main():
    print("="*70)
    print("INTERACTIVE ROBOT COMMAND TOOL")
    print("="*70)
    print(f"Logging to: {LOG_FILE}")
    print("Commands: Type any Dashboard command (or 'help', 'status', 'quit')")
    print("="*70 + "\n")
    
    log_message("="*70, to_screen=False)
    log_message("New session started", to_screen=False)
    log_message("="*70, to_screen=False)
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((HOST, PORT))
        
        # Read initial connection message
        data = s.recv(1024)
        response = data.decode('utf-8').strip()
        log_message(f"CONNECTED: {response}")
        
        # Common commands help
        help_text = """
Common Commands:
  Status Queries:
    - robotmode
    - safetystatus
    - programState
    - running
    - get loaded program
    - is in remote control
    
  Control Commands:
    - power on
    - power off
    - brake release
    - load <program.urp>
    - play
    - pause
    - stop
    - close popup
    - close safety popup
    - unlock protective stop
    
  Special:
    - help (show this help)
    - status (quick status check)
    - quit (exit program)
"""
        
        while True:
            try:
                command = input("\n> ").strip()
                
                if not command:
                    continue
                
                if command.lower() == 'quit':
                    log_message("USER: Quit command received")
                    break
                
                if command.lower() == 'help':
                    print(help_text)
                    continue
                
                if command.lower() == 'status':
                    # Quick status check
                    status_commands = [
                        'robotmode',
                        'safetystatus',
                        'programState',
                        'running',
                        'get loaded program'
                    ]
                    print("\n" + "-"*70)
                    for cmd in status_commands:
                        s.send(str.encode(cmd + '\n'))
                        log_message(f"SENT: {cmd}", to_screen=False)
                        data = s.recv(1024)
                        response = data.decode('utf-8').strip()
                        log_message(f"RECV: {response}", to_screen=False)
                        print(f"{cmd:25s} → {response}")
                    print("-"*70)
                    continue
                
                # Send the command
                s.send(str.encode(command + '\n'))
                log_message(f"SENT: {command}")
                
                # Receive response
                data = s.recv(1024)
                response = data.decode('utf-8').strip()
                log_message(f"RECV: {response}")
                
            except KeyboardInterrupt:
                log_message("\nUSER: Keyboard interrupt")
                break
            except socket.timeout:
                log_message("ERROR: Socket timeout - no response from robot")
            except Exception as e:
                log_message(f"ERROR: {e}")
        
        s.close()
        log_message("Connection closed")
        print(f"\n✓ Session logged to: {LOG_FILE}")
        
    except Exception as e:
        log_message(f"FATAL ERROR: {e}")
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
