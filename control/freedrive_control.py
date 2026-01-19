#!/usr/bin/env python3
"""
Comprehensive Freedrive Control Script for UR10e
This script properly prepares the robot and enables freedrive mode
"""

import socket
import time
import sys

HOST = '192.168.1.102'
DASHBOARD_PORT = 29999
SCRIPT_PORT = 30002

def send_dashboard_command(sock, command):
    """Send Dashboard command and return response"""
    sock.send(str.encode(command + '\n'))
    data = sock.recv(1024)
    return data.decode('utf-8').strip()

def send_script_command(sock, command):
    """Send Script command (no response)"""
    sock.send(str.encode(command + '\n'))
    time.sleep(0.1)  # Small delay for command processing

def get_robot_status(dash_sock):
    """Get comprehensive robot status"""
    status = {}
    status['mode'] = send_dashboard_command(dash_sock, 'robotmode')
    status['safety'] = send_dashboard_command(dash_sock, 'safetystatus')
    status['running'] = send_dashboard_command(dash_sock, 'running')
    status['program_state'] = send_dashboard_command(dash_sock, 'programState')
    return status

def prepare_robot_for_freedrive(dash_sock):
    """Prepare robot for freedrive mode"""
    print("\n" + "="*70)
    print("PREPARING ROBOT FOR FREEDRIVE")
    print("="*70)
    
    status = get_robot_status(dash_sock)
    print(f"\nCurrent state:")
    print(f"  Robot Mode: {status['mode']}")
    print(f"  Safety Status: {status['safety']}")
    print(f"  Program Running: {status['running']}")
    print(f"  Program State: {status['program_state']}")
    
    # Stop any running program
    if "true" in status['running'].lower():
        print("\n→ Stopping running program...")
        response = send_dashboard_command(dash_sock, 'stop')
        print(f"   {response}")
        time.sleep(1)
    
    # Close any popups
    print("\n→ Closing any popups...")
    send_dashboard_command(dash_sock, 'close popup')
    send_dashboard_command(dash_sock, 'close safety popup')
    
    # Check if powered on
    if "POWER_OFF" in status['mode']:
        print("\n→ Powering on robot...")
        response = send_dashboard_command(dash_sock, 'power on')
        print(f"   {response}")
        time.sleep(3)
        
        print("\n→ Releasing brakes...")
        response = send_dashboard_command(dash_sock, 'brake release')
        print(f"   {response}")
        time.sleep(2)
    elif "IDLE" in status['mode'] or "RUNNING" in status['mode']:
        print("\n✓ Robot already powered on")
    else:
        print(f"\n⚠️  Robot in mode: {status['mode']}")
        print("   You may need to resolve issues on teach pendant")
    
    # Final status check
    status = get_robot_status(dash_sock)
    print(f"\nFinal state:")
    print(f"  Robot Mode: {status['mode']}")
    print(f"  Safety Status: {status['safety']}")
    
    # Check if ready
    if "IDLE" in status['mode'] or "RUNNING" in status['mode']:
        if "NORMAL" in status['safety'] or "REDUCED" in status['safety']:
            print("\n✓ Robot is ready for freedrive!")
            return True
    
    print("\n✗ Robot may not be ready for freedrive")
    print("  Check teach pendant for errors or warnings")
    return False

def main():
    print("="*70)
    print("UR10e FREEDRIVE CONTROL")
    print("="*70)
    
    dash_sock = None
    script_sock = None
    
    try:
        # Connect to Dashboard server
        print("\nConnecting to Dashboard server...")
        dash_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        dash_sock.settimeout(5)
        dash_sock.connect((HOST, DASHBOARD_PORT))
        welcome = dash_sock.recv(1024).decode('utf-8').strip()
        print(f"✓ Dashboard: {welcome}")
        
        # Prepare robot
        if not prepare_robot_for_freedrive(dash_sock):
            print("\n⚠️  WARNING: Robot may not be in correct state")
            response = input("Continue anyway? (y/N): ")
            if response.lower() != 'y':
                print("Aborted by user")
                return
        
        # Connect to Script server
        print("\nConnecting to Script server...")
        script_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        script_sock.settimeout(5)
        script_sock.connect((HOST, SCRIPT_PORT))
        print(f"✓ Script server connected")
        
        # Enable freedrive
        print("\n" + "="*70)
        print("FREEDRIVE MODE")
        print("="*70)
        
        input("\nPress RETURN to enable freedrive mode...")
        
        print("\n→ Sending freedrive_mode() command...")
        send_script_command(script_sock, 'freedrive_mode()')
        
        print("\n" + "="*70)
        print("✓ FREEDRIVE MODE ACTIVATED")
        print("="*70)
        print("""
Instructions:
  1. You should now be able to move the robot manually
  2. The teach pendant might show a freedrive icon
  3. Press the button on the teach pendant to exit freedrive
  4. Or press RETURN here to end freedrive mode via script
        """)
        
        input("Press RETURN to disable freedrive mode...")
        
        print("\n→ Sending end_freedrive_mode() command...")
        send_script_command(script_sock, 'end_freedrive_mode()')
        
        print("\n✓ Freedrive mode disabled")
        
        # Final status
        time.sleep(1)
        status = get_robot_status(dash_sock)
        print(f"\nFinal Robot Mode: {status['mode']}")
        
    except socket.timeout:
        print(f"\n✗ ERROR: Connection timeout to {HOST}")
        print("Check network connection and robot IP address")
    except ConnectionRefusedError:
        print(f"\n✗ ERROR: Connection refused by {HOST}")
        print("Check that robot is powered on and network is accessible")
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if script_sock:
            try:
                script_sock.close()
            except:
                pass
        if dash_sock:
            try:
                dash_sock.close()
            except:
                pass
        print("\n" + "="*70)
        print("DISCONNECTED")
        print("="*70)

if __name__ == "__main__":
    main()
