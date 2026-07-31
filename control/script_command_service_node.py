#!/usr/bin/env python3
"""
ROS2 Node for sending SCRIPT commands to UR robot via SCRIPT interface.
Integrates DASHBOARD interface to stop/restart programs during command execution.
"""

import socket
import time
import rclpy
from rclpy.node import Node
from arm_control.srv import ScriptCommand


class ScriptCommandServiceNode(Node):
    """
    ROS2 service node that sends SCRIPT commands to UR robot.
    Manages both SCRIPT (port 30002) and DASHBOARD (port 29999) interfaces.
    """

    def __init__(self):
        super().__init__('script_command_service_node')
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.56.101')
        self.declare_parameter('script_port', 30002)
        self.declare_parameter('dashboard_port', 29999)
        self.declare_parameter('program_name', 'Test_external_control.urp')
        
        # Get parameters
        self.robot_ip = self.get_parameter('robot_ip').value
        self.script_port = self.get_parameter('script_port').value
        self.dashboard_port = self.get_parameter('dashboard_port').value
        self.program_name = self.get_parameter('program_name').value
        
        # Initialize socket connections as None
        self.script_socket = None
        self.dashboard_socket = None
        
        # Define command library
        self.command_library = {
            'popup': {
                'description': 'Display a popup message on the teach pendant',
                'parameters': ['message (string)'],
                'example': 'popup("Hello World")',
                'format': 'popup("{0}")'
            },
            'close_popup': {
                'description': 'Close any open popup on the teach pendant',
                'parameters': [],
                'example': 'close popup',
                'format': 'close popup'
            },
            'set_digital_out': {
                'description': 'Set a digital output to HIGH (True) or LOW (False)',
                'parameters': ['output_number (int)', 'state (True/False)'],
                'example': 'set_digital_out(0, True)',
                'format': 'set_digital_out({0}, {1})'
            },
            'set_tool_digital_out': {
                'description': 'Set a tool digital output to HIGH (True) or LOW (False)',
                'parameters': ['output_number (int)', 'state (True/False)'],
                'example': 'set_tool_digital_out(1, True)',
                'format': 'set_tool_digital_out({0}, {1})'
            },
            'movel': {
                'description': 'Linear move to target pose',
                'parameters': ['x (m)', 'y (m)', 'z (m)', 'rx (rad)', 'ry (rad)', 'rz (rad)', 
                              'a (m/s²)', 'v (m/s)', 't (s)', 'r (m)'],
                'example': 'movel(p[0.2, 0.3, 0.5, 0, 0, 3.14], a=1.2, v=0.25, t=0, r=0)',
                'format': 'movel(p[{0}, {1}, {2}, {3}, {4}, {5}], a={6}, v={7}, t={8}, r={9})'
            },
            'movej': {
                'description': 'Joint move to target joint positions (in radians)',
                'parameters': ['j0 (rad)', 'j1 (rad)', 'j2 (rad)', 'j3 (rad)', 'j4 (rad)', 'j5 (rad)',
                              'a (rad/s²)', 'v (rad/s)', 't (s)', 'r (rad)'],
                'example': 'movej([0, -1.57, 0, -1.57, 0, 0], a=1.4, v=1.05, t=0, r=0)',
                'format': 'movej([{0}, {1}, {2}, {3}, {4}, {5}], a={6}, v={7}, t={8}, r={9})'
            },
            'movej_degrees': {
                'description': 'Joint move with angles in degrees (converted to radians)',
                'parameters': ['j0 (deg)', 'j1 (deg)', 'j2 (deg)', 'j3 (deg)', 'j4 (deg)', 'j5 (deg)',
                              'a (rad/s²)', 'v (rad/s)', 't (s)', 'r (rad)'],
                'example': 'movej([d2r(90), d2r(-90), d2r(0), d2r(-90), d2r(0), d2r(0)], a=1.4, v=1.05, t=0, r=0)',
                'format': 'movej([d2r({0}), d2r({1}), d2r({2}), d2r({3}), d2r({4}), d2r({5})], a={6}, v={7}, t={8}, r={9})'
            },
            'speedl': {
                'description': 'Move with constant linear velocity',
                'parameters': ['vx (m/s)', 'vy (m/s)', 'vz (m/s)', 'vrx (rad/s)', 'vry (rad/s)', 'vrz (rad/s)',
                              'a (m/s²)', 't (s)'],
                'example': 'speedl([0.1, 0, 0, 0, 0, 0], a=0.5, t=10)',
                'format': 'speedl([{0}, {1}, {2}, {3}, {4}, {5}], a={6}, t={7})'
            },
            'speedj': {
                'description': 'Move with constant joint velocities',
                'parameters': ['qd0 (rad/s)', 'qd1 (rad/s)', 'qd2 (rad/s)', 'qd3 (rad/s)', 'qd4 (rad/s)', 'qd5 (rad/s)',
                              'a (rad/s²)', 't (s)'],
                'example': 'speedj([0.1, 0, 0, 0, 0, 0], a=0.5, t=10)',
                'format': 'speedj([{0}, {1}, {2}, {3}, {4}, {5}], a={6}, t={7})'
            },
            'stopj': {
                'description': 'Stop joint movement with specified acceleration',
                'parameters': ['a (rad/s²)'],
                'example': 'stopj(2.0)',
                'format': 'stopj({0})'
            },
            'stopl': {
                'description': 'Stop linear movement with specified acceleration',
                'parameters': ['a (m/s²)'],
                'example': 'stopl(2.0)',
                'format': 'stopl({0})'
            }
        }
        
        # Create service
        self.srv = self.create_service(
            ScriptCommand,
            'send_script_command',
            self.handle_script_command
        )
        
        # Display command library on initialization
        self.display_command_library()
        
        self.get_logger().info('Script Command Service Node initialized')
        self.get_logger().info(f'Robot IP: {self.robot_ip}')
        self.get_logger().info(f'SCRIPT Port: {self.script_port}')
        self.get_logger().info(f'DASHBOARD Port: {self.dashboard_port}')
        self.get_logger().info(f'Program Name: {self.program_name}')
        self.get_logger().info('Service available at: /send_script_command')
        self.get_logger().info('\033[1;32mUse: ros2 service call /send_script_command arm_control/srv/ScriptCommand "{command_name: \'movel\', numeric_params: [0.2, 0.3, 0.5, 0.0, 0.0, 3.14, 1.2, 0.25, 0.0, 0.0], stop_program: true, restart_program: true}"\033[0m')

    def display_command_library(self):
        """Display available commands in the library"""
        self.get_logger().info('=' * 80)
        self.get_logger().info('AVAILABLE SCRIPT COMMANDS LIBRARY:')
        self.get_logger().info('=' * 80)
        
        for cmd_name, cmd_info in self.command_library.items():
            self.get_logger().info(f'\nCommand: {cmd_name}')
            self.get_logger().info(f'  Description: {cmd_info["description"]}')
            if cmd_info['parameters']:
                self.get_logger().info(f'  Parameters: {", ".join(cmd_info["parameters"])}')
            else:
                self.get_logger().info(f'  Parameters: None')
            self.get_logger().info(f'  Example: {cmd_info["example"]}')
        
        self.get_logger().info('=' * 80)

    def connect_script_interface(self):
        """Establish connection to SCRIPT interface (port 30002)"""
        try:
            if self.script_socket:
                self.script_socket.close()
            
            self.script_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.script_socket.settimeout(5.0)
            self.script_socket.connect((self.robot_ip, self.script_port))
            self.get_logger().info(f'Connected to SCRIPT interface at {self.robot_ip}:{self.script_port}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to SCRIPT interface: {e}')
            return False

    def connect_dashboard_interface(self):
        """Establish connection to DASHBOARD interface (port 29999)"""
        try:
            if self.dashboard_socket:
                self.dashboard_socket.close()
            
            self.dashboard_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dashboard_socket.settimeout(5.0)
            self.dashboard_socket.connect((self.robot_ip, self.dashboard_port))
            # Read initial connection message
            data = self.dashboard_socket.recv(1024)
            self.get_logger().info(f'Connected to DASHBOARD interface: {data.decode("utf-8").strip()}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to DASHBOARD interface: {e}')
            return False

    def send_dashboard_command(self, command):
        """Send command to DASHBOARD interface and get response"""
        try:
            self.dashboard_socket.send(str.encode(command + '\n'))
            self.get_logger().debug(f'DASHBOARD >>> {command}')
            data = self.dashboard_socket.recv(1024)
            response = data.decode('utf-8').strip()
            self.get_logger().debug(f'DASHBOARD <<< {response}')
            return True, response
        except Exception as e:
            self.get_logger().error(f'DASHBOARD command failed: {e}')
            return False, str(e)

    def stop_robot_program(self):
        """Stop the currently running program on the robot"""
        self.get_logger().info('Stopping robot program...')
        
        # Close any popups that might interfere
        self.send_dashboard_command('close popup')
        self.send_dashboard_command('close safety popup')
        time.sleep(0.2)
        
        # Stop the program
        success, response = self.send_dashboard_command('stop')
        if success:
            time.sleep(0.5)  # Wait for program to stop
            self.get_logger().info('Robot program stopped')
            return True
        else:
            self.get_logger().error('Failed to stop robot program')
            return False

    def start_robot_program(self):
        """Start/restart the robot program"""
        self.get_logger().info('Starting robot program...')
        
        # Check if program is loaded
        success, response = self.send_dashboard_command('get loaded program')
        
        if 'No program loaded' in response or self.program_name not in response:
            self.get_logger().info(f'Loading program: {self.program_name}')
            success, response = self.send_dashboard_command(f'load {self.program_name}')
            if not success:
                self.get_logger().error(f'Failed to load program: {response}')
                return False
            time.sleep(1.0)  # Wait for program to load
        
        # Close any popups
        self.send_dashboard_command('close popup')
        self.send_dashboard_command('close safety popup')
        time.sleep(0.2)
        
        # Start the program
        success, response = self.send_dashboard_command('play')
        if success:
            time.sleep(0.5)  # Wait for program to start
            self.get_logger().info('Robot program started')
            return True
        else:
            self.get_logger().error('Failed to start robot program')
            return False

    def send_script_command(self, command_string):
        """Send a SCRIPT command to the robot"""
        try:
            self.script_socket.send(str.encode(command_string + '\n'))
            self.get_logger().info(f'SCRIPT >>> {command_string}')
            time.sleep(0.5)  # Wait for command execution
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to send SCRIPT command: {e}')
            return False

    def format_command(self, command_name, numeric_params, string_params):
        """Format command string from library definition and parameters"""
        if command_name not in self.command_library:
            return None
        
        cmd_format = self.command_library[command_name]['format']
        
        try:
            # Combine numeric and string parameters in order
            all_params = list(numeric_params) + list(string_params)
            
            # Format the command with provided parameters
            if all_params:
                command_string = cmd_format.format(*all_params)
            else:
                command_string = cmd_format
            
            return command_string
        except Exception as e:
            self.get_logger().error(f'Invalid parameters for command {command_name}: {e}')
            return None

    def handle_script_command(self, request, response):
        """Handle incoming service requests to send SCRIPT commands"""
        self.get_logger().info(f'Received command request: {request.command_name}')
        self.get_logger().info(f'Stop program: {request.stop_program}, Restart program: {request.restart_program}')
        
        # Validate command exists in library
        if request.command_name not in self.command_library:
            response.success = False
            response.message = f'Unknown command: {request.command_name}. Use display_library service to see available commands.'
            self.get_logger().warn(response.message)
            return response
        
        # Format the command
        command_string = self.format_command(request.command_name, request.numeric_params, request.string_params)
        if not command_string:
            response.success = False
            response.message = 'Invalid parameters for command'
            return response
        
        # Step 1: Handle DASHBOARD operations if stop_program is requested
        if request.stop_program:
            # Connect to DASHBOARD interface
            if not self.connect_dashboard_interface():
                response.success = False
                response.message = 'Failed to connect to DASHBOARD interface'
                return response
            
            # Stop robot program
            if not self.stop_robot_program():
                response.success = False
                response.message = 'Failed to stop robot program'
                self.dashboard_socket.close()
                return response
        
        # Step 2: Connect to SCRIPT interface
        if not self.connect_script_interface():
            response.success = False
            response.message = 'Failed to connect to SCRIPT interface'
            # Try to restart program before closing if we stopped it
            if request.stop_program:
                self.start_robot_program()
                self.dashboard_socket.close()
            return response
        
        # Step 3: Send SCRIPT command
        if not self.send_script_command(command_string):
            response.success = False
            response.message = 'Failed to send SCRIPT command'
            self.script_socket.close()
            # Try to restart program if we stopped it
            if request.stop_program:
                self.start_robot_program()
                self.dashboard_socket.close()
            return response
        
        # Step 4: Close SCRIPT interface
        self.script_socket.close()
        self.get_logger().info('SCRIPT command sent successfully')
        
        # Step 5: Handle program restart if requested
        if request.restart_program:
            # Connect to DASHBOARD if not already connected
            if not request.stop_program:
                if not self.connect_dashboard_interface():
                    response.success = False
                    response.message = 'Command sent but failed to connect to DASHBOARD for restart'
                    return response
            
            # Restart robot program
            if not self.start_robot_program():
                response.success = False
                response.message = 'Command sent but failed to restart robot program'
                self.dashboard_socket.close()
                return response
            
            self.dashboard_socket.close()
        elif request.stop_program:
            # Close DASHBOARD connection if we opened it but don't need to restart
            self.dashboard_socket.close()
        
        # Success
        response.success = True
        response.message = f'Command "{request.command_name}" executed successfully'
        self.get_logger().info(response.message)
        
        return response

    def destroy_node(self):
        """Cleanup on node shutdown"""
        if self.script_socket:
            self.script_socket.close()
        if self.dashboard_socket:
            self.dashboard_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScriptCommandServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
