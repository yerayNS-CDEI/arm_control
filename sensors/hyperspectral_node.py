#!/usr/bin/env python3

##################################################
#### ROS2 HYPERSPECTRAL SENSOR NODE
##################################################

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from navi_wall_interfaces.srv import HyperspectralCommand, HyperspectralConfig
from .sensors_lib.lenz_client import LenzClient
import threading
import time
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


class HyperspectralNode(Node):

    def __init__(self):
        super().__init__("hyperspectral_node")

        # Parameters
        self.declare_parameter("vis_ip", "193.167.0.30")
        self.declare_parameter("nir_ip", "193.167.0.31")
        self.declare_parameter("vis_mtr", 60000)
        self.declare_parameter("vis_mti", 60000)
        self.declare_parameter("nir_mtr", 20000)
        self.declare_parameter("nir_mti", 20000)
        self.declare_parameter("auto_configure", True)
        self.declare_parameter("enable_plotting", False)
        self.declare_parameter("enable_csv_logging", True)
        
        # Default CSV directory: package's resource folder
        try:
            package_name = 'arm_control'
            package_dir = get_package_share_directory(package_name)
            default_csv_dir = os.path.join(package_dir, '..', '..', '..', '..', 'src', package_name, 'resource')
            default_csv_dir = os.path.abspath(default_csv_dir)
        except:
            # Fallback to home directory if package not found
            default_csv_dir = os.path.expanduser("~")
        
        self.declare_parameter("csv_directory", default_csv_dir)

        vis_ip = self.get_parameter("vis_ip").value
        nir_ip = self.get_parameter("nir_ip").value
        self.vis_mtr = self.get_parameter("vis_mtr").value
        self.vis_mti = self.get_parameter("vis_mti").value
        self.nir_mtr = self.get_parameter("nir_mtr").value
        self.nir_mti = self.get_parameter("nir_mti").value
        auto_configure = self.get_parameter("auto_configure").value
        self.enable_plotting = self.get_parameter("enable_plotting").value
        self.enable_csv_logging = self.get_parameter("enable_csv_logging").value
        csv_directory = self.get_parameter("csv_directory").value

        # Publishers (optional - publish after successful measurements)
        self.vis_pub = self.create_publisher(Float32MultiArray, "/hyperspectral/vis_spectrum", 10)
        self.nir_pub = self.create_publisher(Float32MultiArray, "/hyperspectral/nir_spectrum", 10)

        # Wavelength arrays (256 points each)
        self.vis_wavelengths = np.linspace(325.3, 792.6, 256)  # VIS: 325.3-792.6 nm
        self.nir_wavelengths = np.linspace(991, 1707, 256)     # NIR: 991-1707 nm

        # Initialize CSV logging
        self.csv_file_path = None
        if self.enable_csv_logging:
            self._initialize_csv_file(csv_directory)

        # Initialize plotting if enabled
        self.fig = None
        self.ax_vis = None
        self.ax_nir = None
        
        if self.enable_plotting:
            matplotlib.use('TkAgg')  # Use interactive backend
            self.fig, (self.ax_vis, self.ax_nir) = plt.subplots(2, 1, figsize=(10, 8))
            self.fig.suptitle('Hyperspectral Measurements', fontsize=14)
            self.ax_vis.set_title('VIS Spectrum')
            self.ax_vis.set_xlabel('Wavelength (nm)')
            self.ax_vis.set_ylabel('Intensity')
            self.ax_vis.grid(True)
            self.ax_nir.set_title('NIR Spectrum')
            self.ax_nir.set_xlabel('Wavelength (nm)')
            self.ax_nir.set_ylabel('Intensity')
            self.ax_nir.grid(True)
            plt.ion()  # Enable interactive mode
            plt.show(block=False)
            self.get_logger().info("✓ Plotting enabled")

        # Services
        self.configure_srv = self.create_service(
            HyperspectralConfig, 
            "hyperspectral/configure", 
            self.configure_callback
        )
        self.measurement_srv = self.create_service(
            HyperspectralCommand, 
            "hyperspectral/measurement", 
            self.measurement_callback
        )

        # Connect sensors (LenzClient handles keepalive automatically)
        self.get_logger().info("Connecting to VIS and NIR sensors...")
        self.vis = LenzClient(vis_ip)
        self.nir = LenzClient(nir_ip)

        try:
            self.vis.connect()
            self.nir.connect()
            self.get_logger().info("✓ VIS & NIR connected. PNG keepalive running automatically.")
            
            # Auto-configure if enabled
            if auto_configure:
                time.sleep(2)  # Stabilization
                self.get_logger().info("Auto-configuring sensors...")
                self._configure_sensors(self.vis_mtr, self.vis_mti, self.nir_mtr, self.nir_mti)
                self.get_logger().info("✓ Sensors configured and ready.")
            
        except Exception as e:
            self.get_logger().error(f"✗ Connection failed: {e}")
            raise

        self.get_logger().info("HyperspectralNode ready.")


    # ----------------------------------------------------------
    # Helper: Configure sensors
    # ----------------------------------------------------------
    def _configure_sensors(self, vis_mtr, vis_mti, nir_mtr, nir_mti):
        """Internal configuration helper."""
        self.vis.send_config("MTR", vis_mtr); time.sleep(0.2)
        self.vis.send_config("MTI", vis_mti); time.sleep(0.2)
        self.vis.send_config("THP", 1);       time.sleep(0.2)
        
        self.nir.send_config("MTR", nir_mtr); time.sleep(0.2)
        self.nir.send_config("MTI", nir_mti); time.sleep(0.2)
        self.nir.send_config("THP", 1);       time.sleep(0.2)
        
        time.sleep(3)  # Stabilization


    # ----------------------------------------------------------
    # SERVICE: Configuration
    # ----------------------------------------------------------
    def configure_callback(self, request, response):
        """
        Configure sensor parameters.
        Request:
            sensor: "VIS" or "NIR"
            command: "MTR", "MTI", or "THP"
            value: int (e.g., 60000 for MTI)
        Response:
            success: bool
            message: str
        """
        sensor_name = request.sensor.upper()
        cmd = request.command.upper()
        value = request.value

        self.get_logger().info(f"Config request: {sensor_name} {cmd}={value}")

        client = self.vis if sensor_name == "VIS" else self.nir

        try:
            client.send_config(cmd, value)
            time.sleep(0.2)
            
            response.success = True
            response.message = f"OK: {sensor_name} {cmd}={value}"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"ERROR: {str(e)}"
            self.get_logger().error(response.message)

        return response


    # ----------------------------------------------------------
    # SERVICE: Measurement
    # ----------------------------------------------------------
    def measurement_callback(self, request, response):
        """
        Execute measurement command (GDS, GDR, GRF, GSM) with automatic retry.
        Request:
            command: str ("GDS", "GDR", "GRF", "GSM")
        Response:
            vis_ok: bool
            vis_spectrum: uint16[]
            vis_status: int32
            nir_ok: bool
            nir_spectrum: uint16[]
            nir_status: int32
            message: str
        """
        cmd = request.command.upper()
        x_coord = request.x_coord
        y_coord = request.y_coord
        z_coord = request.z_coord
        self.get_logger().info(f"Measurement request: {cmd} at ({x_coord}, {y_coord}, {z_coord})")

        # Try with automatic retry (up to 3 attempts)
        for attempt in range(3):
            try:
                # Clear buffers
                self.vis.flush_buffer()
                self.nir.flush_buffer()
                time.sleep(0.1)

                # Wait for safe timing window after PNG
                self.vis.wait_after_ping()
                self.nir.wait_after_ping()

                # Send commands
                self.get_logger().info(f"[Attempt {attempt+1}/3] Sending {cmd}...")
                self.vis.send_simple(cmd)
                self.nir.send_simple(cmd)
                break  # Success

            except (BrokenPipeError, ConnectionError, OSError) as e:
                self.get_logger().warning(f"[Attempt {attempt+1}/3] FAILED: {e}")
                
                if attempt < 2:  # Retry
                    self.get_logger().info("Reconnecting...")
                    try:
                        self.vis.close()
                        self.nir.close()
                    except:
                        pass
                    
                    time.sleep(2)
                    self.vis = LenzClient(self.get_parameter("vis_ip").value)
                    self.nir = LenzClient(self.get_parameter("nir_ip").value)
                    self.vis.connect()
                    self.nir.connect()
                    time.sleep(3)
                    
                    # Reconfigure
                    self._configure_sensors(
                        self.vis_mtr, self.vis_mti,
                        self.nir_mtr, self.nir_mti
                    )
                    time.sleep(2)
                else:
                    # All attempts failed
                    response.vis_ok = False
                    response.nir_ok = False
                    response.message = f"Failed after 3 attempts: {e}"
                    return response

        # Wait for sensor to complete measurement
        time.sleep(1.2)

        # Read results in parallel threads
        results = {}
        
        def read_sensor(client, label):
            try:
                results[label] = client.read_frame([cmd])
            except Exception as e:
                results[label] = f"ERROR: {e}"

        t_vis = threading.Thread(target=read_sensor, args=(self.vis, "VIS"))
        t_nir = threading.Thread(target=read_sensor, args=(self.nir, "NIR"))

        t_vis.start()
        t_nir.start()
        t_vis.join()
        t_nir.join()

        # Clean up remaining bytes
        time.sleep(0.1)
        self.vis.flush_buffer()
        self.nir.flush_buffer()

        # Process VIS results
        if isinstance(results["VIS"], dict):
            response.vis_ok = True
            response.vis_spectrum = list(results["VIS"]["spectrum"])
            response.vis_status = results["VIS"]["status"]
            
            # Publish VIS spectrum
            vis_msg = Float32MultiArray()
            vis_msg.data = [float(x) for x in response.vis_spectrum]
            self.vis_pub.publish(vis_msg)
        else:
            response.vis_ok = False
            response.vis_spectrum = []
            response.vis_status = -1

        # Process NIR results
        if isinstance(results["NIR"], dict):
            response.nir_ok = True
            response.nir_spectrum = list(results["NIR"]["spectrum"])
            response.nir_status = results["NIR"]["status"]
            
            # Publish NIR spectrum
            nir_msg = Float32MultiArray()
            nir_msg.data = [float(x) for x in response.nir_spectrum]
            self.nir_pub.publish(nir_msg)
        else:
            response.nir_ok = False
            response.nir_spectrum = []
            response.nir_status = -1

        # Set response message
        if response.vis_ok and response.nir_ok:
            response.message = f"OK: {cmd} completed successfully"
            self.get_logger().info(response.message)
        else:
            response.message = f"PARTIAL: VIS={response.vis_ok}, NIR={response.nir_ok}"
            self.get_logger().warning(response.message)

        # Update plots if enabled
        if self.enable_plotting and response.vis_ok and response.nir_ok:
            self._update_plots(response.vis_spectrum, response.nir_spectrum, cmd)

        # Log to CSV if enabled
        if self.enable_csv_logging and response.vis_ok and response.nir_ok:
            self._log_to_csv(cmd, x_coord, y_coord, z_coord, response.vis_spectrum, response.nir_spectrum)

        return response


    # ----------------------------------------------------------
    # Helper: Initialize CSV file
    # ----------------------------------------------------------
    def _initialize_csv_file(self, directory):
        """Create or open CSV file with date-based filename."""
        try:
            # Create filename with current date: Hyperspectral_data_DD_MM_YYYY.csv
            now = datetime.now()
            filename = f"Hyperspectral_data_{now.day:02d}_{now.month:02d}_{now.year}.csv"
            self.csv_file_path = os.path.join(directory, filename)
            
            # Check if file exists to determine if we need to write header
            file_exists = os.path.isfile(self.csv_file_path)
            
            if not file_exists:
                # Create new file with header
                with open(self.csv_file_path, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    
                    # Build header: Measure Type, Date, Time, X, Y, Z, VIS wavelengths, NIR wavelengths
                    header = ['Measure_Type', 'Date', 'Time', 'X_Coordinate', 'Y_Coordinate', 'Z_Coordinate']
                    
                    # Add VIS wavelength columns
                    for wl in self.vis_wavelengths:
                        header.append(f'{wl:.1f}')
                    
                    # Add NIR wavelength columns
                    for wl in self.nir_wavelengths:
                        header.append(f'{wl:.1f}')
                    
                    writer.writerow(header)
                
                self.get_logger().info(f"✓ CSV file created: {self.csv_file_path}")
            else:
                self.get_logger().info(f"✓ CSV file exists, will append data: {self.csv_file_path}")
                
        except Exception as e:
            self.get_logger().error(f"✗ Failed to initialize CSV file: {e}")
            self.csv_file_path = None


    # ----------------------------------------------------------
    # Helper: Log measurement to CSV
    # ----------------------------------------------------------
    def _log_to_csv(self, cmd, x, y, z, vis_spectrum, nir_spectrum):
        """Append measurement data to CSV file."""
        if self.csv_file_path is None:
            return
            
        try:
            now = datetime.now()
            date_str = now.strftime("%Y-%m-%d")
            time_str = now.strftime("%H:%M:%S.%f")[:-3]  # milliseconds precision
            
            with open(self.csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Build row: cmd, date, time, x, y, z, vis_data..., nir_data...
                row = [cmd, date_str, time_str, x, y, z]
                row.extend(vis_spectrum)
                row.extend(nir_spectrum)
                
                writer.writerow(row)
            
            self.get_logger().info(f"✓ Data logged to CSV: {cmd} at ({x}, {y}, {z})")
            
        except Exception as e:
            self.get_logger().error(f"✗ Failed to log to CSV: {e}")


    # ----------------------------------------------------------
    # Helper: Update plots
    # ----------------------------------------------------------
    def _update_plots(self, vis_spectrum, nir_spectrum, cmd):
        """Update matplotlib plots with new spectrum data."""
        try:
            # Clear previous plots
            self.ax_vis.clear()
            self.ax_nir.clear()

            # Plot VIS with wavelength axis (325.3-792.6 nm)
            self.ax_vis.plot(self.vis_wavelengths, vis_spectrum, 'b-', linewidth=1.5, label=f'{cmd}')
            self.ax_vis.set_title(f'VIS Spectrum - {cmd}')
            self.ax_vis.set_xlabel('Wavelength (nm)')
            self.ax_vis.set_ylabel('Intensity')
            self.ax_vis.set_xlim(325.3, 792.6)
            self.ax_vis.grid(True, alpha=0.3)
            self.ax_vis.legend()

            # Plot NIR with wavelength axis (991-1707 nm)
            self.ax_nir.plot(self.nir_wavelengths, nir_spectrum, 'r-', linewidth=1.5, label=f'{cmd}')
            self.ax_nir.set_title(f'NIR Spectrum - {cmd}')
            self.ax_nir.set_xlabel('Wavelength (nm)')
            self.ax_nir.set_ylabel('Intensity')
            self.ax_nir.set_xlim(991, 1707)
            self.ax_nir.grid(True, alpha=0.3)
            self.ax_nir.legend()

            # Update display
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.get_logger().warning(f"Plot update failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HyperspectralNode()
    rclpy.spin(node)

    node.vis.close()
    node.nir.close()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
