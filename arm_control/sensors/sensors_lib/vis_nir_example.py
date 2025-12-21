##################################################
#### VIS + NIR ACQUISITION
##################################################

from .lenz_client import LenzClient
import threading
import time
import matplotlib.pyplot as plt
import numpy as np


def acquire(client, label, result_dict, expected_cmds):
    """Thread to wait for a specific frame from sensor."""
    try:
        result_dict[label] = client.read_frame(expected_cmds)
    except Exception as e:
        result_dict[label] = f"ERROR: {e}"


def wait_for_user(msg):
    input(f"\n>>> {msg} - Press ENTER to continue...")


# ------------------------------
# Plotting
# ------------------------------

def plot_both(vis_spectrum, nir_spectrum, cmd, fig=None, axes=None):
    wavelength_vis = np.linspace(400, 1000, 256)
    wavelength_nir = np.linspace(900, 1700, 256)

    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        axes = (ax1, ax2)
        fig.canvas.manager.set_window_title("VIS + NIR Spectra")

    ax1, ax2 = axes
    ax1.clear(); ax2.clear()

    ax1.plot(wavelength_vis, vis_spectrum)
    ax2.plot(wavelength_nir, nir_spectrum)

    ax1.set_title(f"VIS Spectrum - {cmd}")
    ax2.set_title(f"NIR Spectrum - {cmd}")

    fig.tight_layout()
    plt.pause(0.01)
    return fig, axes


############################################################
# 1. STARTUP
############################################################

print("\n============================================================")
print("STARTING VIS + NIR ACQUISITION")
print("============================================================")

wait_for_user("Step 1: Connect to VIS + NIR sensors")

vis = LenzClient("193.167.0.30")
nir = LenzClient("193.167.0.31")

vis.connect()
nir.connect()

print("\nOK connected. Stabilizing communication...\n")
time.sleep(2)


############################################################
# 2. SENSOR CONFIGURATION
############################################################

def configure_sensors(vis_client, nir_client):
    """Configure both sensors with standard parameters."""
    vis_client.send_config("MTR", 60000); time.sleep(0.2)
    vis_client.send_config("MTI", 60000); time.sleep(0.2)
    vis_client.send_config("THP", 1);     time.sleep(0.2)
    
    nir_client.send_config("MTR", 20000); time.sleep(0.2)
    nir_client.send_config("MTI", 20000); time.sleep(0.2)
    nir_client.send_config("THP", 1);     time.sleep(0.2)

print("CONFIGURING SENSORS...\n")
configure_sensors(vis, nir)
print("OK configuration complete.")
time.sleep(3)


############################################################
# Helper function for safe measurement commands
############################################################

def perform_measurement(cmd, label, plot_fig, plot_axes, retries=3):
    print(f"\n>>> Starting {cmd} ({label})")

    for attempt in range(retries):
        try:
            # Clear old bytes once (don't pause keepalive - PNG must keep running!)
            vis.flush_buffer()
            nir.flush_buffer()
            time.sleep(0.1)  # Brief settle time

            # Wait for safe timing window after PNG (shorter to avoid timeout)
            vis.wait_after_ping(0.5)  # Reduced wait
            nir.wait_after_ping(0.5)

            # Send critical command (NO keepalive pause - let PNG keep connection alive)
            print(f"[Attempt {attempt+1}/{retries}] Sending {cmd}...")
            vis.send_simple(cmd)
            nir.send_simple(cmd)
            break  # Success, exit retry loop
            
        except (BrokenPipeError, ConnectionError, OSError) as e:
            print(f"[Attempt {attempt+1}/{retries}] FAILED: {e}")
            
            if attempt < retries - 1:
                print(f"Reconnecting and retrying...")
                try:
                    vis.close()
                    nir.close()
                except:
                    pass
                time.sleep(2)
                
                # Reconnect
                vis.__init__("193.167.0.30")
                nir.__init__("193.167.0.31")
                vis.connect()
                nir.connect()
                time.sleep(3)
                
                # Reconfigure
                configure_sensors(vis, nir)
                time.sleep(2)
            else:
                raise

    # Integration time — allow full sensor processing
    time.sleep(1.2)  # Increased time for reliable measurement

    # READ SENSOR OUTPUT
    results = {}
    tv = threading.Thread(target=acquire, args=(vis, "VIS", results, [cmd]))
    tn = threading.Thread(target=acquire, args=(nir, "NIR", results, [cmd]))

    tv.start(); tn.start()
    tv.join(); tn.join()

    vis_ok = isinstance(results["VIS"], dict)
    nir_ok = isinstance(results["NIR"], dict)

    if not vis_ok or not nir_ok:
        print(f"ERROR during {cmd}: {results}")
        return plot_fig, plot_axes

    # Clean up any remaining bytes
    time.sleep(0.1)
    vis.flush_buffer()
    nir.flush_buffer()

    print(f"OK {cmd} complete.")

    plot_fig, plot_axes = plot_both(
        results["VIS"]["spectrum"],
        results["NIR"]["spectrum"],
        cmd,
        plot_fig, plot_axes)

    return plot_fig, plot_axes


############################################################
# 3. MANDATORY SEQUENCE: GDS → GDR → GRF
############################################################

sequence = [
    ("GDS", "Dark Sample"),
    ("GDR", "Dark Reference"),
    ("GRF", "White Reference")
]

plot_fig = None
plot_axes = None

for cmd, label in sequence:
    wait_for_user(f"Send {cmd} ({label})")
    plot_fig, plot_axes = perform_measurement(cmd, label, plot_fig, plot_axes)
    
    # Brief recovery time between commands (don't wait too long or PNG times out)
    print("Waiting for sensor recovery...")
    time.sleep(1.5)  # Short wait to keep PNG active


############################################################
# 4. CONTINUOUS GSM LOOP
############################################################

print("\n============================================================")
print("CONTINUOUS GSM MODE")
print("============================================================")

count = 0
while True:
    user = input("Press ENTER for GSM | q to quit: ").strip().lower()
    if user in ("q", "quit", "exit"):
        break

    count += 1
    cmd = "GSM"
    label = f"GSM #{count}"

    plot_fig, plot_axes = perform_measurement(cmd, label, plot_fig, plot_axes)


############################################################
# 5. SHUTDOWN
############################################################

vis.close()
nir.close()

print("\nAll operations completed successfully.")
