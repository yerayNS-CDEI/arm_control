##################################################
#### LENZ HYPERSPECTRAL SENSOR CLIENT (FINAL)
##################################################

import socket
import struct
import threading
import time


# ----------------------------------------------------------
# Command generation (based on real captured frames)
# ----------------------------------------------------------

def build_simple(cmd: str) -> bytes:
    payload = cmd.encode("ascii")
    length = len(payload)
    return bytes([0x25, 0x05, length]) + payload


def build_config(cmd: str, value) -> bytes:
    if cmd in ["MTI", "MTR"]:
        value_str = str(value).zfill(5)
    else:
        value_str = str(value)

    payload = (cmd + value_str).encode("ascii")
    length = len(payload)
    return bytes([0x25, 0x05, length]) + payload


# ----------------------------------------------------------
# Main Lenz client class
# ----------------------------------------------------------

class LenzClient:
    def __init__(self, ip, port=2002, timeout=30.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.sock = None

        self.running = False
        self.tx_lock = threading.Lock()

        # Temporizador absoluto para PNG
        self.next_ping_time = None


    # ------------------------------------------------------
    # Connection and deterministic keepalive
    # ------------------------------------------------------

    def connect(self, retries=3):
        last_error = None

        for attempt in range(retries):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.settimeout(self.timeout)

                print(f"[{self.ip}] Connecting... (attempt {attempt+1}/{retries})")
                self.sock.connect((self.ip, self.port))
                print(f"[{self.ip}] CONNECTED")

                self.running = True
                now = time.time()
                self.last_ping_time = now
                self.next_ping_time = now + 3.0

                threading.Thread(target=self._keepalive_loop, daemon=True).start()
                return

            except Exception as e:
                last_error = e
                print(f"[{self.ip}] Error on attempt {attempt+1}: {e}")

                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
                    self.sock = None

                if attempt < retries - 1:
                    time.sleep(2 * (attempt + 1))

        raise ConnectionError(f"[{self.ip}] Could not connect: {last_error}")


    def _keepalive_loop(self):
        """Deterministic keepalive: PNG cada 3.000s exactos."""
        while self.running:
            now = time.time()

            if now >= self.next_ping_time:
                with self.tx_lock:
                    try:
                        frame = build_simple("PNG")
                        self.sock.sendall(frame)
                        self.last_ping_time = now
                    except Exception:
                        break

                self.next_ping_time += 3.0

            # Pequeña espera para no saturar CPU
            time.sleep(0.01)


    # ------------------------------------------------------
    # Wait for safe sending zone (after last PNG)
    # ------------------------------------------------------

    def wait_after_ping(self, delay=0.5):
        """Wait for safe timing window after last PNG."""
        elapsed = time.time() - self.last_ping_time
        if elapsed < delay:
            time.sleep(delay - elapsed)


    # ------------------------------------------------------
    # Sending commands
    # ------------------------------------------------------

    def send_simple(self, cmd: str):
        frame = build_simple(cmd)
        print(f"[SEND][{self.ip}] CMD={cmd} | HEX: {frame.hex(' ')}")
        with self.tx_lock:
            try:
                self.sock.sendall(frame)
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                print(f"[{self.ip}] ERROR sending {cmd}: {e}")
                raise

    def send_config(self, cmd: str, value):
        frame = build_config(cmd, value)
        print(f"[SEND CONFIG][{self.ip}] {cmd}={value} | HEX: {frame.hex(' ')}")
        with self.tx_lock:
            self.sock.sendall(frame)


    # ------------------------------------------------------
    # Buffer clearing
    # ------------------------------------------------------

    def flush_buffer(self):
        """Clears socket receive buffer."""
        if not self.sock:
            return
        
        original_timeout = self.sock.gettimeout()
        try:
            self.sock.settimeout(0.05)
            discarded = 0
            
            while True:
                try:
                    chunk = self.sock.recv(8192)
                    if not chunk:
                        break
                    discarded += len(chunk)
                except (socket.timeout, BlockingIOError):
                    break
                    
            if discarded > 0:
                print(f"[{self.ip}] Buffer flushed: {discarded} bytes")
        finally:
            try:
                self.sock.settimeout(original_timeout)
            except:
                pass


    # ------------------------------------------------------
    # Frame reading
    # ------------------------------------------------------

    def recv_exact(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError(f"[{self.ip}] Connection lost")
            buf += chunk
        return buf


    def read_frame(self, expected_cmds=None):
        while True:
            header = self.recv_exact(2)
            if header != b"&$":
                raise ValueError(f"[{self.ip}] Unexpected header: {header}")

            cmd = self.recv_exact(3).decode("ascii")
            length = struct.unpack(">H", self.recv_exact(2))[0]
            raw = self.recv_exact(length)
            checksum = self.recv_exact(1)[0]
            status = self.recv_exact(1)[0]

            if length == 512:
                end = self.recv_exact(2)
                if end != b"DR":
                    raise ValueError("Incorrect frame terminator")

            is_spectrum = length == 512
            valid_cmd = expected_cmds is None or cmd in expected_cmds

            if is_spectrum and valid_cmd:
                spectrum = struct.unpack(">256H", raw)
                return {
                    "cmd": cmd,
                    "spectrum": spectrum,
                    "status": status,
                    "raw_data": raw
                }

            print(f"[{self.ip}] Ignoring CMD={cmd}, LENGTH={length}")


    # ------------------------------------------------------
    # Close
    # ------------------------------------------------------

    def close(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        print(f"[{self.ip}] CLOSED")
