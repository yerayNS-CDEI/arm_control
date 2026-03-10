import socket
import struct
import threading
import time

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

class LenzClient:
    # AQUI ESTÀ EL CANVI: timeout=300.0 (5 minuts)
    def __init__(self, ip, port=2002, timeout=300.0):
        self.ip = str(ip)
        self.port = int(port)
        self.timeout = timeout
        self.sock = None
        self.running = False
        self.tx_lock = threading.Lock()
        self.last_ping_time = time.time()
        # Ping més ràpid (cada 1s) per mantenir el sensor ben despert
        self.next_ping_time = time.time() + 1.0 

    def connect(self, retries=3):
        last_error = None
        for attempt in range(retries):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                self.sock.settimeout(self.timeout)
                self.sock.connect((self.ip, self.port))
                self.running = True
                threading.Thread(target=self._keepalive_loop, daemon=True).start()
                return
            except Exception as e:
                last_error = e
                if self.sock: self.sock.close()
                if attempt < retries - 1: time.sleep(2)
                
        raise ConnectionError(f"Error connectant: {last_error}")

    def _keepalive_loop(self):
        while self.running:
            now = time.time()
            if now >= self.next_ping_time:
                with self.tx_lock:
                    try:
                        self.sock.sendall(build_simple("PNG"))
                        self.last_ping_time = now
                    except: 
                        self.running = False
                        break
                self.next_ping_time = now + 1.0 # Ping cada segon
            time.sleep(0.1)

    def wait_after_ping(self, delay=0.4):
        elapsed = time.time() - self.last_ping_time
        if elapsed < delay: time.sleep(delay - elapsed)

    def send_simple(self, cmd: str):
        with self.tx_lock:
            try: self.sock.sendall(build_simple(cmd))
            except Exception as e: raise BrokenPipeError(f"Error enviant: {e}")

    def send_config(self, cmd: str, value):
        frame = build_config(cmd, value)
        with self.tx_lock:
            try: self.sock.sendall(frame)
            except Exception as e: raise BrokenPipeError(f"Error config: {e}")

    def flush_buffer(self):
        if not self.sock: return
        orig = self.sock.gettimeout()
        self.sock.settimeout(0.01)
        try:
            while True:
                if not self.sock.recv(8192): break
        except: pass
        finally: self.sock.settimeout(orig)

    def recv_exact(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk: raise ConnectionError("Connexió perduda")
            buf += chunk
        return buf

    def read_frame(self, expected_cmds=None, max_header_searches=500):
        try:
            searches = 0
            while searches < max_header_searches:
                header = self.recv_exact(2)
                if header != b"&$": 
                    searches+=1
                    continue
                
                cmd = self.recv_exact(3).decode("ascii")
                length = struct.unpack(">H", self.recv_exact(2))[0]
                raw = self.recv_exact(length)
                self.recv_exact(2) # Checksum + Status

                if length == 512:
                    self.recv_exact(2) # Terminator
                    return {"spectrum": struct.unpack(">256H", raw), "cmd": cmd}
                return {"cmd": cmd}
            raise TimeoutError("Header not found")
        except Exception as e: raise BrokenPipeError(f"Error llegint: {e}")

    def close(self):
        self.running = False
        if self.sock:
            try: self.sock.close()
            except: pass
