import socket
import struct
import threading
import time

def build_simple(cmd: str) -> bytes:
    payload = cmd.encode("ascii")
    # Protocol TX: preamble[2] + length[1 byte] + payload
    return bytes([0x25, 0x05, len(payload)]) + payload

def build_config(cmd: str, value) -> bytes:
    if cmd in ["MTI", "MTR"]:
        value_str = str(value).zfill(5)
    else:
        value_str = str(value)
    payload = (cmd + value_str).encode("ascii")
    # Protocol TX: preamble[2] + length[1 byte] + payload
    return bytes([0x25, 0x05, len(payload)]) + payload

def _fmt(data: bytes, max_bytes=32) -> str:
    """Mostra bytes en hex i ASCII imprimible per debugging."""
    hex_part = " ".join(f"{b:02X}" for b in data[:max_bytes])
    asc_part = "".join(chr(b) if 32 <= b < 127 else "." for b in data[:max_bytes])
    suffix = f"  ...+{len(data)-max_bytes}B" if len(data) > max_bytes else ""
    return f"[{hex_part}]  '{asc_part}'{suffix}"

class LenzClient:
    def __init__(self, ip, port=2002, timeout=15.0, debug=True):
        self.ip = str(ip)
        self.port = int(port)
        self.timeout = timeout
        self.debug = debug
        self.sock = None
        self.running = False
        self.tx_lock = threading.Lock()
        self.last_ping_time = time.time()
        self.next_ping_time = time.time() + 1.0
        self.ping_paused = False

    def _dbg(self, msg: str):
        if self.debug:
            ts = time.strftime("%H:%M:%S", time.localtime()) + f".{int(time.time()*1000)%1000:03d}"
            print(f"  [DBG {self.ip} {ts}] {msg}", flush=True)

    def connect(self, retries=3):
        last_error = None
        for attempt in range(retries):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                self.sock.settimeout(self.timeout)
                self._dbg(f"Connectant a {self.ip}:{self.port} (intent {attempt+1}/{retries})...")
                self.sock.connect((self.ip, self.port))
                self.running = True
                self._dbg(f"Connexió TCP establerta.")
                threading.Thread(target=self._keepalive_loop, daemon=True).start()
                return
            except Exception as e:
                last_error = e
                self._dbg(f"Error connexió: {e}")
                if self.sock: self.sock.close()
                if attempt < retries - 1: time.sleep(2)

        raise ConnectionError(f"Error connectant: {last_error}")

    def _keepalive_loop(self):
        while self.running:
            now = time.time()
            if now >= self.next_ping_time:
                with self.tx_lock:
                    now = time.time()
                    if not self.ping_paused and now >= self.next_ping_time:
                        try:
                            frame = build_simple("PNG")
                            # self._dbg(f"TX PNG  {_fmt(frame)}")
                            self.sock.sendall(frame)
                            self.last_ping_time = now
                        except Exception as e:
                            self._dbg(f"ERROR TX PNG: {e}")
                            self.running = False
                            break
                    elif self.ping_paused:
                        self._dbg("PNG suprimit (keepalive pausat)")
                    self.next_ping_time = now + 1.0
            time.sleep(0.1)
    
    def pause_keepalive(self):
        with self.tx_lock:
            self.ping_paused = True
        self._dbg("keepalive PAUSAT")

    def resume_keepalive(self):
        with self.tx_lock:
            self.ping_paused = False
            self.next_ping_time = time.time() + 1.0
        self._dbg("keepalive REPRÈS (proper PNG en ~1s)")

    def wait_after_ping(self, delay=0.4):
        elapsed = time.time() - self.last_ping_time
        remaining = delay - elapsed
        if remaining > 0:
            self._dbg(f"wait_after_ping: esperant {remaining:.3f}s (últim PNG fa {elapsed:.3f}s)")
            time.sleep(remaining)
        else:
            self._dbg(f"wait_after_ping: no cal esperar (últim PNG fa {elapsed:.3f}s)")

    def send_simple(self, cmd: str):
        frame = build_simple(cmd)
        self._dbg(f"TX {cmd:3s}  {_fmt(frame)}")
        with self.tx_lock:
            try:
                self.sock.sendall(frame)
                self._dbg(f"TX {cmd:3s} OK")
            except Exception as e:
                self._dbg(f"TX {cmd:3s} ERROR: {e}")
                raise BrokenPipeError(f"Error enviant: {e}")

    def send_config(self, cmd: str, value):
        frame = build_config(cmd, value)
        self._dbg(f"TX CFG {cmd}={value}  {_fmt(frame)}")
        with self.tx_lock:
            try:
                self.sock.sendall(frame)
                self._dbg(f"TX CFG {cmd}={value} OK")
            except Exception as e:
                self._dbg(f"TX CFG {cmd}={value} ERROR: {e}")
                raise BrokenPipeError(f"Error config: {e}")

    def flush_buffer(self):
        """Drena la cua TCP fins que no hi ha més dades en 10ms.
        Llança ConnectionError si el sensor ha tancat la connexió (recv retorna b'')."""
        if not self.sock: return
        orig = self.sock.gettimeout()
        self.sock.settimeout(0.01)
        total = 0
        try:
            while True:
                data = self.sock.recv(8192)
                if not data:
                    self._dbg(f"FLUSH: b'' rebut → sensor ha tancat la connexió (llegits {total}B fins ara)")
                    self.running = False
                    raise ConnectionError("Sensor ha tancat la connexió durant flush")
                total += len(data)
                self._dbg(f"FLUSH RX {len(data)}B: {_fmt(data)}")
        except ConnectionError:
            raise
        except Exception as e:
            # socket.timeout (cas normal: no hi ha més dades) o altre error de xarxa
            if total == 0:
                self._dbg(f"FLUSH: buit (timeout 10ms, cap dada pendent)")
            else:
                self._dbg(f"FLUSH: fi ({total}B llegits, timeout)")
        finally:
            self.sock.settimeout(orig)

    def recv_exact(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk: raise ConnectionError("Connexió perduda")
            buf += chunk
        return buf

    def read_frame(self, expected_cmds=None, max_header_searches=500):
        self._dbg("read_frame: START — esperant header &$...")
        try:
            searches = 0
            while searches < max_header_searches:
                self._dbg(f"read_frame: [cerca #{searches}] recv_exact(2) header...")
                header = self.recv_exact(2)
                self._dbg(f"read_frame: header rebut: {_fmt(header)}")
                if header != b"&$":
                    self._dbg(f"read_frame: header inesperat (descartant, cerca #{searches})")
                    searches += 1
                    continue

                self._dbg("read_frame: header OK (&$) — recv_exact(3) cmd...")
                cmd = self.recv_exact(3).decode("ascii", errors="replace")
                self._dbg(f"read_frame: cmd={cmd!r}")

                self._dbg("read_frame: recv_exact(2) length...")
                length_raw = self.recv_exact(2)
                length = struct.unpack(">H", length_raw)[0]
                self._dbg(f"read_frame: length={length}")

                self._dbg(f"read_frame: recv_exact({length}) raw...")
                raw = self.recv_exact(length)
                self._dbg(f"read_frame: raw OK ({length}B)")

                self._dbg("read_frame: recv_exact(2) tail...")
                tail = self.recv_exact(2)
                self._dbg(f"read_frame: tail={_fmt(tail)}")

                if length == 512:
                    # Alguns firmwares no envien terminator: capturem el cas graciosament.
                    self._dbg("read_frame: recv_exact(2) terminator (opcional)...")
                    try:
                        term = self.recv_exact(2)
                        self._dbg(f"read_frame: ESPECTRE OK cmd={cmd} tail={_fmt(tail)} term={_fmt(term)}")
                    except ConnectionError:
                        self._dbg(f"read_frame: ESPECTRE OK cmd={cmd} tail={_fmt(tail)} (sense terminator — sensor ha tancat)")
                    return {"spectrum": struct.unpack(">256H", raw), "cmd": cmd}

                # Trama no-espectre: ACK, OK, error... loguem el contingut complet
                self._dbg(
                    f"read_frame: trama no-espectre cmd={cmd!r} length={length} "
                    f"payload={_fmt(raw)} tail={_fmt(tail)} → descartant (cerca #{searches})"
                )
                searches += 1
            raise TimeoutError("Header not found after max_header_searches")
        except Exception as e:
            raise BrokenPipeError(f"Error llegint: {e}")

    def close(self):
        self.running = False
        if self.sock:
            try: self.sock.close()
            except: pass
