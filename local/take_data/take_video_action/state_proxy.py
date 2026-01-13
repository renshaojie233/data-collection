import argparse
import socket
import threading
import time
from typing import List, Optional


class StateProxy:
    def __init__(
        self,
        remote_host: str,
        remote_port: int,
        listen_host: str,
        listen_port: int,
        reconnect_s: float = 1.0,
    ) -> None:
        self._remote_host = remote_host
        self._remote_port = remote_port
        self._listen_host = listen_host
        self._listen_port = listen_port
        self._reconnect_s = reconnect_s
        self._clients: List[socket.socket] = []
        self._clients_lock = threading.Lock()
        self._stop = threading.Event()
        self._last_line: Optional[bytes] = None

    def start(self) -> None:
        accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        accept_thread.start()
        self._relay_loop()

    def _accept_loop(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self._listen_host, self._listen_port))
        server.listen(5)
        while not self._stop.is_set():
            try:
                client, _addr = server.accept()
                client.settimeout(0.5)
                if self._last_line:
                    try:
                        client.sendall(self._last_line)
                    except Exception:
                        client.close()
                        continue
                with self._clients_lock:
                    self._clients.append(client)
            except Exception:
                time.sleep(0.1)

    def _broadcast(self, data: bytes) -> None:
        dead: List[socket.socket] = []
        with self._clients_lock:
            for client in self._clients:
                try:
                    client.sendall(data)
                except Exception:
                    dead.append(client)
            if dead:
                self._clients = [c for c in self._clients if c not in dead]
        for client in dead:
            try:
                client.close()
            except Exception:
                pass

    def _relay_loop(self) -> None:
        while not self._stop.is_set():
            try:
                sock = socket.create_connection(
                    (self._remote_host, self._remote_port), timeout=2.0
                )
                sock.settimeout(1.0)
            except Exception:
                time.sleep(self._reconnect_s)
                continue

            buffer = ""
            try:
                while not self._stop.is_set():
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    buffer += data.decode("utf-8", errors="ignore")
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        payload = (line + "\n").encode("utf-8")
                        self._last_line = payload
                        self._broadcast(payload)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass
                time.sleep(self._reconnect_s)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--remote-host", default="172.16.1.2")
    parser.add_argument("--remote-port", type=int, default=15124)
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=15125)
    parser.add_argument("--reconnect-s", type=float, default=1.0)
    args = parser.parse_args()

    proxy = StateProxy(
        remote_host=args.remote_host,
        remote_port=args.remote_port,
        listen_host=args.listen_host,
        listen_port=args.listen_port,
        reconnect_s=args.reconnect_s,
    )
    print(
        f"State proxy listening on {args.listen_host}:{args.listen_port} "
        f"-> {args.remote_host}:{args.remote_port}"
    )
    proxy.start()


if __name__ == "__main__":
    main()
