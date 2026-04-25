import socket
import threading
import time
import unittest

from racer_companion import msp


class _FakeMspServer:
    """Minimal TCP server that speaks just enough MSP to test our adapter."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 0))
        self.port = self.sock.getsockname()[1]
        self.sock.listen(1)
        self.received = bytearray()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.client = None

    def start(self):
        self.thread.start()

    def _run(self):
        self.client, _ = self.sock.accept()
        self.client.settimeout(0.5)
        try:
            while True:
                try:
                    chunk = self.client.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    return
                self.received.extend(chunk)
        except OSError:
            return

    def send_response(self, cmd: int, payload: bytes):
        size = len(payload)
        chk = size ^ cmd
        for b in payload:
            chk ^= b
        frame = b"$M>" + bytes([size, cmd]) + payload + bytes([chk & 0xFF])
        self.client.sendall(frame)

    def stop(self):
        try:
            if self.client is not None:
                self.client.close()
        except OSError:
            pass
        self.sock.close()


class TestTcpAdapter(unittest.TestCase):
    def test_connects_and_round_trips(self):
        server = _FakeMspServer()
        server.start()
        try:
            client = msp.MspClient(f"tcp://127.0.0.1:{server.port}")
            client.send(msp.MSP_API_VERSION)

            # Wait for server to receive
            for _ in range(20):
                if len(server.received) >= 6:
                    break
                time.sleep(0.05)

            self.assertEqual(server.received[:3], b"$M<")
            self.assertEqual(server.received[4], msp.MSP_API_VERSION)

            # Server replies with a fake API version response
            server.send_response(msp.MSP_API_VERSION, b"\x00\x01\x2d")  # protocol=0, api=1.45

            # Client should poll and get the frame back
            for _ in range(20):
                frames = client.poll()
                if frames:
                    break
                time.sleep(0.05)
            else:
                self.fail("No frame received from server")

            self.assertEqual(frames[0][0], msp.MSP_API_VERSION)
            self.assertEqual(frames[0][1], b"\x00\x01\x2d")
            client.close()
        finally:
            server.stop()

    def test_invalid_uri_raises(self):
        with self.assertRaises(Exception):
            # No port → rsplit fails or connection refuses
            msp.MspClient("tcp://127.0.0.1")


if __name__ == "__main__":
    unittest.main()
