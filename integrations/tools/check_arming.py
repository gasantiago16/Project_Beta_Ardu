"""Connect to BF MSP CLI and dump status every second.

Used to debug arming-disable flags while another process (e.g.
hover_test) is sending RC overrides. Doesn't interfere with the
companion's MSP traffic — opens its own TCP connection.
"""
import re
import socket
import sys
import time


def main() -> int:
    s = socket.create_connection(("127.0.0.1", 5761), timeout=2)
    s.settimeout(1.5)
    s.sendall(b"\n#\n")
    time.sleep(0.3)
    try:
        s.recv(4096)  # discard CLI banner
    except socket.timeout:
        pass

    for i in range(15):
        s.sendall(b"status\n")
        time.sleep(0.5)
        data = b""
        try:
            while True:
                c = s.recv(4096)
                if not c:
                    break
                data += c
        except socket.timeout:
            pass
        text = data.decode(errors="replace")
        flags = re.search(r"Arming disable flags:\s*(.*)", text)
        thr = re.search(r"throttle\s*=\s*(\d+)", text)
        print(
            f"t={i}s flags={flags.group(1).strip() if flags else '?'}",
            flush=True,
        )
        time.sleep(0.5)
    s.sendall(b"exit\n")  # leave CLI cleanly
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
