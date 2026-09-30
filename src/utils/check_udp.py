import json
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 5005))

print("Listening on 127.0.0.1:5005 — Ctrl+C to stop.", flush=True)

try:
    while True:
        data, sender = sock.recvfrom(65535)
        try:
            packet = json.loads(data)
            print(json.dumps(packet, indent=2), flush=True)
        except (ValueError, UnicodeDecodeError):
            print(f"Invalid JSON from {sender}: {data!r}")
except KeyboardInterrupt:
    pass
finally:
    sock.close()