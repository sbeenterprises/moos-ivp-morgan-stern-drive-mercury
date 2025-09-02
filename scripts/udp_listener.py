import socket

def main():
    UDP_IP = "127.0.0.1"
    UDP_PORT = 10112

    # Create and bind a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for incoming UDP data on {UDP_IP}:{UDP_PORT}...")

    while True:
        # Receive up to 1024 bytes
        data, addr = sock.recvfrom(1024)
        print(f"Received message from {addr}: {data.decode(errors='replace')}")

if __name__ == "__main__":
    main()
