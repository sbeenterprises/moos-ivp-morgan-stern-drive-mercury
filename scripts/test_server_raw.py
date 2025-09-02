import socket
import subprocess
import threading

# Define server IP and port
SERVER_IP = '0.0.0.0'
SERVER_PORT = 5000

# Lock for thread-safe command execution
command_lock = threading.Lock()

def send_can_message(command):
    try:
        match(command[:2]):
            case "PA":
                data = f"4B3#7E7EFFFF{round(int(command[-3:]) * 250 / 100):02X}{round(int(command[-3:]) * 250 / 100):02X}FFFF"
            case "PN":
                data = f"4B3#7D7DFFFF{round(int(command[-3:]) * 250 / 100):02X}{round(int(command[-3:]) * 250 / 100):02X}FFFF"
            case "PR":
                data = f"4B3#7C7CFFFF{round(int(command[-3:]) * 250 / 100):02X}{round(int(command[-3:]) * 250 / 100):02X}FFFF"
            case "ST":
                data = f"18F30A1A#{round(int(command[-3:]) * 0x64 / 100):02X}{round(int(command[-3:]) * 0x64 / 100):02X}FFFFFFFFFFFF"
            case _:
                print(f"Invalid command: {command}")
                return

        command_str = f"cansend canb0 {data}"
        subprocess.run(command_str, shell=True, check=True)
        print(f"Executed: {command_str}")

    except Exception as e:
        print(f"Error sending CAN message: {e}")

def handle_client(client_socket):
    """Handle a client connection."""
    try:
        client_socket.sendall(b"Connection accepted. Send commands now.\n")
        print("Client connected.")

        while True:
            data = client_socket.recv(1024).decode('utf-8').strip()
            if not data:
                break

            print(f"Received: {data}")

            with command_lock:
                send_can_message(data)  # Execute cansend command once per received message

            client_socket.sendall(b"Command received.\n")
    
    except Exception as e:
        print(f"Error with client: {e}")
    finally:
        client_socket.close()
        print("Client disconnected.")

def start_tcp_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(5)
    print(f"TCP server listening on {SERVER_IP}:{SERVER_PORT}")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr}")

        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.daemon = True
        client_thread.start()

if __name__ == "__main__":
    start_tcp_server()
