import socket
import threading

# Server details
SERVER_IP = "100.104.250.8"  # Change to the actual server IP if needed
SERVER_PORT = 5000

def receive_messages(client_socket):
    """Continuously receive messages from the server and print them."""
    try:
        while True:
            message = client_socket.recv(1024).decode('utf-8')
            if not message:
                print("Disconnected from server.")
                break
            print(f"\n[SERVER]: {message}")  # Display server messages
    except Exception as e:
        print(f"Error receiving data: {e}")
    finally:
        client_socket.close()

def start_client():
    """Connects to the TCP server and handles sending/receiving messages."""
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print(f"Connected to server at {SERVER_IP}:{SERVER_PORT}")

        # Start thread to receive messages from the server
        receive_thread = threading.Thread(target=receive_messages, args=(client_socket,))
        receive_thread.daemon = True
        receive_thread.start()

        # Allow user to send commands
        while True:
            command = input("\nEnter command (e.g., ST050, PA100, PN000, PR050): ").strip()
            if command.lower() == "exit":
                print("Closing connection.")
                break
            client_socket.sendall(command.encode('utf-8'))
            print(f"Sent: {command}")

    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        client_socket.close()

if __name__ == "__main__":
    start_client()

