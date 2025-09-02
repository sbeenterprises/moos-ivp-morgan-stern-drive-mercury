import socket
import subprocess
import threading
import time

SERVER_IP = '0.0.0.0'
SERVER_PORT = 5000

# --------------------------------------------------------------------
# Global state
# --------------------------------------------------------------------
command_lock = threading.Lock()

# Whether we're in "active" driving mode or not
active = False

# Gear can be "neutral" or "forward" (you could extend this if needed)
gear = "neutral"

# Throttle in [0..0xFA] (hex 00..FA)
throttle_value = 0

# Steering "cached" or last known value – only used if you still want to hold it
steering_value = 50  # A "neutral" 50

# Time when we switched to forward gear, so we can wait ~1s to send throttle
gear_forward_start_time = 0

# Stop event for shutting down the server or the 100ms loop
stop_event = threading.Event()


# --------------------------------------------------------------------
# Helper functions
# --------------------------------------------------------------------
def run_command(cmd):
    """Runs shell command, prints it, and swallows errors as needed."""
    try:
        subprocess.run(cmd, shell=True, check=True)
        print(f"Executed: {cmd}")
    except Exception as e:
        print(f"Error executing '{cmd}': {e}")

def scale_throttle_0_100_to_hex(value_0_100):
    """
    Scale integer [0..100] to a hex [00..FA].
    0 -> 0x00
    100 -> 0xFA
    """
    clamped = max(0, min(value_0_100, 100))
    scaled = int(round(250 * (clamped / 100.0)))  # 0xFA = 250 decimal
    return f"{scaled:02X}"  # Return string like '00', 'FA', etc.

def send_neutral_once():
    """
    Immediately send the PN command to drop speed to zero:
      18F003D0#FF00FFFFFFFFFFFF
    """
    run_command("cansend can1 18F003D0#FF00FFFFFFFFFFFF")

# --------------------------
# Steering-scale helpers
# --------------------------
def scale_to_hex(value):
    """Scale the value from [50..100] to [0xFF..0xE2]."""
    if not (50 <= value <= 100):
        raise ValueError("Value must be between 50 and 100")

    hex_min, hex_max = 0xE2, 0xFF  # per your example
    scaled_value = hex_max - (value - 50) * (hex_max - hex_min) / (100 - 50)
    return hex(int(round(scaled_value)))

def scale_value(x):
    """
    Scales x from range [0..50] to [0x64..0x00].
    Returns an integer which is the decimal equivalent of [0x64..0x00].
    """
    x = max(0, min(x, 50))  # Clamp x within [0, 50]
    scaled_value = int(100 * (1 - (x / 50.0)))  # yields [100..0], i.e. [0x64..0x00]
    return scaled_value


# --------------------------------------------------------------------
# 100ms background loop
# --------------------------------------------------------------------
def can_sender_loop():
    """
    Runs in the background every 100ms, sending the appropriate CAN frames
    based on current global state (e.g., gear forward, throttle).
    
    Note: We REMOVED steering from here so ST commands are sent only once
    within handle_command().
    """
    global active, gear, throttle_value, gear_forward_start_time

    while not stop_event.is_set():
        time.sleep(0.1)  # 100ms loop

        with command_lock:
            if not active:
                # If not active, do nothing
                continue

            # If gear is forward, send gear forward command every 100ms
            if gear == "forward":
                run_command("cansend can1 0CFF05D0#7EFFFFFFFFFFFFFF")

                # Check if 1 second has elapsed since gear_forward_start_time
                if (time.time() - gear_forward_start_time) >= 1.0:
                    # Now send the throttle
                    # 18F003D0#FF<THROTTLE>FFFFFFFFFFFF
                    throttle_hex = scale_throttle_0_100_to_hex(throttle_value)
                    cmd = f"cansend can1 18F003D0#FF{throttle_hex}FFFFFFFFFFFF"
                    run_command(cmd)


# --------------------------------------------------------------------
# Handling incoming commands
# --------------------------------------------------------------------
def handle_command(data):
    """
    Process one line of input from the user (like 'PA050', 'PN', 'ST060', etc.)
    Update global state accordingly, and/or send immediate CAN frames.
    """
    global active, gear, throttle_value, steering_value, gear_forward_start_time

    data = data.strip().upper()
    if not data:
        return

    print(f"Received command: {data}")

    # -----------------------------------------------------
    # Steering command: ST000..ST100
    # -----------------------------------------------------
    if data.startswith("ST"):
        # Example: "ST050" => value = 50
        try:
            value = int(data[-3:])  # Extract value from last 3 chars
        except ValueError:
            value = 50  # fallback

        with command_lock:
            steering_value = max(0, min(value, 100))  # clamp

            if steering_value == 0:
                # "full left"
                cmd = "cansend can1 18F30A1A#0000FFFFFFFFFFFF"
            elif steering_value > 50:
                # Right side => scale using scale_to_hex
                scaled_hex = f"{scale_to_hex(steering_value)[2:].upper():0>2}"
                cmd = f"cansend can1 18F30A1A#{scaled_hex}00FFFFFFFFFFFF"
            else:
                # Left side => scale using scale_value
                scaled_int = scale_value(steering_value)  # returns decimal 0..100
                scaled_hex = f"{hex(scaled_int)[2:].upper():0>2}"
                cmd = f"cansend can1 18F30A1A#{scaled_hex}00FFFFFFFFFFFF"

            run_command(cmd)

    # -----------------------------------------------------
    # PA command
    # -----------------------------------------------------
    elif data.startswith("PA"):
        # "PAxxx" => forward gear with throttle
        try:
            val = int(data[-3:])
        except ValueError:
            val = 0

        if val == 0:
            # treat it like PN
            with command_lock:
                send_neutral_once()
                active = False
                gear = "neutral"
                throttle_value = 0
                steering_value = 50
        else:
            # value > 0 => forward gear
            with command_lock:
                gear = "forward"
                throttle_value = val
                # Mark active = True, set time to now for the 1-second wait
                if not active:
                    gear_forward_start_time = time.time()
                active = True

    # -----------------------------------------------------
    # PN command
    # -----------------------------------------------------
    elif data.startswith("PN"):
        # Immediately do neutral, stop sending
        with command_lock:
            send_neutral_once()
            active = False
            gear = "neutral"
            throttle_value = 0
            steering_value = 50

    # -----------------------------------------------------
    # Remove BU command => ignore it
    # -----------------------------------------------------
    elif data.startswith("BU"):
        print("BU command is removed/ignored.")
        return

    # -----------------------------------------------------
    # Any other command => unknown
    # -----------------------------------------------------
    else:
        print(f"Invalid or unknown command: {data}")


# --------------------------------------------------------------------
# Client handler
# --------------------------------------------------------------------
def handle_client(client_socket):
    global active, gear, throttle_value, steering_value

    try:
        client_socket.sendall(b"Connection accepted. Send commands now.\n")
        print("Client connected.")

        while True:
            data = client_socket.recv(1024)
            if not data:
                # Connection closed by client
                break

            command_str = data.decode('utf-8').strip()
            if not command_str:
                break

            # Handle the command
            handle_command(command_str)

            client_socket.sendall(b"Command received.\n")

    except Exception as e:
        print(f"Error with client: {e}")

    finally:
        # If connection drops, do PN => speed=0, neutral, steering=50, stop sending
        print("Client disconnected. Setting to neutral (PN).")
        with command_lock:
            send_neutral_once()
            active = False
            gear = "neutral"
            throttle_value = 0
            steering_value = 50

        client_socket.close()


# --------------------------------------------------------------------
# Main server
# --------------------------------------------------------------------
def start_tcp_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(5)
    print(f"TCP server listening on {SERVER_IP}:{SERVER_PORT}")

    # Start the background CAN-sender thread (for throttle & gear)
    sender_thread = threading.Thread(target=can_sender_loop, daemon=True)
    sender_thread.start()

    try:
        while True:
            client_socket, addr = server_socket.accept()
            print(f"Connection from {addr}")
            client_thread = threading.Thread(target=handle_client, args=(client_socket,))
            client_thread.daemon = True
            client_thread.start()
    except KeyboardInterrupt:
        print("\nShutting down server.")
    finally:
        stop_event.set()
        server_socket.close()


if __name__ == "__main__":
    start_tcp_server()
