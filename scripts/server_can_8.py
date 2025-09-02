import socket
import subprocess
import threading
import time

# --------------------------------------------------------------------
# Helper functions and CAN interface setup
# --------------------------------------------------------------------

SERVER_IP = '0.0.0.0'
SERVER_PORT = 5000

def run_command(cmd):
    """Runs shell command, prints it, and swallows errors as needed."""
    try:
        subprocess.run(cmd, shell=True, check=True)
        print(f"Executed: {cmd}")
    except Exception as e:
        print(f"Error executing '{cmd}': {e}")


def setup_can_interface(iface, bitrate=250000):
    """Bring up the CAN interface if it’s DOWN."""
    res = subprocess.run(f"ip link show {iface}", shell=True,
                         capture_output=True, text=True)
    if "state DOWN" in res.stdout:
        run_command(f"sudo ip link set {iface} up type can bitrate {bitrate}")


def get_rx_packets(iface):
    """Read the number of received CAN frames from sysfs."""
    path = f"/sys/class/net/{iface}/statistics/rx_packets"
    try:
        with open(path, "r") as f:
            return int(f.read().strip())
    except:
        return 0


def determine_principal_iface():
    """
    Ensure both CAN interfaces are up, wait briefly, then choose the one
    with the higher received-packet count as principal.
    """
    for iface in ("canb0", "canb1"):
        setup_can_interface(iface)

    # Allow a moment for packet counts to update
    time.sleep(1)

    rx0 = get_rx_packets("canb0")
    rx1 = get_rx_packets("canb1")
    chosen = "canb1" if rx1 > rx0 else "canb0"
    print(f"-> Using principal CAN interface: {chosen}")
    return chosen

# Determine principal CAN interface at startup
CAN_IFACE = determine_principal_iface()


def cansend(frame):
    """Send a CAN frame over the chosen interface."""
    run_command(f"cansend {CAN_IFACE} {frame}")


def scale_throttle_0_100_to_hex(value_0_100):
    """
    Scale integer [0..100] to a hex [00..FA].
    0 -> 0x00, 100 -> 0xFA
    """
    clamped = max(0, min(value_0_100, 100))
    scaled = int(round(250 * (clamped / 100.0)))  # 0xFA = 250 decimal
    return f"{scaled:02X}"


def scale_to_hex(value):
    """
    Scale the value from [50..100] to [0xFF..0xE2].
    50 => 0xFF, 100 => 0xD9
    D9 is the maximum for the left, if we go further than it go right
    """
    if not (50 <= value <= 100):
        raise ValueError("Value must be between 50 and 100")

    hex_min, hex_max = 0xD9, 0xFF
    scaled_value = hex_max - (value - 50) * (hex_max - hex_min) / (100 - 50)
    return int(round(scaled_value))


def scale_value(x):
    """
    Scale x from [0..50] to [0x64..0x00] (decimal 100..0).
    """
    x = max(0, min(x, 50))
    return int(100 * (1 - (x / 50.0)))


def send_neutral_once():
    """Immediately send neutral (PN) frame."""
    cansend("18F003D0#FF00FFFFFFFFFFFF")

# --------------------------------------------------------------------
# Global state
# --------------------------------------------------------------------
command_lock = threading.Lock()
active = False
gear = "neutral"
throttle_value = 0
steering_value = 50
gear_start_time = 0
stop_event = threading.Event()

# --------------------------------------------------------------------
# Background CAN-sender loop (100ms)
# --------------------------------------------------------------------
def can_sender_loop():
    global active, gear, throttle_value, gear_start_time

    while not stop_event.is_set():
        time.sleep(0.1)

        with command_lock:
            if not active:
                continue

            if gear == "forward":
                cansend("0CFF05D0#7EFFFFFFFFFFFFFF")
                if (time.time() - gear_start_time) >= 1.0:
                    th_hex = scale_throttle_0_100_to_hex(throttle_value)
                    cansend(f"18F003D0#FF{th_hex}FFFFFFFFFFFF")

            elif gear == "reverse":
                cansend("0CFF05D0#01000000005050FF")
                if (time.time() - gear_start_time) >= 1.0:
                    th_hex = scale_throttle_0_100_to_hex(throttle_value)
                    cansend(f"18F003D0#FF{th_hex}FFFFFFFFFFFF")

# --------------------------------------------------------------------
# Command parser
# --------------------------------------------------------------------
def handle_command(data):
    global active, gear, throttle_value, steering_value, gear_start_time

    data = data.strip().upper()
    if not data:
        return

    print(f"Received command: {data}")

    # --- new STOP/CONTINUE handling ---
    if data == "STOP":
        with command_lock:
            active = False
            # optional: send neutral immediately for safety
            send_neutral_once()
        print("STOP received: CAN communication halted.")
        return

    if data == "CONTINUE":
        with command_lock:
            # reset gear timer so we don't send throttle burst immediately
            gear_start_time = time.time()
            active = True
        print("CONTINUE received: CAN communication resumed.")
        return
    # ----------------------------------

    if data.startswith("ST"):
        try:
            value = int(data[-3:])
        except ValueError:
            value = 50
        with command_lock:
            steering_value = max(0, min(value, 100))
            if steering_value == 0:
                cansend("18F30A1A#0000FFFFFFFFFFFF")
            elif steering_value > 50:
                hex_val = f"{scale_to_hex(steering_value):02X}"
                cansend(f"18F30A1A#{hex_val}00FFFFFFFFFFFF")
            else:
                dec_val = scale_value(steering_value)
                cansend(f"18F30A1A#{dec_val:02X}00FFFFFFFFFFFF")

    elif data.startswith("PA"):
        try:
            val = int(data[-3:])
        except ValueError:
            val = 0
        if val == 0:
            with command_lock:
                send_neutral_once()
                active = False
                gear = "neutral"
                throttle_value = 0
                steering_value = 50
        else:
            with command_lock:
                gear = "forward"
                throttle_value = val
                if not active:
                    gear_start_time = time.time()
                active = True

    elif data.startswith("PN"):
        with command_lock:
            send_neutral_once()
            active = False
            gear = "neutral"
            throttle_value = 0
            steering_value = 50

    elif data.startswith("PR"):
        try:
            val = int(data[-3:])
        except ValueError:
            val = 0
        if val == 0:
            with command_lock:
                send_neutral_once()
                active = False
                gear = "neutral"
                throttle_value = 0
                steering_value = 50
        else:
            with command_lock:
                gear = "reverse"
                throttle_value = val
                if not active:
                    gear_start_time = time.time()
                active = True

    elif data.startswith("BU"):
        print("BU command is removed/ignored.")

    else:
        print(f"Invalid or unknown command: {data}")

# --------------------------------------------------------------------
# Client handler
# --------------------------------------------------------------------
def handle_client(client_socket):
    try:
        client_socket.sendall(b"Connection accepted. Send commands now.\n")
        print("Client connected.")

        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            cmd_str = data.decode('utf-8').strip()
            if not cmd_str:
                break
            handle_command(cmd_str)
            client_socket.sendall(b"Command received.\n")

    except Exception as e:
        print(f"Error with client: {e}")

    finally:
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

    sender_thread = threading.Thread(target=can_sender_loop, daemon=True)
    sender_thread.start()

    try:
        while True:
            client_socket, addr = server_socket.accept()
            print(f"Connection from {addr}")
            t = threading.Thread(target=handle_client, args=(client_socket,))
            t.daemon = True
            t.start()
    except KeyboardInterrupt:
        print("\nShutting down server.")
    finally:
        stop_event.set()
        server_socket.close()

if __name__ == "__main__":
    start_tcp_server()
