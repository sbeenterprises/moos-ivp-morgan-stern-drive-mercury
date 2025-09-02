#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TCP-to-CAN bridge with periodic (100 ms) retransmission of the
last steering, throttle and gear frames until a new command arrives.
"""
import socket
import subprocess
import threading
import time

# --------------------------- Configuration ---------------------------
SERVER_IP   = "0.0.0.0"
SERVER_PORT = 5000
BITRATE     = 250_000

# --------------------------- CAN helpers -----------------------------
comm_enabled = True                       # global TX enable flag


def run_command(cmd: str) -> None:
    """Run a shell command and ignore errors."""
    try:
        subprocess.run(cmd, shell=True, check=True)
        print("Executed:", cmd)
    except Exception as e:
        print(f"Error executing '{cmd}': {e}")


def setup_can_interface(iface: str, bitrate: int = BITRATE) -> None:
    run_command(f"sudo ip link set {iface} up type can bitrate {bitrate}")


def get_rx_packets(iface: str) -> int:
    try:
        with open(f"/sys/class/net/{iface}/statistics/rx_packets") as f:
            return int(f.read().strip())
    except FileNotFoundError:
        return 0


def choose_can_iface() -> str:
    interfaces = ["canb0", "canb1", "can0", "can1"]
    for iface in interfaces:
        setup_can_interface(iface)
    time.sleep(1)
    chosen = max(interfaces, key=get_rx_packets)
    print(f"-> Using CAN interface: {chosen}")
    return chosen


CAN_IFACE = choose_can_iface()


def cansend(frame: str) -> None:
    """Send a CAN frame unless TX is disabled."""
    if comm_enabled:
        run_command(f"cansend {CAN_IFACE} {frame}")


# ------------------------ Throttle lookup table ----------------------
# % → byte  (0 … 100 %)
THROTTLE_MAP = {
      0: "00",  # neutro absoluto
      1: "02",  2: "05",  3: "07",  4: "0A",  5: "0C",  6: "0F",
      7: "11",  8: "14",  9: "16", 10: "19", 11: "1B", 12: "1E",
     13: "20", 14: "23", 15: "25", 16: "28", 17: "2A", 18: "2D",
     19: "2F", 20: "32", 21: "34", 22: "37", 23: "39", 24: "3C",
     25: "3E", 26: "41", 27: "41", 28: "43", 29: "46", 30: "48",
     31: "4B", 32: "50", 33: "52", 34: "55", 35: "57", 36: "57",
     37: "5A", 38: "5C", 39: "5F", 40: "61", 41: "64", 42: "66",
     43: "69", 44: "6B", 45: "6E", 46: "70", 47: "73", 48: "75",
     49: "78", 50: "7A", 51: "7D", 52: "7F", 53: "82", 54: "84",
     55: "87", 56: "89", 57: "8C", 58: "8E", 59: "91", 60: "93",
     61: "93", 62: "96", 63: "9B", 64: "9D", 65: "9D", 66: "A0",
     67: "A5", 68: "A7", 69: "A7", 70: "AC", 71: "AC", 72: "AF",
     73: "B4", 74: "B6", 75: "B9", 76: "B9", 77: "BE", 78: "BE",
     79: "C3", 80: "C3", 81: "C5", 82: "C8", 83: "CA", 84: "CD",
     85: "CF", 86: "D2", 87: "D4", 88: "D7", 89: "D9", 90: "DC",
     91: "DE", 92: "E1", 93: "E3", 94: "E6", 95: "E8", 96: "EB",
     97: "EB", 98: "F0", 99: "F2", 100: "FA"
}


def hex_throttle(percent: int) -> str:
    """Convert 0-100 % → byte defined in THROTTLE_MAP."""
    p = max(0, min(percent, 100))
    return THROTTLE_MAP[p]


# ------------------------ Steering lookup table ----------------------
STEER_FRAMES = {
    #  Right 0‥50
      0: "18F30A1A#0000FFFFFFFFFFFF",
      1: "18F30A1A#0190FFFFFFFFFFFF",
      2: "18F30A1A#0258FFFFFFFFFFFF",
      3: "18F30A1A#0320FFFFFFFFFFFF",
      4: "18F30A1A#04B0FFFFFFFFFFFF",
      5: "18F30A1A#0578FFFFFFFFFFFF",
      6: "18F30A1A#0640FFFFFFFFFFFF",
      7: "18F30A1A#0708FFFFFFFFFFFF",
      8: "18F30A1A#0898FFFFFFFFFFFF",
      9: "18F30A1A#0960FFFFFFFFFFFF",
     10: "18F30A1A#0A28FFFFFFFFFFFF",
     11: "18F30A1A#0AF0FFFFFFFFFFFF",
     12: "18F30A1A#0BB8FFFFFFFFFFFF",
     13: "18F30A1A#0BB8FFFFFFFFFFFF",
     14: "18F30A1A#0C80FFFFFFFFFFFF",
     15: "18F30A1A#0D48FFFFFFFFFFFF",
     16: "18F30A1A#0E10FFFFFFFFFFFF",
     17: "18F30A1A#0ED8FFFFFFFFFFFF",
     18: "18F30A1A#0FA0FFFFFFFFFFFF",
     19: "18F30A1A#1068FFFFFFFFFFFF",
     20: "18F30A1A#1130FFFFFFFFFFFF",
     21: "18F30A1A#11F8FFFFFFFFFFFF",
     22: "18F30A1A#12C0FFFFFFFFFFFF",
     23: "18F30A1A#1388FFFFFFFFFFFF",
     24: "18F30A1A#1450FFFFFFFFFFFF",
     25: "18F30A1A#1518FFFFFFFFFFFF",
     26: "18F30A1A#15E0FFFFFFFFFFFF",
     27: "18F30A1A#16A8FFFFFFFFFFFF",
     28: "18F30A1A#1770FFFFFFFFFFFF",
     29: "18F30A1A#1838FFFFFFFFFFFF",
     30: "18F30A1A#1900FFFFFFFFFFFF",
     31: "18F30A1A#19C8FFFFFFFFFFFF",
     32: "18F30A1A#1A90FFFFFFFFFFFF",
     33: "18F30A1A#1B58FFFFFFFFFFFF",
     34: "18F30A1A#1C20FFFFFFFFFFFF",
     35: "18F30A1A#1CE8FFFFFFFFFFFF",
     36: "18F30A1A#1DB0FFFFFFFFFFFF",
     37: "18F30A1A#1E78FFFFFFFFFFFF",
     38: "18F30A1A#1F40FFFFFFFFFFFF",
     39: "18F30A1A#2008FFFFFFFFFFFF",
     40: "18F30A1A#20D0FFFFFFFFFFFF",
     41: "18F30A1A#2198FFFFFFFFFFFF",
     42: "18F30A1A#2260FFFFFFFFFFFF",
     43: "18F30A1A#2328FFFFFFFFFFFF",
     44: "18F30A1A#23F0FFFFFFFFFFFF",
     45: "18F30A1A#24B8FFFFFFFFFFFF",
     46: "18F30A1A#2580FFFFFFFFFFFF",
     47: "18F30A1A#2648FFFFFFFFFFFF",
     48: "18F30A1A#2710FFFFFFFFFFFF",
     49: "18F30A1A#2710FFFFFFFFFFFF",
     50: "18F30A1A#2810FFFFFFFFFFFF",
    #  Left −1‥−50
     -1: "18F30A1A#FF38FFFFFFFFFFFF",
     -2: "18F30A1A#FE70FFFFFFFFFFFF",
     -3: "18F30A1A#FDA8FFFFFFFFFFFF",
     -4: "18F30A1A#FCE0FFFFFFFFFFFF",
     -5: "18F30A1A#FC18FFFFFFFFFFFF",
     -6: "18F30A1A#FB50FFFFFFFFFFFF",
     -7: "18F30A1A#FA88FFFFFFFFFFFF",
     -8: "18F30A1A#F9C0FFFFFFFFFFFF",
     -9: "18F30A1A#F8F8FFFFFFFFFFFF",
    -10: "18F30A1A#F830FFFFFFFFFFFF",
    -11: "18F30A1A#F768FFFFFFFFFFFF",
    -12: "18F30A1A#F6A0FFFFFFFFFFFF",
    -13: "18F30A1A#F5D8FFFFFFFFFFFF",
    -14: "18F30A1A#F510FFFFFFFFFFFF",
    -15: "18F30A1A#F448FFFFFFFFFFFF",
    -16: "18F30A1A#F380FFFFFFFFFFFF",
    -17: "18F30A1A#F2B8FFFFFFFFFFFF",
    -18: "18F30A1A#F1F0FFFFFFFFFFFF",
    -19: "18F30A1A#F128FFFFFFFFFFFF",
    -20: "18F30A1A#F060FFFFFFFFFFFF",
    -21: "18F30A1A#EF98FFFFFFFFFFFF",
    -22: "18F30A1A#EED0FFFFFFFFFFFF",
    -23: "18F30A1A#EE08FFFFFFFFFFFF",
    -24: "18F30A1A#ED40FFFFFFFFFFFF",
    -25: "18F30A1A#EC78FFFFFFFFFFFF",
    -26: "18F30A1A#EBB0FFFFFFFFFFFF",
    -27: "18F30A1A#EAE8FFFFFFFFFFFF",
    -28: "18F30A1A#EA20FFFFFFFFFFFF",
    -29: "18F30A1A#E958FFFFFFFFFFFF",
    -30: "18F30A1A#E890FFFFFFFFFFFF",
    -31: "18F30A1A#E7C8FFFFFFFFFFFF",
    -32: "18F30A1A#E700FFFFFFFFFFFF",
    -33: "18F30A1A#E638FFFFFFFFFFFF",
    -34: "18F30A1A#E570FFFFFFFFFFFF",
    -35: "18F30A1A#E4A8FFFFFFFFFFFF",
    -36: "18F30A1A#E3E0FFFFFFFFFFFF",
    -37: "18F30A1A#E318FFFFFFFFFFFF",
    -38: "18F30A1A#E250FFFFFFFFFFFF",
    -39: "18F30A1A#E188FFFFFFFFFFFF",
    -40: "18F30A1A#E0C0FFFFFFFFFFFF",
    -41: "18F30A1A#DFF8FFFFFFFFFFFF",
    -42: "18F30A1A#DF30FFFFFFFFFFFF",
    -43: "18F30A1A#DE68FFFFFFFFFFFF",
    -44: "18F30A1A#DDA0FFFFFFFFFFFF",
    -45: "18F30A1A#DCD8FFFFFFFFFFFF",
    -46: "18F30A1A#DC10FFFFFFFFFFFF",
    -47: "18F30A1A#DB48FFFFFFFFFFFF",
    -48: "18F30A1A#DA80FFFFFFFFFFFF",
    -49: "18F30A1A#D9B8FFFFFFFFFFFF",
    -50: "18F30A1A#D9B8FFFFFFFFFFFF",
}

# --------------------------- Global state ----------------------------
command_lock   = threading.Lock()
gear           = "neutral"          # "neutral", "forward", "reverse"
throttle_val   = 0                  # 0-100 %
steer_val      = 0                  # −50 … 50
gear_start_t   = time.time()        # when forward/reverse engaged
stop_event     = threading.Event()

# ------------------------- Periodic sender ---------------------------
def can_sender_loop() -> None:
    """Send current steering + gear/throttle frames every 100 ms."""
    while not stop_event.is_set():
        time.sleep(0.1)
        with command_lock:
            if not comm_enabled:
                continue

            # 1) steering (always)
            cansend(STEER_FRAMES[steer_val])

            # 2) gear/throttle
            if gear == "neutral":
                cansend("18F003D0#FF00FFFFFFFFFFFF")

            elif gear == "forward":
                cansend("0CFF05D0#0100000000F403FF")
                if time.time() - gear_start_t >= 1.0:
                    cansend(f"18F003D0#FF{hex_throttle(throttle_val)}FFFFFFFFFFFF")

            elif gear == "reverse":
                cansend("0CFF05D0#01000000005050FF")
                if time.time() - gear_start_t >= 1.0:
                    cansend(f"18F003D0#FF{hex_throttle(throttle_val)}FFFFFFFFFFFF")


# ------------------------- Command parser ----------------------------
def handle_command(cmd: str) -> None:
    global gear, throttle_val, steer_val, gear_start_t, comm_enabled
    cmd = cmd.strip().upper()
    if not cmd:
        return
    print("RX:", cmd)

    # ----- STOP / CONTINUE -----
    if cmd == "STOP":
        with command_lock:
            comm_enabled = False
        print(">>> STOP: transmissions disabled")
        return

    if cmd == "CONTINUE":
        with command_lock:
            comm_enabled = True
        print(">>> CONTINUE: transmissions resumed")
        return

    # ----- Steering (ST±NNN) -----
    if cmd.startswith("ST"):
        try:
            v = int(cmd[2:])
        except ValueError:
            print("Invalid ST value")
            return
        if v not in STEER_FRAMES:
            print("ST out of range")
            return
        with command_lock:
            steer_val = v
            cansend(STEER_FRAMES[v])           # first shot
        return

    # ----- Forward throttle (PAxxx) -----
    if cmd.startswith("PA"):
        try:
            val = int(cmd[2:])
        except ValueError:
            val = 0
        with command_lock:
            throttle_val = val
            if val == 0:                       # back to neutral
                gear = "neutral"
            else:
                if gear != "forward":
                    gear_start_t = time.time()
                gear = "forward"
            cansend(f"18F003D0#FF{hex_throttle(val)}FFFFFFFFFFFF")
        return

    # ----- Reverse throttle (PRxxx) -----
    if cmd.startswith("PR"):
        try:
            val = int(cmd[2:])
        except ValueError:
            val = 0
        with command_lock:
            throttle_val = val
            if val == 0:
                gear = "neutral"
            else:
                if gear != "reverse":
                    gear_start_t = time.time()
                gear = "reverse"
            cansend(f"18F003D0#FF{hex_throttle(val)}FFFFFFFFFFFF")
        return

    # ----- Neutral (PN) -----
    if cmd == "PN":
        with command_lock:
            gear = "neutral"
            throttle_val = 0
        return

    print("Unknown command")


# --------------------------- TCP server ------------------------------
def client_thread(sock: socket.socket) -> None:
    try:
        sock.sendall(b"Connected. Enter commands.\n")
        while data := sock.recv(1024):
            handle_command(data.decode())
            sock.sendall(b"OK\n")
    except Exception as e:
        print("Client error:", e)
    finally:
        sock.close()
        print("Client disconnected")
        # When the client disconnects, we stop sending frames
        with command_lock:
            comm_enabled = False



def start_server() -> None:
    threading.Thread(target=can_sender_loop, daemon=True).start()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.bind((SERVER_IP, SERVER_PORT))
        srv.listen()
        print(f"TCP server on {SERVER_IP}:{SERVER_PORT}")
        try:
            while True:
                c, a = srv.accept()
                print("Connect", a)
                threading.Thread(target=client_thread,
                                 args=(c,), daemon=True).start()
        except KeyboardInterrupt:
            pass
        finally:
            stop_event.set()


# ---------------------------------------------------------------------
if __name__ == "__main__":
    start_server()
