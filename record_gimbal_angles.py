#!/usr/bin/env python3

import socket
import time
import binascii
import struct
import signal
import sys
import math

# --- Configuration ---
CAMERA_IP = "192.168.144.25"  # Replace with your camera's IP
COMMAND_PORT = 37260
REQUEST_RATE_HZ = 5  # Lowered rate for testing
OUTPUT_FILENAME = "gimbal_angles_debug.csv" # Changed filename
SOCKET_TIMEOUT = 1.0 # Increased timeout for testing

# --- SIYI Protocol Constants (adapted from siyi_message.py) ---
STX = "5566"
CMD_SET_GIMBAL_CONTROL_ANGLE = 0x07
CMD_ACQUIRE_GIMBAL_ATTITUDE = 0x0D
# Add other command IDs if needed

# --- Global flag for graceful shutdown ---
stop_requested = False

def signal_handler(sig, frame):
    """Handle Ctrl+C."""
    global stop_requested
    print("\nCtrl+C detected. Stopping listener...")
    stop_requested = True

def encode_msg(data_hex, cmd_id, seq=0):
    """Encodes a message according to SIYI protocol."""
    data_len = len(data_hex) // 2
    total_len = 7 + data_len
    len_byte = format(total_len & 0xFF, '02x')
    seq_byte = format(seq & 0xFF, '02x')
    cmd_byte = format(cmd_id & 0xFF, '02x')
    crc_input_hex = len_byte + seq_byte + cmd_byte + data_hex
    # --- CRC Calculation Placeholder ---
    # Using 0 for now. Add proper CRC-16-CCITT if needed.
    crc16_val = 0
    crc16_hex = format(crc16_val & 0xFFFF, '04x')
    # --- End CRC Placeholder ---
    message_hex = STX + len_byte + seq_byte + crc16_hex + cmd_byte + data_hex
    return binascii.unhexlify(message_hex)

def decode_msg(response_bytes):
    """Decodes a received SIYI message."""
    response_hex = binascii.hexlify(response_bytes).decode('ascii')
    print(f"DEBUG: Raw received hex: {response_hex}") # DEBUG PRINT

    if not response_hex.startswith(STX):
        print(f"DEBUG: Invalid start bytes: {response_hex[:4]}") # DEBUG PRINT
        return None, None, None, None

    try:
        len_byte = int(response_hex[4:6], 16)
        seq = int(response_hex[6:8], 16)
        cmd_id = int(response_hex[12:14], 16)
        data_hex = response_hex[14:]
        data_len = len(data_hex) // 2

        if len_byte != (7 + data_len):
            print(f"DEBUG: Length mismatch. Header: {len_byte}, Calculated: {7 + data_len}") # DEBUG PRINT
            return None, None, None, None

        print(f"DEBUG: Decoded CMD_ID: {cmd_id:#04x}, SEQ: {seq}, Data: {data_hex}") # DEBUG PRINT
        return data_hex, data_len, cmd_id, seq

    except (ValueError, IndexError) as e:
        print(f"DEBUG: Error decoding message: {e}, Hex: {response_hex}") # DEBUG PRINT
        return None, None, None, None

def gimbal_attitude_msg():
    """Creates the message to request gimbal attitude."""
    return encode_msg("", CMD_ACQUIRE_GIMBAL_ATTITUDE)

def parse_gimbal_attitude(data_hex):
    """Parses gimbal attitude data from hex string."""
    if len(data_hex) < 12:
        print(f"DEBUG: Attitude data too short: {data_hex}") # DEBUG PRINT
        return None

    try:
        yaw_raw = int(data_hex[0:4], 16)
        pitch_raw = int(data_hex[4:8], 16)
        roll_raw = int(data_hex[8:12], 16)

        if yaw_raw > 0x7FFF: yaw_raw -= 0x10000
        if pitch_raw > 0x7FFF: pitch_raw -= 0x10000
        if roll_raw > 0x7FFF: roll_raw -= 0x10000

        yaw = yaw_raw / 10.0
        pitch = pitch_raw / 10.0
        roll = roll_raw / 10.0

        print(f"DEBUG: Parsed angles: Y={yaw:.1f} P={pitch:.1f} R={roll:.1f}") # DEBUG PRINT
        return {'yaw': yaw, 'pitch': pitch, 'roll': roll}

    except ValueError as e:
        print(f"DEBUG: Error parsing attitude data '{data_hex}': {e}") # DEBUG PRINT
        return None

# --- Main Execution ---
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(SOCKET_TIMEOUT)
    server_address = (CAMERA_IP, COMMAND_PORT)
    print(f"Attempting to connect to {CAMERA_IP}:{COMMAND_PORT}")
    print(f"Saving data to {OUTPUT_FILENAME}")
    print("Press Ctrl+C to stop.")

    try:
        with open(OUTPUT_FILENAME, 'w', newline='') as f:
            f.write("timestamp,yaw_deg,pitch_deg,roll_deg\n") # Write header immediately

            while not stop_requested:
                request_msg = gimbal_attitude_msg()
                print(f"\nDEBUG: Sending request: {binascii.hexlify(request_msg).decode('ascii')}") # DEBUG PRINT
                try:
                    sock.sendto(request_msg, server_address)
                except socket.error as e:
                    print(f"ERROR: Socket error sending request: {e}")
                    time.sleep(1)
                    continue

                print("DEBUG: Waiting for response...") # DEBUG PRINT
                try:
                    response_bytes, address = sock.recvfrom(1024)
                    print(f"DEBUG: Received {len(response_bytes)} bytes from {address}") # DEBUG PRINT

                    data_hex, data_len, cmd_id, seq = decode_msg(response_bytes)

                    if cmd_id == CMD_ACQUIRE_GIMBAL_ATTITUDE:
                        print("DEBUG: Received Attitude Message") # DEBUG PRINT
                        angles = parse_gimbal_attitude(data_hex)
                        if angles:
                            timestamp = time.time()
                            yaw = angles['yaw']
                            pitch = angles['pitch']
                            roll = angles['roll']

                            print(f"SUCCESS: Writing to file: T={timestamp:.2f} Y={yaw:.1f} P={pitch:.1f} R={roll:.1f}") # DEBUG PRINT
                            f.write(f"{timestamp:.6f},{yaw:.2f},{pitch:.2f},{roll:.2f}\n")
                            f.flush() # Ensure data is written immediately
                        else:
                            print("DEBUG: Failed to parse valid angles from attitude message.") # DEBUG PRINT
                    elif cmd_id is not None:
                        print(f"DEBUG: Received message with CMD_ID {cmd_id:#04x}, but expected {CMD_ACQUIRE_GIMBAL_ATTITUDE:#04x}") # DEBUG PRINT
                    else:
                        print("DEBUG: Failed to decode received message.") # DEBUG PRINT

                except socket.timeout:
                    print("DEBUG: Socket timeout, no response received.") # DEBUG PRINT
                    pass
                except socket.error as e:
                    print(f"ERROR: Socket error receiving data: {e}")
                    time.sleep(1)
                except Exception as e:
                    print(f"ERROR: An unexpected error occurred in receive loop: {e}")

                # Wait before next request
                time.sleep(1.0 / REQUEST_RATE_HZ)

    except IOError as e:
        print(f"ERROR: Cannot open or write to file {OUTPUT_FILENAME}: {e}")
    finally:
        print("\nClosing socket.")
        sock.close()
        print("Exiting.")