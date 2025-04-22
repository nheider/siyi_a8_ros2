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
REQUEST_RATE_HZ = 10  # How often to request attitude data (e.g., 10Hz)
OUTPUT_FILENAME = "gimbal_angles.csv"
SOCKET_TIMEOUT = 0.5 # Seconds to wait for a response

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
    # Calculate data length (number of hex characters / 2)
    data_len = len(data_hex) // 2
    
    # Header: STX (2 bytes), LEN (1 byte), SEQ (1 byte), CRC16 (2 bytes), CMD_ID (1 byte)
    # Total header length = 7 bytes
    # Total packet length = Header length + Data length
    total_len = 7 + data_len
    
    # Prepare header components
    len_byte = format(total_len & 0xFF, '02x')
    seq_byte = format(seq & 0xFF, '02x')
    cmd_byte = format(cmd_id & 0xFF, '02x')
    
    # Prepare packet for CRC calculation (LEN, SEQ, CMD_ID, DATA)
    crc_input_hex = len_byte + seq_byte + cmd_byte + data_hex
    crc_input_bytes = binascii.unhexlify(crc_input_hex)
    
    # Calculate CRC16 (implementation omitted for brevity - assuming 0 for now)
    # A proper CRC16 implementation (e.g., CRC-16-CCITT) should be used
    # if the camera validates it. For requesting data, it might not be necessary.
    crc16_val = 0 # Replace with actual CRC calculation if needed
    crc16_hex = format(crc16_val & 0xFFFF, '04x')
    
    # Construct final message
    message_hex = STX + len_byte + seq_byte + crc16_hex + cmd_byte + data_hex
    return binascii.unhexlify(message_hex)

def decode_msg(response_bytes):
    """Decodes a received SIYI message."""
    response_hex = binascii.hexlify(response_bytes).decode('ascii')
    
    if not response_hex.startswith(STX):
        # print(f"Warning: Invalid start bytes: {response_hex[:4]}")
        return None, None, None, None
        
    try:
        # Extract header fields
        len_byte = int(response_hex[4:6], 16)
        seq = int(response_hex[6:8], 16)
        # crc16 = int(response_hex[8:12], 16) # CRC ignored for now
        cmd_id = int(response_hex[12:14], 16)
        
        # Extract data payload
        data_hex = response_hex[14:]
        data_len = len(data_hex) // 2
        
        # Basic length check
        if len_byte != (7 + data_len):
            # print(f"Warning: Length mismatch. Header: {len_byte}, Calculated: {7 + data_len}")
            return None, None, None, None

        # TODO: Add CRC validation here if needed

        return data_hex, data_len, cmd_id, seq
        
    except (ValueError, IndexError) as e:
        print(f"Error decoding message: {e}, Hex: {response_hex}")
        return None, None, None, None

def gimbal_attitude_msg():
    """Creates the message to request gimbal attitude."""
    return encode_msg("", CMD_ACQUIRE_GIMBAL_ATTITUDE)

def parse_gimbal_attitude(data_hex):
    """
    Parses gimbal attitude data from hex string.
    Logic from siyi_utils.py (lines 47-64) / siyi_camera_node.py (lines 297-310)
    """
    if len(data_hex) < 12:
        print(f"Warning: Attitude data too short: {data_hex}")
        return None

    try:
        # Correctly parse signed 16-bit integers (scaled by 10)
        yaw_raw = int(data_hex[0:4], 16)
        pitch_raw = int(data_hex[4:8], 16)
        roll_raw = int(data_hex[8:12], 16)

        # Convert from two's complement if necessary
        if yaw_raw > 0x7FFF: yaw_raw -= 0x10000
        if pitch_raw > 0x7FFF: pitch_raw -= 0x10000
        if roll_raw > 0x7FFF: roll_raw -= 0x10000

        # Convert to degrees
        yaw = yaw_raw / 10.0
        pitch = pitch_raw / 10.0
        roll = roll_raw / 10.0
        
        return {'yaw': yaw, 'pitch': pitch, 'roll': roll}
        
    except ValueError as e:
        print(f"Error parsing attitude data '{data_hex}': {e}")
        return None

# --- Main Execution ---
if __name__ == "__main__":
    # Setup signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(SOCKET_TIMEOUT) # Prevent blocking indefinitely
    server_address = (CAMERA_IP, COMMAND_PORT)
    print(f"Listening for gimbal angles from {CAMERA_IP}:{COMMAND_PORT}")
    print(f"Saving data to {OUTPUT_FILENAME}")
    print("Press Ctrl+C to stop.")

    try:
        with open(OUTPUT_FILENAME, 'w', newline='') as f:
            # Write CSV header
            f.write("timestamp,yaw_deg,pitch_deg,roll_deg\n")
            
            while not stop_requested:
                # Send request for gimbal attitude
                request_msg = gimbal_attitude_msg()
                try:
                    sock.sendto(request_msg, server_address)
                except socket.error as e:
                    print(f"Socket error sending request: {e}")
                    time.sleep(1) # Wait before retrying
                    continue

                # Attempt to receive response
                try:
                    response_bytes, address = sock.recvfrom(1024) # Buffer size
                    
                    # Decode the response
                    data_hex, data_len, cmd_id, seq = decode_msg(response_bytes)

                    if cmd_id == CMD_ACQUIRE_GIMBAL_ATTITUDE:
                        angles = parse_gimbal_attitude(data_hex)
                        if angles:
                            timestamp = time.time()
                            yaw = angles['yaw']
                            pitch = angles['pitch']
                            roll = angles['roll']
                            
                            # Write to file
                            f.write(f"{timestamp:.6f},{yaw:.2f},{pitch:.2f},{roll:.2f}\n")
                            # Optional: Print to console
                            print(f"Recv: Yaw={yaw:.1f} Pitch={pitch:.1f} Roll={roll:.1f}", end='\r')
                            
                except socket.timeout:
                    # No response received within timeout, loop again
                    # print("Socket timeout, no response received.")
                    pass 
                except socket.error as e:
                    print(f"Socket error receiving data: {e}")
                    time.sleep(1) # Wait before retrying
                except Exception as e:
                    print(f"An unexpected error occurred: {e}")

                # Wait before next request
                time.sleep(1.0 / REQUEST_RATE_HZ)

    except IOError as e:
        print(f"Error opening or writing to file {OUTPUT_FILENAME}: {e}")
    finally:
        print("\nClosing socket.")
        sock.close()
        print("Exiting.")