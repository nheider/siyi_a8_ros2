#!/usr/bin/env python3

"""
Utility functions for the SIYI A8 camera package
"""

import struct

def bytes_to_hex_string(data_bytes):
    
    """
    Convert bytes to hex string
    
    Args:
        data_bytes: Bytes to convert
    
    Returns:
        str: Hex string representation (e.g. "0102FF")
    """
    return ''.join(format(b, '02x') for b in data_bytes)
    
def hex_string_to_bytes(hex_string):
    """
    Convert hex string to bytes
    
    Args:
        hex_string: Hex string (e.g. "0102FF")
    
    Returns:
        bytes: Bytes representation
    """
    return bytes.fromhex(hex_string)
    
def parse_gimbal_attitude(data_hex):
    """
    Parse gimbal attitude data from hex string
    
    Args:
        data_hex: Hex string with gimbal attitude data
        
    Returns:
        dict: Dictionary with yaw, pitch, roll values in degrees
    """
    if len(data_hex) < 12:
        return None
        
    # Data format is 3 signed 16-bit integers (YPR in tenths of degrees)
    # SIYI protocol uses little-endian byte order (low byte in front)
    
    # Split and swap bytes for yaw
    yaw_low = data_hex[0:2]
    yaw_high = data_hex[2:4]
    yaw_hex = yaw_high + yaw_low
    
    # Split and swap bytes for pitch
    pitch_low = data_hex[4:6]
    pitch_high = data_hex[6:8]
    pitch_hex = pitch_high + pitch_low
    
    # Split and swap bytes for roll
    roll_low = data_hex[8:10]
    roll_high = data_hex[10:12]
    roll_hex = roll_high + roll_low
    
    # Convert hex to signed int
    yaw_raw = int(yaw_hex, 16)
    if yaw_raw > 0x7FFF:
        yaw_raw -= 0x10000
        
    pitch_raw = int(pitch_hex, 16)
    if pitch_raw > 0x7FFF:
        pitch_raw -= 0x10000
        
    roll_raw = int(roll_hex, 16)
    if roll_raw > 0x7FFF:
        roll_raw -= 0x10000
        
    # Convert to degrees
    yaw = yaw_raw / 10.0
    pitch = pitch_raw / 10.0
    roll = roll_raw / 10.0
    
    return {'yaw': yaw, 'pitch': pitch, 'roll': roll}
    
def parse_zoom_level(data_hex):
    """
    Parse zoom level data from hex string
    
    Args:
        data_hex: Hex string with zoom level data
        
    Returns:
        float: Zoom level (e.g. 1.0, 2.0, etc.)
    """
    if len(data_hex) < 4:
        return 1.0
        
    # Data format depends on protocol version
    # Typically two bytes: integer and fractional parts
    integer_part = int(data_hex[0:2], 16)
    fractional_part = int(data_hex[2:4], 16)
    
    zoom_level = integer_part + (fractional_part / 100.0)
    return zoom_level

def Acquire_firmware_version(data_hex):
    """
    Parse firmware version data from hex string
    
    Args:
        data_hex: Hex string with firmware version data
        
    Returns:
        str: Firmware version string (e.g. "1.2.3")
    """
    if len(data_hex) < 6:
        return "Unknown"
    
    # SIYI protocol typically uses 3 bytes for version (major.minor.patch)
    major = int(data_hex[0:2], 16)
    minor = int(data_hex[2:4], 16)
    patch = int(data_hex[4:6], 16)
    
    return f"{major}.{minor}.{patch}"
