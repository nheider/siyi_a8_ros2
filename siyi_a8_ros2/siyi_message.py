#!/usr/bin/env python3

from .siyi_crc16 import CRC16


class SIYIMessage:
    """
    SIYI message handler for A8 Mini camera gimbal
    """
    # Command IDs for SIYI protocol
    ACQUIRE_FIRMWARE_VERSION = "01"
    ACQUIRE_HARDWARE_ID = "02"
    AUTOFOCUS = "04"
    MANUAL_ZOOM = "05"
    ABSOLUTE_ZOOM = "0f"
    ACQUIRE_MAX_ZOOM = "16"
    MANUAL_FOCUS = "06"
    GIMBAL_ROTATION = "07"
    CENTER = "08"
    ACQUIRE_GIMBAL_INFO = "0a"
    FUNCTION_FEEDBACK_INFO = "0b"
    PHOTO_VIDEO = "0c"
    ACQUIRE_GIMBAL_ATTITUDE = "0d"
    CONTROL_ANGLE = "0e"
    
    def __init__(self):
        self.HEADER = "5566"
        self._ctr = "01"
        self._seq = 0
        self.MINIMUM_DATA_LENGTH = 10 * 2
        
    def increment_seq(self, val):
        """
        Increment sequence number for message tracking
        """
        if val < 0 or val > 65535:
            self._seq = 0
            return "0000"
            
        seq = val + 1
        self._seq = seq
        
        # Convert to hex and format appropriately
        seq_hex = format(seq, '04x')
        seq_hex = seq_hex[-4:]  # Take last 4 characters
        
        if len(seq_hex) < 4:
            seq_hex = '0' * (4 - len(seq_hex)) + seq_hex
            
        # Swap bytes for SIYI protocol
        low_b = seq_hex[-2:]
        high_b = seq_hex[:2]
        
        return low_b + high_b
        
    @staticmethod
    def compute_data_len(data):
        """
        Compute data length field for message
        """
        if len(data) % 2 != 0:
            data = "0" + data
            
        L = len(data) // 2
        
        # Convert to hex and format appropriately
        len_hex = format(L, '04x')
        len_hex = len_hex[-4:]  # Take last 4 characters
        
        if len(len_hex) < 4:
            len_hex = '0' * (4 - len(len_hex)) + len_hex
            
        # Swap bytes for SIYI protocol
        low_b = len_hex[-2:]
        high_b = len_hex[:2]
        
        return low_b + high_b
        
    def decode_msg(self, msg):
        """
        Decode a SIYI message
        
        Returns:
            tuple: (data, data_len, cmd_id, seq)
        """
        data = ""
        cmd_id = ""
        data_len = 0
        seq = 0
        
        if len(msg) < self.MINIMUM_DATA_LENGTH:
            print(f"Warning, message length is not long enough for decoding: {len(msg)}")
            return data, data_len, cmd_id, seq
            
        # Check header
        header = msg[0:4]
        if header != self.HEADER:
            print(f"Invalid header: {header}, expected: {self.HEADER}")
            return data, data_len, cmd_id, seq
            
        # Check data length, bytes are reversed
        data_len_hex = msg[6:8]
        data_len = int(data_len_hex, 16)
        
        # Perform CRC16 checkout
        msg_crc = msg[-4:].lower()
        payload = msg[:-4]
        crc = CRC16.compute_str_swap(payload).lower()
        
        if crc != msg_crc:
            print(f"Warning, CRC16 error during message decoding")
            print(f"Calculated CRC: {crc}, Message CRC: {msg_crc}")
            print(f"Message: {msg}")
            return data, data_len, cmd_id, seq
            
        # Get sequence
        seq = msg[8:10]
        
        # Get command ID
        cmd_id = msg[10:12]
        
        # Get data
        if data_len > 0:
            data = msg[12:12 + data_len * 2]  # *2 because each byte is 2 chars
            
        return data, data_len, cmd_id, seq
        
    def encode_msg(self, data, cmd_id):
        """
        Encode a SIYI message
        
        Args:
            data: Hex string of data (e.g., "0102FF")
            cmd_id: Command ID (e.g., "07")
            
        Returns:
            str: Encoded message as hex string
        """
        seq = self.increment_seq(self._seq)
        data_len = self.compute_data_len(data)
        msg_front = self.HEADER + self._ctr + data_len + seq + cmd_id + data
        
        # Insert CRC16 bytes
        crc = CRC16.compute_str_swap(msg_front)
        if crc:
            msg = msg_front + crc
            return msg
        else:
            print("Warning, CRC16 error during message encoding")
            return ""
    
    # Message definition methods
    def firmware_version_msg(self):
        """Get firmware version message"""
        data = ""
        cmd_id = self.ACQUIRE_FIRMWARE_VERSION
        return self.encode_msg(data, cmd_id)
        
    def hardware_id_msg(self):
        """Get hardware ID message"""
        data = ""
        cmd_id = self.ACQUIRE_HARDWARE_ID
        return self.encode_msg(data, cmd_id)
        
    def autofocus_msg(self):
        """Trigger autofocus"""
        data = "01"
        cmd_id = self.AUTOFOCUS
        return self.encode_msg(data, cmd_id)
        
    def zoom_in_msg(self):
        """Zoom in camera"""
        data = "01"
        cmd_id = self.MANUAL_ZOOM
        return self.encode_msg(data, cmd_id)
        
    def zoom_out_msg(self):
        """Zoom out camera"""
        data = "ff"  # -1 in hex
        cmd_id = self.MANUAL_ZOOM
        return self.encode_msg(data, cmd_id)
        
    def zoom_halt_msg(self):
        """Stop zooming"""
        data = "00"
        cmd_id = self.MANUAL_ZOOM
        return self.encode_msg(data, cmd_id)
        
    def absolute_zoom_msg(self, integer, fractional):
        """Set absolute zoom level"""
        # Format integer and fractional parts as hex
        data1 = format(integer & 0xFF, '02x')
        data2 = format(fractional & 0xFF, '02x')
        data = data1 + data2
        cmd_id = self.ABSOLUTE_ZOOM
        return self.encode_msg(data, cmd_id)
        
    def maximum_zoom_msg(self):
        """Get maximum zoom level"""
        data = ""
        cmd_id = self.ACQUIRE_MAX_ZOOM
        return self.encode_msg(data, cmd_id)
        
    def focus_far_msg(self):
        """Focus farther"""
        data = "01"
        cmd_id = self.MANUAL_FOCUS
        return self.encode_msg(data, cmd_id)
        
    def focus_close_msg(self):
        """Focus closer"""
        data = "ff"  # -1 in hex
        cmd_id = self.MANUAL_FOCUS
        return self.encode_msg(data, cmd_id)
        
    def focus_halt_msg(self):
        """Stop focusing"""
        data = "00"
        cmd_id = self.MANUAL_FOCUS
        return self.encode_msg(data, cmd_id)
        
    def gimbal_speed_msg(self, yaw_speed, pitch_speed):
        """Control gimbal by speed
        
        Args:
            yaw_speed: -100 to 100
            pitch_speed: -100 to 100
        """
        # Clamp values
        yaw_speed = max(-100, min(100, yaw_speed))
        pitch_speed = max(-100, min(100, pitch_speed))
        
        # Convert to hex
        data1 = format(yaw_speed & 0xFF, '02x')
        data2 = format(pitch_speed & 0xFF, '02x')
        
        data = data1 + data2
        cmd_id = self.GIMBAL_ROTATION
        return self.encode_msg(data, cmd_id)
        
    def gimbal_center_msg(self):
        """Center the gimbal"""
        data = "01"
        cmd_id = self.CENTER
        return self.encode_msg(data, cmd_id)
        
    def gimbal_info_msg(self):
        """Get gimbal information"""
        data = ""
        cmd_id = self.ACQUIRE_GIMBAL_INFO
        return self.encode_msg(data, cmd_id)
        
    def lock_mode_msg(self):
        """Set gimbal to lock mode"""
        data = "03"
        cmd_id = self.PHOTO_VIDEO
        return self.encode_msg(data, cmd_id)
        
    def follow_mode_msg(self):
        """Set gimbal to follow mode"""
        data = "04"
        cmd_id = self.PHOTO_VIDEO
        return self.encode_msg(data, cmd_id)
        
    def fpv_mode_msg(self):
        """Set gimbal to FPV mode"""
        data = "05"
        cmd_id = self.PHOTO_VIDEO
        return self.encode_msg(data, cmd_id)
        
    def function_feedback_msg(self):
        """Get function feedback information"""
        data = ""
        cmd_id = self.FUNCTION_FEEDBACK_INFO
        return self.encode_msg(data, cmd_id)
        
    def photo_msg(self):
        """Take a photo"""
        data = "00"
        cmd_id = self.PHOTO_VIDEO
        return self.encode_msg(data, cmd_id)
        
    def record_msg(self):
        """Start/stop recording"""
        data = "02"
        cmd_id = self.PHOTO_VIDEO
        return self.encode_msg(data, cmd_id)
        
    def gimbal_attitude_msg(self):
        """Get gimbal attitude"""
        data = ""
        cmd_id = self.ACQUIRE_GIMBAL_ATTITUDE
        return self.encode_msg(data, cmd_id)
        
    def gimbal_angles_msg(self, yaw, pitch):
        """Control gimbal by angles
        
        Args:
            yaw: -135 to 135 degrees
            pitch: -90 to 25 degrees
        """
        # Clamp values
        yaw = max(-135.0, min(135.0, yaw))
        pitch = max(-90.0, min(25.0, pitch))
        
        # Convert to int (×10 for precision)
        control_yaw = int(yaw * 10)
        control_pitch = int(pitch * 10)
        
        # Convert to hex
        data1 = format(control_yaw & 0xFF, '02x')
        data2 = format(control_pitch & 0xFF, '02x')
        
        data = data1 + data2
        cmd_id = self.CONTROL_ANGLE
        return self.encode_msg(data, cmd_id)
        
    def straight_down_msg(self):
        """Point camera straight down (90 degrees pitch)"""
        return self.gimbal_angles_msg(0.0, -90.0)
