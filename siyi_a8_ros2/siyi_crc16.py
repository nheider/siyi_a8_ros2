#!/usr/bin/env python3

class CRC16:
    """
    CRC16 calculator for SIYI messages
    """
    
    @classmethod
    def compute_str_swap(cls, data_str):
        """Compute CRC16 for a string and swap bytes"""
        # Convert hex string to bytes
        data_bytes = bytes.fromhex(data_str)
        
        # Compute CRC16
        crc = cls.compute(data_bytes)
        
        # Convert to hex and swap bytes
        crc_hex = format(crc, '04x')
        if len(crc_hex) < 4:
            crc_hex = '0' * (4 - len(crc_hex)) + crc_hex
            
        # Swap bytes for SIYI protocol
        low_byte = crc_hex[2:]
        high_byte = crc_hex[:2]
        return low_byte + high_byte
        
    @staticmethod
    def compute(data_bytes):
        """Compute CRC16 for byte data"""
        crc = 0x3692  # Initial value for SIYI protocol
        
        for byte in data_bytes:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021  # Polynomial for SIYI protocol
                else:
                    crc = crc << 1
                crc &= 0xFFFF  # Keep only the lower 16 bits
                
        return crc
