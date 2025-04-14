#!/usr/bin/env python3

from siyi_a8_ros2.siyi_message import SIYIMessage
import time

def compare_messages(name, actual, expected):
    """Compare actual message with expected ground truth"""
    expected = expected.replace(" ", "").lower()
    matches = actual.lower() == expected
    result = "PASS ✓" if matches else "FAIL ✗"
    print(f"{name}: {result}")
    if not matches:
        print(f"  Expected: {expected}")
        print(f"  Actual:   {actual.lower()}")
        print(f"  Diff at:  {find_diff_position(actual.lower(), expected)}")
    return matches

def find_diff_position(actual, expected):
    """Find position where strings differ"""
    for i in range(min(len(actual), len(expected))):
        if actual[i] != expected[i]:
            return i
    return len(min(actual, expected, key=len))

def test_decoding(siyi, name, hex_msg, expected_data, expected_data_len, expected_cmd_id, expected_seq):
    """Test decoding a message and compare results with expected values"""
    hex_msg = hex_msg.replace(" ", "").lower()
    data, data_len, cmd_id, seq = siyi.decode_msg(hex_msg)
    
    data_match = data.lower() == expected_data.lower() if expected_data is not None else True
    data_len_match = data_len == expected_data_len
    cmd_id_match = cmd_id.lower() == expected_cmd_id.lower()
    seq_match = str(seq).lower() == expected_seq.lower() if isinstance(expected_seq, str) else seq == expected_seq
    
    all_match = data_match and data_len_match and cmd_id_match and seq_match
    result = "PASS ✓" if all_match else "FAIL ✗"
    
    print(f"Decode {name}: {result}")
    if not all_match:
        if not data_match:
            print(f"  Expected data: {expected_data}")
            print(f"  Actual data:   {data}")
        if not data_len_match:
            print(f"  Expected data_len: {expected_data_len}")
            print(f"  Actual data_len:   {data_len}")
        if not cmd_id_match:
            print(f"  Expected cmd_id: {expected_cmd_id}")
            print(f"  Actual cmd_id:   {cmd_id}")
        if not seq_match:
            print(f"  Expected seq: {expected_seq}")
            print(f"  Actual seq:   {seq}")
    
    return all_match

def main():
    # Create SIYI message handler
    siyi = SIYIMessage()
    
    print("=== Testing SIYI A8 Mini Message Generation vs Documentation ===\n")

    # Ground truth messages from documentation
    ground_truth = {
        "Zoom 1": "55 66 01 01 00 00 00 05 01 8d 64",
        "Zoom -1": "55 66 01 01 00 00 00 05 FF 5c 6a",
        "Absolute Zoom (4.5X)": "55 66 01 02 00 01 00 0F 04 05 60 BB",
        "Acquire Max Zoom": "55 66 01 00 00 00 00 16 B2 A6",
        "Manual Focus +": "55 66 01 01 00 00 00 06 01 de 31",
        "Manual Focus -": "55 66 01 01 00 00 00 06 ff 0f 3f",
        "Take Photo": "55 66 01 01 00 00 00 0c 00 34 ce",
        "Record Video": "55 66 01 01 00 00 00 0c 02 76 ee",
        "Gimbal Speed": "55 66 01 02 00 00 00 07 64 64 3d cf",  # 100, 100
        "Center Gimbal": "55 66 01 01 00 00 00 08 01 d1 12",
        "Gimbal Info": "55 66 01 00 00 00 00 0a 0f 75",
        "Auto Focus": "55 66 01 01 00 00 00 04 01 bc 57",
        "Hardware ID": "55 66 01 00 00 00 00 02 07 f4",
        "Firmware Version": "55 66 01 00 00 00 00 01 64 c4",
        "Lock Mode": "55 66 01 01 00 00 00 0c 03 57 fe",
        "Follow Mode": "55 66 01 01 00 00 00 0c 04 b0 8e",
        "FPV Mode": "55 66 01 01 00 00 00 0c 05 91 9e",
        "Attitude Data": "55 66 01 00 00 00 00 0d e8 05"
    }

    # Test each message generation
    gen_passes = 0
    gen_total = 0
    
    # Set sequence to -1 before each test so it becomes 0 after increment
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Zoom 1", siyi.zoom_in_msg(), ground_truth["Zoom 1"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Zoom -1", siyi.zoom_out_msg(), ground_truth["Zoom -1"])
    
    # For sequence 1, set to 0 so it becomes 1 after increment
    siyi._seq = 0
    gen_total += 1
    gen_passes += compare_messages("Absolute Zoom (4.5X)", siyi.absolute_zoom_msg(4, 5), ground_truth["Absolute Zoom (4.5X)"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Acquire Max Zoom", siyi.maximum_zoom_msg(), ground_truth["Acquire Max Zoom"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Manual Focus +", siyi.focus_far_msg(), ground_truth["Manual Focus +"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Manual Focus -", siyi.focus_close_msg(), ground_truth["Manual Focus -"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Take Photo", siyi.photo_msg(), ground_truth["Take Photo"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Record Video", siyi.record_msg(), ground_truth["Record Video"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Gimbal Speed (100, 100)", siyi.gimbal_speed_msg(100, 100), ground_truth["Gimbal Speed"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Center Gimbal", siyi.gimbal_center_msg(), ground_truth["Center Gimbal"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Gimbal Info", siyi.gimbal_info_msg(), ground_truth["Gimbal Info"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Auto Focus", siyi.autofocus_msg(), ground_truth["Auto Focus"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Hardware ID", siyi.hardware_id_msg(), ground_truth["Hardware ID"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Firmware Version", siyi.firmware_version_msg(), ground_truth["Firmware Version"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Lock Mode", siyi.lock_mode_msg(), ground_truth["Lock Mode"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Follow Mode", siyi.follow_mode_msg(), ground_truth["Follow Mode"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("FPV Mode", siyi.fpv_mode_msg(), ground_truth["FPV Mode"])
    
    siyi._seq = -1
    gen_total += 1
    gen_passes += compare_messages("Attitude Data", siyi.gimbal_attitude_msg(), ground_truth["Attitude Data"])
    
    print(f"\nGeneration Results: {gen_passes}/{gen_total} messages match documentation")
    
    # Test message decoding
    print("\n=== Testing SIYI A8 Mini Message Decoding ===\n")
    
    decode_passes = 0
    decode_total = 0
    
    # Define expected decode results
    decode_test_cases = [
        # name, hex_msg, expected_data, expected_data_len, expected_cmd_id, expected_seq
        ("Zoom 1", ground_truth["Zoom 1"], "01", 1, "05", 0),
        ("Zoom -1", ground_truth["Zoom -1"], "ff", 1, "05", 0),
        ("Absolute Zoom", ground_truth["Absolute Zoom (4.5X)"], "0405", 2, "0f", 1),
        ("Maximum Zoom", ground_truth["Acquire Max Zoom"], "", 0, "16", 0),
        ("Manual Focus +", ground_truth["Manual Focus +"], "01", 1, "06", 0),
        ("Manual Focus -", ground_truth["Manual Focus -"], "ff", 1, "06", 0),
        ("Take Photo", ground_truth["Take Photo"], "00", 1, "0c", 0),
        ("Record Video", ground_truth["Record Video"], "02", 1, "0c", 0),
        ("Gimbal Speed", ground_truth["Gimbal Speed"], "6464", 2, "07", 0),
        ("Center Gimbal", ground_truth["Center Gimbal"], "01", 1, "08", 0),
        ("Gimbal Info", ground_truth["Gimbal Info"], "", 0, "0a", 0),
        ("Auto Focus", ground_truth["Auto Focus"], "01", 1, "04", 0),
        ("Hardware ID", ground_truth["Hardware ID"], "", 0, "02", 0),
        ("Firmware Version", ground_truth["Firmware Version"], "", 0, "01", 0),
        ("Lock Mode", ground_truth["Lock Mode"], "03", 1, "0c", 0),
        ("Follow Mode", ground_truth["Follow Mode"], "04", 1, "0c", 0),
        ("FPV Mode", ground_truth["FPV Mode"], "05", 1, "0c", 0),
        ("Attitude Data", ground_truth["Attitude Data"], "", 0, "0d", 0)
    ]
    
    for test_case in decode_test_cases:
        decode_total += 1
        decode_passes += test_decoding(siyi, *test_case)
    
    print(f"\nDecoding Results: {decode_passes}/{decode_total} messages correctly decoded")
    

if __name__ == "__main__":
    main() 