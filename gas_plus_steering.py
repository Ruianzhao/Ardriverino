import serial
import time
from pynput.keyboard import Controller

# Initialize keyboard controller
keyboard = Controller()

# Replace with your actual Serial port
SERIAL_PORT = "/dev/cu.usbmodem11401"  # macOS Example

BAUD_RATE = 19200  # Must match the Arduino baud rate
PITCH_THRESHOLD = 3  # Change in pitch needed to trigger A/D
FORCE_THRESHOLD = 300   # Default force reading
HOLD_COUNT_THRESHOLD = 20  # Number of readings before holding a key
INIT_READINGS = 20  # Number of readings to determine standard orientation

def read_pitch_and_force():
    """Reads pitch & force values from Arduino and simulates keyboard actions."""
    standard_pitch = None  # Standard pitch orientation
    standard_force = None  # Standard force value
    hold_count = 0  # Counter for detecting prolonged movement
    prev_pitch = None  # Previous pitch value
    prev_force = None  # Previous force value
    initialized = False  # Flag for completing initial calibration
    holding_a = False  # Track if 'A' is held
    holding_d = False  # Track if 'D' is held
    holding_w = False  # Track if 'W' is held
    initial_press_pitch = None  # Pitch when key was first pressed
    initial_press_force = None  # Force when key was first pressed

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05) as ser:
            ser.flush()  # Clear serial buffer
            readings_pitch = []
            readings_force = []

            while True:
                try:
                    if ser.in_waiting > 0:  # Only read if data exists
                        line = ser.readline().decode().strip()  # Read full Serial line
                        if "PITCH:" in line and "FORCE:" in line:
                            # Extract PITCH and FORCE values
                            parts = line.split("|")
                            pitch = float(parts[0].replace("PITCH:", "").strip())
                            force = int(parts[1].replace("FORCE:", "").strip())

                            print(f"Pitch: {pitch}, Force: {force}")  # Debug output

                            # === STEP 1: Determine Standard Orientation === #
                            if not initialized:
                                readings_pitch.append(pitch)
                                readings_force.append(force)
                                if len(readings_pitch) >= INIT_READINGS:
                                    standard_pitch = sum(readings_pitch) / len(readings_pitch)
                                    standard_force = sum(readings_force) / len(readings_force)
                                    initialized = True
                                    print(f"Standard Pitch Set: {standard_pitch}")
                                    print(f"Standard Force Set: {standard_force}")
                                continue  # Skip the rest until initialization is complete

                            # === STEP 2: Detect Changes in Pitch === #
                            pitch_change = pitch - standard_pitch
                            force_change = force - standard_force

                            # Detecting A/D Key Press
                            if pitch_change < -PITCH_THRESHOLD:  # Tilt left
                                if not holding_a:
                                    keyboard.press('a')
                                    print("Pressed: A")
                                    keyboard.release('d')
                                    holding_a = True
                                    holding_d = False
                            elif pitch_change > PITCH_THRESHOLD:  # Tilt right
                                if not holding_d:
                                    keyboard.press('d')
                                    print("Pressed: D")
                                    keyboard.release('a')
                                    holding_d = True
                                    holding_a = False
                            else:
                                keyboard.release('a')
                                keyboard.release('d')
                                holding_a = False
                                holding_d = False

                            # Detecting W Key Press (Force threshold exceeded)
                            if force_change > FORCE_THRESHOLD:
                                if not holding_w:
                                    keyboard.press('w')
                                    print("Pressed: W")
                                    holding_w = True
                            else:
                                if holding_w:
                                    keyboard.release('w')
                                    print("Released: W")
                                    holding_w = False

                            prev_pitch = pitch_change
                            prev_force = force_change
                            time.sleep(0.05)  # Prevent spamming
                except ValueError:
                    continue  # Ignore invalid Serial data
    except serial.SerialException:
        print(f"Error: Could not open port {SERIAL_PORT}. Is the Arduino connected?")


if __name__ == "__main__":
    print("Starting Gyro Wheel Controller...")
    read_pitch_and_force()
