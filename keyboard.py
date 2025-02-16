import serial
import time
from pynput.keyboard import Controller

# Initialize keyboard controller
keyboard = Controller()

# Replace with your actual Serial port
SERIAL_PORT = "/dev/cu.usbmodem11401"  # macOS Example

BAUD_RATE = 19200  # Must match the Arduino baud rate
PITCH_THRESHOLD = 7  # Change in pitch needed to trigger A/D
HOLD_COUNT_THRESHOLD = 20  # Number of readings before holding a key
INIT_READINGS = 50  # Number of readings to determine standard orientation

def read_pitch_from_serial():
    """Reads pitch values from Arduino and simulates 'A' and 'D' key presses based on tilt."""
    standard_pitch = None  # Standard orientation pitch
    hold_count = 0  # Counter for detecting prolonged movement
    prev_pitch = None  # Previous pitch value
    initialized = False  # Flag for completing initial pitch calibration
    holding_a = False  # Track if 'A' is currently held
    holding_d = False  # Track if 'D' is currently held
    initial_press_pitch = None  # Stores the pitch when key was first pressed

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05) as ser:
            ser.flush()  # Clear serial buffer
            readings = []

            while True:
                try:
                    if ser.in_waiting > 0:  # Only read if there is data
                        line = ser.readline().decode().strip()  # Read full Serial line
                        if "PITCH:" in line:
                            pitch = float(line.replace("PITCH:", "").strip())  # Extract the pitch value
                            print(f"Pitch received from Arduino: {pitch}")

                            # === STEP 1: Determine Standard Orientation === #
                            if not initialized:
                                readings.append(pitch)
                                if len(readings) >= INIT_READINGS:
                                    standard_pitch = sum(readings) / len(readings)  # Average initial readings
                                    initialized = True
                                    print(f"Standard Pitch Set: {standard_pitch}")
                                continue  # Skip the rest until initialization is complete

                            # === STEP 2: Detect Changes in Pitch === #
                            pitch_change = pitch - standard_pitch

                            # Reset the initial pitch reference when the direction reverses
                            if prev_pitch is not None:
                                if (pitch_change > 0 and prev_pitch < 0) or (pitch_change < 0 and prev_pitch > 0):
                                    initial_press_pitch = None  # Reset when movement reverses

                            # Detecting Single Key Press Events
                            if pitch_change < -PITCH_THRESHOLD:  # Tilt left
                                if not holding_a:
                                    if initial_press_pitch is None:  # First press, store the initial pitch
                                        initial_press_pitch = pitch
                                    if abs(pitch - initial_press_pitch) >= PITCH_THRESHOLD:
                                        keyboard.press('a')
                                        print("Pressed: A")
                                        keyboard.release('d')
                                        hold_count += 1  # Increase hold counter
                                        holding_a = True
                                        holding_d = False
                            elif pitch_change > PITCH_THRESHOLD:  # Tilt right
                                if not holding_d:
                                    if initial_press_pitch is None:  # First press, store the initial pitch
                                        initial_press_pitch = pitch
                                    if abs(pitch - initial_press_pitch) >= PITCH_THRESHOLD:
                                        keyboard.press('d')
                                        print("Pressed: D")
                                        keyboard.release('a')
                                        hold_count += 1  # Increase hold counter
                                        holding_d = True
                                        holding_a = False
                            else:
                                hold_count = 0  # Reset hold count if near standard orientation

                            # === STEP 3: Detect Holding Keys === #
                            if hold_count >= HOLD_COUNT_THRESHOLD:
                                if pitch_change < -PITCH_THRESHOLD and not holding_a:  # Hold 'A'
                                    keyboard.press('a')
                                    print("Holding: A")
                                    holding_a = True
                                elif pitch_change > PITCH_THRESHOLD and not holding_d:  # Hold 'D'
                                    keyboard.press('d')
                                    print("Holding: D")
                                    holding_d = True

                            # === STEP 4: Release Key When Pitch Starts Returning === #
                            if prev_pitch is not None:
                                if (pitch_change > prev_pitch and pitch_change < 0) or \
                                   (pitch_change < prev_pitch and pitch_change > 0):
                                    if holding_a:
                                        keyboard.release('a')
                                        print("Released: A")
                                        holding_a = False
                                    if holding_d:
                                        keyboard.release('d')
                                        print("Released: D")
                                        holding_d = False
                                    hold_count = 0  # Reset counter

                            prev_pitch = pitch_change  # Store previous pitch value

                            time.sleep(0.05)  # Prevent spamming
                except ValueError:
                    continue  # Ignore invalid Serial data
    except serial.SerialException:
        print(f"Error: Could not open port {SERIAL_PORT}. Is the Arduino connected?")


if __name__ == "__main__":
    print("Starting Gyro Wheel Controller...")
    read_pitch_from_serial()
