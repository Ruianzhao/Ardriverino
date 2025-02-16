import serial 
import time
from  pynput.keyboard import Controller

keyboard = Controller()


SERIAL_PORT = "COM3"

BAUD_RATE = 19200
FORCE_THRESHOLD = 100 
INIT_READINGS = 50
HOLD_COUNT_THRESHOLD = 20


def serial_port_force_reader():
    standard_force = None  # Standard orientation pitch
    hold_count = 0  # Counter for detecting prolonged movement
    prev_force = None  # Previous pitch value
    initialized = False  # Flag for completing initial pitch calibration
    holding_w = False  # Track if 'A' is currently held
    initial_press_force = None  # Stores the pitch when key was first pressed
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05) as ser:
            ser.flush()  # Clear serial buffer
            readings = []

            while True:
                try:
                    if ser.in_waiting > 0:  # Only read if there is data
                        line = ser.readline().decode().strip()  # Read full Serial line
                        if "FORCE:" in line:
                            force = float(line.replace("FORCE:", "").strip())  # Extract the force value
                            print(f"force received from Arduino: {force}")

                            if not initialized:
                                readings.append(force )
                                if len(readings) >= INIT_READINGS:
                                    standard_force = sum(readings) / len(readings)  # Average initial readings
                                    initialized = True
                                    print(f"Standard force Set: {standard_force}")
                                continue  # Skip the rest until initialization is complete


    
                            # === STEP 2: Detect Changes in force === #
                            force_change = force - standard_force

                            # Reset the initial force reference when the direction reverses
                            if prev_force is not None:
                                if (force_change > 0 and prev_force < 0) or (force_change < 0 and prev_force > 0):
                                    initial_press_force = None  # Reset when movement reverses

                            # Detecting Single Key Press Events
                            if force_change > FORCE_THRESHOLD:  # Tilt left
                                if not holding_w:
                                    if initial_press_force is None:  # First press, store the initial force
                                        initial_press_force = force
                                    if abs(force - initial_press_force) >= FORCE_THRESHOLD:
                                        keyboard.press('w')
                                        print("Pressed: 2")
                                        hold_count += 1  # Increase hold counter
                                        holding_w= True
                            else:
                                hold_count = 0  # Reset hold count if near standard orientation

                            # === STEP 3: Detect Holding Keys === #
                            if hold_count >= HOLD_COUNT_THRESHOLD:
                                if force_change < -FORCE_THRESHOLD and not holding_w:  # Hold 'A'
                                    keyboard.press('w')
                                    print("Holding: W")
                                    holding_w = True

                            # === STEP 4: Release Key When force Starts Returning === #
                            if prev_force is not None:
                                if (force_change > prev_force and force_change < 0) or \
                                   (force_change < prev_force and force_change > 0):
                                    if holding_w:
                                        keyboard.release('w')
                                        print("Released: W")
                                        holding_w = False
                                    hold_count = 0  # Reset counter
                            prev_force = force_change  # Store previous force value
                            time.sleep(0.05)  # Prevent spamming
                except ValueError:
                    continue  # Ignore invalid Serial data
    except serial.SerialException:
        print(f"Error: Could not open port {SERIAL_PORT}. Is the Arduino connected?")


if __name__ == "__main__":
    print("Starting Gyro Wheel Controller...")
    serial_port_force_reader()
