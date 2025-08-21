import time
from SM120BL_MB_motor_controller import Feetech_motor

# --- Configuration ---
MOTOR_ID = 1
SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B003LKH9-if00-port0"  # <-- Make sure this is your correct port!

# Let's define two positions to test movement
# We will assume 0 is the starting pulse and 1024 is the target
POSITION_START = 1400
POSITION_TARGET = 10000 # This is the value you need to adjust!

# --- Main Script ---
if __name__ == "__main__":
    print(f"Connecting to motor on port {SERIAL_PORT}...")
    motor = Feetech_motor(serial_port=SERIAL_PORT)

    if not motor.client or not motor.client.is_open:
        print("Failed to connect. Please check the port.")
    else:
        print("Connection successful!")

        # 1. Enable torque
        print(f"Enabling torque for motor ID {MOTOR_ID}...")
        motor.motor_torque_enable(MOTOR_ID)
        time.sleep(0.1)

        # 2. Go to the TARGET position (FORWARD)
        print(f"Moving to target position: {POSITION_TARGET}")
        motor.set_motor_position(MOTOR_ID, POSITION_TARGET)
        print("Waiting for motor to arrive...")
        time.sleep(3) # Give it time to get there

        # 3. Go back to the START position (REVERSE)
        print(f"Moving back to start position: {POSITION_START}")
        motor.set_motor_position(MOTOR_ID, POSITION_START)
        print("Waiting for motor to return...")
        time.sleep(3)

        # 4. Disable torque to let the motor rest
        print(f"Disabling torque for motor ID {MOTOR_ID}.")
        motor.motor_torque_disable(MOTOR_ID)

        print("Script finished. ✨")