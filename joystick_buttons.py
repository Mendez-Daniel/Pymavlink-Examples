"""
Example of how to connect Pymavlink to an autopilot using a UDP in order to determine if certain joystick buttons have been pressed.

Requirements for use in BlueOS:
- You need to create a MAVLink Endpoints in BlueOS, type UDP Client. Example IP 192.168.2.1:14770.
- It is necessary to use the MAVLink router "MAVP2P" to route data, as the MANUAL_CONTROL message will not appear when the joystick is connected.

NOTAS:
All system components that communicate via MAVLink are expected to send a HEARTBEAT message at a constant rate of at least 1 Hz.

The MANUAL_CONTROL message define the target system to be controlled, the movement in four primary axes (x, y, z, r) and two extension axes (s, t), and two 16-bit fields to represent the states of up to 32 buttons (buttons, buttons2).

The axis values can range from -1000 to 1000. The button values are represented by integer values.

Examples of button messages
Button 3 pressed: 4 -> 0b0000000000000100
Button 1 and 5 pressed: 17 -> 0b0000000000010001

Link: https://mavlink.io/en/services/manual_control.html
"""


import time
from pymavlink import mavutil

# Connect to a specific communication port or channel.
# Replace '/dev/ttyUSB0' with the port or IP address of your vehicle.
connection = mavutil.mavlink_connection('udpin:192.168.2.1:14880')

# Wait for the vehicle to send a heartbeat to ensure that the connection is successful.
connection.wait_heartbeat()
print("Conectado al vehículo.")

# Joystick reference buttons with their bit command enumeration from right
# A button = 0
# B button = 1
# X button = 2
# Y button = 3

shift_button = 0
action1_button = 1
action2_button = 3

try:
    status_press = False

    # Loop to receive and process incoming messages.
    while True:
        # Receive a message of type MANUAL_CONTROL with blocking (wait until one is received).
        msg = connection.recv_match(type='MANUAL_CONTROL', blocking=True)
        
        joystick_buttons = msg.buttons # Integer value (16-bit)

        # Status of the buttons
        # If the bit corresponding to the buttons is equal to 1, it is set to true.
        status_shift_button = joystick_buttons & (1 << shift_button)
        status_action1_button = joystick_buttons & (1 << action1_button)
        status_action2_button = joystick_buttons & (1 << action2_button)

        # Execution of button commands (only once per press)
        # Press button A and B
        if status_shift_button and status_action1_button and status_press == False:
            print(f"Laser beam attack activated")
            status_press = True
        # Press button A and Y
        elif status_shift_button and status_action2_button and status_press == False:
            print(f"Laser beam attack desactivated")
            status_press = True
        # Reset the keystroke
        elif (status_action1_button == 0) and (status_action2_button == 0):
            status_press = False
        
        time.sleep(0.01)


except KeyboardInterrupt:
    # Allow loop exit with Ctrl+C
    print("\nExiting the script")


finally:
    # Close the connection before leaving
    connection.close()
    print("Closed connection")