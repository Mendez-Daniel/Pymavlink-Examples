from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Make sure the connection is valid
master.wait_heartbeat()

# Get some information !
print(master.recv_match().to_dict())