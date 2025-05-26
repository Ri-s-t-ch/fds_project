import can
import time

# Create CAN bus interface
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Define data
# Example: Left (-100), Forward (+80), Running (0)
left_right = 156  # 156 = -100 interpreted as unsigned byte (256 - 100)
forward_back = 80
stop_state = 0  # 1 = stop, 0 = run

# Construct message
msg = can.Message(arbitration_id=0x21,
                  data=[left_right, forward_back, stop_state],
                  is_extended_id=False)

try:
    while True:
        bus.send(msg)
        print(f"Sent: {msg.data}")
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopped by user")
except can.CanError as e:
    print(f"CAN error: {e}")
