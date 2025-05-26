import can
import time

def send_can_messages():
    try:
        # Set up a CAN bus on can0 with default settings
        bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # Create a standard CAN frame (11-bit ID)
        msg = can.Message(arbitration_id=0x12,
                          data=[0x68, 0x65, 0x6C, 0x6C, 0x6F],  # 'hello' in ASCII
                          is_extended_id=False)

        print("Starting CAN transmission...")
        while True:
            try:
                bus.send(msg)
                print(f"Sent: {msg}")
            except can.CanError:
                print("Message not sent!")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopped by user")

if __name__ == '__main__':
    send_can_messages()
