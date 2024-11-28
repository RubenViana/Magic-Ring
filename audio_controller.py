import serial
import sys
import subprocess
import matplotlib.pyplot as plt

# Bluetooth setup
# sudo rfcomm bind /dev/rfcomm0 50:02:91:8A:39:3A

# Constants
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
BLUETOOTH_PORT = '/dev/rfcomm0'

# State variables
volume_muted = False
mic_muted = False

connection_type = "bt" # Serial or Bluetooth

dataIMUx = []
dataIMUy = []
dataIMUz = []

def plotIMUData():
    global dataIMU

    plt.figure()
    plt.scatter(dataIMUx, dataIMUy)
    plt.show()



def setupBluetooth():
    print(f"Opening Bluetooth port {BLUETOOTH_PORT} at {BAUD_RATE} baud...")
    bt = serial.Serial(BLUETOOTH_PORT, BAUD_RATE)
    print("Bluetooth port opened successfully")
    return bt

def setupSerial():
    print(f"Opening serial port {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print("Serial port opened successfully")
    return ser

def setup():
    """Setup system volume and microphone state"""
    global volume_muted
    global mic_muted

    # Get system volume state
    volume_state = subprocess.run(['amixer', '-D', 'pulse', 'get', 'Master'], capture_output=True, text=True)
    volume_muted = "[off]" in volume_state.stdout

    # Get microphone state
    mic_state = subprocess.run(['amixer', '-D', 'pulse', 'get', 'Capture'], capture_output=True, text=True)
    mic_muted = "[off]" in mic_state.stdout
                 
def toggle_volume():
    """Toggle system volume mute state"""
    global volume_muted
    volume_muted = not volume_muted
    
    if volume_muted:
        subprocess.run(['amixer', '-D', 'pulse', 'set', 'Master', 'mute'])
        print("Volume muted")
    else:
        subprocess.run(['amixer', '-D', 'pulse', 'set', 'Master', 'unmute'])
        print("Volume unmuted")

def toggle_microphone():
    """Toggle microphone mute state"""
    global mic_muted
    mic_muted = not mic_muted
    
    if mic_muted:
        subprocess.run(['amixer', '-D', 'pulse', 'set', 'Capture', 'nocap'])
        print("Microphone muted")
    else:
        subprocess.run(['amixer', '-D', 'pulse', 'set', 'Capture', 'cap'])
        print("Microphone unmuted")

def increase_volume():
    """Increase system volume"""

    global volume_muted
    volume_muted = False
    subprocess.run(['amixer', '-D', 'pulse', 'set', 'Master', 'unmute'])

    subprocess.run(['amixer', '-D', 'pulse', 'set', 'Master', '5%+'])
    print("Volume increased")

def decrease_volume():
    """Decrease system volume"""

    global volume_muted
    volume_muted = False
    subprocess.run(['amixer', '-D', 'pulse', 'set', 'Master', 'unmute'])

    subprocess.run(['amixer', '-D', 'pulse', 'set', 'Master', '5%-'])
    print("Volume decreased")

def set_volume(volume):
    """Set system volume to a specific value"""
    subprocess.run(['amixer', '-D', 'pulse', 'set', 'Master', str(volume)+'%'])
    print(f"Volume set to {volume}%")

def handle_data(data):
    """Process a command received as a string."""
    command = data.split(" ")[0]
    value = data.split(" ")[1] if len(data.split(" ")) > 1 else None

    print(f"Received Command: {data}")

    # Check if command is valid
    if command == "Toggle-Volume":
        toggle_volume()
    elif command == "Toggle-Microphone":
        toggle_microphone()
    elif command == "Increase-Volume":
        increase_volume()
    elif command == "Decrease-Volume":
        decrease_volume()
    elif command == "Set-Volume":
        set_volume(int(value))
    elif command == "IMU":
        global dataIMUx
        global dataIMUy
        global dataIMUz
        dataIMUx.append(float(value.split(",")[0]))
        dataIMUy.append(float(value.split(",")[1]))
        dataIMUz.append(float(value.split(",")[2]))
        print(f"IMU Data: {value}")
    else:
        print(f"Invalid Command: {data}")

def main():
    global connection_type
    
    # Set connection type
    if len(sys.argv) > 1:
        connection_type = sys.argv[1]
        print(f"Connection type: {connection_type}")

    setup()
    
    try:
        if connection_type == "serial":
            # Setup serial port
            connection = setupSerial()
        elif connection_type == "bt":
            # Setup TCP server
            connection = setupBluetooth()

        # Main loop
        while True:
            try:
                while True:
                    data = connection.readline().decode('utf-8').strip()
                    handle_data(data)
            
            except UnicodeDecodeError as e:
                print(f"Error decoding serial data: {e}")
                continue
            except serial.SerialException as e:
                print(f"Serial/Bluetooth port error: {e}")
                break

    except serial.SerialException as e:
        print(f"Error opening serial/bluetooth port: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
        # plotIMUData()
    finally:
        if 'connection' in locals() and connection.is_open:
            connection.close()
            print("Serial/Bluetooth port closed")

if __name__ == "__main__":
    main()