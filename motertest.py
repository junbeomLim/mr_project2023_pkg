from dynamixel_sdk import *  # Dynamixel SDK library

# Protocol version
PROTOCOL_VERSION = 1.0

# Default setting
DXL1_ID = 1  # Dynamixel ID: 1
DXL2_ID = 2  # Dynamixel ID: 2
DXL3_ID = 3  # Dynamixel ID: 2
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

# Control table addresses
ADDR_AX_TORQUE_ENABLE = 24
ADDR_AX_GOAL_POSITION = 30
ADDR_AX_MOVING_SPEED = 32
DXL_MOVING_STATUS_THRESHOLD = 10

# Speed for maintaining position
SPEED_FOR_MAINTENANCE = 0

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baud rate
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set the baudrate")
    quit()

# Enable torque for all Dynamixels
for dxl_id in [DXL1_ID, DXL2_ID, DXL3_ID]:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_AX_TORQUE_ENABLE, 1)

# Move Dynamixel#2 to position 240 and set its speed to 0 to maintain the position
packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_GOAL_POSITION, 240)
packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_MOVING_SPEED, SPEED_FOR_MAINTENANCE)

def move_robotarm(dxl_goal_position, speed):
    # Move Dynamixel#1 to position and Dynamixel#2 to position
    packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_AX_GOAL_POSITION, dxl_goal_position)
    packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_AX_MOVING_SPEED, speed)
    packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_GOAL_POSITION, 1023-dxl_goal_position)
    packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_MOVING_SPEED, speed)
    packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_AX_GOAL_POSITION, 240)

    # Wait for Dynamixel#1 and Dynamixel#2 to reach their goal positions
    while True:
        if abs(packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_AX_GOAL_POSITION)[0] - dxl_goal_position) <= DXL_MOVING_STATUS_THRESHOLD and \
            abs(packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_GOAL_POSITION)[0] - (1023-dxl_goal_position)) <= DXL_MOVING_STATUS_THRESHOLD and \
            abs(packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_AX_GOAL_POSITION)[0] - 240) <= DXL_MOVING_STATUS_THRESHOLD:
            break

def move_gripper(dxl3_goal_position):
    # Move Dynamixel#3 to position
    packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_AX_GOAL_POSITION, dxl3_goal_position)
    # Wait for Dynamixel#1 and Dynamixel#3 to reach their goal positions
    while True:
        if abs(packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_AX_GOAL_POSITION)[0] - dxl3_goal_position) <= DXL_MOVING_STATUS_THRESHOLD:
            break

# Disable torque for all Dynamixels
for dxl_id in [DXL1_ID, DXL2_ID, DXL3_ID]:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_AX_TORQUE_ENABLE, 0)

move_robotarm(200,1000)

user_input = input('open: o, close: c, end: q :')
while user_input != 1:
    if user_input == 'o' or user_input == 'O':
        move_gripper(20)
    elif user_input == 'c' or user_input == 'C':
        move_gripper(240)
    elif user_input == 'q' or user_input == 'Q':
        move_gripper(240)
        break
    user_input = input('open: o, close: c, end: q :')

move_robotarm(580, 1000)
time.sleep(0.2)
move_gripper(20)
move_robotarm(200, 1000)
# Close port
portHandler.closePort()
