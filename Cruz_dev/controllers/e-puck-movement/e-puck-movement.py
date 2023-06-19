print("starting")
import pdb
"""e-puck-movement controller."""
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot, Motor
#create time_step
time_step = None

# Create Motor devices
left_motor = None
right_motor = None

# Set E-puck angular speed in rad/s
MAX_SPEED = 6.28

# Function to get the time step.
def get_time_step():
    return int(robot.getBasicTimeStep())

# Function to set motor velocity to move forward
def move_forward():
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)

# Initialize robot
def init_robot():
    global left_motor, right_motor
    # time step
    time_step = get_time_step()
    # Get a handler to the motors and set target position to infinity (speed control)
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

# create the Robot instance.
robot = Robot()

# Initialize the robot
init_robot()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(time_step) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    move_forward()

# Enter here exit cleanup code.
robot.cleanup()