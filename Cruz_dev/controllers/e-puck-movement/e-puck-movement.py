import math, random
from controller import Robot, Motor, Supervisor, DistanceSensor

# Create the Robot instance.
robot = Robot()

# Get the time step.
time_step = int(robot.getBasicTimeStep())

# Get the Supervisor instance
supervisor = Supervisor()

# Create Motor devices.
left_motor = None
right_motor = None

# Global position.
position_x = 0.0
position_z = 0.0

# Set E-puck angular speed in rad/s.
MAX_SPEED = 6.28

# Get the e-puck sensors.
#ps = []
#for i in range(8):
#    ps_name = 'ps{}'.format(i)
#    ps.append(robot.getDevice(ps_name))
#    ps[i].enable(time_step)

# Function to set motor velocity to move forward.
def move_forward():
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)
    
# Function to stop the robot from moving.
def motor_stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
# Function to rotate the robot to the left.
def rotate_left():
    left_motor.setVelocity(-MAX_SPEED/10)
    right_motor.setVelocity(MAX_SPEED/10)
    
# Function to rotate the robot to the right.
def rotate_right():
    left_motor.setVelocity(MAX_SPEED/2)
    right_motor.setVelocity(-MAX_SPEED/2)

# Get a handler to the motors and set target position to infinity (speed control).
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Set target position.
target_x = round(random.uniform(-0.5, 0.5),2)
target_z = round(random.uniform(-0.5, 0.5),2)
print("Target Position: ({}, {})".format(target_x, target_z))

#set the desired tolerance
tolerance = 0.01

# Main loop:
# - Perform simulation steps until Webots is stopping the controller.
while robot.step(time_step) != -1:
    # Get the current position of the e-puck
    position = supervisor.getFromDef("epuck").getPosition()
    current_x = position[0]
    current_z = position[2]
    
    # Calculate the difference between the current position and the target position
    diff_x = target_x - current_x
    diff_z = target_z - current_z
    
    # Calculate the distance to the target position
    distance = ((diff_x **2) +  (diff_z **2)) **0.5
    
    # Stop the e-puck if it has reached the target position
    if distance < tolerance:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        print("Reached destination")
        break
        
    # Calculate the desired direction towards the target position
    desired_direction = math.atan2(diff_z, diff_x)
    
    # calculate the difference between the desired and the current direction of the e-puck
    diff_direction = desired_direction - supervisor.getFromDef("epuck").getOrientation()[3]
    
    # Adjust the difference to the range (-pi, pi)
    if diff_direction > math.pi:
        diff_direction -= 2 * math.pi
    elif diff_direction < -math.pi:
        diff_direction += 2 * math.pi
        
    # Set the motor velocitied to rotate the e-puck towards the desired direction
    if diff_direction > 0:
        rotate_left()
    else:
        rotate_right()
        
    if abs(diff_direction) < 0.007:
        move_forward()
 
    
# Enter here exit cleanup code.
robot.cleanup()
