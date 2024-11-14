import rospy
import serial
from geometry_msgs.msg import Twist

# Initialize serial communication
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust the port and baud rate as needed

def send_command(linear, angular):
    command = f"{linear},{angular}\n"
    ser.write(command.encode())

def move_up_squares(squares):
    # Assuming each square is 1 unit distance
    linear_speed = 1.0  # Adjust as needed
    duration = squares  # Adjust based on your speed and distance per square
    send_command(linear_speed, 0)
    rospy.sleep(duration)
    send_command(0, 0)

def move_right_squares(squares):
    # Assuming each square is 1 unit distance
    linear_speed = 1.0  # Adjust as needed
    angular_speed = 1.0  # Adjust as needed
    duration = squares  # Adjust based on your speed and distance per square
    send_command(0, angular_speed)
    rospy.sleep(duration)
    send_command(0, 0)

if __name__ == '__main__':
    rospy.init_node('move_robot')
    move_up_squares(3)
    move_right_squares(5)