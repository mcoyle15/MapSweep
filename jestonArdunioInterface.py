import rospy
import serial
from geometry_msgs.msg import Twist

# Initialize serial communication
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust the port and baud rate as needed

def callback(data):
    # Convert Twist message to motor commands
    linear = data.linear.x
    angular = data.angular.z

    # Create a command string
    command = f"{linear},{angular}\n"
    
    # Send the command to the Arduino
    ser.write(command.encode())

def listener():
    rospy.init_node('motor_command_sender', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass