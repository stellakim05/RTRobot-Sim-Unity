#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys, select, termios, tty

def getKey():
    """Function to get a single key press."""
    tty.setraw(sys.stdin.fileno())
    # Bug fix: Need timeout parameter for select
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # Added timeout
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def handleIJKLKeys(key):
    """Handle IJLK keys for quadrotor movement control."""
    global x, y, z
    # Bug fix: Key sequences need proper handling
    if len(key) == 1:  # Single-character keys
        if key == 'i':  # Move forward
            x = speed
            y = 0
            z = 0
            print("Moving forward")
        elif key == 'k':  # Move backward
            x = -speed
            y = 0
            z = 0
            print("Moving backward")
        elif key == 'j':  # Move left
            x = 0
            y = -speed
            z = 0
            print("Moving left")
        elif key == 'l':  # Move right
            x = 0
            y = speed
            z = 0
            print("Moving right")
        else:
            x = 0
            y = 0
            z = 0
            print("Invalid key. Use i/j/k/l for control.")


def handleVerticalKeys(key):
    """Handle 'w' and 's' keys for vertical movement control (up/down)."""
    global z
    if key == 'w':
        z = speed
        print("Moving up")
    elif key == 's':
        z = -speed
        print("Moving down")
    else:  # Added: Reset z when no vertical movement key is pressed
        z = 0

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('quadrotor_teleop')  # Changed name to be more descriptive
    
    # Added namespace parameter for flexibility
    namespace = rospy.get_param("~namespace", "/quadrotor/")
    cmd_vel_topic = namespace + "cmd_vel"
    feedback_topic = namespace + "movement_feedback"
    
    pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
    unity_feedback_sub = rospy.Subscriber(feedback_topic, Bool,
                                        lambda msg: rospy.loginfo(f"Unity Feedback: {'Moving' if msg.data else 'Stopped'}"))

    # Added more configuration parameters
    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 1.0)
    update_rate = rospy.get_param("~update_rate", 10)  # Hz

    x = y = z = th = 0
    moving = False
    
    try:
        print(f"Quadrotor Teleop Control")
        print(f"Publishing to: {cmd_vel_topic}")
        print("Use arrow keys for movement, 'w'/'s' for up/down")
        print("Press Enter to toggle movement, Ctrl-C to exit")
        
        rate = rospy.Rate(update_rate)
        
        while not rospy.is_shutdown():
            key = getKey()
            
            if key == '\x03':  # Added: Ctrl-C handling
                break
                
            if key == '\r':
                moving = not moving
                if not moving:
                    x = y = z = th = 0  # Reset all velocities when stopping
                print("Movement:", "Enabled" if moving else "Disabled")

            if moving:
                handleIJKLKeys(key)
                handleVerticalKeys(key)
                
                twist = Twist()
                twist.linear.x = x
                twist.linear.y = y
                twist.linear.z = z
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = th
                
                pub.publish(twist)
                
            rate.sleep()

    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        # Send stop command and restore terminal
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)