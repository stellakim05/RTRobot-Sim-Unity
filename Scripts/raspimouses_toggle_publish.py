#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('raspimouse_toggle_publisher')
    pub1 = rospy.Publisher('/raspimouse1/cmd_vel', Twist, queue_size = 1)
    pub2 = rospy.Publisher('/raspimouse2/cmd_vel', Twist, queue_size = 1)
    unity_feedback_sub = rospy.Subscriber('/unity_movement_feedback', Bool, lambda msg: rospy.loginfo(f"Unity Feedback: {'Moving' if msg.data else 'Stopped'}"))

    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    moving = False

    try:
        print(vels(speed,turn))
        print("Press Enter to toggle movement. Press Ctrl-C to exit.")
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            key = getKey()
            if key == '\r':  # Enter key
                moving = not moving
                if moving:
                    x = speed
                    th = 0
                else:
                    x = 0
                    th = 0
                print("ROS: Command sent -", "Moving" if moving else "Stopped")
            elif (key == '\x03'):  # Ctrl+C
                break

            twist = Twist()
            twist.linear.x = x
            twist.linear.y = y
            twist.linear.z = z
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th
            pub1.publish(twist)
            pub2.publish(twist)
            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub1.publish(twist)
        pub2.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)