#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from pcan.msg import Twist1

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w
   a    s    d
        .

'.' : stop
o - Low Beam Light
p - High Beam Light
j - Wiper OFF
k - Wiper Intermittent
l - Wiper High

CTRL-C to quit
"""
moveBindings = {
        'w':(1,0,0),
        'a':(0,-1,0),
        's':(-1,0,0),
        'd':(0,1,0),
        '.':(0,0,0),
        'o':(0,0,-1),
        'p':(0,0,1)
          }
accessories = {
        'j': -1,
        'k':  0,
        'l':  1
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _=select.select([sys.stdin], [], [], 0.1)
    if rlist:
       key = sys.stdin.read(1)
    else:
       key=0
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist1, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print msg
        while(1):
            
            key = getKey()
            print "im here", key

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]

            else:
                x = 0
                y = 0
                z = 0
                if (key == '\x03'):
                    break

            if key in accessories.keys():
                wiper = accessories[key]
                #print wiper

            else:
                wiper = -1
                if (key == '\x03'):
                    break


            twist = Twist1()
            print x,y,z
            twist.linear.x = x; twist.linear.y = y; twist.linear.z = z;
            twist.wiper = wiper
            pub.publish(twist)


    finally:
        twist = Twist1()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
