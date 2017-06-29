#!/usr/bin/env python
import roslib; #roslib.load_manifest('teleop_twist_keyboard')
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
        'w':'w',
        'a':'a',
        's':'s',
        'd':'d',
        ' ':' ',
        'o':'o',
        'p':'p',
        'q':'q',
        'i':'i',
        'r':'r',
        'n':'n',
        'f':'f',
        '1':'1',
        '2':'2',
        '3':'3',
        '4':'4',
        '5':'5',
        '6':'6',
        '7':'7',
        '8':'8',
        '9':'9'
          }
key_print_bindings={
        'w':'w',
        'a':'a',
        's':'s',
        'd':'d',
        ' ':' ',
        'o':'o',
        'p':'p',
        'i':'i',
        'r':'r',
        'n':'n',
        'f':'f',
        '1':'1',
        '2':'2',
        '3':'3',
        '4':'4',
        '5':'5',
        '6':'6',
        '7':'7',
        '8':'8',
        '9':'9'
        
    
}
accessories = {
        'j': 'j',
        'k': 'k',
        'l': 'l'
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _=select.select([sys.stdin], [], [], 0.1)
    if rlist:
       key = sys.stdin.read(1)
    else:
       key='q'
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('can_msg', Twist1, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    x = 0


    try:
        print msg
        while(1):
            
            key = getKey()
            #print key
            if key in moveBindings.keys():
                x = moveBindings[key]
            if key in key_print_bindings.keys():
                print key
            else:
                x = '0'
                if (key == '\x03'):
                    break

            if key in accessories.keys():
                wiper = accessories[key]
                #print wiper

            else:
                wiper = '1'
                if (key == '\x03'):
                    break


            twist = Twist1()
            twist.key=x;
            twist.wip = wiper
            pub.publish(twist)


    finally:
        twist = Twist1()
        twist.key='0'
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
