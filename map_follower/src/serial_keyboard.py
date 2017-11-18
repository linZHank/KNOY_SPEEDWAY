#!/usr/bin/env python
from __future__ import print_function

import rospy
import serial
import sys
import termios
import tty
from select import select

# Enable serial 
console_ser = serial.Serial(port = "/dev/ttyACM0", baudrate=115200)

def getch(timeout=0.01):
    '''Retrieves a character from stdin'''
    if not sys.stdin.isatty():
        return sys.stdin.read(1)
    fileno = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fileno)
    ch = None
    try:
        tty.setraw(fileno)
        rlist = [fileno]
        if timeout >= 0:
            [rlist, _, _] = select(rlist, [], [], timeout)
        elif fileno in rlist:
            ch = sys.stdin.read(1)
    except Exception as ex:
        print("getch", ex)
        raise OSError
    finally:
        termios.tcsetattr(fileno, termios.TCSADRAIN, old_settings)
    return ch


def set_ser_cmd(ser_port, ser_str):
    ser_port.write(ser_str)

def map_keyboard():
    ser_str = ('A-0128+0000', 'A+0000+0000', 'A+0128+0000', 'A+0000+0128')
    bindings = {
        # key: (function, args, description)
        'a': (set_ser_cmd, [console_ser, ser_str[0]], 'turn left'),
        's': (set_ser_cmd, [console_ser, ser_str[1]], 'stop'),
        'd': (set_ser_cmd, [console_ser, ser_str[2]], 'turn right'),
        'w': (set_ser_cmd, [console_ser, ser_str[3]], 'moving forward'),
    }
    done = False
    while not done and not rospy.is_shutdown():
        c = getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown('Example finished.')
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "set_ser_cmd(ser_port, cmd)"
                cmd[0](*cmd[1])
                print('command: %s' % (cmd[2],))
            else:
                print('key bingdings: ')

def main():
    '''Using keyboard combinations"i, j, k, l" to remote control rallycar
    through serial port'''
    print('Initializing node...')
    rospy.init_node('rallycar_keyboard')
    print('Use keyboard to control your rallycar')
    map_keyboard()
       
if __name__=='__main__':
    main()
