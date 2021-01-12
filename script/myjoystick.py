#!/usr/bin/env python

#  Copyright (c) 2017 Jon Cooper
#
#  This file is part of pygame-xbox360controller.
#  Documentation, related files, and licensing can be found at
#
#      <https://github.com/joncoop/pygame-xbox360controller>.


import pygame
import rospy
from sensor_msgs.msg import Joy
import time

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputing the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 30)

    def text_print(self, screen, textString):
        textBitmap = self.font.render(textString, True, WHITE)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 30

    def indent(self):
        self.x += 20

    def unindent(self):
        self.x -= 20


pygame.init()

# Set the width and height of the screen [width, height]
size = [500, 800]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Joystick Tester")
REFRESH_RATE = 60

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()

# Get ready to print
textPrint = TextPrint()

# Game Loop
done = False

# init ROS node
pub = rospy.Publisher('joy', Joy, queue_size=1)
rospy.init_node("Joystic")

while done==False:
    try:
        # Event processing
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        # Drawing code
        screen.fill(BLACK)
        textPrint.reset()

        # Get count of joysticks
        joystick_count = pygame.joystick.get_count()

        textPrint.text_print(screen, "Number of joysticks: {}".format(joystick_count) )
        textPrint.indent()

        # For each joystick:
        for i in range(joystick_count):

            joy = Joy()
            t = rospy.Time.from_sec(time.time())
            sec = t.to_sec()
            joy.header.stamp.secs = int(sec)  # in python, sec -> secs
            joy.header.stamp.nsecs = int((sec - int(sec))*100000000)


            joystick = pygame.joystick.Joystick(i)
            joystick.init()

            textPrint.text_print(screen, "Joystick {}".format(i) )
            textPrint.indent()

            # Get the name from the OS for the controller/joystick
            name = joystick.get_name()
            textPrint.text_print(screen, "Joystick name: {}".format(name) )

            # Usually axis run in pairs, up/down for one, and left/right for the other.
            axes = joystick.get_numaxes()
            textPrint.text_print(screen, "Number of axes: {}".format(axes) )
            textPrint.indent()

            for i in range( axes ):
                axis = joystick.get_axis( i )
                textPrint.text_print(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
                joy.axes.append(axis)
            textPrint.unindent()

            buttons = joystick.get_numbuttons()
            textPrint.text_print(screen, "Number of buttons: {}".format(buttons) )
            textPrint.indent()

            for i in range( buttons ):
                button = joystick.get_button( i )
                textPrint.text_print(screen, "Button {:>2} value: {}".format(i,button) )
                joy.buttons.append(button)
            textPrint.unindent()

            # Hat switch. All or nothing for direction, not like joysticks.
            # Value comes back in a tuple.
            hats = joystick.get_numhats()
            textPrint.text_print(screen, "Number of hats: {}".format(hats) )
            textPrint.indent()

            for i in range( hats ):
                hat = joystick.get_hat( i )
                textPrint.text_print(screen, "Hat {} value: {}".format(i, str(hat)) )
            textPrint.unindent()

            textPrint.unindent()


            pub.publish(joy)

            if joy.buttons[7] == 1: # start button of Xbox joycon pressed
                pygame.quit()
                break

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()
        clock.tick(REFRESH_RATE)

    except KeyboardInterrupt:
        pygame.quit ()
        sys.exit()

pygame.quit ()
