"""
 Copyright (c) 2019 Alan Yorinks All rights reserved.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
 Version 3 as published by the Free Software Foundation; either
 or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
 along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
"""

from pymata_cpx.pymata_cpx import PyMataCpx
import time

"""
This program monitors the switches.
At start up, pixels 6 and 3 will indicate which side the slide
switch is on.
 
If a switch changes state, a message will be
printed to the console and a pixel also will change
state as follows:

Button A pressed:  pixel 1 = red
Button A released: pixel 1 = off

Button B pressed:  pixel 8 = green
Button A released: pixel 8 = off

Slide switch right: pixel 6 = blue, pixel 3 = off
Slide switch left:  pixel 3 = blue, pixel 6 = off

All switches share a common callback function.

"""

# pin numbers for each switch
button_a = 4
button_b = 19
slide = 21

# index into data in the callback
switch_id = 1
switch_value = 2


# the user provided callback
def buttons_callback(data):
    """

    :param data: data[0] = pin type: 32=digital,
                           2 = analog
                 data[1] = the pin where change detected:
                           4 = Button A
                           19 = Button B
                           21 = Slide Switch
                 data[3] = switch state
    """
    # get the switch pin number
    the_switch = data[switch_id]

    # get the reported value for the switch
    the_switch_value = data[switch_value]

    # handle button a changes
    if the_switch == button_a:
        if the_switch_value:
            print('Button A Pressed')
            # turn pixel one red
            p.cpx_pixel_set(1, 255, 0, 0)
            p.cpx_pixels_show()
        else:
            print('Button A Released')
            # turn pixel 1 off
            p.cpx_pixel_set(1, 0, 0, 0)
            p.cpx_pixels_show()
    # handle button b changes
    elif the_switch == button_b:
        if the_switch_value:
            print('Button B Pressed')
            # turn pixel 8 green
            p.cpx_pixel_set(8, 0, 255, 0)
            p.cpx_pixels_show()
        else:
            print('Button B Released')
            # turn pixel 8 off
            p.cpx_pixel_set(8, 0, 0, 0)
            p.cpx_pixels_show()
    # handle the slide switch:
    elif the_switch == slide:
        if the_switch_value:
            print('Switch on left side')
            # turn pixel 3 blue and pixel 6 off
            p.cpx_pixel_set(3, 0, 0, 255)
            p.cpx_pixel_set(6, 0, 0, 0)
            p.cpx_pixels_show()
        else:
            print('Switch on right side')
            # turn pixel 6 blue and pixel 3 off
            p.cpx_pixel_set(6, 0, 0, 255)
            p.cpx_pixel_set(3, 0, 0, 0)
            p.cpx_pixels_show()
    else:
        print('Unknown switch id: ', the_switch)


# create a cpx instance
p = PyMataCpx()

# enable all switches and attach all to the same callback
p.cpx_button_a_start(buttons_callback)
p.cpx_button_b_start(buttons_callback)
p.cpx_slide_switch_start(buttons_callback)

# Set pixel 6 initially on and 3 off.
# If the switch is set to the left at start up
# it will be detected and pixel 3 will be turned
# on and pixel 6 turned off.
p.cpx_pixel_set(6, 0, 0, 255)
p.cpx_pixel_set(3, 0, 0, 0)
p.cpx_pixels_show()

while True:
    # just kill time waiting for a switch change
    try:
        time.sleep(.1)
    except KeyboardInterrupt:
        # If you press control-C, cleanly exit
        p.cpx_close_and_exit()
