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
import math
import time

class LightMeter:
    """
    This program is implemented as a class. Using the light sensor,
    illuminate pixels based on the current light sensor reading.
    As the light sensor values increase, the pixels will be lit
    in a clockwise fashion.
    """
    def __init__(self):
        # a dictionary of pixels as the key and
        # associated rgb color values
        self.pix_d = {
            0: [153, 76, 0],
            1: [153, 0, 0],
            2: [153, 153, 0],
            3: [204, 204, 0],
            4: [0, 153, 0],
            5: [0, 153, 153],
            6: [0, 0, 233],
            7: [153, 0, 153],
            8: [255, 0, 255],
            9: [0, 0, 0],
        }

        # save the previous pixel number that was illuminated
        self.last_pixel_used = 0

        # instantiate pymatacpx
        self.p = PyMataCpx()

        # clear the pixels before monitoring the light
        # sensor values
        self.p.cpx_pixels_clear()
        # set pixel 0 to be the initial pixel and set its rgb
        # from the dictionary
        self.p.cpx_pixel_set(0, *self.pix_d[0])

        self.p.cpx_pixels_show()

        # enable the light sensor and provide a callback.
        # Note: the callback is a member of this class.
        self.p.cpx_light_sensor_start(self.light_sensor_callback)

        while True:
            # just kill time waiting for a light data to arrive
            try:
                time.sleep(.1)
            except KeyboardInterrupt:
                # If you press control-C, cleanly exit
                self.p.cpx_close_and_exit()

    # This is the callback to process light sensor data
    def light_sensor_callback(self, data):
        """
        Light sensor data processor
        :param data: data[2] contains the current light sensor reading
        :return:
        """
        # get the currently reported level
        level = data[2]

        # the level is in the range of 0-1000.
        # adjust the level to map into a valid pixel number
        if level in range(0, 99):
            pixel = 0
        elif level in range(100, 199):
            pixel = 1
        elif level in range(200, 299):
            pixel = 2
        elif level in range(300, 399):
            pixel = 3
        elif level in range(400, 499):
            pixel = 4
        elif level in range(500, 599):
            pixel = 5
        elif level in range(600, 699):
            pixel = 6
        elif level in range(700, 799):
            pixel = 7
        elif level in range(800, 899):
            pixel = 8
        else:
            pixel = 9

        # get the rgb value for the pixel
        rgb = self.pix_d[pixel]

        # if this is not the same pixel as the last one enabled
        # then manipulate the pixels.

        # turn off the current pixel and turn on the new one.
        if pixel != self.last_pixel_used:
            # extinguish the previous pixel
            self.p.cpx_pixel_set(self.last_pixel_used, 0, 0, 0)

            # set the new pixel
            self.p.cpx_pixel_set(pixel, *rgb)

            # control the pixels
            self.p.cpx_pixels_show()
            self.last_pixel_used = pixel

# start the program by instantiating the class
LightMeter()
