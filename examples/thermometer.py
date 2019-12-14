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


class Thermometer:
    """
    Touch the temperature sensor on the playground express.
    As the temperature increases, pixels will illuminate to
    indicate the temperature.
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
            9: [255, 255, 255],
        }

        # save the previous pixel number that was illuminated
        self.last_pixel_used = 0

        # save the first temperature read as the ambient temperature
        # and use that as the basis of comparison.
        self.ambient = 0

        # instantiate pymatacpx
        self.p = PyMataCpx()

        # clear the pixels before monitoring the light
        # sensor values
        self.p.cpx_pixels_clear()
        # set pixel 0 to be the initial pixel and set its rgb
        # from the dictionary
        self.p.cpx_pixel_set(0, *self.pix_d[0])

        self.p.cpx_pixels_show()

        # enable the temperature sensor and provide a callback.
        self.p.cpx_temperature_start(self.temp_callback)

        print()
        print('Touch the temperature sensor. As the temperature changes')
        print('Different neopixels will light up.')
        print()
        print('The current temperature and pixel selected will be shown on the console.')
        print()

        while True:
            # just kill time waiting for a light data to arrive
            try:
                time.sleep(.1)
            except KeyboardInterrupt:
                # If you press control-C, cleanly exit
                self.p.cpx_close_and_exit()

    # This is the callback to process light sensor data
    def temp_callback(self, data):
        """
        Light sensor data processor
        :param data: data[0] = 2 analog mode
                     data[1] = pin 0
                     data[2] = temp in degrees C
        """
        # the current temperature
        the_current_temperature = data[2]

        # save the temperature the first time
        # through as ambient
        if not self.ambient:
            self.ambient = the_current_temperature

        # select the pixel based on the current temperature
        if self.ambient < the_current_temperature < (self.ambient + .6):
            pixel = 0
        elif (self.ambient + 0.6) < the_current_temperature < (self.ambient + 1.2):
            pixel = 1
        elif (self.ambient < the_current_temperature + 1.2) < the_current_temperature < (self.ambient + 1.8):
            pixel = 2
        elif (self.ambient < the_current_temperature + 1.8) < the_current_temperature < (self.ambient + 2.4):
            pixel = 3
        elif (self.ambient < the_current_temperature + 2.4) < the_current_temperature < (self.ambient + 3.0):
            pixel = 4
        elif (self.ambient < the_current_temperature + 3.0) < the_current_temperature < (self.ambient + 3.6):
            pixel = 5
        elif (self.ambient < the_current_temperature + 3.6) < the_current_temperature < (self.ambient + 4.2):
            pixel = 6
        elif (self.ambient < the_current_temperature + 4.2) < the_current_temperature < (self.ambient + 4.8):
            pixel = 7
        elif (self.ambient < the_current_temperature + 4.8) < the_current_temperature < (self.ambient + 5.4):
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

        print('ambient: {} current: {} pixel{}'.format(round(self.ambient, 3),
                                                       round(the_current_temperature, 3),
                                                       pixel))


# start the program by instantiating the class
Thermometer()
