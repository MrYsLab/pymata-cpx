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

import random
import time

from pymata_cpx.pymata_cpx import PyMataCpx


class TheTapper():
    """
    Illuminate the neopixels in a counter-clockwise fashion with randomly generated colors.
    When you tap the playground express, the neopixels will stop changing and the
    program pauses. Tap again and the neopixels will start again.
    """
    def __init__(self):
        # create an instance of the API
        self.p = PyMataCpx()

        print('Tap the playground express to stop the neopixels from moving.')
        print('Tap again, to start them up')
        print('The tap state will be printed to the console')

        # Start monitoring for tap events and
        # send event notifications to the tapped method.
        self.p.cpx_tap_start(self.tapped)
        self.go = True

        while True:

            try:
                # run the light show
                for neopixel in range(0, 10):
                    if self.go:
                        self.p.cpx_pixels_clear()
                        self.p.cpx_pixels_show()
                        r = random.randint(0, 254)
                        g = random.randint(0, 254)
                        b = random.randint(0, 254)
                        self.p.cpx_pixel_set(neopixel, r, g, b)
                        self.p.cpx_pixels_show()
                        time.sleep(.2)
                    else:
                        self.p.cpx_pixels_clear()
                        self.p.cpx_pixels_show()
                        time.sleep(.001)
            except KeyboardInterrupt:
                # If you press control-C, cleanly exit
                self.p.cpx_pixels_clear()
                self.p.cpx_pixels_show()
                self.p.cpx_close_and_exit()

    def tapped(self, data):
        """
        :param data: data[0] = data type (analog = 2, digital =32)
                     data[1] = pin for device 27
                     data[2] = tap data - list of booleans.
                               First value for 1 tap
                               Second value for 2 taps
        """
        # for any taps, toggle the go flag
        # print out the current tap state
        if data[2] != [False, False]:
            self.go = not self.go
            print(self.go)

# start the program
TheTapper()
