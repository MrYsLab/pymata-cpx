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


class TheClapper:
    """
    Turn LEDs in a counter-clockwise fashion with randomly generated colors.
    When you clap your hand loudly, the pixels will stop changing and the
    program exits. Clap again and they start moving again.
    """
    def __init__(self):
        self.p = PyMataCpx()

        print('Clap your hands loudly to stop the neopixels from moving.')
        print('Clap again, to start them up')

        self.p.cpx_microphone_start(self.listen)
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

    def listen(self, data):
        """
        Turn off the light show and exit
        :param data: data[0] = 2 indicating this is analog data
                     data[1] = pin attached to mic- pin 4
                     data[3] = readings from microphone
        """
        if data[2] > 600:
            # self.p.cpx_pixels_clear()
            # self.p.cpx_pixels_show()
            self.go = not self.go


TheClapper()
