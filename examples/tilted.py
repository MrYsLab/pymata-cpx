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


class Tilted:
    """
    Tilt the playground express and see its position written to the
    console. The position will be written only when it changes.
    """

    def __init__(self):
        # instantiate pymatacpx
        self.p = PyMataCpx()

        # start the accelerometer
        self.p.cpx_accel_start(self.accel_callback)

        # create a variable to remember the last position
        self.last_position = 'flat'

        print()
        print('Move the circuit playground express and watch its position')
        print('reported to the console.')

        while True:
            # just kill time waiting for a light data to arrive
            try:
                time.sleep(.001)
            except KeyboardInterrupt:
                # If you press control-C, cleanly exit
                self.p.cpx_close_and_exit()

    def accel_callback(self, data):
        """
        Take the raw xyz data and transform it to
        positional strings.
        :param data: data [0] = data mode 32 is analog.
                     data[1] = the pin number - this is a pseudo pin number
                     data[2] = x value
                     data[3] = y value
                     data[4] = z value
        """
        x = data[2]
        y = data[3]
        z = data[4]

        # Convert raw Accelerometer values to degrees
        x_angle = (math.atan2(y, z) + math.pi) * (180 / math.pi)
        y_angle = (math.atan2(z, x) + math.pi) * (180 / math.pi)

        position = ""

        if 175 < x_angle < 185:
            position = 'flat'
        elif 186 < x_angle < 360:
            position = 'up'
        elif 90 < x_angle < 185:
            position = 'down'

        if position != 'flat':
            if 275 < y_angle < 360:
                position = position + ' right'

            elif 180 < y_angle < 270:
                position = position + ' left'

        if self.last_position != position:
            self.last_position = position
            print(position)


Tilted()
