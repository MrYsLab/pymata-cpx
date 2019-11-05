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

import time

from pymata_cpx.pymata_cpx import PyMataCpx


class TheServoDemo():
    """
    Attach a servo to pin A3. If the slide switch is set to the
    left the servo goes to 0 degrees and to the right, it goes to 180 degrees.
    """

    def __init__(self):
        self.p = PyMataCpx()

        print('Set the slide switch to the left and the servo will move to 0 degrees.')
        print('Set the slide switch to the right and the servo will move to 180 degrees.')

        self.p.cpx_slide_switch_start(self.switch_changed)

        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                # If you press control-C, cleanly exit
                self.p.cpx_pixels_clear()
                self.p.cpx_pixels_show()
                self.p.cpx_close_and_exit()

    def switch_changed(self, data):
        """
        Turn off the light show and exit
        :param data: data[0] = 32 indicating this is digital data
                     data[1] = pin attached slide switch
                     data[2] = 1 = left, 0 = right
        """
        if data[2]:
            self.p.cpx_set_servo_angle(0)
        else:
            self.p.cpx_set_servo_angle(180)


TheServoDemo()
