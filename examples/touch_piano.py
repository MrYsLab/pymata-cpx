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


class TouchPiano:
    """
    As you touch one of the touch inputs, A1 - A7, a note will be played.
    """

    def __init__(self):
        # 7 musical note frequencies - starting at middle c
        self.the_notes = [440, 493, 523, 587, 659, 698, 784]

        # instantiate pymatacpx
        self.p = PyMataCpx()

        # turn on all touch pads 1-7
        for touch_pad in range(1, 8):
            self.p.cpx_cap_touch_start(touch_pad, self.play_notes)
        # self.p.cpx_cap_touch_start(4, self.play_notes)

        self.max = 0

        print()
        print('Touch one of the touch pads, A1 - A7, and a note will play.')

        while True:
            # just kill time waiting for a light data to arrive
            try:
                time.sleep(.001)
            except KeyboardInterrupt:
                # If you press control-C, cleanly exit
                self.p.cpx_close_and_exit()

    def play_notes(self, data):
        # determine if the pad was touched or released
        if data[2]:
            # get the note
            try:
                note = self.the_notes[data[1] - 1]
            except IndexError:
                print(data[1])
                raise

            # play the note
            self.p.cpx_tone(note, 0)

        else:
            # touch pad released - turn off the note
            self.p.cpx_tone_off()


TouchPiano()
