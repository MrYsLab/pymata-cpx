"""
 Copyright (c) 2015-2019 Alan Yorinks All rights reserved.

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

import logging
import threading
import time
import sys
import serial
from serial.tools import list_ports

logger = logging.getLogger(__name__)


class PyMataCpxSerial(threading.Thread):
    """
     This class manages the serial port for Arduino serial communications
    """

    # class variables
    # arduino = serial.Serial()

    def __init__(self, pymata, com_port):
        """
        Constructor:
        :param pymata: the pymata instance

        """
        self.pymata = pymata
        self.command_deque = self.pymata._command_deque

        self.baud_rate = 115200
        self.timeout = 1
        self.com_port = None
        the_ports_list = list_ports.comports()

        if com_port:
            self.com_port = com_port
            print('Using user specified com_port:', com_port)

        else:
            for port in the_ports_list:
                try:
                    if '239A:8018' in port.hwid:
                        self.com_port = port.device
                        if self.pymata.verbose:
                            print('CPx Found on port: ', port.device)
                        else:
                            logger.info('CPx Found on port: ', port.device)
                        break
                except TypeError:
                    continue
            if self.com_port is None:
                if self.pymata.verbose:
                    print('CPE Not Found')
                else:
                    logger.warning('CPE Not Found')

                if self.pymata.exit_on_exception:
                    sys.exit(0)


        self.cpx = serial.Serial(self.com_port, 115200,
                                 timeout=1, writeTimeout=0)

        threading.Thread.__init__(self)
        self.daemon = True
        self.stop_event = threading.Event()

    def stop(self):
        self.stop_event.set()

    def is_stopped(self):
        return self.stop_event.is_set()

    def close(self):
        """
            Close the serial port
            return: None
        """
        try:
            self.cpx.close()
        except OSError:
            pass

    def write(self, data):
        """
            write the data to the serial port
            return: None
        """
        if sys.version_info[0] < 3:
            self.cpx.write(data)
        else:
            self.cpx.write(bytes([ord(data)]))

    # noinspection PyExceptClausesOrder
    def run(self):
        """
        This method continually runs. If an incoming character is available on the serial port
        it is read and placed on the _command_deque
        @return: Never Returns
        """
        while not self.is_stopped():
            # we can get an OSError: [Errno9] Bad file descriptor when shutting down
            # just ignore it
            try:
                if self.cpx.inWaiting():
                    c = self.cpx.read()
                    self.command_deque.append(ord(c))
                else:
                    try:
                        time.sleep(.001)
                    except KeyboardInterrupt:
                        self.pymata.cpx_reset()

            except OSError:
                pass
            except IOError:
                self.stop()
        self.close()
