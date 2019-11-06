"""
 Copyright (c) 2015-2019 Alan Yorinks All rights reserved.

 Portions of this code are Copyright 2016 Adafruit Industries
 Specifically the cp_response_handler

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
import math
import struct
import threading
import time

from .constants import Constants

logger = logging.getLogger(__name__)


class PyMataCpxCommandHandler(threading.Thread):
    """
    This class handles all data interchanges with Firmata
    The receive loop runs in its own thread.

    Messages to be sent to Firmata are queued through a deque to allow for priority
    messages to take precedence. The deque is checked within the receive loop for any
    outgoing messages.

    There is no blocking in either communications direction.

    There is blocking when accessing the data tables through the _data_lock
    """

    # The response tables hold response information for all pins
    # Each table is a table of entries for each pin, which consists of the pin mode, its last value from firmata
    # and a callback function that the user attached to the pin and an internal callback if needed

    # This is a table that stores analog pin modes and data
    # each entry represents ia mode (INPUT or OUTPUT), and its last current value
    # analog_response_table = []

    # This is a table that stores digital pin modes and data
    # each entry represents  its mode  and callback function
    # digital_response_table = []

    # this deque is used by the methods that assemble messages to be sent to Firmata. The deque is filled outside of
    # of the message processing loop and emptied within the loop.
    # command_deque = None

    # firmata version information - saved as a list - [major, minor]
    firmata_version = [None, None, None]

    def __init__(self, pymata):
        """
        constructor for CommandHandler class

        :param pymata: A reference to the pymata instance.

        """
        self.pymata = pymata
        self.command_deque = pymata._command_deque
        self.data_lock = pymata._data_lock

        # create an digital to analog pin map for reporting purposes
        # a0 - a7 for any pin mode
        # key is analog pin number and value is the digital mapping and current mode
        self.dg_pin_map = {12: {'mapped_pin': 0, 'pin_mode': None},
                           6: {'mapped_pin': 1, 'pin_mode': None},
                           9: {'mapped_pin': 2, 'pin_mode': None},
                           10: {'mapped_pin': 3, 'pin_mode': None},
                           3: {'mapped_pin': 4, 'pin_mode': None},
                           2: {'mapped_pin': 5, 'pin_mode': None},
                           0: {'mapped_pin': 6, 'pin_mode': None},
                           1: {'mapped_pin': 7, 'pin_mode': None},
                           }

        # table of methods to handle incoming messages
        self.command_dispatch = {}

        # total number of pins for the discovered board
        self.total_pins_discovered = 30

        # total number of analog pins for the discovered board
        self.number_of_analog_pins_discovered = 12

        self.digital_response_table = []

        self.analog_response_table = []

        self.initialize_response_tables()

        threading.Thread.__init__(self)
        self.daemon = True

        self.stop_event = threading.Event()

    def map_digital_pin_to_analog_pin(self, pin):
        if pin in self.dg_pin_map:
            return self.dg_pin_map[pin]['mapped_pin']
        else:
            raise RuntimeError('Invalid pin number. Must be between 0 and 7.')

    def initialize_response_tables(self):
        # response table indices
        # index 0 = pin type: 32 = digital, 2 = analog
        # index 1 = pin number
        # index 2 = current value
        # index 3 = internal callback association
        # index 4 = external callback association
        # index 5 = debounce time

        self.digital_response_table = [[] for _ in range(self.total_pins_discovered)]
        for entry in range(self.total_pins_discovered):
            self.digital_response_table[entry] = [None, None, 0, None, None, None]

        self.analog_response_table = [[] for _ in range(self.number_of_analog_pins_discovered)]
        for entry in range(self.number_of_analog_pins_discovered):
            self.analog_response_table[entry] = [None, None, 0, None, None, None]

    def stop(self):
        self.stop_event.set()

    def is_stopped(self):
        return self.stop_event.is_set()

    def analog_message(self, data):
        """
        This method handles the incoming analog data message.
        It stores the data value for the pin in the analog response table.
        If a callback function was associated with this pin, the callback function is invoked.
        This method also checks to see if latching was requested for the pin. If the latch criteria was met,
        the latching table is updated. If a latching callback function was provided by the user, a latching
        notification callback message is sent to the user in place of updating the latching table.

        :param data: Message data from Firmata

        :return: No return value.
        """
        with self.data_lock:
            # hold on to the previous value
            previous_value = \
                self.analog_response_table[data[Constants.RESPONSE_TABLE_MODE]][
                    Constants.RESPONSE_TABLE_PREV_DATA_VALUE]
            self.analog_response_table[data[Constants.RESPONSE_TABLE_MODE]][
                Constants.RESPONSE_TABLE_PREV_DATA_VALUE] \
                = (data[Constants.MSB] << 7) + data[Constants.LSB]
            pin = data[Constants.RESPONSE_TABLE_MODE]
            pin_response_data_data = self.analog_response_table[pin]
            value = pin_response_data_data[Constants.RESPONSE_TABLE_PREV_DATA_VALUE]
            # check to see if there is a callback function attached to this pin
            if self.analog_response_table[data[Constants.RESPONSE_TABLE_MODE]][
                    Constants.RESPONSE_TABLE_CALLBACK_INTERNAL]:
                callback = self.analog_response_table[data[Constants.RESPONSE_TABLE_MODE]][
                    Constants.RESPONSE_TABLE_CALLBACK_INTERNAL]
            else:
                callback = self.analog_response_table[data[Constants.RESPONSE_TABLE_MODE]][
                    Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL]
        # send the pin mode, pin number, and current data value
        if callback is not None:
            if value != previous_value:
                # has the value changed since the last report
                callback([Constants.ANALOG, pin, value])

    def digital_message(self, data):
        """
        This method handles the incoming digital message.
        It stores the data values in the digital response table.
        Data is stored for all 8 bits of a  digital port

        :param data: Message data from Firmata

        :return: No return value.
        """
        callback = None
        port = data[0]
        port_data = (data[Constants.MSB] << 7) + data[Constants.LSB]

        # set all the pins for this reporting port
        # get the first pin number for this report
        pin = port * 8
        for pin in range(pin, min(pin + 8, self.total_pins_discovered)):
            # shift through all the bit positions and set the digital response table
            with self.data_lock:
                # look at the previously stored value for this pin
                prev_data = self.digital_response_table[pin][Constants.RESPONSE_TABLE_PREV_DATA_VALUE]
                # get the current value
                self.digital_response_table[pin][Constants.RESPONSE_TABLE_PREV_DATA_VALUE] = port_data & 0x01
                # if the values differ and callback is enabled for the pin, then send out the callback
                if prev_data != port_data & 0x01:
                    if self.digital_response_table[pin][Constants.RESPONSE_TABLE_CALLBACK_INTERNAL]:
                        callback = self.digital_response_table[pin][Constants.RESPONSE_TABLE_CALLBACK_INTERNAL]
                    else:
                        callback = self.digital_response_table[pin][Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL]

                if callback:
                    callback([Constants.DIGITAL, pin,
                              self.digital_response_table[pin][Constants.RESPONSE_TABLE_PREV_DATA_VALUE]])
                    callback = None
                    if self.digital_response_table[pin][Constants.RESPONSE_TABLE_DEBOUNCE_TIME]:
                        time.sleep(self.digital_response_table[pin][Constants.RESPONSE_TABLE_DEBOUNCE_TIME])

            # get the next data bit
            port_data >>= 1

    def send_sysex(self, sysex_command, sysex_data=None):
        """
        This method will send a Sysex command to Firmata with any accompanying data

        :param sysex_command: sysex command

        :param sysex_data: data for command

        :return : No return value.
        """
        if not sysex_data:
            sysex_data = []

        # convert the message command and data to characters
        sysex_message = chr(Constants.START_SYSEX)
        sysex_message += chr(sysex_command)
        if len(sysex_data):
            for d in sysex_data:
                sysex_message += chr(d)
        sysex_message += chr(Constants.END_SYSEX)

        for data in sysex_message:
            self.pymata.transport.write(data)

    def send_command(self, command):
        """
        This method is used to transmit a non-sysex command.

        :param command: Command to send to firmata includes command + data formatted by caller

        :return : No return value.
        """
        send_message = ""
        for i in command:
            send_message += chr(i)

        for data in send_message:
            self.pymata.transport.write(data)

    def system_reset(self):
        """
        Send the reset command to the Arduino.
        It resets the response tables to their initial values

        :return: No return value
        """
        data = chr(Constants.SYSTEM_RESET)
        self.pymata.transport.write(data)

        # response table re-initialization
        # for each pin set the mode to input and the last read data value to zero
        # with self.lock:
        # remove all old entries from existing tables
        # for _ in range(len(self.digital_response_table)):
        # self.digital_response_table.pop()

        # for _ in range(len(self.analog_response_table)):
        #     self.analog_response_table.pop()

        # reinitialize tables
        self.initialize_response_tables()

    # noinspection PyMethodMayBeStatic
    # keeps pycharm happy
    def _string_data(self, data):
        """
        This method handles the incoming string data message from Firmata.
        The string is printed to the console

        :param data: Message data from Firmata

        :return: No return value.s
        """
        print("_string_data:")
        string_to_print = []
        for i in data[::2]:
            string_to_print.append(chr(i))
        print("".join(string_to_print))

    def _therm_handler(self, data):
        """Callback invoked when the thermistor analog input has a new value.
        """
        # Get the raw ADC value and convert to temperature.
        raw = data[2]

        temp_c = self._therm_value_to_temp(raw)
        # Call any user callback
        if self.analog_response_table[0][Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] is not None:
            if self.analog_response_table[0][Constants.RESPONSE_TABLE_PREV_DATA_VALUE] != temp_c:
                self.analog_response_table[0][Constants.RESPONSE_TABLE_PREV_DATA_VALUE] = temp_c
                self.analog_response_table[0][Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL]([2, 0, temp_c])

    def _therm_value_to_temp(self, adc_value):
        """
        Convert a thermistor ADC value to a temperature in Celsius.
        Use Steinhart-Hart thermistor equation to convert thermistor resistance to
        temperature.  See: https://learn.adafruit.com/thermistor/overview
        Handle a zero value which has no meaning (and would cause a divide by zero).

        :param thermistor adc_value:
        :return:
        """

        if adc_value:
            # First calculate the resistance of the thermistor based on its ADC value.
            resistance = ((1023.0 * Constants.THERM_SERIES_OHMS) / adc_value)
            resistance -= Constants.THERM_SERIES_OHMS
            # Now apply Steinhart-Hart equation.
            steinhart = resistance / Constants.THERM_NOMINAL_OHMS
            steinhart = math.log(steinhart)
            steinhart /= Constants.THERM_BETA
            steinhart += 1.0 / (Constants.THERM_NOMINAL_C + 273.15)
            steinhart = 1.0 / steinhart
            steinhart -= 273.15
        else:
            steinhart = 0.0
        return steinhart

    def cp_response_handler(self, data):
        """Callback invoked when a circuit playground sysex command is received.
        """
        # logger.debug('CP response: 0x{0}'.format(hexlify(bytearray(data))))
        if len(data) < 1:
            logger.warning('Received response with no data!')
            return
        # Check what type of response has been received.
        command = data[0] & 0x7F
        if command == Constants.CP_ACCEL_READ_REPLY:
            # Parse accelerometer response.
            if len(data) < 26:
                logger.warning('Received accelerometer response with not enough data.')
                return
            x = self._parse_firmata_float(data[2:10])
            y = self._parse_firmata_float(data[10:18])
            z = self._parse_firmata_float(data[18:26])
            if self.digital_response_table[Constants.ACCEL_PSEUDO_PIN][
                    Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] is not None:
                self.digital_response_table[Constants.ACCEL_PSEUDO_PIN][
                    Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL]([Constants.DIGITAL,
                                                                 Constants.ACCEL_PSEUDO_PIN,
                                                                 x, y, z])
        elif command == Constants.CP_ACCEL_TAP_REPLY:
            # Parse accelerometer tap response.
            if len(data) < 4:
                logger.warning('Received tap response with not enough data!')
                return
            tap = self._parse_firmata_byte(data[2:4])
            if self.digital_response_table[Constants.ACCEL_TAP_PSEUDO_PIN][
                    Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] is not None:
                taps = list(self._tap_register_to_clicks(tap))
                if self.digital_response_table[Constants.ACCEL_TAP_PSEUDO_PIN][
                    Constants.RESPONSE_TABLE_PREV_DATA_VALUE] != taps:
                    self.digital_response_table[Constants.ACCEL_TAP_PSEUDO_PIN][
                        Constants.RESPONSE_TABLE_PREV_DATA_VALUE] = taps
                    self.digital_response_table[Constants.ACCEL_TAP_PSEUDO_PIN][
                        Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL](
                        [Constants.DIGITAL, Constants.ACCEL_TAP_PSEUDO_PIN, taps])
        elif command == Constants.CP_CAP_REPLY:
            # Parse capacitive sensor response.
            if len(data) < 12:
                logger.warning('Received cap touch response with not enough data!')
                return
            input_pin = self._parse_firmata_byte(data[2:4])
            value = self._parse_firmata_long(data[4:12])
            pin = self.map_digital_pin_to_analog_pin(input_pin)

            if value > Constants.CAP_THRESHOLD:
                response = True
            else:
                response = False

            if response != self.analog_response_table[pin][Constants.RESPONSE_TABLE_PREV_DATA_VALUE]:
                self.analog_response_table[pin][Constants.RESPONSE_TABLE_PREV_DATA_VALUE] = response
                self.analog_response_table[pin][Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL](
                    [Constants.ANALOG, pin, response, value])

            time.sleep(.001)

        elif command == Constants.CP_IMPL_VERS_REPLY:
            # Parse implementation version response.
            if len(data) < 8:
                logger.warning('Received color sense response with not enough data!')
                return
            # Parse out the maj, min, and fix
            month = self._parse_firmata_byte(data[2:4])
            day = self._parse_firmata_byte(data[4:6])
            year = self._parse_firmata_byte(data[6:8])
            # if self.implementation_version_callback is not None:
            #     self.implementation_version_callback(major, minor, bugfix)
            self.firmata_version[0] = day
            self.firmata_version[1] = month
            self.firmata_version[2] = year
        else:
            logger.warning('Received unexpected response!')

    def _tap_register_to_clicks(self, register):
        """Convert accelerometer tap register value to booleans that indicate
        if a single and/or double tap have been detected.  Returns a tuple
        of bools with single click boolean and double click boolean.
        """
        single = False
        double = False
        # Check if there is a good tap register value and check the single and
        # double tap bits to set the appropriate bools.
        if register & 0x30 > 0:
            single = True if register & 0x10 > 0 else False
            double = True if register & 0x20 > 0 else False
        return single, double

    def _parse_firmata_byte(self, data):
        """Parse a byte value from two 7-bit byte firmata response bytes."""
        if len(data) != 2:
            raise ValueError('Expected 2 bytes of firmata response for a byte value!')
        return (data[0] & 0x7F) | ((data[1] & 0x01) << 7)

    def _parse_firmata_float(self, data):
        """Parse a 4 byte floating point value from a 7-bit byte firmata response
        byte array.  Each pair of firmata 7-bit response bytes represents a single
        byte of float data so there should be 8 firmata response bytes total.
        """
        if len(data) != 8:
            raise ValueError('Expected 8 bytes of firmata response for floating point value!')
        # Convert 2 7-bit bytes in little endian format to 1 8-bit byte for each
        # of the four floating point bytes.
        raw_bytes = bytearray(4)
        for i in range(4):
            raw_bytes[i] = self._parse_firmata_byte(data[i * 2:i * 2 + 2])
        # Use struct unpack to convert to floating point value.
        return struct.unpack('<f', raw_bytes)[0]

    def _parse_firmata_long(self, data):
        """Parse a 4 byte signed long integer value from a 7-bit byte firmata response
        byte array.  Each pair of firmata 7-bit response bytes represents a single
        byte of long data so there should be 8 firmata response bytes total.
        """
        if len(data) != 8:
            raise ValueError('Expected 8 bytes of firmata response for long value!')
        # Convert 2 7-bit bytes in little endian format to 1 8-bit byte for each
        # of the four floating point bytes.
        raw_bytes = bytearray(4)
        for i in range(4):
            raw_bytes[i] = self._parse_firmata_byte(data[i * 2:i * 2 + 2])
        # Use struct unpack to convert to floating point value.
        return struct.unpack('<l', raw_bytes)[0]

    def run(self):
        """
        This method starts the thread that continuously runs to receive and interpret
        messages coming from Firmata. This must be the last method in this file
        It also checks the deque for messages to be sent to Firmata.
        """
        # To add a command to the command dispatch table, append here.
        self.command_dispatch.update({Constants.ANALOG_MESSAGE: [self.analog_message, 2]})
        self.command_dispatch.update({Constants.DIGITAL_MESSAGE: [self.digital_message, 2]})
        self.command_dispatch.update({Constants.STRING_DATA: [self._string_data, 2]})
        # self.command_dispatch.update({Constants.PIN_STATE_RESPONSE: [self.pin_state_response, 2]})
        self.command_dispatch.update({Constants.CP_COMMAND: [self.cp_response_handler, 2]})

        while not self.is_stopped():
            if len(self.command_deque):
                # get next byte from the deque and process it
                data = self.command_deque.popleft()

                # this list will be populated with the received data for the command
                command_data = []
                # process sysex commands
                if data == Constants.START_SYSEX:
                    # next char is the actual sysex command
                    # wait until we can get data from the deque
                    while len(self.command_deque) == 0:
                        pass
                    sysex_command = self.command_deque.popleft()
                    # retrieve the associated command_dispatch entry for this command
                    dispatch_entry = self.command_dispatch.get(sysex_command)

                    # get a "pointer" to the method that will process this command
                    try:
                        method = dispatch_entry[0]
                    except TypeError:
                        continue

                    # now get the rest of the data excluding the END_SYSEX byte
                    end_of_sysex = False
                    while not end_of_sysex:
                        # wait for more data to arrive
                        while len(self.command_deque) == 0:
                            pass
                        data = self.command_deque.popleft()
                        if data != Constants.END_SYSEX:
                            command_data.append(data)
                        else:
                            end_of_sysex = True

                            # invoke the method to process the command
                            try:
                                method(command_data)
                            except TypeError:
                                continue
                            # go to the beginning of the loop to process the next command
                    continue

                # is this a command byte in the range of 0x80-0xff - these are the non-sysex messages

                elif 0x80 <= data <= 0xff:
                    # look up the method for the command in the command dispatch table
                    # for the digital reporting the command value is modified with port number
                    # the handler needs the port to properly process, so decode that from the command and
                    # place in command_data
                    if 0x90 <= data <= 0x9f:
                        port = data & 0xf
                        command_data.append(port)
                        data = 0x90
                    # the pin number for analog data is embedded in the command so, decode it
                    elif 0xe0 <= data <= 0xef:

                        pin = data & 0xf
                        command_data.append(pin)
                        data = 0xe0
                    else:
                        pass

                    dispatch_entry = self.command_dispatch.get(data)

                    method = dispatch_entry[0]

                    # get the number of parameters that this command provides
                    num_args = dispatch_entry[1]

                    # look at the number of args that the selected method requires
                    # now get that number of bytes to pass to the called method
                    for i in range(num_args):
                        while len(self.command_deque) == 0:
                            pass
                        data = self.command_deque.popleft()
                        command_data.append(data)
                        # go execute the command with the argument list
                    method(command_data)

                    # go to the beginning of the loop to process the next command
                    continue
            else:
                time.sleep(.001)
