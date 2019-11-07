"""
 Copyright (c) 2019 Alan Yorinks All rights reserved.

 Portions of this code are Copyright 2016 Adafruit Industries


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
import sys
import threading
import time
from collections import deque

try:
    from pymata_cpx_command_handler import PyMataCpxCommandHandler
except ImportError:
    from .pymata_cpx_command_handler import PyMataCpxCommandHandler

try:
    from pymata_cpx_serial import PyMataCpxSerial
except ImportError:
    from .pymata_cpx_serial import PyMataCpxSerial

try:
    from constants import Constants
except ImportError:
    from .constants import Constants

logger = logging.getLogger(__name__)


# noinspection PyPep8
class PyMataCpx(object):
    """
    This class contains the complete set of API methods that permit control of the Circuit
    Playground Express Micro-Controller utilizing Firmata or its derivatives.

    """
    # externally accessible entities that are shared by other components in this package

    _data_lock = threading.RLock()
    _command_deque = deque()

    _accel_usage = 0  # 0 = available, 1 = accel, 2 = tap

    _servo_inuse = False  # False = not yet initialized, True = initialized

    _sonar_configured = False

    # noinspection PyPep8Naming
    def __init__(self, verbose=True, exit_on_exception=True, com_port=None):
        """
        The "constructor" instantiates the entire interface. It starts the operational threads for the serial
        interface as well as for the command handler.

        :param verbose: Set to False to suppress output to screen

        :param exit_on_exception: Set to False to suppress exiting on exception

        :param com_port: User specified com port
        """

        try:

            self.verbose = verbose
            self.exit_on_exception = exit_on_exception

            # create an analog to digital pin map so that analog pin numbers
            # a0 - a7 for any pin mode
            # key is analog pin number and value is the digital mapping and current mode
            self.ad_pin_map = {0: {'mapped_pin': 12, 'pin_mode': None},
                               1: {'mapped_pin': 6, 'pin_mode': None},
                               2: {'mapped_pin': 9, 'pin_mode': None},
                               3: {'mapped_pin': 10, 'pin_mode': None},
                               4: {'mapped_pin': 3, 'pin_mode': None},
                               5: {'mapped_pin': 2, 'pin_mode': None},
                               6: {'mapped_pin': 0, 'pin_mode': None},
                               7: {'mapped_pin': 1, 'pin_mode': None},
                               }
            # each byte represents a digital port and its value contains the current port settings
            self._digital_output_port_pins = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

            self._accel_usage = Constants.ACCEL_USAGE_AVAILABLE

            if verbose:
                print('\npymata-cpx version 1.0  Copyright(C) 2019 Alan Yorinks    All rights reserved.')
            else:
                logger.info('pymata-cpx version 1.0  Copyright(C) 2019 Alan Yorinks    All rights reserved.')

            # Instantiate the serial support class
            self.transport = PyMataCpxSerial(self, com_port)

            # Start the data receive thread
            self.transport.start()

            # Instantiate the command handler
            self._command_handler = PyMataCpxCommandHandler(self)
            # Start the command processing thread

            self._command_handler.start()

            # get firmata version in the CPX sketch to be placed in
            # _command_handler.firmata_version

            self._command_handler.send_sysex(Constants.CP_COMMAND,
                                             [Constants.CP_IMPL_VERS])
            # wait for command to process
            time.sleep(1)

            if self._command_handler.firmata_version is None:
                if verbose:
                    print('Circuit Playground not responding - do you have FirmataCPx installed?')
                else:
                    logger.warning('Circuit Playground not responding - do you have FirmataCPx installed?')
            else:
                if verbose:
                    print('FirmataCPx Release Date: ' + str(self._command_handler.firmata_version[0]) + '-' +
                          str(self._command_handler.firmata_version[1]) + '-' +
                          str(self._command_handler.firmata_version[2]))
                else:
                    logger.info('FirmataCPx Release Date: ' + str(self._command_handler.firmata_version[0]) + '-' +
                                str(self._command_handler.firmata_version[1]) + '-' +
                                str(self._command_handler.firmata_version[2]))

            # reset the board
            self.cpx_reset()

            # initialize response tables since the reset cleared them
            self._command_handler.initialize_response_tables()

        except KeyboardInterrupt:
            if verbose:
                print('Program Aborted Before PyMata Instantiated')
            else:
                logger.info('Program Aborted Before PyMata Instantiated')
            if self.exit_on_exception:
                sys.exit()
            else:
                raise KeyboardInterrupt

    # "public" methods

    def cpx_accel_range_set(self, accel_range=0):
        """
        Set the range of the accelerometer.  Accel_range should be a value of:

          - 0 = +/-2G (default)

          - 1 = +/-4G

          - 2 = +/-8G

          - 3 = +/-16G

        :param accel_range:
        """

        assert accel_range in [0, 1, 2, 3], 'Accel range must be one of 0, 1, 2, 3!'
        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_ACCEL_RANGE, accel_range & 0x7F])

    def cpx_accel_start(self, callback):
        """
        Request to start streaming accelerometer data from the board.  Will
        call the provided callback with accelerometer data. You may not
        use the device in both accelerometer and tap mode at the same time.

        :param callback: a list of values described below.

        Parameters sent to callback:

        [Digital Pin Type: 32, Pin Number: 27, x_value, y_value, z_value]

        """
        assert self._accel_usage == Constants.ACCEL_USAGE_AVAILABLE, 'TAP In Use'
        if self._accel_usage == Constants.ACCEL_USAGE_AVAILABLE:
            self._accel_usage = Constants.ACCEL_USAGE_ACCEL
            with self._data_lock:
                self._command_handler.digital_response_table[Constants.ACCEL_PSEUDO_PIN][
                    Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = callback
            self._command_handler.send_sysex(Constants.CP_COMMAND,
                                             [Constants.CP_ACCEL_STREAM_ON])
        else:
            logger.warning('cpx_start_accel: accelerometer is in use')

    def cpx_accel_stop(self):
        """
        Stop the streaming accelerometer data
        """

        with self._data_lock:
            self._command_handler.analog_response_table[Constants.ACCEL_PSEUDO_PIN][
                Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = None
        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_ACCEL_STREAM_OFF])
        self._accel_usage = Constants.ACCEL_USAGE_AVAILABLE

    def cpx_board_light_off(self):
        """
        Set the board red LED OFF
        """
        self._digital_write(13, 0)

    def cpx_board_light_on(self):
        """
        Set the board red LED ON
        """
        self._digital_write(13, 1)

    def cpx_button_a_start(self, callback, debounce_time=.1):

        """
        Enable button and report state changes in the callback.

        :param callback: a list of values described below.

        :param debounce_time: switch debounce time default is .1 seconds

        Parameters sent to callback:

        [Digital Pin Type: 32, Pin Number: 4, switch value: 1 if pressed zero if released.]
        """
        self._cpx_start_sensor(Constants.CPX_BUTTON_A, Constants.DIGITAL, callback, debounce_time)

    def cpx_button_a_stop(self):
        """
        Disable button a reporting.
        """
        self._cpx_stop_sensor(Constants.CPX_BUTTON_A, Constants.DIGITAL)

    def cpx_button_b_start(self, callback, debounce_time=.1):

        """
        Enable button and report state changes in the callback.

        :param callback: a list of values described below.

        :param debounce_time: switch debounce time default is .1 seconds

        Parameters sent to callback:

        [Digital Pin Type: 32, Pin Number: 19, switch value: 1 if pressed zero if released.]

        """
        self._cpx_start_sensor(Constants.CPX_BUTTON_B,
                               Constants.DIGITAL, callback, debounce_time)

    def cpx_button_b_stop(self):
        """
        Disable button b reporting.
        """
        self._cpx_stop_sensor(Constants.CPX_BUTTON_B, Constants.DIGITAL)

    def cpx_cap_touch_start(self, input_pin, callback):
        """
        Start continuous capacitive touch queries for the specified input
        pin.  Will invoke the provided callback each time a new cap touch result
        is available.

        :param input_pin: must be one of the Ax pin numbers. Use only the number.

                          example: cpx_cap_touch_start(5, a_callback_function)

        :param callback: a list of values described below.

        Parameters sent to callback:

        [Analog Pin Type: 2, Pin Number: 1-7, Touched: True or False, Raw data value]


        """

        assert input_pin in [1, 2, 3, 4, 5, 6, 7], \
            'Input pin must be a capacitive input (1, 2, 3, 4, 5, 6, 7)!'
        # translate aX pin number into internal representation - digital pin number

        pin = self._map_analog_pin_to_digital_pin(input_pin)
        with self._data_lock:
            self._command_handler.analog_response_table[input_pin][
                Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = callback
            self._command_handler.analog_response_table[input_pin][
                Constants.RESPONSE_TABLE_CALLBACK_INTERNAL] = None
        # Construct a continuous cap read start command and send it.
        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_CAP_ON, pin & 0x7F])

    def cpx_cap_touch_stop(self, input_pin):
        """
        Stop continuous capacitive touch queries for the specified input
        pin.

        :param input_pin: Ax where x is 1 to 7. Only the numeral is used.

                          example: cpx_cap_touch_stop(5)
        """

        assert input_pin in [1, 2, 3, 4, 5, 6, 7], \
            'Input pin must be a capacitive input (1, 2, 3, 4, 5, 6, 7)!'
        pin = self.ad_pin_map[input_pin]['mapped_pin']
        # Construct a continuous cap read stop command and send it.
        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_CAP_OFF, pin & 0x7F])
        with self._data_lock:
            self._command_handler.analog_response_table[input_pin][
                Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = None
            self._command_handler.analog_response_table[input_pin][
                Constants.RESPONSE_TABLE_CALLBACK_INTERNAL] = None
        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_CAP_OFF, pin & 0x7F])

    def cpx_close_and_exit(self):
        """
        This method will close and stop the command handler,
        reset the Circuit Playground Express and then
        close the transport (serial port). It will then
        reset the circuit playground and exit.
        """
        self._command_handler.system_reset()
        self._command_handler.stop()
        self.cpx_reset()
        self.transport.stop()
        try:
            self.transport.close()
        except TypeError:
            pass
        sys.exit(0)

    def cpx_light_sensor_start(self, callback):
        """
        Enable the light sensor for streaming data and report
        its value in the callback.

        :param callback:a list of values described below.

        Parameters sent to callback:

        [Analog Pin Type: 2, Pin Number: 5, current_value]

        """
        self._cpx_start_sensor(Constants.CPX_LIGHT_SENSOR, Constants.ANALOG, callback, None)

    def cpx_light_sensor_stop(self):
        """
        Disable light sensor reporting.

        """
        self._cpx_stop_sensor(Constants.CPX_LIGHT_SENSOR, Constants.ANALOG)

    def cpx_microphone_start(self, callback):
        """
        Enable the light sensor for streaming data and report
        its value in the callback.

        :param callback: a list of values described below.

        Parameters sent to callback:

        [Analog Pin Type: 2, Pin Number: 4, current_value]

        """
        self._cpx_start_sensor(Constants.CPX_MICROPHONE,
                               Constants.ANALOG, callback, None)

    def cpx_microphone_stop(self):
        """
        Disable the light sensor reporting.
        """
        self._cpx_stop_sensor(Constants.CPX_MICROPHONE, Constants.ANALOG)

    def cpx_pixel_brightness_level(self, brightness):
        """
        Set the brightness of all the NeoPixels.  Brightness will be a value
        from 0-100 where 0 means completely dark/no brightness and 100 is full
        brightness.  Note that animating the brightness won't work the way you
        might expect!  If you go down to 0 brightness you will 'lose' information
        and not be able to go back up to higher brightness levels.  Instead
        this is meant to be called once at the start to limit the brightness
        of pixels that are later set.

        :param brightness: 0-100
        """

        assert 0 <= brightness <= 100, 'Brightness must be a value of 0-100!'
        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_PIXEL_BRIGHTNESS, brightness & 0x7F])

    def cpx_pixel_set(self, pixel, red, green, blue):
        """
        Set the specified pixel of the Circuit Playground board to the
        provided red, green, blue value.

        Note you must call show_pixels() after set_pixel() to
        see the actual pixel colors change!

        :param pixel: 0-9

        :param red: 0-255

        :param green: 0-255

        :param blue: 0-255
        """

        assert 0 <= pixel <= 9, 'pixel must be a value between 0-9!'
        # Pack the pixel and RGB values into a string of 7-bit bytes for the command.
        red &= 0xFF
        green &= 0xFF
        blue &= 0xFF
        pixel &= 0x7F
        b1 = red >> 1
        b2 = ((red & 0x01) << 6) | (green >> 2)
        b3 = ((green & 0x03) << 5) | (blue >> 3)
        b4 = (blue & 0x07) << 4
        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_PIXEL_SET, pixel, b1, b2, b3, b4])

    def cpx_pixels_clear(self):
        """
        Clear all the pixels on the Circuit Playground board.

        Make sure to
        call show_pixels to push the change out to the pixels!

        """

        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_PIXEL_CLEAR])

    def cpx_pixels_show(self):
        """
        Send the previously set pixel color data to the 10 pixels on the
        Circuit Playground board. This includes cpx_clear_pixels.
        """

        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_PIXEL_SHOW])

    def cpx_reset(self):
        """
        This command sends a reset message to the Circuit Playground Express.
        The response tables will be reinitialized.
        """

        # set all output pins to a value of 0
        for pin in range(0, self._command_handler.total_pins_discovered):
            if self._command_handler.digital_response_table[Constants.RESPONSE_TABLE_MODE] \
                    == Constants.PWM:
                self._pwm_write(pin, 0)
            elif self._command_handler.digital_response_table[Constants.RESPONSE_TABLE_MODE] \
                    == Constants.SERVO:
                self._pwm_write(pin, 0)
        self._command_handler.system_reset()

    def cpx_set_servo_angle(self, angle):
        """
        Control an external servo connected to pin
        A3. If calling this method repeatedly, must
        include a sleep of .5 seconds between each
        call.

        :param angle: 0-180

        """
        # check angle range
        assert 0 <= angle <= 180, 'Servo Angle must be between 0 and 180'

        # get the digital pin number for A3
        pin = self.ad_pin_map[3]['mapped_pin']

        # only config the servo pin once
        if not self._servo_inuse:
            self._servo_inuse = True
            self._servo_config(pin)

        # set the angle
        self._pwm_write(pin, angle)

    def cpx_slide_switch_start(self, callback, debounce_time=.1):

        """
        Enable button and report state changes in the callback

        :param callback: a list of values described below.

        :param debounce_time: switch debounce time default is .1 seconds

        Parameters sent to callback:

        [Digital Pin Type: 32, Pin Number: 21, switch value: 1 switch on left side, 0 switch on right side]
        """
        self._cpx_start_sensor(Constants.CPX_SLIDE_SWITCH,
                               Constants.DIGITAL, callback, debounce_time)

    def cpx_slide_switch_stop(self):
        """
        Disable slide switch reporting.
        """

        self._cpx_stop_sensor(Constants.CPX_SLIDE_SWITCH, Constants.DIGITAL)

    def cpx_tap_config_set(self, tap_type=0, threshold=80):
        """
        Set the tap detection configuration.  Tap_type should be a value of:

          - 0 = no tap detection

          - 1 = single tap detection

          - 2 = single & double tap detection (default)

        Threshold controls the sensitivity of the detection and is a value FROM
        0 to 255, the higher the value the less sensitive the detection.  This
        value depends on the accelerometer range and good values are:
          - Accel range +/-16G = 5-10

          - Accel range +/-8G  = 10-20

          - Accel range +/-4G  = 20-40

          - Accel range +/-2G  = 40-80 (80 is the default)

        :param tap_type:

        :param threshold:
        """
        assert tap_type in [0, 1, 2], 'Type must be one of 0, 1, 2!'
        assert 0 <= threshold <= 255, 'Threshold must be a value 0-255!'
        # Assemble data to send by turning each unsigned 8 bit values into two
        # 7-bit values that firmata can understand.  The most significant bits
        # are first and the least significant (7th) bit follows.
        tap_type_low = tap_type & 0x7F
        tap_type_high = (tap_type & 0xFF) >> 7
        threshold_low = threshold & 0x7F
        threshold_high = (threshold & 0xFF) >> 7
        # Send command.
        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_ACCEL_TAP_CONFIG,
                                          tap_type_low, tap_type_high, threshold_low, threshold_high])

    def cpx_tap_start(self, callback):
        """
        Request to start streaming tap data from the board.  Will call the
        provided callback with tap data. You may not
        use the device in both accelerometer and tap mode at the same time.

        :param callback: a list of values described below.


        Parameters sent to callback:

        [Digital Pin Type: 32, Pin Number: 27,
        single_tap: True or False, double_tap: True or False]

        """
        assert self._accel_usage == Constants.ACCEL_USAGE_AVAILABLE, 'Accelerometer In Use'

        if self._accel_usage == Constants.ACCEL_USAGE_AVAILABLE:
            self._accel_usage = Constants.ACCEL_USAGE_TAP
            with self._data_lock:
                self._command_handler.digital_response_table[Constants.ACCEL_TAP_PSEUDO_PIN][
                    Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = callback
                self._command_handler.digital_response_table[Constants.ACCEL_TAP_PSEUDO_PIN][
                    Constants.RESPONSE_TABLE_PREV_DATA_VALUE] = [False, False]
            self._command_handler.send_sysex(Constants.CP_COMMAND,
                                             [Constants.CP_ACCEL_TAP_STREAM_ON])
        else:
            logger.warning('cpx_start_tap: accelerometer is in use')

    def cpx_tap_stop(self):
        """
        Stop streaming tap data from the board.
        """
        with self._data_lock:
            self._command_handler.digital_response_table[Constants.ACCEL_PSEUDO_PIN][
                Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = None
        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_ACCEL_TAP_STREAM_OFF])
        self._accel_usage = Constants.ACCEL_USAGE_AVAILABLE

    def cpx_temperature_start(self, callback):
        """
        Enable reading data from the thermistor.
        The callback should take two arguments,
        the temperature in celsius, and the raw ADC value of the thermistor.

        :param callback: a list of values described below.

        Parameters sent to callback:

        [Analog Pin Type: 2, Pin Number: 0, temperature in degrees C]

        """
        try:
            self._set_pin_mode(0, Constants.INPUT,
                               Constants.ANALOG,
                               callback,
                               self._command_handler._therm_handler, None)
        except IndexError:
            raise

    def cpx_temperature_stop(self):
        """
        Stop streaming temperature data from the thermistor.
        """
        with self._data_lock:
            self._command_handler.digital_response_table[0][
                Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = None
            self._command_handler.digital_response_table[0][
                Constants.RESPONSE_TABLE_CALLBACK_INTERNAL] = None
        self._disable_analog_reporting(0)

    def cpx_tone(self, frequency_hz, duration_ms=0):
        """
        Play a tone with the specified frequency (in hz) for the specified
        duration (in milliseconds) using the Circuit Playground board speaker.
        Both frequency and duration can be at most 16,384.  Duration is optional
        and if not specified the tone will continue to play forever (or until
        cpx_tone_off is called).

        :param frequency_hz:

        :param duration_ms:
        """

        # Pack 14-bits into 2 7-bit bytes.
        frequency_hz &= 0x3FFF
        f1 = frequency_hz & 0x7F
        f2 = frequency_hz >> 7
        # Again pack 14-bits into 2 7-bit bytes.
        duration_ms &= 0x3FFF
        d1 = duration_ms & 0x7F
        d2 = duration_ms >> 7
        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_TONE, f1, f2, d1, d2])

    def cpx_tone_off(self):
        """
        Stop all tone playback on the Circuit Playground board speaker.
        :return:
        """

        self._command_handler.send_sysex(Constants.CP_COMMAND,
                                         [Constants.CP_NO_TONE])

    # "private methods"
    def _analog_read(self, pin):
        """
        Retrieve the last analog data value received for the specified pin.

        :param pin: Selected pin

        :return: The last value entered into the analog response table.
        """
        with self._data_lock:
            data = self._command_handler.analog_response_table[pin][Constants.RESPONSE_TABLE_PREV_DATA_VALUE]
        return data

    def _cpx_start_sensor(self, pin, pin_type, callback, debounce_time=None):
        """
        Start sensor streaming for dedicated sensors
        :param pin:
        :param pin_type: digital or analog
        :param callback:
        :return:
        """

        if pin_type == Constants.DIGITAL:
            # get record for this pin
            the_record = self._command_handler.digital_response_table[pin]
            if the_record[Constants.RESPONSE_TABLE_MODE] is None:
                self._set_pin_mode(pin, Constants.INPUT, Constants.DIGITAL, callback, None, debounce_time)

            # self._command_handler.digital_response_table[pin][Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = \
            #     callback
            self._enable_digital_reporting(pin)
        else:
            the_record = self._command_handler.analog_response_table[pin]
            if the_record[Constants.RESPONSE_TABLE_MODE] is None:
                self._set_pin_mode(pin, Constants.INPUT, Constants.ANALOG, callback, None, None)
            # self._command_handler.analog_response_table[pin][Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = \
            #     callback
            self._enable_analog_reporting(pin)

    def _cpx_stop_sensor(self, pin, pin_type):
        """
        Stop sensor streaming for dedicated sensors

        :param pin:

        """
        if pin_type == Constants.DIGITAL:
            self._disable_digital_reporting(pin)
            self._command_handler.digital_response_table[pin][Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = \
                None
            self._command_handler.digital_response_table[pin][Constants.RESPONSE_TABLE_CALLBACK_INTERNAL] = \
                None
        else:
            self._disable_analog_reporting(pin)
            self._command_handler.analog_response_table[pin][Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = \
                None
            self._command_handler.digital_response_table[pin][Constants.RESPONSE_TABLE_CALLBACK_INTERNAL] = \
                None

    def _digital_read(self, pin):
        """
        Retrieve the last digital data value received for the specified pin.
        NOTE: This command will return values for digital, pwm, etc,  pin types

        :param pin: Selected pin

        :return: The last value entered into the digital response table.
        """
        with self._data_lock:
            data = \
                self._command_handler.digital_response_table[pin][Constants.RESPONSE_TABLE_PREV_DATA_VALUE]
        return data

    def _digital_write(self, pin, value):
        """
        Set the specified pin to the specified value.

        :param pin: pin number

        :param value: pin value

        :return: No return value
        """
        # The command value is not a fixed value, but needs to be calculated using the
        # pin's port number
        #
        #
        port = pin // 8

        calculated_command = Constants.DIGITAL_MESSAGE + port
        mask = 1 << (pin % 8)
        # Calculate the value for the pin's position in the port mask
        if value == 1:
            self._digital_output_port_pins[port] |= mask

        else:
            self._digital_output_port_pins[port] &= ~mask

        # Assemble the command
        command = (calculated_command, self._digital_output_port_pins[port] & 0x7f,
                   (self._digital_output_port_pins[port] >> 7) & 0x7f)

        self._command_handler.send_command(command)

    def _disable_analog_reporting(self, pin):
        """
        Disables analog reporting for a single analog pin.

        :param pin: Analog pin number. For example for A0, the number is 0.

        :return: No return value
        """
        self._command_handler.analog_response_table[pin] = [None, None, 0, None, None, None]

        command = [Constants.REPORT_ANALOG + pin, Constants.REPORTING_DISABLE]
        self._command_handler.send_command(command)
        time.sleep(.2)

    def _disable_digital_reporting(self, pin):
        """
        Disables digital reporting. By turning reporting off for this pin, reporting
        is disabled for all 8 bits in the "port" -

        :param pin: Pin and all pins for this port
        :return: No return value
        """
        self._command_handler.digital_response_table[pin] = [None, None, 0, None, None, None]

        port = pin // 8
        command = [Constants.REPORT_DIGITAL + port, Constants.REPORTING_DISABLE]
        self._command_handler.send_command(command)

        time.sleep(.2)
        # self._command_handler.digital_response_table[pin] = [0, 0, None]

    def _enable_analog_reporting(self, pin):
        """
        Enables analog reporting. By turning reporting on for a single pin.

        :param pin: Analog pin number. For example for A0, the number is 0.

        :return: No return value
        """

        command = [Constants.REPORT_ANALOG + pin, Constants.REPORTING_ENABLE]
        self._command_handler.send_command(command)

    def _enable_digital_reporting(self, pin):
        """
        Enables digital reporting. By turning reporting on for all 8 bits in the "port" -
        this is part of Firmata's protocol specification.

        :param pin: Pin and all pins for this port

        :return: No return value
        """
        port = pin // 8
        command = [Constants.REPORT_DIGITAL + port, Constants.REPORTING_ENABLE]
        self._command_handler.send_command(command)

    def _extended_analog(self, pin, data):
        """
        This method will send an extended data analog output command to the selected pin

        :param pin: 0 - 127
        :param data: 0 - 0xfffff
        """
        analog_data = [pin, data & 0x7f, (data >> 7) & 0x7f, (data >> 14) & 0x7f]
        self._command_handler.send_sysex(Constants.EXTENDED_ANALOG, analog_data)

    def _map_analog_pin_to_digital_pin(self, pin):
        """
        The user expresses all pins using their A number.
        Translate a digital pin number to its analog equivalent.
        :param pin:
        :return: analog pin number
        """
        if pin in self.ad_pin_map:
            return self.ad_pin_map[pin]['mapped_pin']
        else:
            raise RuntimeError('Invalid pin number. Must be between 0 and 7.')

    def _pwm_write(self, pin, value):
        """
        Set the specified pin to the specified value.

        :param pin: Pin number

        :param value: Pin value
        """

        if Constants.ANALOG_MESSAGE + pin < 0xf0:
            command = [Constants.ANALOG_MESSAGE + pin, value & 0x7f, (value >> 7) & 0x7f]
            self._command_handler.send_command(command)
        else:
            self._extended_analog(pin, value)

    def _set_pin_mode(self, pin, mode, pin_type, cb_external=None, cb_internal=None, debounce_time=None):
        """
        This method sets a pin to the desired pin mode for the pin_type.
        It automatically enables data reporting.

        :param pin: Pin number (for analog use the analog number, for example A4: use 4)

        :param mode: INPUT, OUTPUT, PWM, PULLUP

        :param pin_type: ANALOG or DIGITAL

        :param cb_external: User's call back routine

        :param cb_internal: used by cpx type functions call back routine


        :return: No return value
        """

        if mode == Constants.INPUT and pin_type == Constants.ANALOG:
            command = [Constants.SET_PIN_MODE, pin, pin_type]
        else:
            command = [Constants.SET_PIN_MODE, pin, mode]
        self._command_handler.send_command(command)

        # enable reporting for input pins
        if mode == Constants.INPUT or mode == Constants.PULLUP:
            if pin_type == Constants.ANALOG:

                # set analog response table to show this pin is an input pin
                with self._data_lock:
                    self._command_handler.analog_response_table[pin][Constants.RESPONSE_TABLE_MODE] = \
                        Constants.INPUT
                    self._command_handler.analog_response_table[pin][
                        Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = cb_external
                    self._command_handler.analog_response_table[pin][
                        Constants.RESPONSE_TABLE_CALLBACK_INTERNAL] = cb_internal
                    self._enable_analog_reporting(pin)
            # if not analog it has to be digital
            else:
                with self._data_lock:
                    try:
                        self._command_handler.digital_response_table[pin][Constants.RESPONSE_TABLE_MODE] = \
                            Constants.INPUT
                        self._command_handler.digital_response_table[pin][
                            Constants.RESPONSE_TABLE_CALLBACK_EXTERNAL] = cb_external
                        self._command_handler.digital_response_table[pin][
                            Constants.RESPONSE_TABLE_CALLBACK_INTERNAL] = cb_internal
                    except IndexError:
                        pass

                    self._enable_digital_reporting(pin)

        else:  # must be output - so set the tables accordingly
            if pin_type == Constants.ANALOG:
                self._command_handler.analog_response_table[pin][Constants.RESPONSE_TABLE_MODE] = mode

            else:
                self._command_handler.digital_response_table[pin][Constants.RESPONSE_TABLE_MODE] = mode

    def _set_sampling_interval(self, interval):
        """
        This method sends the desired sampling interval to Firmata.
        Note: Standard Firmata  will ignore any interval less than 10 milliseconds

        :param interval: Integer value for desired sampling interval in milliseconds

        :return: No return value.
        """
        data = [interval & 0x7f, (interval >> 7) & 0x7f]
        self._command_handler.send_sysex(Constants.SAMPLING_INTERVAL, data)

    def _servo_config(self, pin, min_pulse=544, max_pulse=2400):
        """
        Configure a pin as a servo pin. Set pulse min, max in ms.

        :param pin: Servo Pin.

        :param min_pulse: Min pulse width in ms.

        :param max_pulse: Max pulse width in ms.

        :return: No return value
        """
        self._set_pin_mode(pin, Constants.SERVO, Constants.OUTPUT)
        command = [pin, min_pulse & 0x7f, (min_pulse >> 7) & 0x7f,
                   max_pulse & 0x7f, (max_pulse >> 7) & 0x7f]

        self._command_handler.send_sysex(Constants.SERVO_CONFIG, command)
