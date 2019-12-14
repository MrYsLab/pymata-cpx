# some state variables

class Constants:
    # pin states
    HIGH = 1  # digital pin state high value
    LOW = 0  # digital pin state low value

    # reporting flags
    REPORTING_ENABLE = 1  # enable reporting for REPORT_ANALOG or REPORT_DIGITAL message sent to firmata
    REPORTING_DISABLE = 0  # disable reporting for REPORT_ANALOG or REPORT_DIGITAL message sent to firmata

    # pin modes
    INPUT = 0x00  # pin set as input
    OUTPUT = 0x01  # pin set as output
    ANALOG = 0x02  # analog pin in analogInput mode
    PWM = 0x03  # digital pin in PWM output mode
    SERVO = 0x04  # digital pin in Servo output mode
    I2C = 0x06  # pin included in I2C setup
    ONEWIRE = 0x07  # possible future feature
    STEPPER = 0x08  # any pin in stepper mode
    ACCEL = 0x09  # accelerometer
    SERIAL = 0x0a
    PULLUP = 0x0b  # Any pin in pullup mode
    SONAR = 0x0c  # Any pin in SONAR mode
    TONE = 0x0d  # Any pin in tone mode
    ANALOG_INPUT = 0x0e  # pin set as analog input
    ANALOG_OUTPUT = 0x0f  # pin set as analog input
    IGNORE = 0x7f

    # the following pin modes are not part of or defined by Firmata
    # but used by PyMata
    DIGITAL = 0x20

    # I2C command operation modes
    I2C_WRITE = 0B00000000
    I2C_READ = 0B00001000
    I2C_READ_CONTINUOUSLY = 0B00010000
    I2C_STOP_READING = 0B00011000
    I2C_READ_WRITE_MODE_MASK = 0B00011000

    # message command bytes (128-255/ 0x80- 0xFF)
    # from this client to firmata
    MSG_CMD_MIN = 0x80  # minimum value for a message from firmata
    REPORT_ANALOG = 0xC0  # enable analog input by pin #
    REPORT_DIGITAL = 0xD0  # enable digital input by port pair
    SET_PIN_MODE = 0xF4  # set a pin to INPUT/OUTPUT/PWM/etc

    START_SYSEX = 0xF0  # start a MIDI Sysex message
    END_SYSEX = 0xF7  # end a MIDI Sysex message
    REPORT_VERSION = 0xF9  # get firmware version of sketch
    SYSTEM_RESET = 0xFF  # reset from MIDI

    # messages from firmata
    DIGITAL_MESSAGE = 0x90  # send or receive data for a digital pin
    ANALOG_MESSAGE = 0xE0  # send or receive data for a PWM configured pin
    #  REPORT_VERSION = 0xF9  # report protocol version

    # user defined SYSEX commands
    CP_COMMAND = 0x40  # circuit playground command id - followed by sub commands
    TONE_PLAY = 0x5F  # play a tone at a specified frequency and duration
    SONAR_CONFIG = 0x62  # configure pins to control a Ping type sonar distance device

    # ENCODER_DATA = 0x61  # current encoder position data
    SONAR_DATA = 0x63  # distance data returned

    SERVO_CONFIG = 0x70  # set servo pin and max and min angles
    STRING_DATA = 0x71  # a string message with 14-bits per char
    STEPPER_DATA = 0x72  # Stepper motor command
    I2C_REQUEST = 0x76  # send an I2C read/write request
    I2C_REPLY = 0x77  # a reply to an I2C read request
    I2C_CONFIG = 0x78  # config I2C settings such as delay times and power pins
    REPORT_FIRMWARE = 0x79  # report name and version of the firmware
    SAMPLING_INTERVAL = 0x7A  # modify the sampling interval

    EXTENDED_ANALOG = 0x6F  # analog write (PWM, Servo, etc) to any pin
    PIN_STATE_QUERY = 0x6D  # ask for a pin's current mode and value
    PIN_STATE_RESPONSE = 0x6E  # reply with pin's current mode and value
    CAPABILITY_QUERY = 0x6B  # ask for supported modes and resolution of all pins
    CAPABILITY_RESPONSE = 0x6C  # reply with supported modes and resolution
    ANALOG_MAPPING_QUERY = 0x69  # ask for mapping of analog to pin numbers
    ANALOG_MAPPING_RESPONSE = 0x6A  # reply with analog mapping data

    # reserved values
    SYSEX_NON_REALTIME = 0x7E  # MIDI Reserved for non-realtime messages
    SYSEX_REALTIME = 0x7F  # MIDI Reserved for realtime messages

    # circuit playground sub-commands
    CP_PIXEL_SET = 0x10  # Set NeoPixel, expects the following bytes as data:
    #  - Pixel ID (0-9)
    #  - Pixel RGB color data as 4 7-bit bytes.  The upper
    #    24 bits will be mapped to the R, G, B bytes.
    CP_PIXEL_SHOW = 0x11  # Update NeoPixels with their current color values.
    CP_PIXEL_CLEAR = 0x12  # Clear all NeoPixels to black/off.  Must call show pixels after this to see the change!
    CP_PIXEL_BRIGHTNESS = 0x13  # Set the brightness of the NeoPixels, just like calling the
    # NeoPixel library setBrightness function.  Takes one parameter
    # which is a single byte with a value 0-100.
    CP_TONE = 0x20  # Play a tone on the speaker, expects the following bytes as data:
    #  - Frequency (hz) as 2 7-bit bytes (up to 2^14 hz, or about 16khz)
    #  - Duration (ms) as 2 7-bit bytes.
    CP_NO_TONE = 0x21  # Stop playing anything on the speaker.
    CP_ACCEL_READ = 0x30  # Return the current x, y, z accelerometer values.
    CP_ACCEL_TAP = 0x31  # Return the current accelerometer tap state.
    CP_ACCEL_READ_REPLY = 0x36  # Result of an accelerometer read.
    # Includes 3 floating point values
    # (4 bytes each) with x, y, z
    # acceleration in meters/second^2.
    CP_ACCEL_TAP_REPLY = 0x37  # Result of the tap sensor read.  I
    # includes a byte with the tap register value.
    CP_ACCEL_TAP_STREAM_ON = 0x38  # Turn on continuous streaming of tap data.
    CP_ACCEL_TAP_STREAM_OFF = 0x39  # Turn off streaming of tap data.
    CP_ACCEL_STREAM_ON = 0x3A  # Turn on continuous streaming of accelerometer data.
    CP_ACCEL_STREAM_OFF = 0x3B  # Turn off streaming of accelerometer data.
    CP_ACCEL_RANGE = 0x3C  # Set the range of the accelerometer, takes one byte as a parameter.
    # Use a value 0=+/-2G, 1=+/-4G, 2=+/-8G, 3=+/-16G
    CP_ACCEL_TAP_CONFIG = 0x3D  # Set the sensitivity of the tap detection, takes 4 bytes of 7-bit firmata
    # data as parameters which expand to 2 unsigned 8-bit bytes value to set:
    #   - Type of click: 0 = no click detection, 1 = single click, 2 = single & double click (default)
    #   - Click threshold: 0-255, the higher the value the less sensitive.  Depends on the accelerometer
    #     range, good values are: +/-16G = 5-10, +/-8G = 10-20, +/-4G = 20-40, +/-2G = 40-80
    #     80 is the default value (goes well with default of +/-2G)
    CP_CAP_READ = 0x40  # Read a single capacitive input.  Expects a byte as a parameter with the
    # cap touch input to read (0, 1, 2, 3, 6, 9, 10, 12).  Will respond with a
    # CP_CAP_REPLY message.
    CP_CAP_ON = 0x41  # Turn on continuous cap touch reads for the specified input (sent as a byte parameter).
    CP_CAP_OFF = 0x42  # Turn off continuous cap touch reads for the specified input (sent as a byte parameter).
    CP_CAP_REPLY = 0x43  # Capacitive input read response.  Includes a byte with the pin # of the cap input, then
    # four bytes of data which represent an int32_t value read from the cap input.
    CP_SENSECOLOR = 0x50  # Perform a color sense using the NeoPixel and light sensor.
    CP_SENSECOLOR_REPLY = 0x51  # Result of a color sense, will return the red, green, blue color
    # values that were read from the light sensor.  This will return
    # 6 bytes of data:
    #  - red color (unsigned 8 bit value, split across 2 7-bit bytes)
    #  - green color (unsigned 8 bit value, split across 2 7-bit bytes)
    #  - blue color (unsigned 8 bit value, split across 2 7-bit bytes)
    CP_IMPL_VERS = 0x60  # Get the implementation version, 3 bytes of Major, Minor, Bugfix
    CP_IMPL_VERS_REPLY = 0x61

    # Accelerometer constants to be passed to set_accel_range.
    ACCEL_2G = 0
    ACCEL_4G = 1
    ACCEL_8G = 2
    ACCEL_16G = 3

    CAP_THRESHOLD = 33000000  # Threshold for considering a cap touch input pressed.
    # If the cap touch value is above this value it is
    # considered touched.

    # The response tables hold response information for all pins
    # Each table is a table of entries for each pin, which consists of the pin mode, its last value from firmata
    # and a callback function that the user attached to the pin and an internal callback if needed

    # These values are indexes into the response table entries
    RESPONSE_TABLE_MODE = 0
    RESPONSE_TABLE_PIN_NUMBER = 1
    RESPONSE_TABLE_PREV_DATA_VALUE = 2
    RESPONSE_TABLE_CALLBACK_EXTERNAL = 3
    RESPONSE_TABLE_CALLBACK_INTERNAL = 4
    RESPONSE_TABLE_DEBOUNCE_TIME = 5

    ACCEL_USAGE_AVAILABLE = 0  # accel not in use
    ACCEL_USAGE_ACCEL = 1  # accel is being used as an accelerometer
    ACCEL_USAGE_TAP = 2  # accel is being used as a tap detector
    ACCEL_PSEUDO_PIN = 11  # to store info in analog
    ACCEL_TAP_PSEUDO_PIN = 12 # store tap data separately

    # These values are the index into the data passed by firmata and used to reassemble integer values
    MSB = 2
    LSB = 1

    # temperature calculation constants
    THERM_SERIES_OHMS = 10000.0  # Resistor value in series with thermistor.
    THERM_NOMINAL_OHMS = 10000.0  # Thermistor resistance at 25 degrees C.
    THERM_NOMINAL_C = 25.0  # Thermistor temperature at nominal resistance.
    THERM_BETA = 3950.0  # Thermistor beta coefficient.

    # pin numbers for dedicated sensors.
    # the may be mapped to the actual pin numbers by the .ino sketch
    """
    The mapping from CP classic to express pins is as follows.  This is intended to
    match the usage of the digital and analog pins.
  - Digital inputs:
    - Firmata D4 -> CP Classic D4/left button -> CP Express D4/button A (no change, direct mapping)
    - Firmata D19 -> CP Classic D19/right button -> CP Express D5/button B
    - Firmata D21 -> CP Classic D21/slide switch -> CP Express D7/slide switch
    - Firmata D13 -> CP Classic D13/red LED -> CP Express D13/red LED (no change, direct mapping)
  - Analog inputs:
    - Firmata A5 -> CP Classic A5/light sensor -> CP Express A8/light sensor
    - Firmata A0 -> CP Classic A0/temp sensor -> CP Express A9/temp sensor
    - Firmata A4 -> CP Classic A4/microphone -> CP Express synthesized 10-bit PDM sample
    - Firmata A11 -> CP Classic A11/D12 -> CP Express A0
    - Firmata A7 -> CP Classic A7/D6 -> CP Express A1
    - Firmata A9 -> CP Classic A9/D9 -> CP Express A2
    - Firmata A10 -> CP Classic A10/D10 -> CP Express A3
    """
    # Dedicated cpx sensor pin numbers and mapped to pin type in
    # the command handler
    CPX_MICROPHONE = 10  # this is a virtual pin
    CPX_TEMPERATURE = 9 # this is analog
    CPX_BUTTON_A = 4  # this is digital
    CPX_BUTTON_B = 5  # this is digital
    CPX_SLIDE_SWITCH = 7  # this is digital
    CPX_LIGHT_SENSOR = 8  # this is analog
