/*
  Circuit Playground Express Firmata Firmware

  This is a modified, abbreviated, special version of the Firmata firmware
  for Adafruit's Circuit Playground Express physical computing board.

  NOTE: This version ONLY works with Circuit Playground Express!
  https://github.com/MrYsLab/pymata-cpx

  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
  Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
  Copyright (C) 2009-2015 Jeff Hoefs.  All rights reserved.
  Copyright (C) 2016 Tony DiCola.  All rights reserved.
  Copyright (C) 2019 Alan Yorinks.  All rights reserved.


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
*/

#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_CircuitPlayground.h>


// These defines setup debug output if enabled above (otherwise it
// turns into no-ops that compile out).
#define DEBUG_OUTPUT Serial1
#define DEBUG_BAUD   115200

#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) { DEBUG_OUTPUT.print(__VA_ARGS__); }
#define DEBUG_PRINTLN(...) { DEBUG_OUTPUT.println(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...) {}
#define DEBUG_PRINTLN(...) {}
#endif

#define TOTAL_ANALOG_PINS       13 // a0 - a9 + virtual for sound and accelorometer and tap
#define TOTAL_PINS              31 // 14 digital + 13 analog + 4 SPI (D14-D17 on ISP header)
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           ((p) == 0 || (p) == 1 || (p) == 9 || (p) == 10 || (p) == 13)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 2 || (p) == 3)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (p) - 14
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define TOTAL_PORTS             ((TOTAL_PINS + 7) / 8)

#define I2C_WRITE                   B00000000
#define I2C_READ                    B00001000
#define I2C_READ_CONTINUOUSLY       B00010000
#define I2C_STOP_READING            B00011000
#define I2C_READ_WRITE_MODE_MASK    B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
#define I2C_MAX_QUERIES             8
#define I2C_REGISTER_NOT_SPECIFIED  -1

// Circuit Playground Express configuration:
#define CPLAY_SPEAKER           12  // Output connected to the speaker.
#define CPLAY_SPEAKER_SHUTDOWN  11  // Pin connected to speaker amp shutdown.
#define CAP_COUNT      8            // Number of cap touch inputs.
#define CAP_SAMPLES    30           // Number of samples to take for a cap touch input.
#define ANALOG_SAMPLES 5            // How many times to sample an ADC input and average its results.
// Smoothes out the noise from express's faster processing and ADC sampling.

// Circuit playground-specific Firmata SysEx commands:
#define CP_COMMAND              0x40  // Byte that identifies all Circuit Playground commands.
#define CP_PIXEL_SET            0x10  // Set NeoPixel, expects the following bytes as data:
//  - Pixel ID (0-9)
//  - Pixel RGB color data as 4 7-bit bytes.  The upper
//    24 bits will be mapped to the R, G, B bytes.
#define CP_PIXEL_SHOW           0x11  // Update NeoPixels with their current color values.
#define CP_PIXEL_CLEAR          0x12  // Clear all NeoPixels to black/off.  Must call show pixels after this to see the change!
#define CP_PIXEL_BRIGHTNESS     0x13  // Set the brightness of the NeoPixels, just like calling the
// NeoPixel library setBrightness function.  Takes one parameter
// which is a single byte with a value 0-100.
#define CP_TONE                 0x20  // Play a tone on the speaker, expects the following bytes as data:
//  - Frequency (hz) as 2 7-bit bytes (up to 2^14 hz, or about 16khz)
//  - Duration (ms) as 2 7-bit bytes.
#define CP_NO_TONE              0x21  // Stop playing anything on the speaker.
#define CP_ACCEL_READ           0x30  // Return the current x, y, z accelerometer values.
#define CP_ACCEL_TAP            0x31  // Return the current accelerometer tap state.
#define CP_ACCEL_READ_REPLY     0x36  // Result of an acceleromete read.  Includes 3 floating point values (4 bytes each) with x, y, z
// acceleration in meters/second^2.
#define CP_ACCEL_TAP_REPLY      0x37  // Result of the tap sensor read.  Includes a byte with the tap register value.
#define CP_ACCEL_TAP_STREAM_ON  0x38  // Turn on continuous streaming of tap data.
#define CP_ACCEL_TAP_STREAM_OFF 0x39  // Turn off streaming of tap data.
#define CP_ACCEL_STREAM_ON      0x3A  // Turn on continuous streaming of accelerometer data.
#define CP_ACCEL_STREAM_OFF     0x3B  // Turn off streaming of accelerometer data.
#define CP_ACCEL_RANGE          0x3C  // Set the range of the accelerometer, takes one byte as a parameter.
// Use a value 0=+/-2G, 1=+/-4G, 2=+/-8G, 3=+/-16G
#define CP_ACCEL_TAP_CONFIG     0x3D  // Set the sensitivity of the tap detection, takes 4 bytes of 7-bit firmata
// data as parameters which expand to 2 unsigned 8-bit bytes value to set:
//   - Type of click: 0 = no click detection, 1 = single click, 2 = single & double click (default)
//   - Click threshold: 0-255, the higher the value the less sensitive.  Depends on the accelerometer
//     range, good values are: +/-16G = 5-10, +/-8G = 10-20, +/-4G = 20-40, +/-2G = 40-80
//     80 is the default value (goes well with default of +/-2G)
#define CP_CAP_READ             0x40  // Read a single capacitive input.  Expects a byte as a parameter with the
// cap touch input to read (0, 1, 2, 3, 6, 9, 10, 12).  Will respond with a
// CP_CAP_REPLY message.
#define CP_CAP_ON               0x41  // Turn on continuous cap touch reads for the specified input (sent as a byte parameter).
#define CP_CAP_OFF              0x42  // Turn off continuous cap touch reads for the specified input (sent as a byte parameter).
#define CP_CAP_REPLY            0x43  // Capacitive input read response.  Includes a byte with the pin # of the cap input, then
// four bytes of data which represent an int32_t value read from the cap input.
#define CP_SENSECOLOR           0x50  // Perform a color sense using the NeoPixel and light sensor.
#define CP_SENSECOLOR_REPLY     0x51  // Result of a color sense, will return the red, green, blue color
// values that were read from the light sensor.  This will return
// 6 bytes of data:
//  - red color (unsigned 8 bit value, split across 2 7-bit bytes)
//  - green color (unsigned 8 bit value, split across 2 7-bit bytes)
//  - blue color (unsigned 8 bit value, split across 2 7-bit bytes)
#define CP_IMPL_VERS            0x60  // Get the implementation version
#define CP_IMPL_VERS_REPLY      0x61  // 3 bytes from IMPLEMENTATION_VERSION


// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL   1


/*==============================================================================
   GLOBAL VARIABLES
  ============================================================================*/

// Circuit playground globals:
bool streamTap = false;
bool streamAccel = false;
// Define type for the cap touch sensor state of each cap touch input.
typedef struct {
  bool streaming;
  uint8_t pin;
} cap_state_type;

cap_state_type cap_state[CAP_COUNT] = {
  {
    .streaming = false,
    .pin       = 0
  },
  {
    .streaming = false,
    .pin       = 1
  },
  {
    .streaming = false,
    .pin       = 2
  },
  {
    .streaming = false,
    .pin       = 3
  },
  {
    .streaming = false,
    .pin       = 6
  },
  {
    .streaming = false,
    .pin       = 9
  },
  {
    .streaming = false,
    .pin       = 10
  },
  {
    .streaming = false,
    .pin       = 12
  }
};


/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 19; // how often to run the main loop (in ms)

/* i2c data */
struct i2c_device_info {
  byte addr;
  int reg;
  byte bytes;
};

/* for i2c read continuous more */
i2c_device_info query[I2C_MAX_QUERIES];

byte i2cRxData[64];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
// default delay time between i2c read request and Wire.requestFrom()
unsigned int i2cReadDelayTime = 0;

Servo servos[MAX_SERVOS];
byte servoPinMap[TOTAL_PINS];
byte detachedServos[MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;

boolean isResetting = false;

/* utility functions */
void wireWrite(byte data)
{
#if ARDUINO >= 100
  Wire.write((byte)data);
#else
  Wire.send(data);
#endif
}

byte wireRead(void)
{
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

// touch channel to be processed in the loop
int touch_channel = 0;

// number of loops before accelerometer is processed
int numLoops = 20 ;

// count of loops for accelerometer processing
int accLoopCounter = 0 ;


/*==============================================================================
   FUNCTIONS
  ============================================================================*/

// The functions that begin with cpx are specific to this version of firmata

// map the pseudo sound pin to start getting a sound sample
int cpxAnalogPinMapping(int pin) {
  if (pin == 10){
      return -1 ;
  }
  else{
      return pin;
  }
}

// the cpx version of the readPort function
uint8_t cpxReadPort(uint8_t port, uint8_t bitmask) {

  uint8_t portValue = readPort(port, bitmask);
  for (int i = 0; i < 8; ++i) {
    int pin = port * 8 + i;


    if (digitalRead(pin) == HIGH) {
        portValue |= (1 << i);
    }
    else {
        portValue &= ~(1 << i);
    }
  }
  return portValue;
}

// Cpx version of the pinMode function.
void cpxPinMode(int pin, int mode) {
  // printData((char*)"cpxPinMode", pin);
  //printData((char*)"cpxPinMode", mode);

  // Ignore pins above 13 as they are internal
  if (pin > 13) {
    return;
  }
  if ((mode == INPUT) || (mode == INPUT_PULLUP)) {
    switch (pin) {
      case 4:
      case 5:
        mode = INPUT_PULLDOWN;
        break;
      case 7:
        mode = INPUT_PULLUP;
        break;
    }
  }
  pinMode(pin, mode);
}


// ignore any pins which are internal to express.
void cpxDigitalWrite(uint8_t pin, uint8_t value) {
  // Only write to pins below 13 as other pins are internal like chip selects, etc.
  if (pin > 30) {
    return;
  }
  digitalWrite(pin, value);
}

// cpx verison of  writePort function
uint8_t cpxWritePort(uint8_t port, uint8_t value, uint8_t bitmask) {
  byte pin = port * 8;
  if ((bitmask & 0x01)) cpxDigitalWrite(PIN_TO_DIGITAL(pin + 0), (value & 0x01));
  if ((bitmask & 0x02)) cpxDigitalWrite(PIN_TO_DIGITAL(pin + 1), (value & 0x02));
  if ((bitmask & 0x04)) cpxDigitalWrite(PIN_TO_DIGITAL(pin + 2), (value & 0x04));
  if ((bitmask & 0x08)) cpxDigitalWrite(PIN_TO_DIGITAL(pin + 3), (value & 0x08));
  if ((bitmask & 0x10)) cpxDigitalWrite(PIN_TO_DIGITAL(pin + 4), (value & 0x10));
  if ((bitmask & 0x20)) cpxDigitalWrite(PIN_TO_DIGITAL(pin + 5), (value & 0x20));
  if ((bitmask & 0x40)) cpxDigitalWrite(PIN_TO_DIGITAL(pin + 6), (value & 0x40));
  if ((bitmask & 0x80)) cpxDigitalWrite(PIN_TO_DIGITAL(pin + 7), (value & 0x80));
  return 1;
}

// Cpx version of  analog read to capture the PDM mic.
int cpxAnalogRead(uint8_t pin) {
  int cpxPin = cpxAnalogPinMapping(pin);
  if (cpxPin == -1) {
    // Special case to read the microphone and scale to an unsigned 10-bit value.
    // Note that we get _signed_ 10-bit values from the mic capture call below, so just add 512 to readjust to unsigned.
    // Read a single sample from the mic.
    int16_t sample;
    //printData((char*)"getting sample", 0);
    CircuitPlayground.mic.capture(&sample, 1);
    return sample + 512;
  }
  else {
    // Just read the input.
    // However we'll take a few samples and average them as the ADC on the SAMD21 is a lot faster
    // and gets more noisey results.  Just one sample usually reads a really low value.
    // Take 5 readings from the sensor and average them.
    uint32_t total = 0;
    for (int i = 0; i < ANALOG_SAMPLES; ++i) {
      total += analogRead(cpxPin);
    }
    total /= ANALOG_SAMPLES;
    return total;
  }
}

// cpx version of  analogWrite
void cpxAnalogWrite(uint8_t pin, uint8_t value) {
  // Only write to pins below 13 as other pins are internal like chip selects, etc.
  if (pin > 30) {
    return;
  }
  analogWrite(pin, value);
}

void attachServo(byte pin, int minPulse, int maxPulse)
{
  if (servoCount < MAX_SERVOS) {
    // reuse indexes of detached servos until all have been reallocated
    if (detachedServoCount > 0) {
      servoPinMap[pin] = detachedServos[detachedServoCount - 1];
      if (detachedServoCount > 0) detachedServoCount--;
    } else {
      servoPinMap[pin] = servoCount;
      servoCount++;
    }
    if (minPulse > 0 && maxPulse > 0) {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
    } else {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin));
    }
  } else {
    Firmata.sendString("Max servos attached");
  }
}

void detachServo(byte pin)
{
  servos[servoPinMap[pin]].detach();
  // if we're detaching the last servo, decrement the count
  // otherwise store the index of the detached servo
  if (servoPinMap[pin] == servoCount && servoCount > 0) {
    servoCount--;
  } else if (servoCount > 0) {
    // keep track of detached servos because we want to reuse their indexes
    // before incrementing the count of attached servos
    detachedServoCount++;
    detachedServos[detachedServoCount - 1] = servoPinMap[pin];
  }

  servoPinMap[pin] = 255;
}

void readAndReportData(byte address, int theRegister, byte numBytes) {
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()
  if (theRegister != I2C_REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
    wireWrite((byte)theRegister);
    Wire.endTransmission();
    // do not set a value of 0
    if (i2cReadDelayTime > 0) {
      // delay is necessary for some devices such as WiiNunchuck
      delayMicroseconds(i2cReadDelayTime);
    }
  } else {
    theRegister = 0;  // fill the register with a dummy value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (numBytes < Wire.available()) {
    Firmata.sendString("I2C: Too many bytes received");
  } else if (numBytes > Wire.available()) {
    Firmata.sendString("I2C: Too few bytes received");
  }

  i2cRxData[0] = address;
  i2cRxData[1] = theRegister;

  for (int i = 0; i < numBytes && Wire.available(); i++) {
    i2cRxData[2 + i] = wireRead();
  }

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
   check all the active digital inputs for change of state, then add any events
   to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
     The compiler will apply substantial optimizations if the inputs
     to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, cpxReadPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, cpxReadPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, cpxReadPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, cpxReadPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, cpxReadPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, cpxReadPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, cpxReadPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, cpxReadPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, cpxReadPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, cpxReadPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, cpxReadPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, cpxReadPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, cpxReadPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, cpxReadPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, cpxReadPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, cpxReadPort(15, portConfigInputs[15]), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
   two bit-arrays that track Digital I/O and PWM status
*/
void setPinModeCallback(byte pin, int mode)
{
  if (pinConfig[pin] == PIN_MODE_IGNORE)
    return;

  if (pinConfig[pin] == PIN_MODE_I2C && isI2CEnabled && mode != PIN_MODE_I2C) {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_DIGITAL(pin) && mode != PIN_MODE_SERVO) {
    if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
      detachServo(pin);
    }
  }
  if (IS_PIN_ANALOG(pin)) {
    //printData((char*)"analog", pin)
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == PIN_MODE_PULLUP) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  pinState[pin] = 0;
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          cpxPinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
          // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
          cpxDigitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        }
        pinConfig[pin] = PIN_MODE_ANALOG;
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        cpxPinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
        // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
        cpxDigitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        pinConfig[pin] = INPUT;
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        cpxPinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
        pinConfig[pin] = PIN_MODE_PULLUP;
        pinState[pin] = 1;
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        cpxDigitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
        cpxPinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        pinConfig[pin] = OUTPUT;
      }
      break;
    case PIN_MODE_PWM:
      if (IS_PIN_PWM(pin)) {
        cpxPinMode(PIN_TO_PWM(pin), OUTPUT);
        cpxAnalogWrite(PIN_TO_PWM(pin), 0);
        pinConfig[pin] = PIN_MODE_PWM;
      }
      break;
    case PIN_MODE_SERVO:
      if (IS_PIN_DIGITAL(pin)) {
        pinConfig[pin] = PIN_MODE_SERVO;
        if (servoPinMap[pin] == 255 || !servos[servoPinMap[pin]].attached()) {
          // pass -1 for min and max pulse values to use default values set
          // by Servo library
          attachServo(pin, -1, -1);
        }
      }
      break;
    case PIN_MODE_I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        pinConfig[pin] = PIN_MODE_I2C;
      }
      break;

    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

/*
   Sets the value of an individual pin. Useful if you want to set a pin value but
   are not tracking the digital port state.
   Can only be used on pins configured as OUTPUT.
   Cannot be used to enable pull-ups on Digital INPUT pins.
*/
void setPinValueCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (pinConfig[pin] == OUTPUT) {
      pinState[pin] = value;
      cpxDigitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch (pinConfig[pin]) {
      case PIN_MODE_SERVO:
        if (IS_PIN_DIGITAL(pin))
          servos[servoPinMap[pin]].write(value);
        pinState[pin] = value;
        break;
      case PIN_MODE_PWM:
        if (IS_PIN_PWM(pin))
          cpxAnalogWrite(PIN_TO_PWM(pin), value);
        pinState[pin] = value;
        break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, pinValue, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT) {
          pinValue = ((byte)value & mask) ? 1 : 0;
          if (pinConfig[pin] == OUTPUT) {
            pinWriteMask |= mask;
          } else if (pinConfig[pin] == INPUT && pinValue == 1 && pinState[pin] != 1) {
            // only handle INPUT here for backwards compatibility
#if ARDUINO > 100
            cpxPinMode(pin, INPUT_PULLUP);
#else
            // only write to the INPUT pin to enable pullups if Arduino v1.0.0 or earlier
            pinWriteMask |= mask;
#endif
          }
          pinState[pin] = pinValue;
        }
      }
      mask = mask << 1;
    }
    cpxWritePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
*/
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, cpxAnalogRead(analogPin));
      }
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, cpxReadPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}


/*==============================================================================
   Circuit Playground commands
  ============================================================================*/

void circuitPlaygroundCommand(byte command, byte argc, byte* argv) {
  switch (command) {
    case CP_PIXEL_SET:
      // Set a NeoPixel to the specified RGB color.
      // Expect: 1 byte pixel number, 4 bytes pixel RGB value (as 7-bit bytes)
      if (argc >= 5) {
        // Parse out the pixel number and R, G, B bytes.
        uint8_t pixel = argv[0] & 0x7F;
        uint8_t r = (argv[1] << 1) | ((argv[2] & 0x7F) >> 6);  // Red = 7 bits from byte 4 and 1 bit from byte 5
        uint8_t g = ((argv[2] & 0x3F) << 2) | (((argv[3]) & 0x7F) >> 5);  // Green = 6 bits from byte 5 and 2 bits from byte 6
        uint8_t b = ((argv[3] & 0x1F) << 3) | (((argv[4]) & 0x7F) >> 4);  // Blue = 5 bits from byte 6 and 3 bits from byte 7
        CircuitPlayground.strip.setPixelColor(pixel, r, g, b);
      }
      break;
    case CP_PIXEL_SHOW:
      // Light up the neopixels with their current buffer values.
      CircuitPlayground.strip.show();
      break;
    case CP_PIXEL_CLEAR:
      // Clear all the pixel color values.
      CircuitPlayground.strip.clear();
      break;
    case CP_PIXEL_BRIGHTNESS:
      // Set pixel brightness.
      // Expects 1 byte with the brightness as a value 0-100.
      if (argc >= 1) {
        uint8_t brightness = argv[0];
        if (brightness > 100) {
          return;
        }
        CircuitPlayground.strip.setBrightness(brightness);
      }
    case CP_TONE:
      // Play a tone on the speaker.
      // Expect: 2 bytes tone frequency, 2 bytes tone duration
      if (argc >= 4) {
        uint16_t frequency = ((argv[1] & 0x7F) << 7) | (argv[0] & 0x7F);
        uint16_t duration = ((argv[3] & 0x7F) << 7) | (argv[2] & 0x7F);
        // If duration is zero then interpret that as continuous tone playback.
        // Force output mode of speaker and speaker enable pin as it might have
        // been clobbered by user code since A0 / D12 is also the speaker output.
        pinMode(CPLAY_SPEAKER, OUTPUT);
        pinMode(CPLAY_SPEAKER_SHUTDOWN, OUTPUT);
        digitalWrite(CPLAY_SPEAKER_SHUTDOWN, HIGH);
        if (duration == 0) {
          tone(CPLAY_SPEAKER, frequency);
        }
        else {
          tone(CPLAY_SPEAKER, frequency, duration);
        }
      }
      break;
    case CP_NO_TONE:
      // Stop tone playback.
      // Force output mode of speaker and speaker enable pin as it might have
      // been clobbered by user code since A0 / D12 is also the speaker output.
      pinMode(CPLAY_SPEAKER, OUTPUT);
      pinMode(CPLAY_SPEAKER_SHUTDOWN, OUTPUT);
      digitalWrite(CPLAY_SPEAKER_SHUTDOWN, LOW);
      tone(CPLAY_SPEAKER, 100, 1);  // Very short tone to stop playback as noTone locks the board.
      //noTone(CPLAY_SPEAKER);  TODO: Notone broken! Locks up
      break;
    case CP_IMPL_VERS:
      sendImplemenationVersionResponse();
      break;
    case CP_ACCEL_READ:
      sendAccelResponse();
      break;
    case CP_ACCEL_TAP:
      sendTapResponse();
      break;
    case CP_ACCEL_TAP_STREAM_ON:
      streamTap = true;
      break;
    case CP_ACCEL_TAP_STREAM_OFF:
      streamTap = false;
      break;
    case CP_ACCEL_STREAM_ON:
      streamAccel = true;
      break;
    case CP_ACCEL_STREAM_OFF:
      streamAccel = false;
      break;
    case CP_CAP_READ:
      // Flip the streaming bool on for the specified input.
      // First check we have enough parameters and grab the input from the first byte.
      if (argc >= 1) {
        uint8_t input = argv[0] & 0x7F;
        // Now find the specified cap input and send a response with its current cap touch value.
        for (int i = 0; i < CAP_COUNT; ++i) {
          if (cap_state[i].pin == input) {
            sendCapResponse(input);
          }
        }
      }
      break;
    case CP_CAP_ON:
      // Flip the streaming bool on for the specified input.
      // First check we have enough parameters and grab the input from the first byte.
      if (argc >= 1) {
        uint8_t input = argv[0] & 0x7F;
        // Now find the specified cap input and flip on its streaming bit.
        for (int i = 0; i < CAP_COUNT; ++i) {
          if (cap_state[i].pin == input) {
            cap_state[i].streaming = true;
          }
        }
      }
      break;
    case CP_CAP_OFF:
      // Flip the streaming bool off for the specified input.
      // First check we have enough parameters and grab the input from the first byte.
      if (argc >= 1) {
        uint8_t input = argv[0] & 0x7F;
        // Now find the specified cap input and flip on its streaming bit.
        for (int i = 0; i < CAP_COUNT; ++i) {
          if (cap_state[i].pin == input) {
            cap_state[i].streaming = false;
          }
        }
      }
      break;
    case CP_ACCEL_RANGE:
      // Set the range of the accelerometer based on the passed in value.
      // First check we have enough parameters and grab the input from the first byte.
      if (argc >= 1) {
        uint8_t range = argv[0] & 0x7F;
        // Check the range is an allowed value (0, 1, 2, 3).
        if (range > 3) {
          // Unknown range value, stop processing!
          return;
        }
        // Set the range of the accelerometer.
        CircuitPlayground.lis.setRange((lis3dh_range_t)range);
      }
      break;
    case CP_ACCEL_TAP_CONFIG:
      // Set the sensitivity of tap detection.
      // Expects 4 7-bit firmata bytes as input, which expand to 2 bytes of 8-bit unsigned data.
      // First check we have enough parameters.
      if (argc >= 4) {
        // Parse out paramemters.
        uint8_t type = ((argv[1] & 0x01) << 7) | (argv[0] & 0x7F);
        uint8_t threshold = ((argv[3] & 0x01) << 7) | (argv[2] & 0x7F);
        // Set the click threshold values.
        CircuitPlayground.lis.setClick(type, threshold);
      }
      break;
    case CP_SENSECOLOR:
      // Sense the color of an object over the light sensor and send back
      // a CP_SENSECOLOR_REPLY response.
      sendColorSenseResponse();
      break;
  }
}

// Send a color sense response back to the host computer.
void sendColorSenseResponse() {
  // Perform a color sense with NeoPixel #1 and the light sensor.
  uint8_t red, green, blue;
  CircuitPlayground.senseColor(red, green, blue);
  // Construct a response data packet and send it.
  uint8_t data[4] = {0};
  data[0] = CP_SENSECOLOR_REPLY;
  data[1] = red;
  data[2] = green;
  data[3] = blue;
  // Send the response.
  Firmata.sendSysex(CP_COMMAND, 4, data);
}


// Read the version and send a response packet
void sendImplemenationVersionResponse() {
  // Construct a response data packet and send it.
  uint8_t data[4] = {0};
  data[0] = CP_IMPL_VERS_REPLY;
  data[1] = 12;
  data[2] = 14;
  data[3] = 19;
  // Send the response.
  Firmata.sendSysex(CP_COMMAND, 4, data);
}

// Read the accelerometer and send a response packet.
void sendAccelResponse() {
  // Get an accelerometer X, Y, Z reading.
  sensors_event_t event;
  CircuitPlayground.lis.getEvent(&event);
  // Construct a response data packet.
  uint8_t data[13] = {0};
  data[0] = CP_ACCEL_READ_REPLY;
  // Put the three 32-bit float X,Y,Z reading into the packet.
  // Note that Firmata.sendSysex will automatically convert bytes into
  // two 7-bit bytes that are Firmata/MIDI compatible.
  // Use a union to easily grab the bytes of the float.
  union {
    float value;
    uint8_t bytes[4];
  } reading;
  // Grab each X, Y, Z float byte value and copy it into the response.
  reading.value = event.acceleration.x;
  memcpy(data + 1, reading.bytes, 4);
  reading.value = event.acceleration.y;
  memcpy(data + 5, reading.bytes, 4);
  reading.value = event.acceleration.z;
  memcpy(data + 9, reading.bytes, 4);
  // Send the response.
  Firmata.sendSysex(CP_COMMAND, 13, data);
}

// Read the accelerometer tap detection and send a response packet.
void sendTapResponse() {
  // Get the accelerometer tap detection state.
  uint8_t click = CircuitPlayground.lis.getClick();
  // Construct a response data packet and send it.
  uint8_t data[2] = {0};
  data[0] = CP_ACCEL_TAP_REPLY;
  data[1] = click;
  // Send the response.
  Firmata.sendSysex(CP_COMMAND, 2, data);
}

// Read the capacitive sensor state and send a response packet.
void sendCapResponse(uint8_t pin) {
  // Get the cap sense value for the provided input pin.
  int32_t value = CircuitPlayground.readCap(pin, CAP_SAMPLES);
  // Build a response data packet and send it.  The response includes:
  // - uint8_t: CP_CAP_REPLY value
  // - uint8_t: pin number of the read input
  // - int32_t: cap sensor value, large values mean the input was touched
  union {
    struct {
      uint8_t type;
      uint8_t pin;
      int32_t value;
    } data;
    uint8_t bytes[6];
  } response;
  response.data.type = CP_CAP_REPLY;
  response.data.pin = pin;
  response.data.value = value;
  // Send the response, this will expand each byte into 2 bytes of 7-bit data.
  Firmata.sendSysex(CP_COMMAND, 6, response.bytes);
}

/*==============================================================================
   SYSEX-BASED commands
  ============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;

  switch (command) {
    // Circuit playground command processing:
    case CP_COMMAND:
      if (argc < 1) return;
      circuitPlaygroundCommand(argv[0], argc - 1, argv + 1);
      break;
    case I2C_REQUEST:
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
        Firmata.sendString("10-bit addressing not supported");
        return;
      }
      else {
        slaveAddress = argv[0];
      }

      switch (mode) {
        case I2C_WRITE:
          Wire.beginTransmission(slaveAddress);
          for (byte i = 2; i < argc; i += 2) {
            data = argv[i] + (argv[i + 1] << 7);
            wireWrite(data);
          }
          Wire.endTransmission();
          delayMicroseconds(70);
          break;
        case I2C_READ:
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          readAndReportData(slaveAddress, (int)slaveRegister, data);
          break;
        case I2C_READ_CONTINUOUSLY:
          if ((queryIndex + 1) >= I2C_MAX_QUERIES) {
            // too many queries, just ignore
            Firmata.sendString("too many queries");
            break;
          }
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = (int)I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          queryIndex++;
          query[queryIndex].addr = slaveAddress;
          query[queryIndex].reg = slaveRegister;
          query[queryIndex].bytes = data;
          break;
        case I2C_STOP_READING:
          byte queryIndexToSkip;
          // if read continuous mode is enabled for only 1 i2c device, disable
          // read continuous reporting for that device
          if (queryIndex <= 0) {
            queryIndex = -1;
          } else {
            // if read continuous mode is enabled for multiple devices,
            // determine which device to stop reading and remove it's data from
            // the array, shifiting other array data to fill the space
            for (byte i = 0; i < queryIndex + 1; i++) {
              if (query[i].addr == slaveAddress) {
                queryIndexToSkip = i;
                break;
              }
            }

            for (byte i = queryIndexToSkip; i < queryIndex + 1; i++) {
              if (i < I2C_MAX_QUERIES) {
                query[i].addr = query[i + 1].addr;
                query[i].reg = query[i + 1].reg;
                query[i].bytes = query[i + 1].bytes;
              }
            }
            queryIndex--;
          }
          break;
        default:
          break;
      }
      break;
    case I2C_CONFIG:
      delayTime = (argv[0] + (argv[1] << 7));

      if (delayTime > 0) {
        i2cReadDelayTime = delayTime;
      }

      if (!isI2CEnabled) {
        enableI2CPins();
      }

      break;
    case SERVO_CONFIG:
      if (argc > 4) {
        // these vars are here for clarity, they'll optimized away by the compiler
        byte pin = argv[0];
        int minPulse = argv[1] + (argv[2] << 7);
        int maxPulse = argv[3] + (argv[4] << 7);

        if (IS_PIN_DIGITAL(pin)) {
          if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
            detachServo(pin);
          }
          attachServo(pin, minPulse, maxPulse);
          setPinModeCallback(pin, PIN_MODE_SERVO);
        }
      }
      break;

    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      } else {
        //Firmata.sendString("Not enough data");
      }
      break;
    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;
  }
}

void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing
  // Arduino.h to get SCL and SDA pins
  for (i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, PIN_MODE_I2C);
    }
  }

  isI2CEnabled = true;

  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
}

/*==============================================================================
   SETUP()
  ============================================================================*/

void systemResetCallback()
{
  isResetting = true;

  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default

  // Reset circuit playground components to a default state with nothing running.
  // (i.e. no pixels lit, no sound, no data streaming back)

  digitalWrite(13, LOW);
  delay(100);
  digitalWrite(13, HIGH);
  delay(100);

  circuitPlaygroundReset();

  if (isI2CEnabled) {
    disableI2CPins();
  }

  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, PIN_MODE_ANALOG);
    } else if (IS_PIN_DIGITAL(i)) {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }

    servoPinMap[i] = 255;
  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;

  detachedServoCount = 0;
  servoCount = 0;

  /* send digital inputs to set the initial state on the host computer,
     since once in the loop(), this firmware will only send on change */
  isResetting = false;
}

void circuitPlaygroundReset() {
  // Reset the circuit playground components into a default state
  // with none of the pixels lit, no tones playing, and no cap touch
  // or accelerometer data streaming back.

  // Make speaker shutdown an output and turn off the speaker.
  pinMode(CPLAY_SPEAKER_SHUTDOWN, OUTPUT);
  digitalWrite(CPLAY_SPEAKER_SHUTDOWN, LOW);
  //noTone(CPLAY_SPEAKER); //TODO: NOTONE BROKEN! LOCKS

  // Turn off all the NeoPixels.
  CircuitPlayground.strip.clear();
  CircuitPlayground.strip.show();

  // Reset the accelerometer to a default range.
  CircuitPlayground.lis.setRange(LIS3DH_RANGE_2_G);
  delay(100);
  CircuitPlayground.lis.setClick(2, 80);
  delay(100);

  // Turn off streaming of tap, accel, and cap touch data.
  streamTap = false;
  streamAccel = false;
  for (int i = 0; i < CAP_COUNT; ++i) {
    cap_state[i].streaming = false;
  }
}

void setup()
{

  // Circuit playground setup:
  if (!CircuitPlayground.begin()) {
    // Failed to initialize circuit playground, fast blink the red LED on the board.
    pinMode(13, OUTPUT);
    while (1) {
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      delay(100);
    }
  }

  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);

  Firmata.begin(115200);

  systemResetCallback();  // reset to default config
}

/*==============================================================================
   LOOP()
  ============================================================================*/
void loop()
{
  byte pin, analogPin;

  /* DIGITALREAD - as fast as possible, check for changes and output them to the
     FTDI buffer using Serial.print()  */
  checkDigitalInputs();

  /* STREAMREAD - processing incoming messagse as soon as possible, while still
     checking digital inputs.  */
  while (Firmata.available()) {
    Firmata.processInput();
  }

  // TODO - ensure that Stream buffer doesn't go over 60 bytes

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && pinConfig[pin] == PIN_MODE_ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          Firmata.sendAnalog(analogPin, cpxAnalogRead(analogPin));
        }
      }
    }
    // report i2c data for all device with read continuous mode enabled
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes);
      }
    }

    if ( accLoopCounter++ > numLoops) {
        accLoopCounter = 0 ;

        // Check if an accelerometer event should be streamed to the firmata client.
        if (streamAccel) {
            sendAccelResponse();
        }
    }

      // Check if a tap event should be streamed to the firmata client.
      if (streamTap) {
          sendTapResponse();
      }

    // Check if any cap touch inputs should be streamed to the firmata client.

    if (cap_state[touch_channel].streaming)
    {
      sendCapResponse(cap_state[touch_channel].pin);
    }
    touch_channel++;
    if (touch_channel >= CAP_COUNT){
      touch_channel = 0;
    }

  }

}

void printData(char * id,  long data)
{
  char myArray[64] ;

  String myString = String(data);
  myString.toCharArray(myArray, 64) ;
  Firmata.sendString(id) ;
  Firmata.sendString(myArray);
}
