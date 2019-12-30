<div style="text-align:left;color:#990033; font-family:times, serif; font-size:3.0em">The pymata-cpx API</div>

<br>
You may view the complete pymata-cpx API by clicking on
[this link.](https://htmlpreview.github.io/?https://github.com/MrYsLab/pymata-cpx/blob/master/docs/api.html)
The API supports both the Circuit Playground Express input sensors and
output devices. Let's start by exploring the input sensors.

## Circuit Playground Express Input Sensors
Below is the list of the Circuit Playground Express input devices
supported by pymata-cpx:

* The Accelerometer
* The Push Buttons And Slide Switch
* The Touchpad Sensors
* The Light Sensor
* The Temperature Sensor
* The Sound Sensor
* The Tap Sensor


## The API Input Device Start And Stop Methods
All of these devices have two methods associated with them. One to start
continuous reporting for the sensor, and the second to stop
continuous reporting.

For example, to continuously receive change reports for Button B, call

**cpx_button_b_start** 

All the *start* methods require that you provide a callback method.
When the sensor changes value, the callback is invoked with information about the 
change. More about this in a moment.

If you no longer wish to receive change notifications, one would call the *stop* method for the sensor.
Continuing with using Button B as an example, to discontinue Button B from sending value change
notifications, call:

**cpx_button_b_stop**


## Understanding Callbacks
When using any of the input device **start** methods, you must provide a
callback method. The callback method is a method that you write to accept 
sensor change notifications. The format for all input callback methods is the same. If
a callback is created with the name ***my_callback***, its signature
looks as follows : 

**my_callback(data)**<br>

The data parameter is provided to the callback method in the form of a
list. The length of the list and the data it contains varies depending
upon which input device generated the report. 

You may specify a separate callback for each of the input devices you've
enabled, or you can share a callback across multiple sensors. The choice
is yours.

## Understanding The Callback Data
For each input device type, the API specifies the list data received by
the callback method. The format for the data is as follows:

**Index 0** is the **pin type** for the input device. A value of
**32** indicates that this is a digital type device, and a value of **2**
indicates that this device is an analog device. Knowing the pin type is useful
if you wish to have your callback method handle multiple sensors. 

**Index 1** is the internal pin number for the device. See the tables below.

**Index 2 and above*** contain the data value of the sensor. The data is sensor dependent.

For each **start** method, there is a comment in the API that describes the format for the
data parameter that will be provided when your callback method is invoked.

For example, looking at the API entry for [cpx_button_b_start](https://htmlpreview.github.io/?https://github.com/MrYsLab/pymata-cpx/blob/master/docs/api.html#pymata_cpx.PyMataCpx.cpx_button_b_start)
, here is what you would see:

```python
 def cpx_button_b_start(self, callback)

    Enable button and report state changes in the callback.

    :param callback: a list of values described below.

    Parameters sent to callback:

    [Digital Pin Type: 32, Pin Number: 5, switch value: 1 if pressed zero if released.]

```
The last line describes the values placed in the data parameter for the callback method. We would expect to see
a list that contains: [32, 5, DATA_VALUE], where DATA_VALUE is either a zero or 1.

Let's look at an example that returns multiple values. Here is the API entry for receiving notification 
of changes to a 
[capacitive touch pad:](https://htmlpreview.github.io/?https://github.com/MrYsLab/pymata-cpx/blob/master/docs/api.html#pymata_cpx.PyMataCpx.cpx_cap_touch_start)

```python
 def cpx_cap_touch_start(self, input_pin, callback)

    Start continuous capacitive touch queries for the specified input pin. Will invoke the provided callback each time a new cap touch result is available.

    :param input_pin: must be one of the Ax pin numbers. Use only the number.

                  example: cpx_cap_touch_start(5, a_callback_function)

    :param callback: a list of values described below.

    Parameters sent to callback:

    [Analog Pin Type: 2, Pin Number: 1-7, Touched: True or False, Raw data value]

```

For a capacitive touchpad callback, we would expect the data to look like
[2, PIN_NUMBER, BOOLEAN_STATE, RAW_DATA_VALUE]

Index 0 identifies the event coming from an analog device. Index 1 contains the pin number where
the change was detected. Index 2 contains either True or False to indicate the current state
of the touch sensor, and Index 3 contains the raw analog value for the sensor if you wish to use that
value.

All of the **start** methods in the API identify the items you can expect to find in the 
callback list.

##Pin Numbers
Here are the pin numbers you would expect to see in the callback data list for each of the sensor types.
Notice that pin numbers are unique across both digital and analog sensors.


### Digital Inputs
|   Digital Pin Number  |  Device   |
|------------------  |------------ |
|          4         |Button A|
|5 |Button B  |
|7 |Slide Switch|
|12|Tap Sensor|


### Analog Inputs
| Analog Pin Number 	|    Sensor    	|
|------------------	|------------	|
|          1 - 7         	|   Touch |
|          8         	|   Light |
|              9     	|   Temperature | 
|          10         	|   Microphone |
|          11         	|   Accelerometer |



<br>
##The Circuit Playground Express Output Actuators

Below is the list of the Circuit Playground Express on-board actuators supported by pymata-cpx:

* Red "Board LED"
* On-board Neopixels
* Tone Generator

# The Examples
The best way to understand how to use the API is to see some example code.
The following table contains a link to source code for each example and a list of the sensors and actuators
that the example uses.

With the exception of the *buttons_switch_and_pixels* example, all of the example code is
contained within a class. If you prefer to work in a procedural style, the *buttons_switch_and_pixels*
provides an illustration of the procedural style.

|   Example  |  Sensors   |  Actuators     |
|------------------  |------------ | ------------ |
| [buttons_switch_and_pixels](https://github.com/MrYsLab/pymata-cpx/blob/master/examples/buttons_switch_and_pixels.py) | Button A, Button B, Slide Switch | Neopixels 1, 3, 6 and 8 |
| [light_meter](https://github.com/MrYsLab/pymata-cpx/blob/master/examples/light_meter.py) | Light Sensor | All 10 Neopixels |
| [the clapper](https://github.com/MrYsLab/pymata-cpx/blob/master/examples/the_clapper.py) | Sound sensor | All 10 Neopixels |
| [the tapper](https://github.com/MrYsLab/pymata-cpx/blob/master/examples/the_tapper.py) | Accelorometer tap sensor | All 10 Neopixels | 
| [thermometer](https://github.com/MrYsLab/pymata-cpx/blob/master/examples/thermometer.py) | Temperature sensor | All 10 Neopixels |
| [tilted](https://github.com/MrYsLab/pymata-cpx/blob/master/examples/tilted.py) | Accelerometer | None |
| [touch_piano](https://github.com/MrYsLab/pymata-cpx/blob/master/examples/touch_piano.py) | All 7 Touchpads | Tone Generator |




<br> <br>

Copyright (C) 2019 Alan Yorinks. All Rights Reserved.
