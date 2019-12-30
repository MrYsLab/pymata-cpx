<div style="text-align:left;color:#990033; font-family:times, serif; font-size:3.0em">The pymata-cpx API</div>

<br>
You may view the complete pymata-cpx API by clicking on
[this link.](https://htmlpreview.github.io/?https://github.com/MrYsLab/pymata-cpx/blob/master/docs/api.html)
The API supports both the Circuit Playground Express input sensors and
output devices. Let's start by exploring the input sensors.

## Circuit Playground Express Input Sensors
This is the list of the Circuit Playground Express input devices
supported by pymata-cpx:

* The Accelerometer
* The Push Buttons And Slide Switch
* The Touch Sensors
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

All **start** methods require a callback to be provided to receive the
change notifications. Any time the state of Button B changes, pymata-cpx
calls the callback method. More on that in a moment. When a *start*
method is called the pin mode is automatically set for the pin. If a pin 

To stop the change
notifications for Button B, one would call the *cpx_button_b_stop*
method.

## Understanding Callbacks
When using any of the input device **start** methods, you must provide a
callback method. This is a method that you write to accept sensor change
notifications The format for all input callback methods is the same. If
a callback is created with the name ***my_callback***, its signature
would look as follows : 

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

**Index 0** is the pin type for the input device. A value of
**32** indicates that this is a digital type device, and a value of **2**
indicates that this device is an analog device. 

**Index 1** is the internal pin number for the device.

Here is a list of pin
numbers for the various devices:

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



### The Accelerometer
### The Push Buttons And Slide Switch
### The Touch Sensors
### The Light Sensor
### The Temperature Sensor
### The Sound Sensor
### The Tap Sensor

<br>
## Circuit Playground Express Output Devices

## Pin Number Summary
<br>

## Callback Parameters Summary
<br>

# Using Callbacks
For enhanced real-time support when monitoring the Circuit Playground
Express input devices, such as button presses, changes in temperature,
or any of the many other integrated input devices, you must provide a
callback function to receive and process those changes.

Let's look at a simple working example that will monitor


<br> <br>

Copyright (C) 2019 Alan Yorinks. All Rights Reserved.
