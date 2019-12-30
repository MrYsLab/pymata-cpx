![logo](https://github.com/MrYsLab/pymata-cpx/blob/master/docs/images/cpx.jpg)

## Control A Circuit Playground Express From Your PC With An Easy To Use Python 3 API

It supports the following CPX devices:
* The Buttons and Slide Switch.
* The D13 Board LED.
* The 10 onboard neo-pixels.
* Tone generation using the onboard speaker.
* The accelerometer, including tap sensing.
* The temperature sensor.
* The light sensor.
* The sound sensor.
* Touchpad sensors.

An example to animate the neopixels and to start and stop the animation
by simply tapping the Playground Express:

```python
import random
import time

from pymata_cpx.pymata_cpx import PyMataCpx


class TheTapper():
    """
    Illuminate the neopixels in a counter-clockwise fashion with randomly generated colors.
    When you tap the playground express, the neopixels will stop changing and the
    program pauses. Tap again and the neopixels will start again.
    """
    def __init__(self):
        # create an instance of the API
        self.p = PyMataCpx()

        print('Tap the playground express to stop the neopixels from moving.')
        print('Tap again, to start them up')
        print('The tap state will be printed to the console')

        # Start monitoring for tap events and
        # send event notifications to the "tapped" callback method.
        self.p.cpx_tap_start(self.tapped)
        
        # flag to start and stop the light show
        self.go = True

        while True:
            try:
                # run the light show
                for neopixel in range(0, 10):
                    # check the go flag
                    if self.go:
                        self.p.cpx_pixels_clear()
                        self.p.cpx_pixels_show()
                        r = random.randint(0, 254)
                        g = random.randint(0, 254)
                        b = random.randint(0, 254)
                        self.p.cpx_pixel_set(neopixel, r, g, b)
                        self.p.cpx_pixels_show()
                        time.sleep(.2)
                    else:
                        self.p.cpx_pixels_clear()
                        self.p.cpx_pixels_show()
                        time.sleep(.001)
            except KeyboardInterrupt:
                # If you press control-C, cleanly exit
                self.p.cpx_pixels_clear()
                self.p.cpx_pixels_show()
                self.p.cpx_close_and_exit()

    def tapped(self, data):
        """
        :param data: data[0] = data type (analog = 2, digital =32)
                     data[1] = pin for device 27
                     data[2] = tap data - list of booleans.
                               First value for 1 tap
                               Second value for 2 taps
        """
        # for any taps, toggle the go flag
        # print out the current go state
        if data[2] != [False, False]:
            self.go = not self.go
            print(self.go)

# start the program
TheTapper()

```

This project was developed with
[Pycharm](https://www.jetbrains.com/pycharm/?from=pymata-cpx)
![logo](https://github.com/MrYsLab/python_banyan/blob/master/images/icon_PyCharm.png)
