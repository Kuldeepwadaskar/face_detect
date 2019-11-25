#usr/bin/env python

import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=4)

pwm.set_pwm_freq(25)
pwm.set_pwm(1 ,0 ,100)
pwm.set_pwm(0 ,0 ,140)

