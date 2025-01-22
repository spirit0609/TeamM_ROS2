#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import threading
import pigpio
import time

ch1_out = 19
ch2_out = 13
ch3_out = 12
ch4_out = 18
gpio_sw = 5

minPulse = 700
maxPulse = 2000

ch1_in = 23
ch2_in = 24
ch3_in = 17
ch4_in = 27

ch1_rise_tick = 0
ch2_rise_tick = 0
ch3_rise_tick = 0
ch4_rise_tick = 0

pwm1_width = 0
pwm2_width = 0
pwm3_width = 0
pwm4_width = 0

pi = pigpio.pi()

pi.set_mode(gpio_sw, pigpio.INPUT)
pi.set_pull_up_down(gpio_sw, pigpio.PUD_UP)

print("Initializing th ESC. Please turn off the switch.")

while(True):
    sw = pi.read(gpio_sw)
    if(sw == 0): # the switch is on
        print("The switch is not turned off.\n Turn off the switch.")
        time.sleep(0.5)
    else:
        pi.set_servo_pulsewidth(ch1_out, maxPulse) # aileron
        pi.set_servo_pulsewidth(ch2_out, maxPulse) # elevator
        pi.set_servo_pulsewidth(ch3_out, maxPulse) # throttle
        pi.set_servo_pulsewidth(ch4_out, maxPulse) # rudder
        print("Initialization complete.")
        break

print("Now, turn on the switch.")

pi.set_servo_pulsewidth(ch1_out, minPulse)
pi.set_servo_pulsewidth(ch2_out, minPulse)
pi.set_servo_pulsewidth(ch3_out, minPulse)
pi.set_servo_pulsewidth(ch4_out, minPulse)
time.sleep(3)

while(True):
    sw = pi.read(gpio_sw)
    if(sw == 1): # the switch is off
        print("The switch is not turned on. Turn on the switch.")
        print("Make sure that the throttle stick is in the lowest position.")
        time.sleep(0.5)
    else:
        print("Now you are ready to go.")
        break

def gpiocallback(gpio, level, tick):
    global ch1_rise_tick, ch2_rise_tick, ch3_rise_tick, ch4_rise_tick
    global ch1_in, ch2_in, ch3_in, ch4_in
    global ch1_out, ch2_out, ch3_out, ch4_out
    global minPulse, maxPulse
    global pwm1_width, pwm2_width, pwm3_width, pwm4_width
    if gpio == ch1_in :
        if level==1 :
            ch1_rise_tick = tick
        elif level==0 : 
            pwm1_width = tick - ch1_rise_tick
        else :
            pwm1_width = 0

    if gpio == ch2_in :
        if level==1 :
            ch2_rise_tick = tick
        elif level==0 : 
            pwm2_width = tick - ch2_rise_tick
        else :
            pwm2_width = 0

    if gpio == ch3_in :
        if level==1 :
            ch3_rise_tick = tick
        elif level==0 : 
            pwm3_width = tick - ch3_rise_tick
        else :
            pwm3_width = 0

    if gpio == ch4_in :
        if level==1 :
            ch4_rise_tick = tick
        elif level==0 : 
            pwm4_width = tick - ch4_rise_tick
        else :
            pwm4_width = 0

class Receiver(Node):

    def __init__(self):
        super().__init__('Receiver')

        pi = pigpio.pi()
        pi.set_mode(ch1_out, pigpio.OUTPUT)
        pi.set_mode(ch2_out, pigpio.OUTPUT)
        pi.set_mode(ch3_out, pigpio.OUTPUT)
        pi.set_mode(ch4_out, pigpio.OUTPUT)
        pi.set_mode(ch1_in, pigpio.INPUT)
        pi.set_mode(ch2_in, pigpio.INPUT)
        pi.set_mode(ch3_in, pigpio.INPUT)
        pi.set_mode(ch4_in, pigpio.INPUT)

        pi.callback(ch1_in, pigpio.EITHER_EDGE, gpiocallback)
        pi.callback(ch2_in, pigpio.EITHER_EDGE, gpiocallback)
        pi.callback(ch3_in, pigpio.EITHER_EDGE, gpiocallback)
        pi.callback(ch4_in, pigpio.EITHER_EDGE, gpiocallback)

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        time.sleep(1)

class Brushless_controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.ch1_mid = 1452
        self.ch2_mid = 1489
        self.ch4_mid = 1515
        self.range = 500
        timer_period = 0.005
        self.bluetooth_timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global pwm1_width, pwm2_width, pwm3_width, pwm4_width

        if (pwm1_width > 0 and pwm2_width > 0):
            c1 = (pwm1_width - self.ch1_mid)#/self.range
            c2 = (pwm2_width - self.ch2_mid)#/self.range
        else:
            c1 = 0
            c2 = 0
                       
        self.get_logger().info(f"c1 : {c1}, c2 : {c2}, pwm3_width : {pwm3_width}")
        pi.set_servo_pulsewidth(ch1_out, pwm3_width+c1)
        pi.set_servo_pulsewidth(ch2_out, pwm3_width+c2)
        pi.set_servo_pulsewidth(ch3_out, pwm3_width+c1)
        pi.set_servo_pulsewidth(ch4_out, pwm3_width+c2)

if __name__ == '__main__':
    rclpy.init(args=None)
    
    receiver = Receiver()
    brushless_controller = Brushless_controller()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(receiver)
    executor.add_node(brushless_controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = brushless_controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

