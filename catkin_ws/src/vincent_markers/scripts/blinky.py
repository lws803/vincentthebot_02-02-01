import RPi.GPIO as GPIO
import time
from std_msgs.msg import Int32
import rospy


LedPin = 11    # pin11
def setup():
  GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
  GPIO.setup(LedPin, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led


def listener(data):
    GPIO.output(LedPin, GPIO.HIGH)  # led on
    time.sleep(2) # On the LED for 2 seconds
    GPIO.output(LedPin, GPIO.LOW) # led off


if __name__ == '__main__':     # Program start from here
  setup()
  rospy.init_node('light_listener_node', anonymous=True)
  rospy.Subscriber("light_cue", Int32, listener)
  rospy.spin()