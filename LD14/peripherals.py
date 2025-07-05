import RPi.GPIO as GPIO
import threading
import time

class Peripherals:
    def __init__(self):
        # Initialize GPIO if not already setup
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BCM)
            
        # Define pins for buzzer and LED
        self.buzzer_pin = 25  # Adjust as needed for your hardware
        self.led_pin = 12     # Adjust as needed for your hardware
        
        # Setup the pins
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        GPIO.setup(self.led_pin, GPIO.OUT)
        
        # Initialize PWM for buzzer (frequency control)
        self.buzzer_pwm = GPIO.PWM(self.buzzer_pin, 440)  # Start at 440Hz (A4 note)
        self.buzzer_pwm.start(0)  # Start with 0% duty cycle (off)
        
        # Initialize LED state
        self.led_state = False
        GPIO.output(self.led_pin, self.led_state)
        
        # Thread for timed operations
        self.timer_thread = None
        self.running = True
    
    def buzzer_on(self, frequency=440, duration=None):
        """Turn on buzzer with specified frequency"""
        self.buzzer_pwm.ChangeFrequency(frequency)
        self.buzzer_pwm.ChangeDutyCycle(50)  # 50% duty cycle
        
        if duration:
            if self.timer_thread and self.timer_thread.is_alive():
                self.running = False
                self.timer_thread.join(0.1)
            
            self.running = True
            self.timer_thread = threading.Thread(target=self._buzzer_timer, args=(duration,))
            self.timer_thread.daemon = True
            self.timer_thread.start()
    
    def _buzzer_timer(self, duration):
        """Timer thread for buzzer"""
        time.sleep(duration)
        if self.running:
            self.buzzer_off()
    
    def buzzer_off(self):
        """Turn off buzzer"""
        self.buzzer_pwm.ChangeDutyCycle(0)
    
    def led_on(self):
        """Turn on LED"""
        self.led_state = True
        GPIO.output(self.led_pin, True)
    
    def led_off(self):
        """Turn off LED"""
        self.led_state = False
        GPIO.output(self.led_pin, False)
    
    def led_toggle(self):
        """Toggle LED state"""
        self.led_state = not self.led_state
        GPIO.output(self.led_pin, self.led_state)
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        if self.timer_thread and self.timer_thread.is_alive():
            self.timer_thread.join(0.1)
        self.buzzer_off()
        self.led_off()
        # Don't call GPIO.cleanup() here to avoid conflicts with other components