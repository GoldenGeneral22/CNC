from machine import Pin, PWM, I2C
import time
from pico_i2c_lcd import I2cLcd
import rotary_irq_rp2 as RotaryIRQ
import mp_button as Button

# Constants
pulsesPerRevolution = 1 # Anzahl der Magnete am Spindelrad
averageWindowSize = 10  # Anzahl Messwerte fuer gleitenden Mittelwert
minDelta = 500  # Mindestabstand der Impulse in us
toleranz = 50
targetRPMmin = 0
targetRPMmax = 30000
targetRPMincr = 250

# Pins
hall_pin = 9
i2c_sda_pin = 26
i2c_scl_pin = 27
rotary_clk_pin = 4
rotary_dt_pin = 5
pwm_pin = 21
button_pin = 6

# Globals
lastPulseTime = 0
rpmHistory = [0] * averageWindowSize
rpmHistoryIndex = 0
smoothedRPM = 0
pwmPercentage = 0
pwmPercentageChange = 1
engagedMatching = False
matchCharacter = "*"

# Function Setup
def hallInterrupt(pin):
    global lastPulseTime, rpmHistory, rpmHistoryIndex, smoothedRPM
    
    now = time.ticks_us()
    deltaTime = time.ticks_diff(now, lastPulseTime)
    if deltaTime > minDelta:
        lastPulseTime = now

        pulseFrequency = 1_000_000 / deltaTime  # in Hz
        currentRPM = (pulseFrequency * 60) / pulsesPerRevolution

        # Gleitender Mittelwert aktualisieren
        rpmHistory[rpmHistoryIndex] = currentRPM
        rpmHistoryIndex = (rpmHistoryIndex + 1) % averageWindowSize
        smoothedRPM = sum(rpmHistory) / averageWindowSize

def buttonInterrupt(pin, event):
    global engagedMatching, matchCharacter
    if(event == Button.PRESSED):
        if(engagedMatching):
            matchCharacter = "*"
        else:
            matchCharacter = " "
        engagedMatching = not engagedMatching

def setPwmPercent(percent):
    if percent < 0:
        percent = 0
    elif percent > 100:
        percent = 100
    duty_u16 = int((percent/100)*65535)
    pwmPin.duty_u16(duty_u16)

# Class and Interrupt Setup
hallPin = Pin(hall_pin, Pin.IN, Pin.PULL_UP)
hallPin.irq(trigger=Pin.IRQ_FALLING, handler=hallInterrupt)
i2c = I2C(0, sda=Pin(i2c_sda_pin), scl=Pin(i2c_scl_pin), freq=400000)
I2C_ADDR = i2c.scan()[0]
lcd = I2cLcd(i2c, I2C_ADDR, 2, 16)
rotaryEncoder = RotaryIRQ(pin_num_clk=rotary_clk_pin, pin_num_dt=rotary_dt_pin, min_val=targetRPMmin, max_val=targetRPMmax, incr=targetRPMincr, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
pwmPin = PWM(Pin(pwm_pin))
pwmPin.freq(2000)
buttonPin = Button(button_pin, callback=buttonInterrupt, internal_pulldown=True)

# Mainloop
lcd.backlight_on()
lcd.hide_cursor()
while True:
    targetRPM = rotaryEncoder.value()

    if(engagedMatching):
        if(smoothedRPM < targetRPM - toleranz):
            pwmPercentage = min(100, pwmPercentage + pwmPercentageChange)
        elif(smoothedRPM > targetRPM + toleranz):
            pwmPercentage = max(0, pwmPercentage - pwmPercentageChange)
        setPwmPercent(pwmPercentage)

    lcd.move_to(0, 0)
    lcd.putstr(matchCharacter + f"Target RPM: {targetRPM:<5}")
    lcd.move_to(0, 1)
    lcd.putstr(f"Smoothed RPM: {int(smoothedRPM):<5}")

    time.sleep_ms(50)