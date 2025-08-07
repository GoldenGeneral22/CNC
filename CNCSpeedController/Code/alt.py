from machine import Pin, PWM, I2C
import time
from pico_i2c_lcd import I2cLcd
from rotary_irq_rp2 import RotaryIRQ
from mp_button import Button

import rotary_irq_rp2
print("rotary_irq_rp2 is a", type(rotary_irq_rp2))
print("Attributes:", dir(rotary_irq_rp2))
print("RotaryIRQ is a", type(rotary_irq_rp2.RotaryIRQ))

# Constants
pulsesPerRevolution = 1 # Anzahl der Magnete am Spindelrad
averageWindowSize = 10  # Anzahl Messwerte fuer gleitenden Mittelwert
minDelta = 500  # Mindestabstand der Impulse in us
toleranz = 50
targetRPMmin = 0
targetRPMmax = 5000
targetRPMincr = 250

# Pins
hall_pin = 11
i2c_sda_pin = 0
i2c_scl_pin = 1
rotary_clk_pin = 2
rotary_dt_pin = 3
pwm_pin = 16
button_pin = 4

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
    print(matchCharacter)

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


addresses = i2c.scan()
print("I2C addresses found:", addresses)
if len(addresses) == 0:
    print("Kein I2C-Gerät gefunden! Bitte Verkabelung prüfen.")
    while True:
        time.sleep(1)  # Programm hier anhalten

I2C_ADDR = 0x27 # i2c.scan()[0]
lcd = I2cLcd(i2c, I2C_ADDR, 2, 16)
rotaryEncoder = rotary_irq_rp2.RotaryIRQ(pin_num_clk=rotary_clk_pin, pin_num_dt=rotary_dt_pin, min_val=targetRPMmin, max_val=targetRPMmax, incr=targetRPMincr, reverse=False)
pwmPin = PWM(Pin(pwm_pin))
pwmPin.freq(2000)
buttonPin = Button(button_pin, callback=buttonInterrupt, internal_pullup=True)

# Mainloop
lcd.backlight_on()
lcd.hide_cursor()
while True:
    buttonPin.update()
    rotaryEncoder.update()
    
    targetRPM = rotaryEncoder.value()

    if(engagedMatching):
        if(smoothedRPM < targetRPM - toleranz):
            pwmPercentage = min(100, pwmPercentage + pwmPercentageChange)
        elif(smoothedRPM > targetRPM + toleranz):
            pwmPercentage = max(0, pwmPercentage - pwmPercentageChange)
        setPwmPercent(pwmPercentage)

    lcd.clear()
    lcd.move_to(0, 0)
    lcd.putstr(matchCharacter + "Target RPM: " + str(targetRPM))
    lcd.move_to(0, 1)
    lcd.putstr(" Smooth RPM: " + str(int(smoothedRPM)))
    #print(f"Target: {targetRPM}", f" Smoothed: {int(smoothedRPM)}")

    time.sleep_ms(200)