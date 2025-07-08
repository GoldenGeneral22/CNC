# Initialisieren von Bibliotheken
import time
from machine import I2C, Pin, PWM
from lib.rotary import Rotary
from lib.pio import rpmReader
from lib.machine_i2c_lcd import I2cLcd

# Festlegung der Pinbelegung
lcdSDA = 4
lcdSCL = 5
rotaryDT = 18
rotaryCLK = 19
rotarySW = 17
hallPin = 15
pwmPin = 16

# Initialisieren von RPM-Variablen
targetRPM = 10_000
targetRPMModifier = 200
currentRPM = 0

reader = rpmReader(pinNum = hallPin, stateMachineID = 0, frequency = 1_000_000, smoothingFactor = 10)

rotary = Rotary(rotaryDT, rotaryCLK, rotarySW)

pwm = PWM(Pin(pwmPin))
pwm.freq(1_000)

i2c = I2C(0, sda=Pin(lcdSDA), scl=Pin(lcdSCL), freq=100_000)
lcd = I2cLcd(i2c, 0x27, 2, 16)
lcd.backlight_on()

def rotaryChanged(change):
    global targetRPM
    if change == Rotary.ROT_CW:
        targetRPM = targetRPM + targetRPMModifier
    elif change == Rotary.ROT_CCW:
        targetRPM = targetRPM - targetRPMModifier
    elif change == Rotary.SW_PRESS:
        return # Stuff happens here
    elif change == Rotary.SW_RELEASE:
        return # Stuff happens here
rotary.add_handler(rotaryChanged)

while True:
    rpm = reader.read()
    if rpm is not None:
        currentRPM = rpm
    
    if currentRPM < targetRPM:
        diff = targetRPM - currentRPM
        dutyRatio = currentRPM / targetRPM
        dutyRatio = max(0.1, min(dutyRatio, 1))
        pwm.duty_u16(int(dutyRatio * 65535))
    elif currentRPM > targetRPM:
        dutyRatio = currentRPM / targetRPM
        dutyRatio = max(0, min(dutyRatio, 1))
        pwm.duty_u16(int(dutyRatio * 65535))

    lcd.clear()
    lcd.putstr("Target: " + str(targetRPM) + "\n" + "Current: " + str(int(currentRPM)))

    time.sleep(0.02)