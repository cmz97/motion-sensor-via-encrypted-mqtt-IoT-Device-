import network
import ubinascii
import ntptime
import machine
import esp32
import upip
import sys
import time
from machine import Timer
from machine import TouchPad, Pin
from machine import sleep
from math import atan2
from math import sqrt
from math import acos
from machine import PWM
from math import floor

global ssid
global wlan
global touchThreshold

FIRSTIME = True

import micropython

# Button A: 15
# Button B: 32
# Button C: 14
# onBoard LED: 13
# ADXL343 IMU I2C Addr 0x53 (83)

# Green LED: 12
# Red LED: 27
# Yellow LED: 33

ssid = 'LAWRENCE'
passphrase = '11111111'

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print('connecting to network...')
    wlan.connect(ssid, passphrase)
    while not wlan.isconnected():
        pass

print('Oh Yes! Get connected')
print('Connected to ' + ssid)
print('MAC Address: ' + ubinascii.hexlify(network.WLAN().config('mac'),':').decode())
print('IP Address: ' + wlan.ifconfig()[0])
print('')

try:
    import umqtt.simple
    import umqtt.robust
    import hmac
except:
    upip.install("micropython-umqtt.simple")
    upip.install("micropython-umqtt.robust")
    upip.install("micropython-hmac")

from umqtt.simple import MQTTClient
from crypt import CryptAes


STATE = 0


i2cAddrGolden = [60, 72, 83]
IMU_I2C_ADDR = 83
TMP_I2C_ADDR = 72

IMU_DATA_FORMAT_REG = 0x31
IMU_BW_RATE_REG = 0x2C
IMU_POWER_CTL_REG = 0x2D
TMP_CONFIG_REG = 0x03


onBoardLed = Pin(13, Pin.OUT, Pin.PULL_DOWN)
yellowLed = Pin(27, Pin.OUT, Pin.PULL_DOWN)
yellowLed.off()
redLed = Pin(33, Pin.OUT, Pin.PULL_DOWN)
redLed.off()
greenLed = Pin(12, Pin.OUT, Pin.PULL_DOWN)
greenLed.off()

# --------- Helper Functions  ----------

def from2sCompBits2SignedInt(tempByteArr, bits):
    val = int.from_bytes(tempByteArr, 'big', True)
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val # return positive value as is

def twosCompliments(num):
    if num >= 0:
        return num
    else:
        return int(bin(num % (1<<8))) #invert and plus 1, 16 bits

def getRightValueIn10Bit(byteArray):
    tempByteArr = bytes([byteArray[1],byteArray[0]])
    temp16bitVal = int.from_bytes(tempByteArr, 'big', False) # take all 16 value
    temp10bitVal = temp16bitVal & 0b1111111111 #the front is padding, take the 10 from the callback
    tempByteArr = bytes([temp10bitVal >> 8, temp10bitVal & 0b11111111]) #split the high order bit and the lower order bits
    return from2sCompBits2SignedInt(tempByteArr,10) #signed 10 bit bytes array to int

def sampleAndHoldAccel(samples, xAverage = 0, yAverage = 0, zAverage = 0):
    timer = machine.RTC()
    start = timer.datetime()
    for _ in range(samples):
        xAverage += getRightValueIn10Bit(i2c.readfrom_mem(IMU_I2C_ADDR, 0x32, 2))
        yAverage += getRightValueIn10Bit(i2c.readfrom_mem(IMU_I2C_ADDR, 0x34, 2))
        zAverage += getRightValueIn10Bit(i2c.readfrom_mem(IMU_I2C_ADDR, 0x36, 2))
    end = timer.datetime()
    delT = (end[7]-start[7])*0.000001
    accelSums = ((int(xAverage) >> 2) * 0.0156, (int(yAverage) >> 2) * 0.0156, (int(zAverage) >> 2) * 0.0156)
    xAverage = (int(xAverage / samples) >> 2) * 0.0156
    yAverage = (int(yAverage / samples) >> 2) * 0.0156
    zAverage = (int(zAverage / samples) >> 2) * 0.0156
    return xAverage,yAverage,zAverage, delT, accelSums

def getAngle(xAverage, yAverage, zAverage):
    roll = atan2(yAverage , zAverage) * 57.3
    pitch = atan2((- xAverage) , sqrt(yAverage * yAverage + zAverage * zAverage)) * 57.3
    if zAverage >= 1 :
        zAverage = 1.0
    elif zAverage <= -1:
        zAverage = -1.0

    theta = acos(zAverage) * 57.3#radian 2 degree
    return roll, pitch, theta

def getRightValueIn13Bit(byteArray):
    temp16bitVal = int.from_bytes(byteArray, 'big', False) # take all 16 value
    temp13bitVal = temp16bitVal >> 3 #shift out the 3 lsb event flag
    tempByteArr = bytes([temp13bitVal >> 8, temp13bitVal & 0b11111111]) #split the high order bit and the lower order bits
    return from2sCompBits2SignedInt(tempByteArr,13) #signed 10 bit bytes array to int

def sampleAndHoldTemperature(samples, tempAverage = 0):
    for _ in range(samples):
        tempAverage += getRightValueIn13Bit(i2c.readfrom_mem(TMP_I2C_ADDR, 0x00, 2))
    tempAverage = tempAverage / (samples * 16)
    return tempAverage

# --------- Senor Initialization ----------
def interfacingSensor():
    global i2c
    i2c = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(23), freq=400000)
    print('scanning I2C Now ...')
    i2cAddrs = i2c.scan()
    print(i2cAddrs)

    for addr in i2cAddrGolden:
        if addr not in i2cAddrs:
                print('I2C Device Error!, Incorrect or not enough I2C Connected!')
                sys.exit()

    print('\nCorrect I2C Device Detected!\n')

    #Set Resolution and Range
    data = i2c.readfrom_mem(IMU_I2C_ADDR, IMU_DATA_FORMAT_REG, 1) #read Register 0x31—DATA_FORMAT (Read/Write)
    data = int.from_bytes(data, 'big')
    print('data:' + str(data))
    data = data & 0b11110100 # 2-g range and 10 bit mode
    i2c.writeto_mem(IMU_I2C_ADDR, IMU_DATA_FORMAT_REG, bytes([data]))
    print('Resolution and Range Set!')

    #Set ODR Speed
    data = i2c.readfrom_mem(IMU_I2C_ADDR, IMU_BW_RATE_REG, 1) #Register 0x2C—BW_RATE (Read/Write)
    data = int.from_bytes(data, 'big')
    print('data:' + str(data))
    data &= 0b11110000 # clear last four bit
    data |= 0b00001101 # 800HZ Rate
    i2c.writeto_mem(IMU_I2C_ADDR, IMU_BW_RATE_REG, bytes([data]))
    print('ODR Speed Set!')

    #Set Measurement Mode
    data = i2c.readfrom_mem(IMU_I2C_ADDR, IMU_POWER_CTL_REG, 1) #Register 0x2C—BW_RATE (Read/Write)
    data = int.from_bytes(data, 'big')
    print('data:' + str(data))
    data |= 0b00001000 # Enable Measurement Mode
    i2c.writeto_mem(IMU_I2C_ADDR, IMU_POWER_CTL_REG, bytes([data]))
    print('measurement mode on!')

    print('sample and holding ... (1000 sample)')
    xAverage, yAverage, zAverage, _ , _ = sampleAndHoldAccel(1000) #first hardware offset
    print('xyz 10 sample hold avg: '+ str(xAverage) + ', ' + str(yAverage) + ', ' + str(zAverage))

    #write offset to memory
    i2c.writeto_mem(IMU_I2C_ADDR, 0x1E, bytes([twosCompliments(int(xAverage / 4))])) # concatenate to 1 byte, MSB
    i2c.writeto_mem(IMU_I2C_ADDR, 0x1F, bytes([twosCompliments(int(yAverage / 4))])) # concatenate to 1 byte, MSB
    i2c.writeto_mem(IMU_I2C_ADDR, 0x20, bytes([twosCompliments(int((zAverage + 32) / 4))])) # concatenate to 1 byte, MSB

    xAverage, yAverage, zAverage, _ , _ = sampleAndHoldAccel(1000) #then software offset
    print('xyz 10 sample hold avg: '+ str(xAverage) + ', ' + str(yAverage) + ', ' + str(zAverage))
    print('now software offset the values')

    print('setting temperature chip resolution ...')

    #Set Resolution and Range
    data = i2c.readfrom_mem(TMP_I2C_ADDR, TMP_CONFIG_REG, 1) #read Register 0x31—DATA_FORMAT (Read/Write)
    data = int.from_bytes(data, 'big')
    print('data:' + str(data))
    data = data | 0b10000000 # set the high resolution mode
    i2c.writeto_mem(TMP_I2C_ADDR, TMP_CONFIG_REG, bytes([data]))
    print('Resolution Set! Waiting for button input')

    print('All initiation is set!')


# --------- Irq Handler and Functions ----------
def button1_int_handler(pin):
    global STATE
    global onBoardLed
    print('button one pressed')
    if STATE is not 1:
        STATE = 1
        onBoardLed.on()
        machine.enable_irq(machine.disable_irq())

def button2_int_handler(pin):
    global STATE
    global onBoardLed
    print('button two pressed')
    if STATE is not 2:
        STATE = 2
        onBoardLed.off()
        machine.enable_irq(machine.disable_irq())

# ---------- Init GPIO ------------
button1 = Pin(15, Pin.IN, Pin.PULL_UP)
button1.irq(trigger=Pin.IRQ_RISING, handler=button1_int_handler)

button2 = Pin(32, Pin.IN, Pin.PULL_UP)
button2.irq(trigger=Pin.IRQ_RISING, handler=button2_int_handler)

#----------MQTT Shit-----------
sessionIDTopic = 'SPINNER/SESSIONID'
sensorDataTopic = 'SPINNER/SENSORDATA'
acknowledgeTopic = 'SPINNER/ACKNOW'
WAIT4ACKNOW = True;

client_id = ubinascii.hexlify(machine.unique_id())

def spinnerDemo(sessionID):
    xAverage, yAverage, zAverage, _, _ = sampleAndHoldAccel(10)
    #format its value to compress
    dataX, dataY, dataZ = "{0:.2f}".format(xAverage), "{0:.2f}".format(yAverage), "{0:.2f}".format(zAverage)
    dataTemp = "{0:.2f}".format(sampleAndHoldTemperature(10))

    #Checksum
    print(xAverage, yAverage, zAverage, dataTemp)
    sensor_data = ubinascii.hexlify(dataX + dataY+ dataZ+ dataTemp)
    print('Hex: ' + str(sensor_data) + '[' + str(len(sensor_data)) + ']')

    #AES
    myCrypto = CryptAes(sessionID)
    myCrypto.encrypt(sensor_data)
    myHMAC = myCrypto.sign_hmac(sessionID)
    jsonData = myCrypto.send_mqtt(myHMAC)

    #send to Topic
    client.publish(topic=sensorDataTopic,msg=jsonData)


def sessionIDTopicInterruptHandler(topic, msg):
    strTopic = str(topic).strip('b\'').strip('\'')
    if strTopic == sessionIDTopic:
        print('Msg From SessionID Topic: ' + str(msg))
        spinnerDemo(msg)

    elif strTopic == acknowledgeTopic:
        print('Msg From Acknowledge Topic: ' + str(msg))
        WAIT4ACKNOW = False

def connect2broker(client_id, mqtt_server, username, password, port):
    client = MQTTClient(client_id, server = mqtt_server, user = username, password = password, port = port)
    client.set_callback(sessionIDTopicInterruptHandler)
    client.connect()
    client.subscribe(sessionIDTopic)
    client.subscribe(acknowledgeTopic)
    print('Connected to %s,  to %s topic' % (mqtt_server, sessionIDTopic))
    return client

client = connect2broker(client_id, "farmer.cloudmqtt.com", 'swpdieal', 'MdwlLdTQWzbI', '14584')

while True:
    if STATE is 1:
        print('------ !!!! START Iteration !!!! -----')

        interfacingSensor()
        print('------ Waiting for Acknowledgement! -----')
        while (WAIT4ACKNOW):
            pass
        WAIT4ACKNOW = True
        FIRSTIME = False
        print('------ !!!! END Iteration !!!! -----\n')

    elif STATE is 2 and not FIRSTIME:
        try:
            client.check_msg()
            time.sleep(1)
        except OSError as e:
            print('Failed to connect to MQTT broker. Reconnecting...')
            time.sleep(1)
            machine.reset()


print('\n\n')
