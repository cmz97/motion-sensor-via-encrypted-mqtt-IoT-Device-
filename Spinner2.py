import ubinascii
import network
import machine
import utime
import esp32
import upip
from machine import Pin, Timer, I2C, PWM, TouchPad
from ntptime import settime
from time import sleep
import random
import micropython
from crypt import *#CryptAes
from umqtt.simple import MQTTClient
import urequests
import json

try:
    import umqtt.simple
    import umqtt.robust
    import hmac, hashlib
except:
    upip.install("micropython-umqtt.simple")
    upip.install("micropython-umqtt.robust")
    upip.install("hmac")


#Initialization
switch1 = Pin(12, Pin.IN, Pin.PULL_DOWN)
switch2 = Pin(27, Pin.IN, Pin.PULL_DOWN)
g_led = Pin(33, Pin.OUT)
r_led = Pin(15, Pin.OUT)
led_board = Pin(13, Pin.OUT)
pwm = PWM(Pin(33), freq=10, duty=512)
pwm.deinit()
interrupt2 = 0
addr_list = []
temp_last = 0

sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
if not sta_if.isconnected():
    sta_if.connect('LAWRENCE', '11111111')
    while not sta_if.isconnected():
        sleep(1)
    print("Oh Yes! Get connected")
    print("Connected to LAWRENCE")
    print("MAC Address: {0}".format(ubinascii.hexlify(sta_if.config('mac'),':').decode()))
    print("IP Address: {0}".format(sta_if.ifconfig()[0]))

def call(pin):
    pwm.deinit()
    led_board = Pin(13, Pin.OUT)
    led_board.value(1)
    initialize()
    print("Waiting for calibrating")
    x,y,z = calibration()
    print("Calibration Done. The offset value is x:{},y:{},z:{}".format(x,y,z))
    
def cancel_de(pin):
    tim.init(period=300, mode=Timer.ONE_SHOT, callback=call)

#Switch 2 interrupt
def call2(pin):
    global interrupt2, pub
    led_board.value(0)
    pub = 1
    interrupt2 = 1

def cancel_de2(pin):
    tim2.init(period=300, mode=Timer.ONE_SHOT, callback=call2)

tim = Timer(1)
tim2 = Timer(2)
switch1.irq(trigger=Pin.IRQ_FALLING, handler=cancel_de)
switch2.irq(trigger=Pin.IRQ_FALLING, handler=cancel_de2)
i2c = I2C(scl=Pin(22), sda=Pin(23),freq=400000)
acc_id = list(i2c.scan())[1]
temp_id = list(i2c.scan())[0]

def initialize():
    addr_list = list(i2c.scan())
    acc_addr = i2c.readfrom_mem(addr_list[1],0,1)
    temp_addr = i2c.readfrom_mem(addr_list[0],0x0b,1)

    if acc_addr != b'\xe5':
        raise error('Wrong accelerometer device')
    if temp_addr != b'\xcb':
        raise error('Wrong temperature device')

    i2c.writeto_mem(acc_id,0x2d, b'\x08') #Mearsure
    i2c.writeto_mem(acc_id,0x2c, b'\x13') #800Hz-ODR
    i2c.writeto_mem(acc_id,0x31, b'\x08') #Full_Res & Range
    print("Accelerometer Sensor Initialized, Sensor id:{}".format(acc_addr))
    i2c.writeto_mem(temp_id,0x03, b'\x80') #16-bit-Res
    print("Temperature Sensor Initialized, Sensor id:{}".format(temp_addr))

def temp_trans(data,bit):
    if (data & (1 << (bit-1))):
        data = data - (1 << bit)
        return (data - 65536) / 128
    else:
        return data / 128
    return data

def acc_trans(data, bit):
    if (data & (1 << (bit-1))):
        data = data - (1 << bit)
    return data

def calibration():
    clear = bytearray([0, 0, 0])
    i2c.writeto_mem(acc_id,0x1E,clear)
    read = bytearray(6)
    x_offset = 0
    y_offset = 0
    z_offset = 0
    c = 0
    while c < 5000:
        i2c.readfrom_mem_into(acc_id,0x32,read)
        value_x = (read[1] << 8 | read[0]) & 1023
        value_x = acc_trans(value_x, 10)
        value_y = (read[3] << 8 | read[2]) & 1023
        value_y = acc_trans(value_y, 10)
        value_z = (read[5] << 8 | read[4]) & 1023
        value_z = acc_trans(value_z,10)
        x_offset = x_offset + value_x
        y_offset = y_offset + value_y
        z_offset = z_offset + value_z
        c = c + 1
    x_offset = -int((x_offset / 5000) / 4)
    y_offset = -int((y_offset / 5000) / 4)
    z_offset = -int((z_offset / 5000) / 4)
    offset = bytearray([x_offset])
    i2c.writeto_mem(acc_id,0x1E,offset)
    offset = bytearray([y_offset])
    i2c.writeto_mem(acc_id,0x1F,offset)
    offset = bytearray([z_offset])
    i2c.writeto_mem(acc_id,0x20,offset)

    i2c.readfrom_mem_into(acc_id, 0x32,read)
    x = (read[1] << 8 | read[0]) & 1023
    x = acc_trans(x, 10)
    y = (read[3] << 8 | read[2]) & 1023
    y = acc_trans(y, 10)
    z = (read[5] << 8 | read[4]) & 1023
    z = acc_trans(z, 10) - offset[0]
    sleep(1)
    return x,y,z
<<<<<<< HEAD

##MQTTFunc:

=======
        
##MQTTFunc:  
>>>>>>> 36a3b7473de84820c2a0116f6cfe32e2cdc42275
sessionIDTopic_pub = 'SPINNER/SESSIONID'
sensorTopic_sub = 'SPINNER/SENSORDATA'
akTopic_pub = 'SPINNER/ACKNOW'
client_id = ubinascii.hexlify(machine.unique_id())
mqtt_server ='farmer.cloudmqtt.com'
temp_last = 0
pub = 0

def receive_handler(topic,msg):
    global num,client,pub,temp_last 
    print('Msg From Sensordata Topic:  ' + str(msg))
    r_led.value(0)
    pwm.init()
    #Crypt & Demo
    mycrp = CryptAes(bytes(str(num),'uft-8'))
    fail_check = mycrp.verify_hmac(msg)
    if fail_check == 0:
<<<<<<< HEAD
        client.pulish(topic=akTopic_pub, msg="Fail Authentication")
    #client.pulish(topic=akTopic_pub,msg=new_msg)
    mycrp.decrypt(msg)
    print("depted")

#Publish
=======
        client.publish(topic=akTopic_pub, msg="Fail Authentication")
        pub = 1
    
    check,node_id,data_x,data_y,data_z,temp = mycrp.decrypt(msg)
    
    #Demo
    if (data_x > 0.8 or data_y > 0.8 or data_z > 1.2):
        r_led.value(1)
    if (temp_last):        
        freq = pwm.freq() + int((temp_last - temp)) * 5
        pwm.freq(freq) 
    if check:
        client.publish(topic=akTopic_pub, msg="Successful Decryption")
        temp_last = temp
        data = {}
        data['value1'] = node_id + '|||' + str(num) + '|||' + str(data_x) + '|||' + str(data_y) + '|||' + str(data_z) + '|||' + str(temp) 
        r = urequests.request("POST", "https://maker.ifttt.com/trigger/decryption/with/key/l2IcanZRKg_-1ny_9_Dhew-gadfK0vblGjpIJtCK0-w", json=data, headers={"Content-Type": "application/json"})
        print(r.text)
        pub = 1
    
#Connect to the mqtt    
>>>>>>> 36a3b7473de84820c2a0116f6cfe32e2cdc42275
def mqtt_pub(client_id,maqtt_server,username,password,port,Topic_pub,Topic_sub):
    client = MQTTClient(client_id, server = mqtt_server,user = username, password = password, port = port)
    client.connect()
    client.set_callback(receive_handler)
    client.subscribe(Topic_sub)
    print('Connected to %s MQTT broker, subscribed to %s topic' % (mqtt_server, Topic_sub))
    return client

##Random Number Generate and publish
def send_sessionid(pin):
    global pub, num
    if pub > 0:
        print("Session ID published")
        num = random.randint(0,2024)
        client.publish(topic=sessionIDTopic_pub,msg=str(num))
        pub = 0
        print(num)
    else:
        pass

client = mqtt_pub(client_id,mqtt_server,'swpdieal','MdwlLdTQWzbI','14584',sessionIDTopic_pub, sensorTopic_sub)
tim3 = Timer(3)
tim3.init(period=1000, mode=1, callback=send_sessionid)

while True:
    if interrupt2 > 0:
        client.check_msg()
        sleep(1)
<<<<<<< HEAD


#         except:
#             print("Fail to subscribe, waitting for information")
#             machine.reset()
=======
        
>>>>>>> 36a3b7473de84820c2a0116f6cfe32e2cdc42275
