#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import os
import sys
from threading import Timer, Thread, Event
import smbus
from piusv import PiUSV
import paho.mqtt.client as mqttClient
import json
import glob
#import sn3218

def logstd(message):
  print(time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()) + " " + message)

def find_devices():
    return glob.glob('/sys/bus/w1/devices/28-*')

def read_temp(path):
    lines = []

    with open(path + '/w1_slave') as f:
        lines = f.readlines()
    if len(lines) != 2:
        return False, 0
    if lines[0].find('YES') == -1:
        return False, 0
    d = lines[1].strip().split('=')
    if len(d) != 2:
        return False, 0
    return True, int(d[1])

bus = smbus.SMBus(1) # 1 indicates /dev/i2c-1
try:
  #bus.read_byte(0x28)
  bus.read_byte(0x48)
  import automationhat
  logstd("Automation Hat detected")
  if automationhat.is_automation_hat():
    automationhat.light.power.write(1)
    automationhat.relay.one.off()
    automationhat.relay.two.off()
    #automationhat.enable_auto_lights(False)
except:
  logstd("Automationhat missing")
  #pass

piusv = PiUSV()

devices = find_devices()

def temp1():
  global devices
  c = 0.0

  for device in devices:
    valid, raw = read_temp(device)
    if valid:
      c = raw / 1000.0
  return c

# Host als Parameter fuer online Check
if len(sys.argv) == 1:
  host = "8.8.8.8"
else:
  host = str(sys.argv[1])

# Variablen definieren
StartTime = 0
BtnTimer = 0
offline = 1
i2c = "OK " + time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime())

Connected = False   #global variable for the state of the connection

broker_address= "mqtt.example.com"    #Broker address
port = 1883                           #Broker port
user = "user"                         #Connection username
password = "password"                 #Connection password
report_intervall = 0
relay_one_auto = True
pump_on = False

INA226_ADDR = 0x40
INA226_CONFIG = 0x2741
INA226_CALIBRATION = 0x0002
INA226_REG_CONFIG = 0x00
INA226_REG_BUSVOLTAGE = 0x02
INA226_REG_POWER = 0x03
INA226_REG_CURRENT = 0x04
INA226_REG_CALIBRATION = 0x05
currentLSB = 0.0001
powerLSB = 0.0025

try:
  bus.read_byte(INA226_ADDR)
  logstd("INA226 detected")
  ina226 = True
  #bus.write_word_data(INA226_ADDR, INA226_REG_CONFIG, bus.read_word_data(INA226_ADDR, INA226_REG_CONFIG) | 0x0080)
  bus.write_word_data(INA226_ADDR, INA226_REG_CONFIG, INA226_CONFIG)
  bus.write_word_data(INA226_ADDR, INA226_REG_CALIBRATION, INA226_CALIBRATION)
  #print("%04x" % bus.read_word_data(INA226_ADDR, INA226_REG_CONFIG))
  #print("%04x" % bus.read_word_data(INA226_ADDR, INA226_REG_CALIBRATION))
except:
  logstd("INA226 not detected")
  ina226 = False
  pass

def busVoltage():
  if ina226:
   val = bus.read_word_data(INA226_ADDR, INA226_REG_BUSVOLTAGE)
  else:
   val = 0
  valh = (val & 0xff00) >> 8
  vall = (val & 0x00ff)
  val = (vall << 8) | valh
  return val * 0.00125

def busPower():
  if ina226:   
   val = bus.read_word_data(INA226_ADDR, INA226_REG_POWER)
  else:
   val = 0
  valh = (val & 0xff00) >> 8
  vall = (val & 0x00ff)
  val = (vall << 8) | valh
  return val * powerLSB

def shuntCurrent():
  if ina226:
   val = bus.read_word_data(INA226_ADDR, INA226_REG_CURRENT)
  else:
   val = 0
  valh = (val & 0xff00) >> 8
  vall = (val & 0x00ff)
  val = (vall << 8) | valh
  return val * currentLSB

class checkTimer():
  def __init__(self,t,hFunction):
    self.t=t
    self.hFunction = hFunction
    self.thread = Timer(self.t,self.handle_function)

  def handle_function(self):
    self.hFunction()
    self.thread = Timer(self.t,self.handle_function)
    self.thread.start()

  def start(self):
    self.thread.start()

  def cancel(self):
    self.thread.cancel()

# pump control
def pumpOn():
  global pump_on
  automationhat.relay.one.on()
  pump_on = True

def pumpOff():
  global pump_on
  automationhat.relay.one.off()
  pump_on = False
  
# mqtt
def on_connect(client, userdata, flags, rc):
  global Connected
  if rc == 0:
    logstd("Connected to broker")
    Connected = True                #Signal connection
  else:
    logstd("Connection failed")
    Connected = False

# Licht
def on_message1(client, userdata, message):
  logstd("Message1 received: "  + message.topic + " " + message.payload)
  if message.payload == "On":
    automationhat.relay.two.on()
    client.publish("domoticz/in", '{"command": "switchlight", "idx": 97, "switchcmd": "On"}')
  if message.payload == "Off":
    automationhat.relay.two.off()
    client.publish("domoticz/in", '{"command": "switchlight", "idx": 97, "switchcmd": "Off"}')

# Pumpe
def on_message2(client, userdata, message):
  global relay_one_auto

  logstd("Message2 received: "  + message.topic + " " + message.payload)
  if message.payload == "On":
    relay_one_auto = False
    pumpOn()
    client.publish("domoticz/in", '{"command": "switchlight", "idx": 100, "switchcmd": "Set Level", "level": 10}')
  if message.payload == "Off":
    relay_one_auto = False
    pumpOff()
    client.publish("domoticz/in", '{"command": "switchlight", "idx": 100, "switchcmd": "Set Level", "level": 0}')
  else:
    relay_one_auto = True
    client.publish("domoticz/in", '{"command": "switchlight", "idx": 100, "switchcmd": "Set Level", "level": 20}')

def on_message3(client, userdata, message):
  global relay_one_auto

  #print "Message3 received: "  + message.topic + " " + message.payload
    
  list = []
  list = json.loads(message.payload)

  try:
    if list["name"] == "raspi2 Licht" and list["nvalue"] == 1:
      logstd("raspi2 Licht an")
      automationhat.relay.two.on()
    if list["name"] == "raspi2 Licht" and list["nvalue"] == 0:
      logstd("raspi2 Licht aus")
      automationhat.relay.two.off()

    if list["name"] == "raspi2 Pumpe" and list["svalue1"] == "10":
      logstd("raspi2 Pumpe an")
      pumpOn()
      relay_one_auto = False
    if list["name"] == "raspi2 Pumpe" and list["svalue1"] == "0":
      logstd("raspi2 Pumpe aus")
      pumpOff()
      relay_one_auto = False
    if list["name"] == "raspi2 Pumpe" and list["svalue1"] == "20":
      logstd("raspi2 Pumpe auto")
      relay_one_auto = True
  except:
    logstd("no valid mqtt value found")
    pass

client = mqttClient.Client("Python")               #create new instance
client.username_pw_set(user, password=password)    #set username and password
client.on_connect= on_connect                      #attach function to callback
#client.on_message= on_message                      #attach function to callback
client.message_callback_add('cmnd/raspi2/light', on_message1)
client.message_callback_add('cmnd/raspi2/pump', on_message2)
client.message_callback_add('domoticz/out', on_message3)

try:
  client.connect(broker_address, port=port)          #connect to broker

  client.loop_start()        #start the loop

  while Connected != True:    #Wait for connection
    time.sleep(0.1)

  client.subscribe("#")
  time.sleep(0.1)
  client.publish("domoticz/in", '{"command": "switchlight", "idx": 100, "switchcmd": "Set Level", "level": 20}')
  client.publish("domoticz/in", '{"command": "getdeviceinfo", "idx": 97}')
except:
  print("no connection to broker")

# syslog routine
def log(message):
  os.system("logger -t led " + message)
log("process started, checking host " + host)


### set sensor timeout of cap1208 reg 0x22 to max = 11200 ms ###
#bus.write_byte_data(0x28, 0x22, bus.read_byte_data(0x28, 0x22) | 0xf0)

def TouchButtonEvent(channel, event):
  global StartTime, BtnTimer

  if channel > 3:
    return
  if event == 'press':
    print("Button " + str(channel) + " pressed")
    automationhat.light[channel - 1].on()
    if channel == 1:
      BtnTimer = 0
      StartTime = time.time()
  if event == 'release':
    print("Button " + str(channel) + " released")
    automationhat.light[channel - 1].off()
    if channel == 1 and StartTime > 0:
      BtnTimer = time.time() - StartTime
      StartTime = 0

def partition(array, begin, end):
  pivot = begin
  for i in xrange(begin+1, end+1):
    if array[i] <= array[begin]:
      pivot += 1
      array[i], array[pivot] = array[pivot], array[i]
    array[pivot], array[begin] = array[begin], array[pivot]
  return pivot

def quicksort(array, begin=0, end=None):
  if end is None:
    end = len(array) - 1
  def _quicksort(array, begin, end):
    if begin >= end:
      return
    pivot = partition(array, begin, end)
    _quicksort(array, begin, pivot-1)
    _quicksort(array, pivot+1, end)
  return _quicksort(array, begin, end)

def analog(chan):
  val = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
  for i in range(0, 10):
    val[i] = automationhat.analog[chan].read()
    time.sleep(0.01)
  quicksort(val)
  #for i in range(0, 10):
  #  print(str(val[i]) + " ")
  #print("")
  return (val[3] + val[4] + val[5] + val[6]) / 4.0

# check ping thread
def pingcheck():
  global offline, BtnTimer

  offline = os.system("ping -q -w 1 -c 1 " + host + " > /dev/null 2>&1")

def readval():
  global i2c, report_intervall, pump_on, Connected, relay_one_auto

  a1 = 0
  a2 = 0
  a3 = 0
  a4 = 0
  t1 = 0

  try:
    a1 = analog(0)
    a2 = analog(1)
    a3 = analog(2)
    a4 = analog(3)
    t1 = temp1()
    piusv.get_parameter()
    piusv.word2float()
    s1 = piusv.line()
  except IOError:
    s1 = "i2c Error"
    i2c = "last error at " + time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime())
    pass

  h1 = 100.0 - (a1 - 1.40) * 80.0
  h2 = 100.0 - (a2 - 1.40) * 80.0
  f = open('/tmp/workfile', 'w')
  f.write("%s\n\n" % s1)
  f.write("Analog1:   %6.3f V\n" % a1)
  f.write("Sensor2:   %6.3f V\n" % a2)
  f.write("Input:     %6.3f V\n" % busVoltage())
  f.write("Current:   %6.3f A\n" % shuntCurrent())
  f.write("Humidity1: %6.3f %%\n" % h1)
  f.write("Humidity2: %6.3f %%\n" % h2)
  f.write("Temp1:     %6.3f C\n" % t1)
  f.write("\ni2cState: %s\n" % i2c)
  if Connected == True:
    f.write("mqtt broker connected")
  else:
    f.write("mqtt broker disconnected")
  f.close()

  if report_intervall == 0:
    client.publish("stat/raspi2/humidity",h2)
    if relay_one_auto == True:
      client.publish("stat/raspi2/relay_one", "auto")
    else:
      client.publish("stat/raspi2/relay_one", "manual")
    if pump_on == True:
      client.publish("stat/raspi2/pump", "on")
    else:
      client.publish("stat/raspi2/pump", "off")
      client.publish("domoticz/in", '{"idx":96, "nvalue":' + str(int(h2)) + '}')
      client.publish("domoticz/in", '{"idx":102,"nvalue":' + str(int(h1)) + '}')
      client.publish("domoticz/in", '{"idx":98, "nvalue":0,"svalue":"' + str(shuntCurrent()) + '"}')
      client.publish("domoticz/in", '{"idx":99, "nvalue":0,"svalue":"' + str(busVoltage()) + '"}')
      client.publish("domoticz/in", '{"idx":101,"nvalue":0,"svalue":"' + str(t1) + '"}')
  report_intervall = report_intervall + 1
  if report_intervall >= 60:
    report_intervall = 0

  if h2 < 50.0 and relay_one_auto == True and pump_on == False:
    client.publish("stat/raspi2/pump", "on")
    client.publish("domoticz/in", '{"idx":100,"nvalue":1}')
    print(time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()) + " h2 < 50.0 -> Pumpe an")
    pumpOn()
  if h2 > 60.0 and relay_one_auto == True and pump_on == True:
    client.publish("stat/raspi2/pump", "off")
    client.publish("domoticz/in", '{"idx":100,"nvalue":0}')
    print(time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()) + " h2 > 60.0 -> Pumpe aus")
    pumpOff()

# Endlosschleife
try:
  t = checkTimer(10, pingcheck)
  t.start()
  t1 = checkTimer(1, readval)
  t1.start()
  automationhat.relay.two.off()
  while BtnTimer <= 7:
    if offline:
      # falls offline gruene LED schnell blinken
      automationhat.light.comms.on()
      time.sleep(0.1)
      automationhat.light.comms.off()
      time.sleep(0.4)
    else:
      # falls online gruene LED langsam blinken
      automationhat.light.comms.on()
      time.sleep(1)
      automationhat.light.comms.off()
      time.sleep(1)

    # Button laenger als 3 Sekunden gedrueckt, gelbe LED an
    if (StartTime > 0) and (time.time() - StartTime) >= 3:
      automationhat.light.warn.on()

    # Button laenger als 7 Sekunden gedrueckt, gelbe LED aus
    if (StartTime > 0) and (time.time() - StartTime) >= 7:
      automationhat.light.warn.off()

    # Button kuerzer als 1 Sekunde gedrueckt, netcheck
    if (BtnTimer > 0) and (BtnTimer < 1):
      log("button pressed " + str(BtnTimer) + " seconds - netcheck")
      BtnTimer = 0
      os.system("sudo /usr/local/bin/netcheck")

    # Button mindestens 3, jedoch unter 7 Sekunden gedrueckt, gelbe LED aus, Netzwerkneustart
    if (BtnTimer >= 3) and (BtnTimer < 7):
      log("button pressed " + str(BtnTimer) + " seconds - restarting network")
      BtnTimer = 0
      os.system("sudo service networking restart")
      os.system("sudo service isc-dhcp-server restart")
      automationhat.light.warn.off()

except KeyboardInterrupt:
  print("\nCTRL+C")

#t.cancel()
print("Beendet")

if BtnTimer == 100:
  t.cancel()
  t1.cancel()
  os.execv(sys.executable, ['python'] + sys.argv)

# Button laenger als 7 Sekunden gedrueckt, poweroff
if BtnTimer >= 7:
  print("Button pressed " + str(BtnTimer) + " seconds")
  log("Button pressed " + str(BtnTimer) + " seconds - poweroff initiated")
  #os.system("sudo poweroff")
  #os.system("sudo /usr/local/bin/stopusv")
t.cancel()
t1.cancel()
pumpOff()
