# This code is copyright Simon Walters under GPL v2
# This code is derived from scratch_gpio_handler for the RPi
# Version 0.3 alpha
# V 0.35 - Add in support for PWM on Pins 9,10 and 11 (temp remove analog input)
# V0.36 - Add in support for usings pins 7,8,12 and 13 for servos instead of digital out
# V0.37 - Add in stepper motor trial support
# V 0.38  Add in delay to only send digital inputs every 0.2 sec
# V0.39  Trying to check com port
# V0.40 - Possible auto com working version
# V0.41 - Looks like it works on Windows XP
# V0.42e1 - Add in ability to set pullups pulldowns
# V0.43 - playing with stepper
# V0.44 - move code into thread
# V0.45 - Fix PWM regression and introduce experimental continous stepper
# V0.46 - StepperB variable now working using pins 10,11,12,13 value -100 to 100
#         Data inputs only sent back every 0.1 secs to stop chatter
# V0.47 - more mods
# V0.50 - radical program restructuring with additional threads for steppers
# V0.51 - Decrease CPU usuage in threads and add invert function for digital outs
# V0.52 - mode mods


from array import *
import threading
import socket
import time
import sys
import struct
import time
import datetime as dt
#import pyfirmata
import Arduino as Shrimp

import platform #SDM
if platform.system() == 'Windows': #SDM
    import _winreg as winreg #SDM
if platform.system() == 'Linux': #SDM
    import glob #SDM
import itertools

'''
from Tkinter import Tk
from tkSimpleDialog import askstring
root = Tk()
root.withdraw()
'''

PORT = 42001
DEFAULT_HOST = '127.0.0.1'
#HOST = askstring('Scratch Connector', 'IP:')
BUFFER_SIZE = 240 #used to be 100
SOCKET_TIMEOUT = 1



#Map pin usuage
PIN_NUM = array('i',[2,3,4,5,6,7,8,9,10,11,12,13])#list of Arduino Uno pin nums
PIN_USE = array('i',[0,0,0,0,2,1,1,2,2, 2, 1, 1 ])#1 indicates output , 0 indicates input
ANALOG_PIN_NUM = array('i',[0,1,2,3,4,5])

PINS = len(PIN_NUM)
ANALOG_PINS = len(ANALOG_PIN_NUM)
DIGITAL_IN = [None] * PINS
DIGITAL_OUT = [None] * PINS
ANALOG_IN = [None] * ANALOG_PINS
LAST_ANALOG_VALUE = [float] * ANALOG_PINS
CURRENT_ANALOG_VALUE = [float] * ANALOG_PINS

STEPPERA=0
STEPPERB=1

stepper_value = array('i',[0,0])

invert = False

global pause
pause = False

def isNumeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def enumerate_serial_ports():
    """ Uses the Win32 registry to return a iterator of serial 
        (COM) ports existing on this computer.


    """
    path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
    except WindowsError:
        raise IterationError

    for i in itertools.count():
        try:
            val = winreg.EnumValue(key, i)
            yield (str(val[1]))#, str(val[0]))
        except EnvironmentError:
            break




class MyError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)

def physical_pin_update(pin_index, value,debug_info=True):
    global invert
    pin = PIN_NUM[pin_index]
    pinUse = PIN_USE[pin_index]
    if value == 1:
        convValue = 'HIGH'
    else:
        convValue = 'LOW'
    #change pin back to digital output if prev used as servo
    if (pinUse == 3):
        board.Servo.detach(pin)
        board.pinMode(pin, "OUTPUT")
        PIN_USE[pin_index] = 1
        if debug_info:
            print 'g pin' , pin , ' set to be digital out'
    
    if (pinUse == 1) or (pinUse == 0):
        if invert == True:
            value = 1 - value
        #print 'setting physical pin %d to %d' % (PIN_NUM[pin_index],value)
        if debug_info:
            print 'g setting physical pin %d to %d' % (pin,value)
        board.digitalWrite(pin, convValue)
    elif (pinUse == 2):
        if invert == True:
            value = float(1.0 - value)
        if debug_info:
            print 'g setting physical pin %d to %d' % (pin,value)
        board.analogWrite(pin, float(value))
            

def step_coarse(pinA,pinB,pinC,pinD,delay):
    board.digitalWrite(pinA, "HIGH")
    board.digitalWrite(pinD, "LOW")
    time.sleep(delay)

    board.digitalWrite(pinB, "HIGH")
    board.digitalWrite(pinA, "LOW")
    time.sleep(delay)

    board.digitalWrite(pinC, "HIGH")
    board.digitalWrite(pinB, "LOW")
    time.sleep(delay)

    board.digitalWrite(pinD, "HIGH")
    board.digitalWrite(pinC, "LOW")
    time.sleep(delay)


def step_fine(pinA,pinB,pinC,pinD,delay):
    board.digitalWrite(pinD, "LOW")
    board.digitalWrite(pinA, "HIGH")
    time.sleep(delay)

    board.digitalWrite(pinB, "HIGH")
    time.sleep(delay)

    board.digitalWrite(pinA, "LOW")
    time.sleep(delay)

    board.digitalWrite(pinC, "HIGH")
    time.sleep(delay)

    board.digitalWrite(pinB, "LOW")
    time.sleep(delay)

    board.digitalWrite(pinD, "HIGH")
    time.sleep(delay)

    board.digitalWrite(pinC, "LOW")
    time.sleep(delay)

    board.digitalWrite(pinA, "HIGH")
    time.sleep(delay)
    
#----------------------------- STEPPER CONTROL --------------
class StepperControl(threading.Thread):
    def __init__(self,stepper_num):
        self.stepper_num = stepper_num # find which stepper a or b
        threading.Thread.__init__(self)
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        global step_delay
        time.sleep(2) # just wait till board likely to be up and running
        stepper_start_pin = 6 + (self.stepper_num * 4) # use pins6-9 for steppera, 10-13 for stepperb

        while not self.stopped():
            local_stepper_value=stepper_value[self.stepper_num] # get stepper value in case its changed during this thread
            if local_stepper_value != 0: #if stepper_value non-zero
                if local_stepper_value > 0: # if positive value
                    step_coarse(stepper_start_pin,stepper_start_pin+1,stepper_start_pin+2,stepper_start_pin+3,step_delay) #step forward
                else:
                    step_coarse(stepper_start_pin+3,stepper_start_pin+2,stepper_start_pin+1,stepper_start_pin,step_delay) #step forward
                if abs(local_stepper_value) != 100: # Only introduce delay if motor not full speed
                    time.sleep(10*step_delay*((100/abs(local_stepper_value))-1))                       
            else:
                time.sleep(1) # sleep if stepper value is zero
                 
# --------------------MAIN INPUT PIN DETECTION CODE
class ScratchSender(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)
        self.scratch_socket = socket
        self._stop = threading.Event()


    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        global step_delay
        time.sleep(2)
        print 'Checking Inputs'
        last_bit_pattern=0L
        for p in range(PINS):
            pinUse = PIN_USE[p]
            pin = PIN_NUM[p]
            if (pinUse == 0):
                pin_value = 0;
                try:
                    pin_value = int(board.digitalRead(pin))
                    #print 'pin' , pin , ' ' ,pin_value
                except:
                    #print 'pin' , pin , ' ' , 'no value found'
                    pass
                self.broadcast_pin_update(pin, pin_value)
                last_bit_pattern += pin_value << p
            #else:
                #last_bit_pattern += 1 << i
            #print 'lbp %s' % bin(last_bit_pattern)

        last_bit_pattern = last_bit_pattern ^ -1

        last_time_sent = time.time()
        global pause
        while not self.stopped():
            while not pause:
                if (time.time() - last_time_sent) > 0.1:            
                    pin_bit_pattern = 0L
                    changed_pins = 0L
                    for p in range(PINS):
                        pinUse = PIN_USE[p]
                        pin = PIN_NUM[p]
                        if (pinUse == 0):
                            #print 'pin_sender' , pin

                            #pin_bit_pattern += int(board.digital[2].read())
                            pin_value = 0;
                            try:
                                pin_value = int(board.digitalRead(pin))
                                #print 'pin' , pin , ' ' ,pin_value
                            except:
                                #print 'pin' , pin , ' ' , 'no value found'
                                pass
                            pin_bit_pattern += pin_value << p
        ##                #else:
        ##                    #pin_bit_pattern += 1 << i
        ##            #print bin(pin_bit_pattern)
                    #if there is a change in the input pins
                    changed_pins = pin_bit_pattern ^ last_bit_pattern
                    #print "changed pins" , changed_pins
                    if (changed_pins > 0):
                        #print 'pin bit pattern %d' % pin_bit_pattern

                        try:
                            self.broadcast_changed_pins(changed_pins, pin_bit_pattern)
                        except Exception as e:
                            print e
                            break

                    last_bit_pattern = pin_bit_pattern
    ##            #Analog Section
    ##            for i in range(ANALOG_PINS):
    ##                #print 'pin' , ANALOG_PIN_NUM[i]
    ##
    ##                pin_value_analog = 0.0;
    ##                try:
    ##                    board.analog[ANALOG_PIN_NUM[i]].enable_reporting()
    ##                    pin_value_analog = float(ANALOG_IN[i].read())
    ##                    #print 'pin' , ANALOG_PIN_NUM[i] , ' ' ,pin_value_analog
    ##                    CURRENT_ANALOG_VALUE[i] = pin_value_analog
    ##                except:
    ##                    #print 'pin' , PIN_NUM[i] , ' ' , 'no value found'
    ##                    pass
    ##
    ##                try:
    ##                    if (abs(CURRENT_ANALOG_VALUE[i] - LAST_ANALOG_VALUE[i]) > 0.05):
    ##                        sensor_name = "analogpin" + str(ANALOG_PIN_NUM[i])
    ##                        bcast_str = 'sensor-update "%s" %f' % (sensor_name, pin_value_analog)
    ##                        print 'sending: %s' % bcast_str
    ##                        self.send_scratch_command(bcast_str)
    ##                        LAST_ANALOG_VALUE[i] = CURRENT_ANALOG_VALUE[i]
    ##                except Exception as e:
    ##                    print e , 'but carry on :)'
    ##                    break
                    last_time_sent = time.time()
                else:
                    time.sleep(0.1) # give cpu time to breathe           

    def broadcast_changed_pins(self, changed_pin_map, pin_value_map):
        for p in range(PINS):
            pinUse = PIN_USE[p]
            pin = PIN_NUM[p]
            # if we care about this pin's value
            if (changed_pin_map >> p) & 0b1:
                pin_value = (pin_value_map >> p) & 0b1
                if (pinUse == 0):
                    self.broadcast_pin_update(pin, pin_value)

    def broadcast_pin_update(self, pin, value):
        sensor_name = "pin" + str(pin)
        bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        print 'sending: %s' % bcast_str
        self.send_scratch_command(bcast_str)

    def send_scratch_command(self, cmd):
        n = len(cmd)
        a = array('c')
        a.append(chr((n >> 24) & 0xFF))
        a.append(chr((n >> 16) & 0xFF))
        a.append(chr((n >>  8) & 0xFF))
        a.append(chr(n & 0xFF))
        self.scratch_socket.send(a.tostring() + cmd)




class ScratchListener(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)
        self.scratch_socket = socket
        self._stop = threading.Event()
        
    def send_scratch_command(self, cmd):
        n = len(cmd)
        a = array('c')
        a.append(chr((n >> 24) & 0xFF))
        a.append(chr((n >> 16) & 0xFF))
        a.append(chr((n >>  8) & 0xFF))
        a.append(chr(n & 0xFF))
        self.scratch_socket.send(a.tostring() + cmd)


    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()


    def run(self):
        global cycle_trace,motorA,motorB,stepperb_value,step_delay,invert


        stepperb_count=100

        #initilise pin states
        print 'Setting Inital Pin States'
        for p in range(PINS):
            pinUse = PIN_USE[p]
            pin = PIN_NUM[p]
            if (pinUse == 1):
                physical_pin_update(p,0,True)
            elif (pinUse == 0):
                physical_pin_update(p,pin%2,True)
                
            elif (pinUse == 2):
                
                print 'pin' , pin , ' PWM/MOTOR'
            elif (pinUse == 3):
                
                print 'pin' , pin , ' servo'

        for i in range(ANALOG_PINS):
                LAST_ANALOG_VALUE[i] = -1.1
                #print tempstr , ANALOG_IN[i]
                
        #This is main listening routine
        while not self.stopped():
            try:
                data = self.scratch_socket.recv(BUFFER_SIZE)
                dataraw = data[4:].lower()
                #print "Listening", str(dataraw)
                #print 'Length: %d, Data: %s' % (len(dataraw), dataraw)
                #print 'Cycle trace' , cycle_trace
                if len(dataraw) == 0:
                    #This is probably due to client disconnecting
                    #I'd like the program to retry connecting to the client
                    #tell outer loop that Scratch has disconnected
                    if cycle_trace == 'running':
                        cycle_trace = 'disconnected'
                        break

            except socket.timeout:
                #print "No data received: socket timeout"

                    
                continue
            except IOError, e:
                if cycle_trace == 'running':
                    cycle_trace = 'disconnected'
                break

                

            if 'sensor-update' in dataraw:
                #gloablly set all ports
                if 'allpins" 1' in dataraw:
                    for p in range(PINS):
                        pinUse = PIN_USE[p]
                        if (pinUse == 1):
                            physical_pin_update(p,1)
                if 'allpins" 0' in dataraw:
                    for p in range(PINS):
                        pinUse = PIN_USE[p]
                        if (pinUse == 1):
                            physical_pin_update(p,0)
                if 'allpins" "on' in dataraw:
                    for p in range(PINS):
                        pinUse = PIN_USE[p]
                        if (pinUse == 1):
                            physical_pin_update(p,1)
                if 'allpins" "off' in dataraw:
                    for p in range(PINS):
                        pinUse = PIN_USE[p]
                        if (pinUse == 1):
                            physical_pin_update(p,0)
                if 'allpins" "high' in dataraw:
                    for p in range(PINS):
                        pinUse = PIN_USE[p]
                        if (pinUse == 1):
                            physical_pin_update(p,1)
                if 'allpins" "low' in dataraw:
                    for p in range(PINS):
                        pinUse = PIN_USE[p]
                        if (pinUse == 1):
                            physical_pin_update(p,0)
                
                
                #check for individual port commands
                for p in range(PINS):
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                    pin = PIN_NUM[p]
                    if 'pin' + str(pin) + '" 1' in dataraw:
                        if (pinUse >= 1):
                            physical_pin_update(p,1)
                    if 'pin' + str(pin) + '" 0' in dataraw:
                        if (pinUse >= 1):
                            physical_pin_update(p,0)
                    if 'pin' + str(pin) + '" "on' in dataraw:
                        if (pinUse >= 1):
                            physical_pin_update(p,1)
                    if 'pin' + str(pin) + '" "off' in dataraw:
                        if (pinUse >= 1):
                            physical_pin_update(p,0)
                    if 'pin' + str(pin) + '" "high' in dataraw:
                        if (pinUse >= 1):
                            physical_pin_update(p,1)
                    if 'pin' + str(pin) + '" "low' in dataraw:
                        if (pinUse >= 1):
                            physical_pin_update(p,0)
                        
                #Use bit pattern to control ports
                if 'pinpattern' in dataraw:
                    #print 'Found pinpattern'
                    num_of_bits = PINS
                    outputall_pos = dataraw.find('pinpattern')
                    sensor_value = dataraw[(outputall_pos+12):].split()
                    #print sensor_value[0]
                    if isNumeric(sensor_value[0]) == True: #SDM
                        bit_pattern = ('00000000000000000000000000'+sensor_value[0])[-num_of_bits:]
                        #print 'bit_pattern %s' % bit_pattern
                        j = 0
                        for p in range(PINS):
                        #bit_state = ((2**i) & sensor_value) >> i
                        #print 'dummy pin %d state %d' % (i, bit_state)
                            if (PIN_USE[p] == 1):
                                if bit_pattern[-(j+1)] == '0':
                                    physical_pin_update(p,0)
                                else:
                                    physical_pin_update(p,1)
                                j = j + 1

                #Check for motor commands
                for p in range(PINS):
                    if (PIN_USE[p] == 2):
                    #check for individual port commands
                    
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                        pin = PIN_NUM[p]
                        if 'motor' + str(pin) in dataraw:
                            outputall_pos = dataraw.find('motor'+str(pin))
                            sensor_value = dataraw[(1+outputall_pos+len('motor'+str(pin))):].split()
                            print "sensor_value" , sensor_value[0]
                            if isNumeric(sensor_value[0]):
                                motorvalue = float(max(0,min(100,int(sensor_value[0]))))/100.0
                                print motorvalue
                                try:
                                    physical_pin_update(p,motorvalue)
                                except:
                                    print 'Motor Write failed'
                                    pass
                #Check for servo commands
                for p in range(PINS):
                    if ((PIN_USE[p] == 1) or (PIN_USE[p] == 3)):
                    #check for individual port commands
                    
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                        pin = PIN_NUM[p]
                        if 'servo' + str(pin) in dataraw:
                            if (PIN_USE[p] == 1):
                                board.Servo.attach(pin)
                                PIN_USE[p] = 3
                                print 'pin' , pin , ' servo'
                            outputall_pos = dataraw.find('servo'+str(pin))
                            sensor_value = dataraw[(1+outputall_pos+len('servo'+str(pin))):].split()
                            print "sensor_value" , sensor_value[0]
                            if isNumeric(sensor_value[0]):
                                motorvalue = int(sensor_value[0])#int(max(0,min(100,int(sensor_value[0]))))
                                print motorvalue
                                try:
                                    board.Servo.write(pin, motorvalue)
                                except:
                                    pass
                                
                #Check for stepper commands

                if 'stepdelay' in dataraw:
                    outputall_pos = dataraw.find('stepdelay')
                    sensor_value = dataraw[(1+outputall_pos+len('stepdelay')):].split()
                    print "stepdelay" , sensor_value[0]
                    if isNumeric(sensor_value[0]):
                        step_delay = float(sensor_value[0])

                if 'stepperb' in dataraw:
                    outputall_pos = dataraw.find('stepperb')
                    sensor_value = dataraw[(1+outputall_pos+len('stepperb')):].split()
                    print "stepperb" , sensor_value[0]
                    if isNumeric(sensor_value[0]):
                        stepper_value[STEPPERB] =  int(max(-100,min(100,int(sensor_value[0]))))

   

            #Check for Broadcasts from Scratch
            if 'broadcast' in dataraw:
                print 'received broadcast: %s' % data
                if ('inverton' in dataraw):
                     invert = True
                if ('invertoff' in dataraw):
                     invert = False
                if (('allon' in dataraw) or ('allhigh' in dataraw)):
                    for p in range(PINS):
                        if (PIN_USE[p] >= 1):
                            physical_pin_update(p,1)
                if (('alloff' in dataraw) or ('alllow' in dataraw)):
                    for p in range(PINS):
                        if (PIN_USE[p] >= 1):
                            physical_pin_update(p,0)
                for p in range(PINS):
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                    pin = PIN_NUM[p]
                    if 'pin' + str(pin)+'high' in dataraw:
                        physical_pin_update(p,1)
                    if 'pin' + str(pin)+'low' in dataraw:
                        physical_pin_update(p,0)
                    if 'pin' + str(pin)+'on' in dataraw:
                        physical_pin_update(p,1)
                    if 'pin' + str(pin)+'off' in dataraw:
                        physical_pin_update(p,0)
                    if 'sonar' + str(pin) in dataraw:
                        print "sonar detected"
                        global pause
                        pause = True
                        durations = []
                        time.sleep(0.5)
                        duration = board.pulseIn_set(pin, "HIGH", 4)
                        duration = board.pulseIn_set(pin, "HIGH", 4) #two times, because the first time the values are off
                        distance = duration * 0.01716
                        global pause
                        pause = False
                        if (distance > 500) or (distance < 1.1):
                            distance = 0
                        print'Distance:',round(distance), 'cm'
                        sensor_name = "pin" + str(pin) 
                        bcast_str = 'sensor-update "%s" %d' % (sensor_name, round(distance))
                        self.send_scratch_command(bcast_str)            
                    
                if (('stepfine' in dataraw)):
                    print 'stepfine rcvd'
                    step_fine(10,11,12,13,step_delay )
                    
                    
                if (('stepcoarse' in dataraw)):
                    print 'stepcoarse rcvd'
                    step_coarse(10,11,12,13,step_delay )


                if (('spinfine' in dataraw)):
                    print 'spinfine rcvd'
                    for i in range(1,512):
                        step_fine(10,11,12,13,step_delay )
                       
                if (('spincoarse' in dataraw)):
                    print 'spincoasre rcvd'
                    for i in range(1,512):
                        step_fine(10,11,12,13,step_delay ) 
                   
                    
            if 'stop handler' in dataraw:
                cleanup_threads((listener, sender))
                sys.exit()

            #else:
                #print 'received something: %s' % dataraw


def create_socket(host, port):
    while True:
        try:
            print 'Trying'
            scratch_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            scratch_sock.connect((host, port))
            break
        except socket.error:
            print "There was an error connecting to Scratch!"
            print "I couldn't find a Mesh session at host: %s, port: %s" % (host, port) 
            time.sleep(3)
            #sys.exit(1)

    return scratch_sock

def cleanup_threads(threads):
    for thread in threads:
        thread.stop()

    for thread in threads:
        thread.join()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = DEFAULT_HOST

#start program
com_port_open = False
try:
##    #Try and find open com port and then try to open them up     
##    if platform.system() == 'Windows': #SDM
##        esp = enumerate_serial_ports() # create a generator #SDM
##    if platform.system() == 'Linux': #SDM
##        esp = glob.glob("/dev/ttyUSB*") #SDM
##    for i in esp:
##        print 'Found ' , i
##        try:
##            print "Testing " , i
##            board = pyfirmata.Arduino(i, baudrate=57600) # Baudrate must match rate set in sketch
##            if board.get_firmata_version() == None:
##                raise Exception('spam', 'eggs')
##
##            print i , 'passed'
##            break
##        except Exception , e:
##            print '"Exception ' , e
##            pass
##
##    #If the above code errors on you - rem the lines out and use this instead
##    #board = pyfirmata.Arduino("COM26", baudrate=57600) # Replace COM26 with your Arduino port
##
##    #carry on assuming an Arduino or Shrimp is connected 
##    com_port_open = True
##    it = pyfirmata.util.Iterator(board)
##    it.start()

    board = Shrimp.Arduino()
    com_port_open = True

    print 'Defining Inital Pin Usage'
    for p in range(PINS):
        pin = PIN_NUM[p]
        pinUse = PIN_USE[p]
        if (pinUse == 1):
            board.pinMode(pin, "OUTPUT")
            print 'pin', pin, ' out'
        elif (pinUse == 0):
            board.pinMode(pin, "INPUT") 
            print 'pin', pin, ' in'
        elif (pinUse == 2):
            board.pinMode(pin, "OUTPUT") 
            print 'pin', pin, ' PWM/MOTOR'
        elif (pinUse == 3):
            print 'pin', pin, ' servo'

##    for aPin in range(ANALOG_PINS):
##            board.analog[ANALOG_PIN_NUM[aPin]].enable_reporting()
##            tempstr = 'a:' + str(ANALOG_PIN_NUM[i]) + ':i'
##            ANALOG_IN[i] = board.get_pin(tempstr)
            

    cycle_trace = 'start'
#    motorA = 0;
#    motorB = 0;
#    motor_timing = array('i',[0,0,100])
#    motor_order = array('i',[0,1])
    stepperb_value=0
    steppera_value=0
    step_delay = 0.005 # delay used between steps in stepper motor functions
    while True:

        if (cycle_trace == 'disconnected'):
            print "Scratch disconnected"
            cleanup_threads((listener, sender,steppera,stepperb))
            time.sleep(1)
            cycle_trace = 'start'

        if (cycle_trace == 'start'):
            # open the socket
            print 'Starting to connect...' ,
            the_socket = create_socket(host, PORT)
            print 'Connected!'
            the_socket.settimeout(SOCKET_TIMEOUT)
            listener = ScratchListener(the_socket)
    #        data = the_socket.recv(BUFFER_SIZE)
    #        print "Discard 1st data buffer" , data[4:].lower()
            sender = ScratchSender(the_socket)
            steppera = StepperControl(STEPPERA)
            stepperb = StepperControl(STEPPERB)
            cycle_trace = 'running'
            print "Running...."
            listener.start()
            sender.start()
            steppera.start()
            stepperb.start()

##        # wait for ctrl+c
##        try:
##            #do nothing
##            time.sleep(0.05)
##        except KeyboardInterrupt:
##            print 'Ctrl-C pressed - cleaning up'
##            cleanup_threads((listener,sender,steppera,stepperb))
##            board.close()
##            com_port_open = False
##            sys.exit()
except:
    print 'Final exception reached'
    #cleanup_threads((listener,sender))
    if  com_port_open:
        board.close()
        com_port_open = False
    board.close()
    sys.exit()


