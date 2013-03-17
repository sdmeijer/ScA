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
import pyfirmata

import _winreg as winreg
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
    #change pin back to digital output if prev used as servo
    if (PIN_USE[i] == 3):
        tempstr = 'd:' + str(PIN_NUM[i]) + ':o'
        dummy =board.get_pin(tempstr)
        PIN_USE[i] = 1
        if debug_info:
            print 'g pin' , PIN_NUM[i] , ' set to be digital out'
    
    if (PIN_USE[pin_index] == 1) or (PIN_USE[pin_index] == 0):
        if invert == True:
            value = 1 - value
        #print 'setting physical pin %d to %d' % (PIN_NUM[pin_index],value)
        if debug_info:
            print 'g setting physical pin %d to %d' % (PIN_NUM[pin_index],value)
        board.digital[PIN_NUM[pin_index]].write(value)
    elif (PIN_USE[pin_index] == 2):
        if invert == True:
            value = float(1.0 - value)
        if debug_info:
            print 'g setting physical pin %d to %d' % (PIN_NUM[pin_index],value)
        board.digital[PIN_NUM[pin_index]].write(float(value))
            

def step_coarse(a,b,c,d,delay):
    physical_pin_update(PIN_NUM.index(a),1,False)
    physical_pin_update(PIN_NUM.index(d),0,False)
    time.sleep(delay)

    physical_pin_update(PIN_NUM.index(b),1,False)
    physical_pin_update(PIN_NUM.index(a),0,False)
    time.sleep(delay)
    
    physical_pin_update(PIN_NUM.index(c),1,False)
    physical_pin_update(PIN_NUM.index(b),0,False)
    time.sleep(delay)
    
    physical_pin_update(PIN_NUM.index(d),1,False)
    physical_pin_update(PIN_NUM.index(c),0,False)
    time.sleep(delay)


def step_fine(a,b,c,d,delay):
    physical_pin_update(PIN_NUM.index(d),0,False)
    physical_pin_update(PIN_NUM.index(a),1,False)
    time.sleep(delay)

    physical_pin_update(PIN_NUM.index(b),1,False)
    time.sleep(delay)
    
    physical_pin_update(PIN_NUM.index(a),0,False)
    time.sleep(delay)
    
    physical_pin_update(PIN_NUM.index(c),1,False)
    time.sleep(delay)

    physical_pin_update(PIN_NUM.index(b),0,False)
    time.sleep(delay)
    
    physical_pin_update(PIN_NUM.index(d),1,False)
    time.sleep(delay)

    physical_pin_update(PIN_NUM.index(c),0,False)
    time.sleep(delay)
    
    physical_pin_update(PIN_NUM.index(a),1,False)
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
        for i in range(PINS):
            #print 'i %d' % i
            #print 'GPIO PIN %d' % GPIO_PIN_INPUT[i]
            if (PIN_USE[i] == 0):
                pin_value = 0;
                try:
                    pin_value = int(DIGITAL_IN[i].read())
                    #print 'pin' , PIN_NUM[i] , ' ' ,pin_value
                except:
                    #print 'pin' , PIN_NUM[i] , ' ' , 'no value found'
                    pass
                self.broadcast_pin_update(i, pin_value)
                last_bit_pattern += pin_value << i
            #else:
                #last_bit_pattern += 1 << i
            #print 'lbp %s' % bin(last_bit_pattern)

        last_bit_pattern = last_bit_pattern ^ -1

        last_time_sent = time.time()
        while not self.stopped():

            if (time.time() - last_time_sent) > 0.1:            
                pin_bit_pattern = 0L
                changed_pins = 0L
                for i in range(PINS):
                    if (PIN_USE[i] == 0):
                        #print 'pin' , PIN_NUM[i]

                        #pin_bit_pattern += int(board.digital[2].read())
                        pin_value = 0;
                        try:
                            pin_value = int(DIGITAL_IN[i].read())
                            #print 'pin' , PIN_NUM[i] , ' ' ,pin_value
                        except:
                            #print 'pin' , PIN_NUM[i] , ' ' , 'no value found'
                            pass
                        pin_bit_pattern += pin_value << i
    ####                    pin_bit_pattern += GPIO.input(PIN_NUM[i]) << i
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
        for i in range(PINS):
            # if we care about this pin's value
            if (changed_pin_map >> i) & 0b1:
                pin_value = (pin_value_map >> i) & 0b1
                if (PIN_USE[i] == 0):
                    self.broadcast_pin_update(i, pin_value)

    def broadcast_pin_update(self, pin_index, value):
        #sensor_name = "gpio" + str(GPIO_NUM[pin_index])
        #bcast_str = 'sensor-update "%s" %d' % (sensor_name, value)
        #print 'sending: %s' % bcast_str
        #self.send_scratch_command(bcast_str)
        sensor_name = "pin" + str(PIN_NUM[pin_index])
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
        for i in range(PINS):
            if (PIN_USE[i] == 1):
                physical_pin_update(i,0,True)
            elif (PIN_USE[i] == 0):
                physical_pin_update(i,PIN_NUM[i]%2,True)
                
            elif (PIN_USE[i] == 2):
                
                print 'pin' , PIN_NUM[i] , ' PWM/MOTOR'
            elif (PIN_USE[i] == 3):
                
                print 'pin' , PIN_NUM[i] , ' servo'

        for i in range(ANALOG_PINS):
                LAST_ANALOG_VALUE[i] = -1.1
                #print tempstr , ANALOG_IN[i]
                
        #This is main listening routine
        while not self.stopped():
            try:
                data = self.scratch_socket.recv(BUFFER_SIZE)
                dataraw = data[4:].lower()
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
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            physical_pin_update(i,1)
                if 'allpins" 0' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            physical_pin_update(i,0)
                if 'allpins" "on' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            physical_pin_update(i,1)
                if 'allpins" "off' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            physical_pin_update(i,0)
                if 'allpins" "high' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            physical_pin_update(i,1)
                if 'allpins" "low' in dataraw:
                    for i in range(PINS):
                        if (PIN_USE[i] == 1):
                            physical_pin_update(i,0)
                
                
                #check for individual port commands
                for i in range(PINS):
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                    physical_pin = PIN_NUM[i]
                    if 'pin' + str(physical_pin) + '" 1' in dataraw:
                        if (PIN_USE[i] >= 1):
                            physical_pin_update(i,1)
                    if 'pin' + str(physical_pin) + '" 0' in dataraw:
                        if (PIN_USE[i] >= 1):
                            physical_pin_update(i,0)
                    if 'pin' + str(physical_pin) + '" "on' in dataraw:
                        if (PIN_USE[i] >= 1):
                            physical_pin_update(i,1)
                    if 'pin' + str(physical_pin) + '" "off' in dataraw:
                        if (PIN_USE[i] >= 1):
                            physical_pin_update(i,0)
                    if 'pin' + str(physical_pin) + '" "high' in dataraw:
                        if (PIN_USE[i] >= 1):
                            physical_pin_update(i,1)
                    if 'pin' + str(physical_pin) + '" "low' in dataraw:
                        if (PIN_USE[i] >= 1):
                            physical_pin_update(i,0)
                        
                #Use bit pattern to control ports
                if 'pinpattern' in dataraw:
                    #print 'Found pinpattern'
                    num_of_bits = PINS
                    outputall_pos = dataraw.find('pinpattern')
                    sensor_value = dataraw[(outputall_pos+12):].split()
                    #print sensor_value[0]
                    bit_pattern = ('00000000000000000000000000'+sensor_value[0])[-num_of_bits:]
                    #print 'bit_pattern %s' % bit_pattern
                    j = 0
                    for i in range(PINS):
                    #bit_state = ((2**i) & sensor_value) >> i
                    #print 'dummy pin %d state %d' % (i, bit_state)
                        if (PIN_USE[i] == 1):
                            if bit_pattern[-(j+1)] == '0':
                                physical_pin_update(i,0)
                            else:
                                physical_pin_update(i,1)
                            j = j + 1

                #Check for motor commands
                for i in range(PINS):
                    if (PIN_USE[i] == 2):
                    #check for individual port commands
                    
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                        physical_pin = PIN_NUM[i]
                        if 'motor' + str(physical_pin) in dataraw:
                            outputall_pos = dataraw.find('motor'+str(physical_pin))
                            sensor_value = dataraw[(1+outputall_pos+len('motor'+str(physical_pin))):].split()
                            print "sensor_value" , sensor_value[0]
                            if isNumeric(sensor_value[0]):
                                motorvalue = float(max(0,min(100,int(sensor_value[0]))))/100.0
                                print motorvalue
                                #board.digital[PIN_NUM[i]].write(motorvalue)
                                try:
                                    physical_pin_update(i,motorvalue)
                                    #board.digital[PIN_NUM[i]].write(motorvalue)
                                except:
                                    print 'Motor Write failed'
                                    pass
                #Check for servo commands
                for i in range(PINS):
                    if ((PIN_USE[i] == 1) or (PIN_USE[i] == 3)):
                    #check for individual port commands
                    
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                        physical_pin = PIN_NUM[i]
                        if 'servo' + str(physical_pin) in dataraw:
                            if (PIN_USE[i] == 1):
                                tempstr = 'd:' + str(PIN_NUM[i]) + ':s'
                                dummy =board.get_pin(tempstr)
                                PIN_USE[i] = 3
                                print 'pin' , PIN_NUM[i] , ' servo'
                            outputall_pos = dataraw.find('servo'+str(physical_pin))
                            sensor_value = dataraw[(1+outputall_pos+len('servo'+str(physical_pin))):].split()
                            print "sensor_value" , sensor_value[0]
                            if isNumeric(sensor_value[0]):
                                motorvalue = int(sensor_value[0])#int(max(0,min(100,int(sensor_value[0]))))
                                print motorvalue
                                #board.digital[PIN_NUM[i]].write(motorvalue)
                                try:
                                    board.digital[PIN_NUM[i]].write(motorvalue)
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
                #print 'received broadcast: %s' % data
                if ('inverton' in dataraw):
                     invert = True
                if ('invertoff' in dataraw):
                     invert = False
                if (('allon' in dataraw) or ('allhigh' in dataraw)):
                    for i in range(PINS):
                        if (PIN_USE[i] >= 1):
                            physical_pin_update(i,1)
                if (('alloff' in dataraw) or ('alllow' in dataraw)):
                    for i in range(PINS):
                        if (PIN_USE[i] >= 1):
                            physical_pin_update(i,0)
                for i in range(PINS):
                    #check_broadcast = str(i) + 'on'
                    #print check_broadcast
                    physical_pin = PIN_NUM[i]
                    if 'pin' + str(physical_pin)+'high' in dataraw:
                        physical_pin_update(i,1)
                    if 'pin' + str(physical_pin)+'low' in dataraw:
                        physical_pin_update(i,0)
                    if 'pin' + str(physical_pin)+'on' in dataraw:
                        physical_pin_update(i,1)
                    if 'pin' + str(physical_pin)+'off' in dataraw:
                        physical_pin_update(i,0)
##                    if 'sonar' + str(physical_pin) in dataraw:
##                        if (PIN_USE[i] == 0):
##                            #print "sonar pulse" , physical_pin
##                            GPIO.output(23, True)
##                            time.sleep(0.00001)
##                            GPIO.output(23, False)
##                            t0=dt.datetime.now()
##                            t1=t0
##                            while ((GPIO.input(physical_pin)==False) and ((t1-t0).microseconds < 100000)):
##                                t1=dt.datetime.now()
##                            t1=dt.datetime.now()
##                            t2=t1
##                            while ((GPIO.input(physical_pin)==True) and ((t2-t1).microseconds < 100000)):
##                                t2=dt.datetime.now()
##                            t2=dt.datetime.now()
##                            t3=(t2-t1).microseconds
##                            distance=t3/58
##                            if (distance < 500) and (distance > 2):
##                                #print'Distance:',distance,'cm'
##                                sensor_name = "pin" + str(physical_pin) 
##                                bcast_str = 'sensor-update "%s" %d' % (sensor_name, distance)
##                                #print 'sending: %s' % bcast_str
##                                self.send_scratch_command(bcast_str)            


                    
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
                       
                    #self.physical_pin_update(PIN_NUM.index(11),0,False)


                if (('spincoarse' in dataraw)):
                    print 'spincoasre rcvd'
                    for i in range(1,512):
                        step_fine(10,11,12,13,step_delay ) 
                   
##                    time.sleep(delay)
##                    self.physical_pin_update(PIN_NUM.index(8),0)
                    
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
    #Try and find open com port and then try to open them up     
    esp = enumerate_serial_ports() # create a generator
    for i in esp:
        print 'Found ' , i
        try:
            print "Testing " , i
            board = pyfirmata.Arduino(i, baudrate=57600) # Baudrate must match rate set in sketch
            if board.get_firmata_version() == None:
                raise Exception('spam', 'eggs')

            print i , 'passed'
            break
        except Exception , e:
            print '"Exception ' , e
            pass

    #If the above code errors on you - rem the lines out and use this instead
    #board = pyfirmata.Arduino("COM26", baudrate=57600) # Replace COM26 with your Arduino port

    #carry on assuming an Arduino or Shrimp is connected 
    com_port_open = True
    it = pyfirmata.util.Iterator(board)
    it.start()

    print 'Defining Inital Pin Usage'
    for i in range(PINS):
        if (PIN_USE[i] == 1):
            print 'pin' , PIN_NUM[i] , ' out'
        elif (PIN_USE[i] == 0):
            tempstr = 'd:' + str(PIN_NUM[i]) + ':i'
            DIGITAL_IN[i] =board.get_pin(tempstr) 
            print 'pin' , PIN_NUM[i] , ' in'
        elif (PIN_USE[i] == 2):
            tempstr = 'd:' + str(PIN_NUM[i]) + ':p'
            DIGITAL_IN[i] =board.get_pin(tempstr) 
            print 'pin' , PIN_NUM[i] , ' PWM/MOTOR'
        elif (PIN_USE[i] == 3):
            print 'pin' , PIN_NUM[i] , ' servo'

    for i in range(ANALOG_PINS):
            board.analog[ANALOG_PIN_NUM[i]].enable_reporting()
            tempstr = 'a:' + str(ANALOG_PIN_NUM[i]) + ':i'
            ANALOG_IN[i] = board.get_pin(tempstr)
            

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

        # wait for ctrl+c
        try:
            #do nothing
            time.sleep(0.05)
        except KeyboardInterrupt:
            print 'Ctrl-C pressed - cleaning up'
            cleanup_threads((listener,sender,steppera,stepperb))
            board.exit()
            com_port_open = False
            sys.exit()
except:
    print 'Final exception reached'
    #cleanup_threads((listener,sender))
    if  com_port_open:
        board.exit()
        com_port_open = False
    sys.exit()


