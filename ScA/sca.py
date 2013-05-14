# This code is copyright Simon Walters under GPL v2
# This code is derived from scratch_gpio_handler for the RPi (GPL v2)
#
#

from array import *
import threading
import socket
import time
import sys
import struct
import time
import datetime as dt
import Arduino as Shrimp
import platform
if platform.system() == 'Windows':
    import _winreg as winreg
if platform.system() == 'Linux':
    import glob
import itertools

PORT = 42001
DEFAULT_HOST = '127.0.0.1'
BUFFER_SIZE = 240
SOCKET_TIMEOUT = 1

#Map pin usuage
PIN_NUM = array('i',[2,3,4,5,6,7,8,9,10,11,12,13])#list of Arduino Uno pin nums
PIN_USE = array('i',[0,0,0,0,2,1,1,2, 2, 2, 1, 1])#1 indicates output , 0 indicates input, 4 = capacitive pin
ANALOG_PIN_NUM = array('i',[0,1,2,3,4,5])

PINS = len(PIN_NUM)
ANALOG_PINS = len(ANALOG_PIN_NUM)
DIGITAL_IN = [None] * PINS
DIGITAL_OUT = [None] * PINS
ANALOG_IN = [None] * ANALOG_PINS
LAST_ANALOG_VALUE = [float] * ANALOG_PINS
CURRENT_ANALOG_VALUE = [float] * ANALOG_PINS
LAST_CAP_VALUE = {}
LAST_PIN_USE = {}
MELODY = {}
DURATIONS = {}

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
            print 'pin' , pin , ' set to be digital out'
    
    if (pinUse == 1) or (pinUse == 0):
        if invert == True:
            value = 1 - value
        #print 'setting physical pin %d to %d' % (PIN_NUM[pin_index],value)
        if debug_info:
            print 'setting physical pin %d to %d' % (pin,value)
        board.digitalWrite(pin, convValue)
    elif (pinUse == 2):
        if invert == True:
            value = float(1.0 - value)
        if debug_info:
            print 'setting physical pin (PWM) %d to %d' % (pin,value)
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
                except:
                    pass
                self.broadcast_pin_update(pin, pin_value)
                last_bit_pattern += pin_value << p
            elif (pinUse == 4):
                LAST_CAP_VALUE[pin] = board.capacitivePin(pin)

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
                            pin_value = 0;
                            try:
                                pin_value = int(board.digitalRead(pin))
                            except:
                                pass
                            pin_bit_pattern += pin_value << p
                        elif (pinUse == 4):
                            pin_capvalue = 0
                            try:
                                pin_capvalue = board.capacitivePin(pin)
                            except:
                                pass
                            if (LAST_CAP_VALUE[pin] <> pin_capvalue) and (pin_capvalue > 0):
                                LAST_CAP_VALUE[pin] = pin_capvalue
                                self.broadcast_pin_update(pin, pin_capvalue)
                                
                    #if there is a change in the input pins
                    changed_pins = pin_bit_pattern ^ last_bit_pattern
                    if (changed_pins > 0):
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


    def playMelody(self, pin):
        '''
        How to use in Scratch:
            broadcast "note10 [C4,G3]"
            wait 1 sec.
            broadcast "dur10 [4,8]"
        first broadcast sends the notes (C4 is middle C) (CS is C Sharp, etc.)
        second broadcast sends the durations of the notes (4 = quarter, 8 = eigtht, etc.)
        When sending a long melody and playing it multiple times,
        not all the times the melody will be played.
        Short melodies (1 of 2 notes) plays all the time.
        '''
        if (len(DURATIONS[pin]) == len(MELODY[pin])):
            print "Melody received"
            board.Melody(pin, MELODY[pin], DURATIONS[pin]) 


    def run(self):
        global cycle_trace,motorA,motorB,stepperb_value,step_delay,invert
        stepperb_count=100

        #initilise pin states
        print 'Setting Initial Pin States'
        for p in range(PINS):
            pinUse = PIN_USE[p]
            pin = PIN_NUM[p]
            #print pin
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
                if len(dataraw) == 0:
                    #This is probably due to client disconnecting
                    #I'd like the program to retry connecting to the client
                    #tell outer loop that Scratch has disconnected
                    if cycle_trace == 'running':
                        cycle_trace = 'disconnected'
                        break

            except socket.timeout:
                #No data received: socket timeout
                continue
            except IOError, e:
                if cycle_trace == 'running':
                    cycle_trace = 'disconnected'
                break

            if 'sensor-update' in dataraw:
                #globally set all ports
                if ('allpins" 1' in dataraw) \
                   or ('allpins" "on' in dataraw) \
                   or ('allpins" "high' in dataraw):
                    for p in range(PINS):
                        pinUse = PIN_USE[p]
                        if (pinUse == 1):
                            physical_pin_update(p,1)
                if ('allpins" 0' in dataraw) \
                   or ('allpins" "off' in dataraw) \
                   or ('allpins" "low' in dataraw):
                    for p in range(PINS):
                        pinUse = PIN_USE[p]
                        if (pinUse == 1):
                            physical_pin_update(p,0)
                
                #check for individual port commands
                for p in range(PINS):
                    pin = PIN_NUM[p]
                    if ('pin' + str(pin) + '" 1' in dataraw) \
                       or ('pin' + str(pin) + '" "on' in dataraw) \
                       or ('pin' + str(pin) + '" "high' in dataraw):
                        if (pinUse >= 1):
                            physical_pin_update(p,1)
                    if 'pin' + str(pin) + '" 0' in dataraw \
                       or ('pin' + str(pin) + '" "off' in dataraw) \
                       or ('pin' + str(pin) + '" "low' in dataraw):
                        if (pinUse >= 1):
                            physical_pin_update(p,0)
                        
                #Use bit pattern to control ports
                if 'pinpattern' in dataraw:
                    num_of_bits = PINS
                    outputall_pos = dataraw.find('pinpattern')
                    sensor_value = dataraw[(outputall_pos+12):].split()
                    if isNumeric(sensor_value[0]) == True:
                        bit_pattern = ('00000000000000000000000000'+sensor_value[0])[-num_of_bits:]
                        j = 0
                        for p in range(PINS):
                            if (PIN_USE[p] == 1):
                                if bit_pattern[-(j+1)] == '0':
                                    physical_pin_update(p,0)
                                else:
                                    physical_pin_update(p,1)
                                j = j + 1

                #Check for motor commands
                for p in range(PINS):
                    if (PIN_USE[p] == 2):
                        pin = PIN_NUM[p]
                        if ('motor' + str(pin) in dataraw) \
                           or ('power' + str(pin) in dataraw):
                            try:
                                outputall_pos = dataraw.index('motor'+str(pin))
                                t = 'motor'
                            except ValueError:
                                outputall_pos = dataraw.find('power'+str(pin))
                                t = 'power'
                            sensor_value = dataraw[(1+outputall_pos+len(t+str(pin))):].split()
                            if isNumeric(sensor_value[0]):
                                motorvalue = 2.55 * max(0,min(100,int(sensor_value[0])))
                                if motorvalue >= 0.0 and motorvalue <= 255.0:
                                    physical_pin_update(p,motorvalue)
                                else:
                                    print t.capitalize() + 'write failed'
                                    pass

                #Check for servo commands
                for p in range(PINS):
                    if ((PIN_USE[p] == 1) or (PIN_USE[p] == 3)):
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
                                motorvalue = int(sensor_value[0])
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

                #Check for config commands
                if ('config' in dataraw):
                    for p in range(PINS):
                        #check_broadcast = str(i) + 'on'
                        #print check_broadcast
                        pin = PIN_NUM[p]
                        pinUse = PIN_USE[p]
                        if 'config' + str(pin)+'out' in dataraw: # change pin to output from input
                            if pinUse == 0: # check to see if it is an input at moment
                                board.pinMode(pin, "OUTPUT") # make it an output
                                print 'pin' , pin , ' out'
                                PIN_USE[p] = 1
                        if 'config' + str(pin)+'in' in dataraw: # change pin to input from output
                            if pinUse != 0: # check to see if it not an input already
                                board.pinMode(pin, "INPUT") # make it an input
                                print 'pin' , pin , ' in'
                                PIN_USE[p] = 0


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
                if ('touchoffall' in dataraw):
                    for p in range(PINS):
                        pin = PIN_NUM[p]
                        try:
                            PIN_USE[p] = LAST_PIN_USE[pin]
                        except KeyError:
                            #No touch active
                            pass
                    
                for p in range(PINS):
                    pin = PIN_NUM[p]
                    if ('pin' + str(pin)+'high' in dataraw) \
                       or ('pin' + str(pin)+'on' in dataraw):
                        physical_pin_update(p,1)
                    if ('pin' + str(pin)+'low' in dataraw) \
                       or ('pin' + str(pin)+'off' in dataraw):
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
                        pause = False
                        if (distance > 500) or (distance < 1.1):
                            distance = 0
                        print'Distance:',round(distance), 'cm'
                        sensor_name = "pin" + str(pin) 
                        bcast_str = 'sensor-update "%s" %d' % (sensor_name, round(distance))
                        self.send_scratch_command(bcast_str)
                    if 'touch' + str(pin) in dataraw:
                        print "touch received, pin: " + str(pin)
                        p = PIN_NUM.index(pin)
                        if PIN_USE[p] != 4 and int(pin) <> 13:
                            LAST_PIN_USE[pin] = PIN_USE[p]
                            LAST_CAP_VALUE[pin] = 99
                            self.broadcast_pin_update(pin, LAST_CAP_VALUE[pin])
                            PIN_USE[p] = 4
                    if 'touchoff' + str(pin) in dataraw:
                        print "touch off received, pin: " + str(pin)
                        p = PIN_NUM.index(pin)
                        try:
                            PIN_USE[p] = LAST_PIN_USE[pin]
                        except KeyError:
                            #No touch active
                            pass
                        
                    if 'note' + str(pin) in dataraw:
                        i = data.index('[')
                        note = data[i+1:-2]
                        notes = note.split(',')
                        MELODY[pin] = notes
                        if len(MELODY[pin]) == len(DURATIONS[pin]):
                            self.playMelody(pin)
                    if 'dur' + str(pin) in dataraw:
                        i = data.index('[')
                        dur = data[i+1:-2]
                        durs = dur.split(',')
                        DURATIONS[pin] = durs
                        if len(DURATIONS[pin]) == len(MELODY[pin]):
                            self.playMelody(pin)
                    
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
global pause
pause = False

board = Shrimp.Shrimp()
version = board.version()

if version == 'version':
    try:
        com_port_open = True

        print 'Defining Initial Pin Usage'
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
                pause = True
                board.close()
                cleanup_threads((listener, sender,steppera,stepperb))
                time.sleep(2)
                board = Shrimp.Shrimp()
                cycle_trace = 'start'

            if (cycle_trace == 'start'):
                pause = False
                # open the socket
                print 'Starting to connect...' ,
                the_socket = create_socket(host, PORT)
                print 'Connected!'
                the_socket.settimeout(SOCKET_TIMEOUT)
                listener = ScratchListener(the_socket)
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
                pause = True
                cleanup_threads((listener,sender,steppera,stepperb))
                board.close()
                print 'Shrimp/Arduino closed'
                com_port_open = False
                sys.exit()
    except:
        print 'Final exception reached'
        if  com_port_open:
            board.close()
            print 'Shrimp/Arduino closed'
            com_port_open = False
        sys.exit()

