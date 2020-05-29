import time
import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import Adafruit_ADS1x15
import pigpio
import numpy as np
import math
from math import atan2, degrees


DIR = 20     # Direction GPIO Pin
STEP = 21    # Step GPIO Pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation 
MS0=14	#Mode GPIO Pin 
MS1=15	#Mode GPIO Pin

pi = pigpio.pi()

# Set up pins as an output
pi.set_mode(DIR, pigpio.OUTPUT)
pi.set_mode(STEP, pigpio.OUTPUT)
pi.write(DIR,CCW) #set clockwise or anticlockwise

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm303_accel.LSM303_Accel(i2c)
sensor1 = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
adc = Adafruit_ADS1x15.ADS1015()
GAIN = 1
voltage=[]

def vector_2_degrees(x, y):
    angle = degrees(atan2(y, x))
    if angle < 0:
        angle += 360
    return angle

def get_heading(_sensor):
    magnet_x, magnet_y, _ = _sensor.magnetic
    return vector_2_degrees(magnet_x, magnet_y)

def get_inclination(_sensor):
    x, y, z = _sensor.acceleration
    return vector_2_degrees(x, z), vector_2_degrees(y, z)
MODE = (MS0,MS1)   # Microstep Resolution GPIO Pins
RESOLUTION = {'Full': (0, 0),
              'Half': (1,0),
              '1/4': (0,1),
              '1/8': (1, 1)
                }
for i in range(2):
    pi.write(MODE[i], RESOLUTION['Full'][i])
while True:
    for y in range(1648):
        print ('ADC channel 0 value:')
        print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*range(4)))
        print('-' * 37)
        values = [0]*8
        for i in range(4):
            values[i] = adc.read_adc(i, gain=GAIN)
            if  i>0:
                values[i]=0
        print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))
        print('')
        time.sleep(0.5)
        v=(values[0]*3.3)/1647 #FOR GAIN=1, 2047 adc counts=4.096v
        voltage.append(v)
        x= values[0]/1647; #1645=Maximum ADC counts for 3.3v 
        x=x*255;# 255=
        pi.set_PWM_dutycycle(STEP, x)
        pi.set_PWM_frequency(STEP, 500)
        acc_x, acc_y, acc_z = sensor.acceleration
        mag_x, mag_y, mag_z = sensor1.magnetic
        print('Acceleration (m/s^2):({0:10.3f},{1:10.3f},{2:10.3f})'.format(acc_x, acc_y, acc_z))
        angle_xz, angle_yz = get_inclination(sensor)
        print("XZ angle = {:6.2f}deg   YZ angle = {:6.2f}deg".format(angle_xz, angle_yz))
        print('')
        time.sleep(1.0)
        print('Magnetometer (gauss):({0:10.3f},{1:10.3f},{2:10.3f})'.format(mag_x, mag_y, mag_z))
        print("heading: {:.2f} degrees".format(get_heading(sensor1)))  
        print('')
        time.sleep(1.0)
    f = np.fft.fft(voltage)
    power_spectrum = []
    entire_energy = 0
    highest_frequency_energy = 0
    for i in range(0,1648):#1647 adc counts=3.3v
        power_spectrum.append(math.sqrt((f.real[i] * f.real[i])+(f.imag[i] * f.imag[i])))
        total_energy += power_spectrum[i]
        if(i==1023):#(N/2)-1
            highest_frequency_energy=power_spectrum[i]
    eta = (highest_frequency_energy/(total_energy))*100
    print("ADC Validation Efficiency:" + str(eta) + "%")
    if eta>80 and eta <100:
        print("ADC data is valid")
    else:
        print("ADC data is not valid")
