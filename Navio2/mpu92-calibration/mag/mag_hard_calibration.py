##############################################################
# Esta parte de código viene en mpu9250_i2c.py, pero como daba
# problema a la hora de importarlo porque no lo cogía bien,
# lo puse todo en el mismo fichero
##############################################################

import smbus,time
# MPU6050 Registers
MPU6050_ADDR = 0x69
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_PIN_CFG  = 0x37
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
#AK8963 registers
AK8963_ADDR   = 0x0C
AK8963_ST1    = 0x02
HXH          = 0x04
HYH          = 0x06
HZH          = 0x08
AK8963_ST1   = 0x02
AK8963_ST2   = 0x09
AK8963_CNTL  = 0x0A
AK8963_ASAX = 0x10

mag_sens = 4800.0 # magnetometer sensitivity: 4800 uT

"""

def MPU6050_start():
    # reset all sensors
    bus.write_byte_data(MPU6050_ADDR,PWR_MGMT_1,0x80)
    time.sleep(0.1)
    bus.write_byte_data(MPU6050_ADDR,PWR_MGMT_1,0x01)
    time.sleep(0.1)
    # power management and crystal settings
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    # alter sample rate (stability)
    samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, samp_rate_div)
    time.sleep(0.1)
    #Write to Configuration register
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
    time.sleep(0.1)
    #Write to Gyro configuration register
    gyro_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
    gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
    gyro_indx = 0
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
    time.sleep(0.1)
    #Write to Accel configuration register
    accel_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
    accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
    accel_indx = 0
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
    time.sleep(0.1)
    # interrupt register (related to overflow of data [FIFO])
    bus.write_byte_data(MPU6050_ADDR,INT_PIN_CFG,0x22)
    time.sleep(0.1)
    # enable the AK8963 magnetometer in pass-through mode
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)
    time.sleep(0.1)
    return gyro_config_vals[gyro_indx],accel_config_vals[accel_indx]
    
def read_raw_bits(register):
    # read accel and gyro values
    high = bus.read_byte_data(MPU6050_ADDR, register)
    low = bus.read_byte_data(MPU6050_ADDR, register+1)

    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    
    # convert to +- value
    if(value > 32768):
        value -= 65536
    return value

def mpu6050_conv():
    # raw acceleration bits
    acc_x = read_raw_bits(ACCEL_XOUT_H)
    acc_y = read_raw_bits(ACCEL_YOUT_H)
    acc_z = read_raw_bits(ACCEL_ZOUT_H)
    
    # raw gyroscope bits
    gyro_x = read_raw_bits(GYRO_XOUT_H)
    gyro_y = read_raw_bits(GYRO_YOUT_H)
    gyro_z = read_raw_bits(GYRO_ZOUT_H)

    #convert to acceleration in g and gyro dps
    a_x = (acc_x/(2.0**15.0))*accel_sens
    a_y = (acc_y/(2.0**15.0))*accel_sens
    a_z = (acc_z/(2.0**15.0))*accel_sens

    w_x = (gyro_x/(2.0**15.0))*gyro_sens
    w_y = (gyro_y/(2.0**15.0))*gyro_sens
    w_z = (gyro_z/(2.0**15.0))*gyro_sens
    
    return a_x,a_y,a_z,w_x,w_y,w_z
"""
def AK8963_start():
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
    time.sleep(0.1)
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x0F)
    time.sleep(0.1)
    coeff_data = bus.read_i2c_block_data(AK8963_ADDR,AK8963_ASAX,3)
    AK8963_coeffx = (0.5*(coeff_data[0]-128)) / 256.0 + 1.0
    AK8963_coeffy = (0.5*(coeff_data[1]-128)) / 256.0 + 1.0
    AK8963_coeffz = (0.5*(coeff_data[2]-128)) / 256.0 + 1.0
    time.sleep(0.1)
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
    time.sleep(0.1)
    AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
    AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
    AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,AK8963_mode)
    time.sleep(0.1)
    return [AK8963_coeffx,AK8963_coeffy,AK8963_coeffz] 
    
def AK8963_reader(register):
    # read magnetometer values
    low = bus.read_byte_data(AK8963_ADDR, register-1)
    high = bus.read_byte_data(AK8963_ADDR, register)
    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    # convert to +- value
    if(value > 32768):
        value -= 65536
    
    return value

def AK8963_conv():
    # raw magnetometer bits
    while 1:
##        if ((bus.read_byte_data(AK8963_ADDR,AK8963_ST1) & 0x01))!=1:
##            return 0,0,0
        mag_x = AK8963_reader(HXH)
        mag_y = AK8963_reader(HYH)
        mag_z = AK8963_reader(HZH)

        # the next line is needed for AK8963
        if (bus.read_byte_data(AK8963_ADDR,AK8963_ST2)) & 0x08!=0x08:
            break
        
    #convert to acceleration in g and gyro dps
##    m_x = AK8963_coeffs[0]*(mag_x/(2.0**15.0))*mag_sens
##    m_y = AK8963_coeffs[1]*(mag_y/(2.0**15.0))*mag_sens
##    m_z = AK8963_coeffs[2]*(mag_z/(2.0**15.0))*mag_sens
    m_x = (mag_x/(2.0**15.0))*mag_sens
    m_y = (mag_y/(2.0**15.0))*mag_sens
    m_z = (mag_z/(2.0**15.0))*mag_sens
    return m_x,m_y,m_z
    
# start I2C driver
bus = smbus.SMBus(1) # start comm with i2c bus
time.sleep(0.1)
#gyro_sens,accel_sens = MPU6050_start() # instantiate gyro/accel
time.sleep(0.1)
AK8963_coeffs = AK8963_start() # instantiate magnetometer
time.sleep(0.1)


##############################################################
# Esta parte de código viene en mpu9250_i2c.py, pero como daba
# problema a la hora de importarlo porque no lo cogía bien,
# lo puse todo en el mismo fichero
##############################################################



######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU9250/MPU9265 board
# (MPU6050 - accel/gyro, AK8963 - mag)
# and solves for tje hard iron offset for a
# magnetometer using a calibration block
#
#
######################################################
#
# wait 5-sec for IMU to connect
import time,sys
#from mpu9250_i2c import *

sys.path.append("../")
t0 = time.time()
start_bool = False # if IMU start fails - stop calibration
while time.time()-t0<5:
    try: 
        #from mpu9250_i2c import *
        start_bool = True
        break
    except:
        continue
import numpy as np
import csv
import matplotlib.pyplot as plt

time.sleep(2) # wait for mpu to load
# 
#####################################
# Mag Calibration Functions
#####################################
#
def outlier_removal(x_ii,y_ii):
    x_diff = np.append(0.0,np.diff(x_ii)) # looking for outliers
    stdev_amt = 5.0 # standard deviation multiplier
    x_outliers = np.where(np.abs(x_diff)>np.abs(np.mean(x_diff))+\
                          (stdev_amt*np.std(x_diff)))
    x_inliers  = np.where(np.abs(x_diff)<np.abs(np.mean(x_diff))+\
                          (stdev_amt*np.std(x_diff)))
    y_diff     = np.append(0.0,np.diff(y_ii)) # looking for outliers
    y_outliers = np.where(np.abs(y_diff)>np.abs(np.mean(y_diff))+\
                          (stdev_amt*np.std(y_diff)))
    y_inliers  = np.abs(y_diff)<np.abs(np.mean(y_diff))+\
                 (stdev_amt*np.std(y_diff))
    if len(x_outliers)!=0:
        x_ii[x_outliers] = np.nan # null outlier
        y_ii[x_outliers] = np.nan # null outlier
    if len(y_outliers)!=0:
        y_ii[y_outliers] = np.nan # null outlier
        x_ii[y_outliers] = np.nan # null outlier
    return x_ii,y_ii

def mag_cal():
    print("-"*50)
    print("Magnetometer Calibration")
    mag_cal_rotation_vec = [] # variable for calibration calculations
    for qq,ax_qq in enumerate(mag_cal_axes):
        input("-"*8+" Press Enter and Start Rotating the IMU Around the "+ax_qq+"-axis")
        print("\t When Finished, Press CTRL+C")
        mag_array = []
        t0 = time.time()
        while True:
            try:
                mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
            except KeyboardInterrupt:
                break
            except:
                continue
            mag_array.append([mx,my,mz]) # mag array
        mag_array = mag_array[20:] # throw away first few points (buffer clearing)
        mag_cal_rotation_vec.append(mag_array) # calibration array
        print("Sample Rate: {0:2.0f} Hz".format(len(mag_array)/(time.time()-t0)))
        
    mag_cal_rotation_vec = np.array(mag_cal_rotation_vec) # make numpy array
    ak_fit_coeffs = []
    indices_to_save = [0,0,1] # indices to save as offsets
    for mag_ii,mags in enumerate(mag_cal_rotation_vec):
        mags = np.array(mags) # mag numpy array
        x,y = mags[:,cal_rot_indices[mag_ii][0]],\
                        mags[:,cal_rot_indices[mag_ii][1]] # sensors to analyze
        x,y = outlier_removal(x,y) # outlier removal
        y_0 = (np.nanmax(y)+np.nanmin(y))/2.0 # y-offset
        x_0 = (np.nanmax(x)+np.nanmin(x))/2.0 # x-offset
        ak_fit_coeffs.append([x_0,y_0][indices_to_save[mag_ii]]) # append to offset
        
    return ak_fit_coeffs,mag_cal_rotation_vec
#
#########################################
# Plot Values to See Calibration Impact
#########################################
#
def mag_cal_plot():
    plt.style.use('ggplot') # start figure
    fig,axs = plt.subplots(1,2,figsize=(12,7)) # start figure
    for mag_ii,mags in enumerate(mag_cal_rotation_vec):
        mags = np.array(mags) # magnetometer numpy array
        x,y = mags[:,cal_rot_indices[mag_ii][0]],\
                    mags[:,cal_rot_indices[mag_ii][1]]
        x,y = outlier_removal(x,y) # outlier removal 
        axs[0].scatter(x,y,
                       label='Rotation Around ${0}$-axis (${1},{2}$)'.\
                    format(mag_cal_axes[mag_ii],
                           mag_labels[cal_rot_indices[mag_ii][0]],
                           mag_labels[cal_rot_indices[mag_ii][1]]))
        axs[1].scatter(x-mag_coeffs[cal_rot_indices[mag_ii][0]],
                    y-mag_coeffs[cal_rot_indices[mag_ii][1]],
                       label='Rotation Around ${0}$-axis (${1},{2}$)'.\
                    format(mag_cal_axes[mag_ii],
                           mag_labels[cal_rot_indices[mag_ii][0]],
                           mag_labels[cal_rot_indices[mag_ii][1]]))
    axs[0].set_title('Before Hard Iron Offset') # plot title
    axs[1].set_title('After Hard Iron Offset') # plot title
    mag_lims = [np.nanmin(np.nanmin(mag_cal_rotation_vec)),
                np.nanmax(np.nanmax(mag_cal_rotation_vec))] # array limits
    mag_lims = [-1.1*np.max(mag_lims),1.1*np.max(mag_lims)] # axes limits
    for jj in range(0,2):
        axs[jj].set_ylim(mag_lims) # set limits
        axs[jj].set_xlim(mag_lims) # set limits
        axs[jj].legend() # legend
        axs[jj].set_aspect('equal',adjustable='box') # square axes
    fig.savefig('mag_cal.png',dpi=300,bbox_inches='tight',
                facecolor='#FFFFFF') # save figure
    plt.show() #show plot

if __name__ == '__main__':
    if not start_bool:
        print("IMU not Started - Check Wiring and I2C")
    else:
        #
        ###################################
        # Magnetometer Calibration
        ###################################
        #
        mag_labels = ['m_x','m_y','m_z'] # mag labels for plots
        mag_cal_axes = ['z','y','x'] # axis order being rotated
        cal_rot_indices = [[0,1],[1,2],[0,2]] # indices of heading for each axis
        mag_coeffs,mag_cal_rotation_vec = mag_cal() # grab mag coefficients
        #
        ###################################
        # Plot with and without offsets
        ###################################
        #
        mag_cal_plot() # plot un-calibrated and calibrated results
        #
        

