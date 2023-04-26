"""
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi

To run this example navigate to the directory containing it and run following commands:
sudo python Barometer_example.py
"""

import time
import matplotlib.pyplot as plt
import navio.ms5611
import navio.util
from numpy import mean
import numpy as np 
navio.util.check_apm()


baro = navio.ms5611.MS5611()
baro.initialize()

# importing the required module
import matplotlib.pyplot as plt

# Calcular las medias
media = 0
T_mean = [] # Lista vacía para calcular la temperatura media.
P_mean = [] # Lista vacía para calcular la presión media.
A_mean = [] # Lista vacía para calcular la altitud media.
Altitud = 0
timer = []
y2 = []

count_i = 0
count_j = 0

segs_calib = input(" Introduce el tiempo de calibración [s] \n")
print(" Calibrando...")

while(count_i < int(segs_calib)):
	baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()
	baro.calculatePressureAndTemperature()
	
	P_mean.append(baro.returnPressure()) # Calculamos la media de la presión. Sirve para calibrar el barómetro.
	count_i +=  1
	print(count_i)
 
	time.sleep(1)

media = mean(P_mean) # Lo introducimos 
print(" Barómetro calibrado.")
print(" Presión base [mbar]: %.6f" % media)
print("----------------")
P_mean = [] # Reseteamos la media de presión.
print(" Tiene 10s para colocarlo a la altura deseada")
time.sleep(10)
print(" Comienza a testear.")
print("----------------")

while(count_j < 120):
    
	baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()

	baro.refreshTemperature()
	time.sleep(0.01) # Waiting for temperature data ready 10ms
	baro.readTemperature()

	baro.calculatePressureAndTemperature()

	presion = baro.returnPressure()
	altitud = baro.returnAltitude(presion,media)
	
	T_mean.append(baro.returnTemperature())
	P_mean.append(baro.returnPressure()) 
	A_mean.append(altitud)
 
	# Descomentar esta línea si queremos obtener los resultados de CADA medición.
	print (" Temperature(ºC): %.6f" % (baro.returnTemperature()), "Pressure(millibar): %.6f" % (baro.returnPressure()), "Altitude(m): %.6f" % (altitud))
	print ("Mean temperature: %.6f" % (mean(T_mean)), "Mean pressure: %.6f" % (mean(P_mean)), "Mean altitude(m): %.6f" % (mean(A_mean)))	
	print ("Desv temperature: %.6f" % (np.std(T_mean)), "Desv pressure: %.6f" % (np.std(P_mean)), "Desv altitude(m): %.6f" % (np.std(A_mean)))	
	

	y2.append(altitud)
	time.sleep(0.2)
	timer.append(count_j)
	count_j += 1
	print(count_j)
	time.sleep(1)

print("-"*100)
print("Informe resultados ")
print("  Medida media temperatura","{:3.3f}".format(mean(T_mean)) +" [ºC]", "Desviacion típica temperatura","{:+3.3f}".format(np.std(T_mean)) + " [ºC]")
print("  Medida media presión:","{:3.3f}".format(mean(P_mean)) +" [mbar]", "Desviacion típica presión:","{:3.3f}".format(np.std(P_mean)) + " [mbar]")
print("  Medida media altitud:","{:8.3f}".format(mean(A_mean)) +" [m]", "\tDesviacion típica altitud:","{:3.3f}".format(np.std(A_mean)) + " [m]")

media = 0	
 
plt.figure(figsize=(12,8), dpi=80)
plt.plot(timer, A_mean,'r.')
plt.xlabel("tiempo [s]")
plt.ylabel("altura [m]")
plt.title("Medidas realizadas de altura")
plt.savefig('A_media4.png')


plt.figure(figsize=(12,8), dpi=80)
plt.plot(timer,y2,'b.')
plt.xlabel("tiempo [s]")
plt.ylabel("altura [m]")
plt.title("Medidas realizadas de altura")
plt.savefig('Altitud4.png')
