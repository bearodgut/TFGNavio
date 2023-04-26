
# import time
# import matplotlib.pyplot as plt
# import navio.ms5611
# import navio.util
# from numpy import mean
# import numpy as np 
# navio.util.check_apm()


# baro = navio.ms5611.MS5611()
# baro.initialize()
# # importing the required module
# import matplotlib.pyplot as plt


# # Calcular las medias
# media = 0
# meadiaAltitud = 0
# T_mean = [] # Lista vacía para calcular la temperatura media.
# P_mean = [] # Lista vacía para calcular la presión media.
# A_mean = [] # Lista vacía para calcular la altitud media.
# Altitud = 0
# timer = []
# y2 = []

# count_i = 0
# count_j = 0

# segs_calib = input(" Introduce el tiempo de calibración [s] \n")
# print(" Calibrando...")

# while(count_i < int(segs_calib)):
# 	baro.refreshPressure()
# 	time.sleep(0.01) # Waiting for pressure data ready 10ms
# 	baro.readPressure()
# 	baro.calculatePressureAndTemperature()
	
# 	P_mean.append(baro.returnPressure()) # Calculamos la media de la presión. Sirve para calibrar el barómetro.
# 	count_i +=  1
# 	presion2 = baro.returnPressure()
 	
# 	A_mean.append(baro.returnAltitude(presion2,mean(P_mean)))
# 	print(count_i)
 
# 	time.sleep(1)

# media = mean(P_mean) # Lo introducimos 
# mediaAltitud =mean(A_mean)
# print(" Barómetro calibrado.")
# print(" Presión base [mbar]: %.6f" % media)
# #pre=baro.returnPressure()
# #altitudInicial= baro.returnAltitude(pre,media)
# print("----------------")
# print(" Altitud base [m]: %.6f" % mediaAltitud )
# print("----------------")

# P_mean = [] # Reseteamos la media de presión.
# A_mean = []
# print(" Tiene 15s para colocarlo a la altura deseada")
# time.sleep(15)
# print(" Comienza a testear.")
# print("----------------")
# while(count_j < 20):
    
# 	baro.refreshPressure()
# 	time.sleep(0.01)
# 	baro.readPressure()

# 	baro.refreshTemperature()
# 	time.sleep(0.01) # Waiting for temperature data ready 10ms
# 	baro.readTemperature()

# 	baro.calculatePressureAndTemperature()

# 	presion = baro.returnPressure()
# 	altitud = baro.returnAltitude(presion,media)
 
# 	T_mean.append(baro.returnTemperature())
# 	P_mean.append(baro.returnPressure()) 
# 	A_mean.append(altitud)
 
# 	# Descomentar esta línea si queremos obtener los resultados de CADA medición.
# 	#print (" Temperature(ºC): %.6f" % (baro.returnTemperature()), "Pressure(millibar): %.6f" % (baro.returnPressure()), "Altitude(m): %.6f" % (altitud))
# 	#print ("Mean temperature: %.6f" % (mean(T_mean)), "Mean pressure: %.6f" % (mean(P_mean)), "Mean altitude(m): %.6f" % (mean(A_mean)))	
# 	#print ("Desv temperature: %.6f" % (np.std(T_mean)), "Desv pressure: %.6f" % (np.std(P_mean)), "Desv altitude(m): %.6f" % (np.std(A_mean)))	
	

# 	y2.append(altitud)
# 	time.sleep(0.2)
# 	timer.append(count_j)
# 	count_j += 1
# 	#print(count_j)
# 	time.sleep(1)

# print("-"*100)
# print("Informe resultados ")
# print("  Medida media temperatura","{:3.3f}".format(mean(T_mean)) +" [ºC]", "Desviacion típica temperatura","{:+3.3f}".format(np.std(T_mean)) + " [ºC]")
# print("  Medida media presión:","{:3.3f}".format(mean(P_mean)) +" [mbar]", "Desviacion típica presión:","{:3.3f}".format(np.std(P_mean)) + " [mbar]")
# print("  Medida media altitud:","{:8.3f}".format(mean(A_mean)) +" [m]", "\tDesviacion típica altitud:","{:3.3f}".format(np.std(A_mean)) + " [m]")
# #print("Presion incial - presion media:" ,(abs(altitudInicial)-A_mean))
# media = 0	
 
# # plt.figure(figsize=(12,8), dpi=80)
# # plt.plot(timer, A_mean,'r.')
# # plt.xlabel("tiempo [s]")
# # plt.ylabel("altura [m]")
# # plt.title("Medidas realizadas de altura")
# # plt.savefig('A_media4.png')


# # plt.figure(figsize=(12,8), dpi=80)
# # plt.plot(timer,y2,'b.')
# # plt.xlabel("tiempo [s]")
# # plt.ylabel("altura [m]")
# # plt.title("Medidas realizadas de altura")
# # plt.savefig('Altitud4.png')


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
presiones = []
altitudCalibracion=[]
mediaAltitudCalibracion=0
count_i = 0
count_j = 0

segs_calib = input(" Introduce el tiempo de calibración [s] \n")
print(" Calibrando...")

## calibrar
for i in range(0,int(segs_calib)):
    baro.refreshPressure()
    time.sleep(0.01)# Waiting for pressure data ready 10ms
    baro.readPressure()
    baro.calculatePressureAndTemperature()
    presiones.append(baro.returnPressure())
    #altitudCalibracion.append(baro.A)
    print(i)
    time.sleep(1)
    print(baro.A)
    time.sleep(1)
## Calibracion hecha
#CAlcular PResion media de CAlibracion
#CALCULAR ALTITUD MEDIA DE CALIBRACION

media = mean(presiones)
#mediaAltitudCalibracion = mean(altitudCalibracion)

print(" Barómetro calibrado.")
print(" Presión base [mbar]: %.6f" % media)
print("----------------")
#print(" Altitud base [mbar]: %.6f" % mediaAltitudCalibracion)
# print("----------------")

print(" Tiene 10s para colocarlo a la altura deseada")
time.sleep(10)
print(" Comienza a testear.")
print("----------------")


## Calculo una vez que se mueve el dron
for j in range(0,int(segs_calib)):
	baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()

	baro.calculatePressureAndTemperature()
	presiones.append(baro.returnPressure())

	T_mean.append(baro.returnTemperature())
	P_mean.append(baro.returnPressure()) 
	presion = baro.returnPressure()
	A_mean.append(baro.returnAltitude(presion,media))

	print(" Temperature(ºC): %.6f" % (baro.returnTemperature()), "Pressure(millibar): %.6f" % (baro.returnPressure()), "Altitude(m): %.6f" % (baro.returnAltitude(presion,media)))
	print(j)
	time.sleep(1)


print("-"*100)
print("Informe resultados ")
print("  Medida media temperatura","{:3.3f}".format(mean(T_mean)) +" [ºC]", "Desviacion típica temperatura","{:+3.3f}".format(np.std(T_mean)) + " [ºC]")
print("  Medida media presión:","{:3.3f}".format(mean(P_mean)) +" [mbar]", "Desviacion típica presión:","{:3.3f}".format(np.std(P_mean)) + " [mbar]")
print("  Medida media altitud:","{:8.3f}".format(mean(A_mean)) +" [m]", "\tDesviacion típica altitud:","{:3.3f}".format(np.std(A_mean)) + " [m]")

#media = 0