import time
import navio.ms5611
import navio.util
from numpy import mean
import numpy as np 


class Barometro:

    baro = navio.ms5611.MS5611()
    altitudBase = 0.0
    presionInicial=0.0
    
    presionF=0.0
    ## altitud a la que despega el dron, restar con la actual para conocer a que altura se encuentra

    def __init__(self):
        navio.util.check_apm()
        self.baro.initialize()
        time.sleep(0.1)
        self.setAltitudIni(20,2)

    def getAltitud_abs(self, muestras, decimales):
        presiones = []
        for i in range(0,muestras):
            presiones.append(self._getPres_())
            
            
        self.presionF = mean(presiones)
        presionRef = sum(presiones)/muestras
        return round(self._getAltitud_(presionRef), decimales)

    def getAltitud_real(self, muestras, decimales):
        altitud = self.getAltitud_abs(muestras, decimales)
        return altitud - self.altitudBase

    def _getAltitud_(self, pressure):
        #return 44330 * (1 - (pressure / 1013.25)**(1/5.255))
        return (1-(pressure/1014.565974)**(1/5.255))/0.0000225577 
    def _getPres_(self):
        self.baro.refreshPressure()
        time.sleep(0.01) # Waiting for pressure data ready 10ms
        self.baro.readPressure()

        self.baro.refreshTemperature()
        time.sleep(0.01) # Waiting for temperature data ready 10ms
        self.baro.readTemperature()

        self.baro.calculatePressureAndTemperature()
        return self.baro.PRES

    def setAltitudIni(self, muestras, decimales):
        altitudAbs = self.getAltitud_abs(muestras, decimales)
        self.altitudBase = altitudAbs
        
    def returnAltitude(self,presion,media):
        return (1-(presion/media)**(1/5.255))/0.0000225577 # Calcular altitud respecto presión base.
	    
        #altitud = (1-(presion/media)**(1/5.255))/0.0000225577 # Calcular altitud respecto presión base.
	    #print(media)
        #return altitud
    def getPresionInicial(self):
        self.presionInicial= self._getPres_()
        return self.presionInicial
    
    def Altitud_abs(self, muestras, decimales,presionCal):
        presiones = []
        
        for i in range(0,muestras):
            presiones.append(self._getPres_())

        self.presionRef = sum(presiones)/muestras
        return round(self.returnAltitude(presionCal,mean(presiones)),decimales)
        #return round(self._getAltitud_(self.presionRef), decimales)
    def Altitud_R(self, muestras, decimales,presionCal):
        altitud = self.Altitud_abs(muestras, decimales,presionCal)
        return altitud - self.altitudBase     
    
        
bar = Barometro()        

Cal=input(" Introduce OK para calibrar \n")
if Cal=="OK":
    #bar.baro.readPressure()
    bar.__init__() 
    time.sleep(0.01) # Waiting for pressure data ready 10ms
    bar.baro.readPressure()
    bar.baro.calculatePressureAndTemperature()
    
    presionInicial = bar.getPresionInicial()
    bar.setAltitudIni(50,3)
    print(presionInicial)
    altitudInicial=bar.getAltitud_real(50,3)
    print(altitudInicial)
    #print(AltitudInicial)
    print(bar.altitudBase)
    print(" Calibrado...")
    
    Muestras= input("introduzca muestras para el calculo de presion: \n")
    print("Tiene 15s para poner el dron en el nuevo sitio \n")
    time.sleep(15)
    #altitud = bar.Altitud_R(50, 3,presionInicial)
    presionFinal=bar.getAltitud_abs(50,3)
    altitudFinal= bar.returnAltitude(bar.presionF,presionInicial) 
    altitud = bar.getAltitud_real(50,3)
    print("Altitud: ", altitud)
    print(bar.altitudBase)
    print(altitudFinal)
    print(abs(altitudFinal)-abs(altitudInicial))
    print("PresionMEdia: ", bar.presionF)
    print(bar.getAltitud_real(50,3))
    
    
#time.sleep(10)
# bar.__init__()  
# print(bar.altitudBase)
# presionInicial = bar._getPres_
# print(bar._getPres_())
#altitudInicial = bar.setAltitudIni(MuestrasCal,3)
#print(altitudInicial)
# print(" Barómetro calibrado.")
# print(" Presión base [mbar]: %.6f" % baro._getPres_)
# print("----------------")
# P_mean = [] # Reseteamos la media de presión.
# print(" Tiene 10s para colocarlo a la altura deseada")
# time.sleep(10)
# print(" Comienza a testear.")
# print("----------------")
# print(altitudInicial)

