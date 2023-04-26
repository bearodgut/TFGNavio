import time
import navio.ms5611
import navio.util

class Barometro:

    baro = navio.ms5611.MS5611()
    altitudBase = 0.0 
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

        presionRef = sum(presiones)/muestras
        return round(self._getAltitud_(presionRef), decimales)

    def getAltitud_real(self, muestras, decimales):
        altitud = self.getAltitud_abs(muestras, decimales)
        return altitud - self.altitudBase

    def _getAltitud_(self, pressure):
        return 44330 * (1 - (pressure / 1013.25)**(1/5.255))

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