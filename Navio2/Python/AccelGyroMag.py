"""
MS5611 driver code is placed under the BSD license.
Copyright (c) 2014, Emlid Limited, www.emlid.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.
	* Neither the name of the Emlid Limited nor the names of its contributors
	may be used to endorse or promote products derived from this software
	without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from matplotlib.ft2font import BOLD
import spidev
import time
import argparse 
import sys
import navio.mpu9250
import navio.util
import numpy as np
import matplotlib.pyplot as plt
import math 
navio.util.check_apm()

parser = argparse.ArgumentParser()
parser.add_argument("-i", help = "Sensor selection: -i [sensor name]. Sensors names: mpu is MPU9250, lsm is LSM9DS1")

if len(sys.argv) == 1:
    print ("Enter parameter")
    parser.print_help()
    sys.exit(1)
elif len(sys.argv) == 2:
    sys.exit("Enter sensor name: mpu or lsm")

args = parser.parse_args()

if args.i == 'mpu':
    print ("Selected: MPU9250")
    imu = navio.mpu9250.MPU9250()
elif args.i == 'lsm':
    print ("Selected: LSM9DS1")
    imu = navio.lsm9ds1.LSM9DS1()
else:
    print ("Wrong sensor name. Select: mpu or lsm")
    sys.exit(1)

if imu.testConnection():
    print ("Connection established: True")
else:
    print(imu.response)
    sys.exit("Connection established: False")

imu.initialize()
time.sleep(1)

count = 0
Acc_lista = [] # Lista para guardar datos del acelerómetro.
Mag_lista = [] # # Lista para guardar datos del magnetómetro.
tiempo_prev = time.time()
gx_prev,gy_prev,gz_prev = 0,0,0 # Angulo inicial para calcular rot con giróscopio.

t1 = time.time() # Para calcular nº muestras/s
ii = 1000 # Numero de puntos
mpu9250_vec_OX,mpu9250_vec_OY,t_vec = [],[],[]

print('\x1b[1;31m' +
'\x1b[1;31m' + '**************************************************************\n\
**' + '\033[0m' + '      Comprobación del funcionamiento de 9DOF IMU         ' + '\x1b[1;31m'  + '**\n\
**************************************************************' + '\033[0m' + '\n')

execute = True

while execute:
    caso = input('\033[0m' + '\
    1) Valores de acelerómetro, giróscopo y magnetómetro.\n\
    2) Comprobación del error en la medición estática de la gravedad en el acelerómetro.\n\
    3) Comportamiento dinámico del acelerómetro y giróscopo. \n\
    4) Brújula usango magnetómetro. \n\
    5) Salir de la ejecución.\n' + '\033[32m' + '\
    Seleccione el tipo de resolución que desee ejecutar: '  + '\033[0m')

    if int(caso) == 1:   
        num_c1 = input('\x1b[1;35m' + '\n---> Introduzca numero de pruebas a realizar: '+ '\033[0m')
        c1 = 0
        while (c1 < int(num_c1)):
            m9a, m9g, m9m, m9t = imu.getMotion9() # Guardamos info acel,giros,magne,temperatura.
            print(" Acc [m/seg^2]:", "{:+9.3f}".format(m9a[0]), "{:+7.3f}".format(m9a[1]), "{:+7.3f}".format(m9a[2])),
            print (" Gyr [º/seg]:", "{:+9.3f}".format(m9g[0]), "{:+7.3f}".format(m9g[1]), "{:+7.3f}".format(m9g[2])), 
            print (" Mag [uTesla]:", "{:+9.3f}".format(m9m[0]), "{:+7.3f}".format(m9m[1]), "{:+7.3f}".format(m9m[2])),
            print("--"*16)
            time.sleep(0.5)
            c1 += 1
        print("Fin de las mediciones.\n")
        
    elif int(caso) == 2:
        num_c2 = input('\x1b[1;35m' + '\n---> Introduzca numero de pruebas a realizar. A mayor pruebas, más fiabilidad: '+ '\033[0m')        
    
        print("Realizando las mediciones...\n")
        c2 = 0
        # Creamos una lista para poder calcular la estadística con la funcion dear()
        Acc_lista_x,Acc_lista_y,Acc_lista_z = [],[],[] 
        while (c2 < int(num_c2)):
            m9a,_,_,_ = imu.getMotion9()
            # Con cada iteración, guardamos en la lista Acc_lista_x lo que se lee en el eje OX de la aceleración.
            Acc_lista_x.append(m9a[0]) 
            # Con cada iteración, guardamos en la lista Acc_lista_y lo que se lee en el eje OY de la aceleración.
            Acc_lista_y.append(m9a[1]) 
            # Con cada iteración, guardamos en la lista Acc_lista_z lo que se lee en el eje OZ de la aceleración.
            Acc_lista_z.append(m9a[2]) 
            c2 += 1
            time.sleep(0.01)
            
        eje =  input('\x1b[1;35m' + '\n---> Elija sobre qué eje desea realizar la medición 1: OX, 2: OY, 3: OZ: '+ '\033[0m')    
        if int(eje) == 1: # Eje OX
            # Le pasamos la lista a la función que calcula todos esos parametros.
            media, desv_tipi, err_abso, err_rela = imu.dear(Acc_lista_x) 
            eje = "OX"
        elif int(eje) == 2: # EJE OY
            # Le pasamos la lista a la función que calcula todos esos parametros.
            media, desv_tipi, err_abso, err_rela = imu.dear(Acc_lista_y) 
            eje = "OY"
        elif int(eje) == 3: # EJE OZ
            # Le pasamos la lista a la función que calcula todos esos parametros.  
            media, desv_tipi, err_abso, err_rela = imu.dear(Acc_lista_z) 
            eje = "OZ"
        else:
            print("Asegúrese de que ejecuta una de las tres opciones, tecleando 1 ó 2 ó 3 y luego pulse ENTER.")
            print("Saliendo...\n")
            time.sleep(1) 
        
        print("Informe resultados " + eje +"." )
        print("  Medida media accl:","{:3.3f}".format(media) +" [m/seg^2]")
        print("  Desviación típica:", "{:3.3f}".format(desv_tipi) +" [m/seg^2]")
        print("  Error absoluto:", "{:8.3f}".format(err_abso) + " [m/seg^2]")
        print("  Error relativo:", "{:8.3f}".format(err_rela) + " % \n")
        time.sleep(2)
    
    elif int(caso) == 3:   
        opt = input('\x1b[1;35m' + '\n---> Mostrar los valores leídos [1], plotear los valores [2]: '+ '\033[0m')
        
        if int(opt) == 1:
            tiempo_prev = time.time()
            c3 = 0
            num_c3_1 = input('\x1b[1;35m' + '\n---> Seleccione numero de pruebas a realizar:  '+ '\033[0m')
            # Angulo inicial para calcular rot con giróscopo.
            gx_prev,gy_prev,gz_prev = 0,0,0 
            while c3 < int(num_c3_1):
                m9a, m9g,_,_ = imu.getMotion9() # Solo nos interesa datos acelerom y girosco
                # La función calcula el ángulo inclinación pasándole la aceleración en los 3 ejes.
                accel_incl_ang_x, accel_incl_ang_y,accel_incl_ang_z = imu.ang_inclinacion(m9a[0],m9a[1],m9a[2])
                dt = time.time() - tiempo_prev
                tiempo_prev = time.time()
                # Calcula angulo de rotación de los tres ejes.
                gyros_rot_ang_x,gyros_rot_ang_y,gyros_rot_ang_z = imu.ang_rotacion(m9g[0],m9g[1],m9g[2],gx_prev,gy_prev,gz_prev,dt)
                gx_prev = gyros_rot_ang_x
                gy_prev = gyros_rot_ang_y
                gz_prev = gyros_rot_ang_z
                print (" Ángulo de inclinación OX [º]:", "{:+9.3f}".format(math.degrees(accel_incl_ang_x))),
                print (" Ángulo de inclinación OY [º]:", "{:+9.3f}".format(math.degrees(accel_incl_ang_y))),
                #print("-----------------------------")
                print (" Ángulo de rotación OX [º]:", "{:+9.3f}".format(math.degrees(gyros_rot_ang_x))),
                print (" Ángulo de rotación OY [º]:", "{:+9.3f}".format(math.degrees(gyros_rot_ang_y))),
                print("-----------------------------")
                x,y,_= imu.filtro_complementario()
                print(math.degrees(x))
                print(math.degrees(y))
                c3 += 1
                time.sleep(0.45)
        
        elif int(opt) == 2:
            tiempo_prev = time.time()
            # t_vec:  vector de tiempo en la gráfica (eje OX)
            # mpu9250_vec_OX: Lista que guarda en GRADOS los datos de acel, giros y filtro en el OX.
            # mpu9250_vec_OY: Lista que guarda en GRADOS los datos de acel, giros y filtro en el OY.
            t_vec,mpu9250_vec_OX,mpu9250_vec_OY = [],[],[]
            gx_prev,gy_prev,gz_prev = 0,0,0 # Angulo inicial para calcular rot con giróscopio.
            ii = 1000
            c3_2 = 0
            num_c3_2 = input('\x1b[1;35m' + '\n---> Introduzca numero de pruebas a realizar:  '+ '\033[0m')
            while c3_2 < int(num_c3_2):
                m9a, m9g,_,_ = imu.getMotion9() # Solo nos interesa datos acelerom y girosco
                # La función calcula el ángulo inclinación pasándole la aceleración en los 3 ejes.
                accel_incl_ang_x, accel_incl_ang_y,accel_incl_ang_z = imu.ang_inclinacion(m9a[0],m9a[1],m9a[2])
                dt = time.time() - tiempo_prev
                tiempo_prev = time.time()
                # Calcula angulo de rotación de los tres ejes.
                gyros_rot_ang_x,gyros_rot_ang_y,gyros_rot_ang_z = imu.ang_rotacion(m9g[0],m9g[1],m9g[2],gx_prev,gy_prev,gz_prev,dt)
                # Calcula angulo con el filtro complementario.
                ang_filtro_x, ang_filtro_y,ang_filtro_z = imu.filtro_complementario()
                gx_prev = gyros_rot_ang_x 
                gy_prev = gyros_rot_ang_y 
                gz_prev = gyros_rot_ang_z            
                t_vec.append(time.time())
                mpu9250_vec_OX.append([math.degrees(accel_incl_ang_x),math.degrees(gyros_rot_ang_x),math.degrees(ang_filtro_x)])
                mpu9250_vec_OY.append([math.degrees(accel_incl_ang_y),math.degrees(gyros_rot_ang_y),math.degrees(ang_filtro_y)])
                c3_2 +=1
                print(c3_2)
                time.sleep(0.01)
            
            # plt.style.use(‘style_name”): Lista de estilos dosponibles en matplotlib. El nuestro es ggplot
            # https://www.geeksforgeeks.org/style-plots-using-matplotlib/
            plt.style.use('ggplot')
            
            # Los tres string de la leyenda.
            mpu9250_str = ['Ángulo incl. acelerómetro','Ángulo rotac. giróscopo','Ángulo filtro'] 
            # Eje de tiempo OX
            t_vec = np.subtract(t_vec,t_vec[0]) 
            # Ploteamos el resultado en 2 gráficas. 
            fig,axs = plt.subplots(2,1,figsize=(14,9), sharex = True)
            plt.subplots_adjust(hspace=0.3) # Espaciado entre gráficas 
            cmap = plt.cm.Set1 # Mapa de colores incorporado.

            ax = axs[0]
            data_vec = []
            for zz in range(0,np.shape(mpu9250_vec_OX)[1]): # Construye la gráfica. range (0,3) (incl,rotac,filtro)
               # Recorre ii putnos de accel_incl_ang_x, luego recorre ii puntos de gyros_rot_ang_x, y finalmente el del filtro.
               data_vec = [ii[zz] for ii in mpu9250_vec_OX] 
               ax.plot(t_vec,data_vec,label=mpu9250_str[zz],color=cmap(zz))
            
            ax.set_title("ANGULOS CON RESPECTO EJE X", fontsize = 16, fontweight='bold',position=(0.5,0.9))
            ax.set_ylabel('Ángulo [º]',fontsize = 17, fontweight = "bold")
            ax.yaxis.set_label_coords(x=-0.055,y=0.5) # Coordenadas de la etiqueta eje OY
            ax.yaxis.set_tick_params(labelsize=12) # Tamaño fuente de los valores eje OY 
            plt.savefig("AccelGyro")

            ax1 = axs[1]
            data_vec = []
            for zz in range(0,np.shape(mpu9250_vec_OY)[1]):
    
                data_vec = [ii[zz] for ii in mpu9250_vec_OY]
                ax1.plot(t_vec,data_vec,label=mpu9250_str[zz],color=cmap(zz))
                
            # bbox_to_anchor: coloca la leyenda en la posición (x, x) en la coordenada de los ejes. (0, 0) es
            # la esquina inferior izquierda, y (1.0, 1.0) es la esquina superior derecha de la coordenada del eje. 
            ax1.legend(bbox_to_anchor=(1,1.3)) # Situa la leyenda entre medio de las dos gráficas.
            ax1.set_title("ÁNGULOS CON RESPECTO EJE Y", fontsize = 16, fontweight='bold',position=(0.5,1.25)) # Añade el título
            ax1.set_ylabel('Ángulo [º]',fontsize = 17, fontweight = "bold") # Añade titulo a OY
            ax1.set_xlabel('Tiempo [s]', fontsize = 15, fontweight = 'bold') # Añade titulo a OX
            ax1.yaxis.set_label_coords(x=-0.055,y=0.5) # Coordenadas de la etiqueta eje OY
            ax1.xaxis.set_label_coords(x=0.5,y=-0.15) # Coordenadas de la etiqueta eje OX
            ax1.xaxis.set_tick_params(labelsize=12) # Tamaño fuente de los valores eje OX
            ax1.yaxis.set_tick_params(labelsize=12) # Tamaño fuente de los valores eje OY

            plt.savefig("AccelGyro")
        else:
            print("Asegurese de que ejecuta una de las dos opciones, tecleando 1 o 2 y luego pulse ENTER.")
            time.sleep(1)
        
    elif int(caso) == 4:
        c4 = 0
        num_c4 = input('\x1b[1;35m' + '\n---> Introduzca numero de pruebas a realizar:  '+ '\033[0m')
        print("Brújula activada")
        while c4 < int(num_c4):
            # Obtiene las tres componentes del campo magnetico (OX,OY,OZ)
            _,_,m9m,_ = imu.getMotion9() 
            # Con la función calculamos la brújula
            angulo = imu.brujula(m9m[0],m9m[1])
            print("  ", "{:+7.3f}".format(angulo) +" [º]")
            print("------------------------------")
            c4 +=1
            time.sleep(1)
        print("Fin de las mediciones.\n")
        time.sleep(2)
        
    elif int(caso) == 5:
        execute = False
        print("Saliendo de la ejecución...")
        time.sleep(2)
    else:
        print("\nAsegúrese de que ejecuta una de las tres acciones, tecleando 1 ó 2 ó 3 y luego pulse ENTER.")
        print("Saliendo...")
        time.sleep(1)
        print('\x1b[1;35m' + '___________________________________________________________' + '\033[0m\n')
        
