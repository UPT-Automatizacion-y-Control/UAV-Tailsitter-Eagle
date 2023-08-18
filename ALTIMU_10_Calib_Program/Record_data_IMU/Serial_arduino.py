import serial
import os
import pandas
import math

MAX_MEAS = 200  # max number of readings in the session, so that we don't create an infinite loop
AVG_MEAS = 25 
FILENAME = os.path.join(os.getcwd(), 'acceldata.txt')  # output file
# Configuración del puerto serial
puerto = '/dev/ttyUSB0' # Reemplazar con el puerto serial correspondiente
baud_rate = 115200

# Crear objeto Serial
ser = serial.Serial(puerto, baud_rate)

while True:
    # Leer datos del puerto serial
    datos = ser.readline()
    
    # Decodificar los datos recibidos
    datos_decodificados = datos.decode('utf-8').rstrip()
    
    # Imprimir los datos recibidos
    print(datos_decodificados)

    for _ in range(25):
        user = input(
            '[INPUT]: Ready for measurement? Type \'m\' to measure or \'q\' to save and quit: ').lower()
        if user == 'm':
            # record data to list
            ax, ay, az = RecordDataPt(ser)
            magn = math.sqrt(ax**2 + ay**2 + az**2)
            print('[INFO]: Avgd Readings: {:.4f}, {:.4f}, {:.4f} Magnitude: {:.4f}'.format(
                ax, ay, az, magn))
            data.append([ax, ay, az])
        elif user == 'q':
            # save, then quit
            print('[INFO]: Saving data and exiting...')
            List2DelimFile(data, FILENAME, delimiter='\t')
            ser.Close()
            print('[INFO]: Done!')
            return
        else:
            print('[ERROR]: \'{}\' is an unknown input. Terminating!'.format(user))
            List2DelimFile(data, FILENAME, delimiter='\t')
            ser.Close()
            return
