# import serial
#
# arduino = serial.Serial('/dev/ttys000', 9600)
#
# print("Starting!")
#
# comando = input('Introduce un comando: ') #Input
# arduino.write(comando) #Mandar un comando hacia Arduino
# if comando == "sonar":
# 	print("Imprimiendo datos del sonar")
#
# elif comando == "sensor":
# 	print("Imprimiendo datos del sensor")
# txt = ''
# while arduino.inWaiting() > 0:
#       txt += arduino.read(1)
#       print txt
#       txt = ''
#
# arduino.close() #Finalizamos la comunicacion
import serial

arduino = serial.Serial('/dev/cu.usbmodem1421', 9600)

print("Starting!")

while True:
      comando = str(input('Introduce un comando: ')) #Input
      arduino.write(comando.encode()) #Mandar un comando hacia Arduino
      if comando == 'H':
            print('LED ENCENDIDO')
      elif comando == 'L':
            print('LED APAGADO')

arduino.close() #Finalizamos la comunicacion
