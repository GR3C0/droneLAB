import serial

arduino = serial.Serial('/dev/ttys000', 9600)

print("Starting!")


comando = input('Introduce un comando: ') #Input
arduino.write(comando) #Mandar un comando hacia Arduino
if comando == "sonar":
	print("Imprimiendo datos del sonar")

elif comando == "sensor":
	print("Imprimiendo datos del sensor")
txt = ''
while arduino.inWaiting() > 0:
      txt += arduino.read(1)
      print txt
      txt = ''

arduino.close() #Finalizamos la comunicacion