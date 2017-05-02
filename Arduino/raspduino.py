import serial

arduino = serial.Serial('/dev/ttys000', 9600)

print("Starting!")


comando = input('Introduce un comando: ') #Input
arduino.write(comando) #Mandar un comando hacia Arduino
if comando == "encender motores":
	print("Encendiendo motores")

elif comando == "apagar motores":
	print("Apagando motores")

arduino.close() #Finalizamos la comunicacion