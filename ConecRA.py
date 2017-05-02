# Interfaz de usuario para usar Arduino y Raspi
import serial
# Conexion con la Arduino, pendiente cambiar el puerto
arduino = serial.Serial('/dev/tty', 9600)

print("Conectando")

while True:
	usuario = input("Introduce un comando: ")
	arduino.write(usuario) # Enviar datos a la Arduino

	if usuario == "Desconectar" or "desconectar":
		arduino.close() # Finalizamos la comunicación

