from django.shortcuts import render
import serial

def post_list(request):
    return render(request, 'drone/index.html', {})

def volar(request):
    return render(request, 'drone/volar.html', {}) # Devuelve el html

def raspduino(request):
    # Comunicación con la arduino
    print("Hola Mundo")
    raspi = serial.Serial('/dev/cu.wchusbserial1420', '9600') # Puerto de la arduino
    print("Starting!")

    while True:
        comando = str(input('Introduce un comando: ')) #Input
        raspi.write(comando.encode()) #Mandar un comando hacia Arduino
        if comando == 'H':
            print('LED ENCENDIDO')
        elif comando == 'L':
            print('LED APAGADO')

    raspi.close() #Finalizamos la comunicacion
