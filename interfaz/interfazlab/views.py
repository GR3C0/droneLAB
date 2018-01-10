from django.shortcuts import render
from django.shortcuts import get_object_or_404, render_to_response
from django.http import HttpResponseRedirect, HttpResponse
# from django.core.urlresolvers import reverse
from django.template import RequestContext
from .form import SearchForm
from threading import Thread
from pyfirmata import *
from pyfirmata import Arduino

def post_list(request):
    return render(request, 'drone/index.html', {})

def volar(request):
    return render(request, 'drone/volar.html', {}) # Devuelve el html
    # Comunicacion con la arduino
    print("Hola Mundo")
    raspi = serial.Serial('/dev/cu.usbmodem1421', '9600') # Puerto de la arduino
    print("Starting!")

    while True:
        comando = str(input('Introduce un comando: ')) #Input
        raspi.write(comando.encode()) #Mandar un comando hacia Arduino
        if comando == 'H':
            print('LED ENCENDIDO')
        elif comando == 'L':
            print('LED APAGADO')

    raspi.close() #Finalizamos la comunicacion

a = Arduino('/dev/cu.usbmodem1421')
a.pin_mode(13, pyfirmata.OUTPUT)
def dot():
    a.digital_write(13, pyfirmata.HIGH)
    a.delay(0.5)
    a.digital_write(13, pyfirmata.LOW)
    a.delay(0.5)
def slash():
    a.digital_write(13, pyfirmata.HIGH)
    a.delay(1.5)
    a.digital_write(13, pyfirmata.LOW)
    a.delay(0.5)

class ArduinoMorse (Thread):
    def __init__(self, text):
        Thread.__init__(self)
        self.text=text
    def run(self):
            for x in self.text :
                if x=='a' or x=='A':
                    dot()
                    slash()
                    spaceL()
                elif x=='z' or x=='Z':
                    print('Hola Mundo')

def encode(request):
    if request.method == 'GET':
        form = SearchForm(request.GET)
        if form.is_valid():
            text = form.cleaned_data['text']
            arduino=ArduinoMorse(text)
            arduino.start()
        else:
            queryset = []
    form = SearchForm()
    return render_to_response("drone/volar.html", {'form': form})
