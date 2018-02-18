from flask import Flask, render_template, request, redirect
import serial # Libreria para la comunicacion con arduino

app = Flask(__name__, static_url_path='')

@app.before_first_request
def setupSerial():
	global arduino
	arduino = serial.Serial('/dev/cu.wchusbserial1420', 9600)

@app.route('/')
def datosArduino():
    print("Enviando datos a la arduino")
    return page()


@app.route("/") # La raíz
def page(name=None):
    print("Render uno")
    #return contact()
    return render_template('index.html', name=name)

@app.route('/', methods=['GET', 'POST'])
def contact(form=None): # Recoge datos del boton
	if request.method == 'POST':
		if 'encender' in request.form: # Si el boton se activa
			print("Encendido")
			arduino.write("H".encode('utf-8')) # Enviar datos a la Arduino
			return render_template('index.html', form=form)

		elif 'apagar' in request.form:
			print("Apagado")
			arduino.write("L".encode('utf-8'))
			return render_template('index.html', form=form)

		else:
			print('No pasa nada')
			return render_template('index.html', form=form)

	elif request.method == 'GET':
		return render_template('index.html', form=form)

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=80, debug=True) # Nos permite acceder fuera de la raspi
