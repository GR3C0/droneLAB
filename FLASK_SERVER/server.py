from flask import Flask, render_template, request, redirect
import serial  # Libreria para la comunicacion con arduino

app = Flask(__name__, static_url_path='')


@app.before_first_request
def setupSerial():
    global arduino
    arduino = serial.Serial('/dev/cu.wchusbserial1420', 9600)


@app.route('/', methods=['GET', 'POST'])  # La raíz
def contact(form=None):  # Recoge datos del boton
    if request.method == 'POST':
        if 'encender' in request.form:  # Si el boton se activa
            print("Encendido")
            arduino.write("0".encode('utf-8'))  # Enviar datos a la Arduin
            return render_template('index.html', form=form, name="Encendido")

        elif 'apagar' in request.form:
            print("Apagado")
            arduino.write("1".encode('utf-8'))
            return render_template('index.html', form=form, name="Apagado")

        else:
            print('No pasa nada')
            return render_template('index.html', form=form)

    elif request.method == 'GET':
        return render_template('index.html', form=form)


if __name__ == "__main__":
    # Nos permite acceder fuera de la raspi
    app.run(host='0.0.0.0', port=80, debug=True)
