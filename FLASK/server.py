from flask import Flask, render_template, request
import serial # Libreria para la comunicacion con arduino

app = Flask(__name__, static_url_path='')

@app.before_first_request
def setupSerial():
	global arduino
	arduino = serial.Serial('/dev/cu.wchusbserial1420', 9600)

@app.route('/arduino', methods = ['POST'])
def contact():

        # if 'Do Something' in request.form:
        #     print("ARRIBA")
        #     arduino.wite('H')
        #     return render_template('index.html')
		#
        # elif 'Do Something Else' in request.form:
        #     arduino.write('L')
        #     return render_template('index.html')

@app.route('/')
def datosArduino():
    print("Enviando datos a la arduino")
    return page()


@app.route("/") # La raíz
def page(name=None):
    print("Render uno")
    #return contact()
    return render_template('index.html', name=name)

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=80, debug=True) # Nos permite acceder fuera de la raspi
