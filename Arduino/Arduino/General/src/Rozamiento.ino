

float rozamiento_cero() // Función de rozamiento cero
{
  imu::Vector<3> acelerometro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // Obtiene la aceleración
  unsigned long t_inicio = millis(); // Obtiene el tiempo de ejecución del programa al iniciarse, este valor es 0
  unsigned long t_final; // Variable que almacenará el tiempo final para restarlo al inicial
  Serial.println(acelerometro.x()); // Solo para pruebas

  if (acelerometro.x()>AX_MIN || acelerometro.y()>AY_MIN || acelerometro.z()>AZ_MIN) // Si uno de los tres ejes sufre una aceleración
  {
    t_final = millis();
  }
  // Aquí va el tema de motores
  unsigned long aceleracion = t_final-t_inicio;
  float Fuerza_motores = aceleracion * masa;
}
