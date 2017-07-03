void setup() {
  Serial.begin(9600);

}

void loop() {
  // read from port 0, send to port 1:
  if (Serial.available() > 0)
  {
        // read the incoming byte:
        int c = Serial.read();

        if (c == 107)
        {
          Serial.println("He recibido h");
        }
        // say what you got:
        // Serial.print("I received: ");
        // Serial.println(incomingByte, DEC);
  }
}
