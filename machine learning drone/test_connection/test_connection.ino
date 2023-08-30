int x;
int Kp_int, Kd_int, Ki_int;
float Kp, Kd, Ki;

unsigned long timer;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
}

void loop() {
  
  while (!Serial.available());
  timer = millis();
  String receivedData = Serial.readString(); // Read String
      // Find semicolon
    int firstSemicolon = receivedData.indexOf(';');
    int secondSemicolon = receivedData.indexOf(';', firstSemicolon + 1);
    
    if (firstSemicolon != -1 && secondSemicolon != -1) {
      String strValue1 = receivedData.substring(0, firstSemicolon);
      String strValue2 = receivedData.substring(firstSemicolon + 1, secondSemicolon);
      String strValue3 = receivedData.substring(secondSemicolon + 1);
      
      Kp_int = strValue1.toInt();
      Kd_int = strValue2.toInt();
      Ki_int= strValue3.toInt();

      Kp = float(Kp_int)/100;
      Kd = float(Kd_int)/100;
      Ki = float(Ki_int)/100;
    }

  while(millis() - timer < 1000);
  Serial.print(Kp+Kd+Ki);
  Serial.flush();
}
