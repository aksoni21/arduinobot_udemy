//Flask Webapp -> sends goal to service_client.py -> sends goal to service_server.py -> sends the goal to arduino.ino via serial  

#include <Servo.h>

Servo motor;
Servo motor2;
Servo motor3;


void setup() {
  // put your setup code here, to run once:
  motor.attach(10);
  // motor2.attach(5);
  // motor3.attach(11);
  motor.write(90);
  // motor2.write(90);
  // motor3.write(90);

  Serial.begin(9600);
  Serial.setTimeout(1);

}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available())
  {
    int angle= Serial.readString().toInt();
    motor.write(angle);
    // motor2.write(angle);
    // motor3.write(angle);
  }
  delay(1000);
}