int VRx = A0;//2; // analog
int VRy = A1;//1; // analog
int SW  = 42; // digital

int y = 0;
int x = 0;
// digital outputs
void setup(){
Serial.begin(9600);
}

void loop() {
  y = analogRead(VRy);
  x = analogRead(VRx);

  Serial.println(x);
  Serial.println(y);
  Serial.println();
  delay(500);
}
