int VRx = 2; // analog
int VRy = 1; // analog
int SW  = 42; // digital

// digital outputs
int LF = 13;
int LB = 14;
int RF = 20;
int RB = 19;
// digital outputs
void setup(){
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RB, OUTPUT);
}

void loop() {
  digitalWrite(LF, LOW);
  delay(100);
  digitalWrite(LF, HIGH);
  delay(100);
}
