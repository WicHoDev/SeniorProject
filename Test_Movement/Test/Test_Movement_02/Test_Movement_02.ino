int VRx = 2; // analog
int VRy = 1; // analog
int SW  = 42; // digital

// digital outputs
int LF = 13;
int LB = 14;
int RF = 20;
int RB = 19;

 // 0 - 1890 - 4096
int x = 0; 
int y = 0; 

void setup(){
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RB, OUTPUT);
}

void loop() {
  x = analogRead(VRx);
  y = analogRead(VRy);

  // fowards
  if(y >= 2200 && y < 4095){
    //analogWrite(RF, map(y, 2200, 4096, 0, 255));
    //analogWrite(LF, map(y, 2200, 4096, 0, 255));
    digitalWrite(LF, HIGH);
    digitalWrite(RF, HIGH);
    delay(1000);
  }
  // backwards
  else if(y >= 0 && y < 1800){
    //analogWrite(LB, map(y, 0, 1800, 255, 0));
    //analogWrite(RB, map(y, 0, 1800, 255, 0));
    digitalWrite(LB, HIGH)
    digitalWrite(RB, HIGH)
    delay(1000);
  }
  // left
  else if(x >= 2200 && x < 4096){
    // analogWrite(RF, map(x, 2200, 4096, 0, 255));
    digitalWrite(RFa], HIGH);
    delay(1000);
  }
  // right
  else if(x >= 0 && x < 1800){
    // analogWrite(LF, map(x, 0, 1800, 255, 0));
    digitalWrite(LF, HIGH);
    delay(1000);
  }
  // off
    /*analogWrite(LF, 0);
    analogWrite(LB, 0);
    analogWrite(RF, 0);
    analogWrite(RB, 0);*/
  digitalWrite(LF, LOW);
  digitalWrite(LB, LOW);
  digitalWrite(RF, LOW);
  digitalWrite(RB, LOW);
}