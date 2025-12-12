int VRx = A0;//2; // analog
int VRy = A1;//1; // analog
int SW  = 42; // digital

// digital outputs
int in1 = 3; // LF 
int in2 = 4; // LB
int enableA = 2;

int in3 = 5; // RF
int in4 = 6; // RB
int enableB = 7;

int rMax = 1023;
int rMin = 0;
int center = (rMax/2);
int deadBand = 100;

 // 0 - 1890 - 4096
int x = 0; 
int y = 0; 

void setup(){
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enableA, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enableB, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
  x = analogRead(A0);
  y = analogRead(A1);

  if(x < 10)
    analogWrite(enableA, 0);
  if(y < 10)
    analogWrite(enableB, 0);

// fowards
  if(y > center + deadBand){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enableA, map(y, center + deadBand, rMax, 0, 255));
    analogWrite(enableB, map(y, center + deadBand, rMax, 0, 255));
  }
  // backwards
  else if(y < center - deadBand){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enableA, map(y, rMin, center - deadBand, 255, 0));
    analogWrite(enableB, map(y, rMin, center - deadBand, 255, 0));
  }
  // left
  else if(x > center + deadBand){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(enableA, map(x, center + deadBand, rMax,0, 255));
    analogWrite(enableB, map(x, center + deadBand, rMax, 0, 255));
  }
  // right
  else if(x < center - deadBand){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enableA, map(x, rMin, center - deadBand, 255, 0));
    analogWrite(enableB, map(x, rMin, center - deadBand, 255, 0));
  }else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enableB, 0);
    analogWrite(enableA, 0);
  }
}
