#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

int VRx = 2; // analog
int VRy = 1; // analog
int SW  = 42; // digital

// digital outputs
int in1 = 36; // LF 
int in2 = 35; // LB
int enableA = 37;

// int in3 = 17; // RF
// int in4 = 18; // RB
// int enableB = 8;
int in3 = 21; // RF
int in4 = 47; // RB
int enableB = 48;

// int rMax = 4095;
int rMax = 1024;
int rMin = 0;
int center = (rMax/2);
int deadBand = 200;

 // 0 - 1890 - 4096
int x = 0; 
int y = 0; 

void RGB_LED(){
    #define RGB_PIN 48        // ESP32-S3-DevKitC onboard RGB
    #define NUM_PIXELS 1
    Adafruit_NeoPixel pixel(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);
    pixel.begin();
    pixel.setBrightness(50);
    pixel.setPixelColor(0, pixel.Color(0, 50, 0)); // Green
    pixel.show(); // turn off
}

void setup(){
    RGB_LED();
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
