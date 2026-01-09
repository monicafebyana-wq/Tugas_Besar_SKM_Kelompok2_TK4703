#include <Arduino.h>
#include <KY040.h>
#include <PID_v1.h>

#define CLK_PIN 3
#define DT_PIN 2
KY040 g_rotaryEncoder(CLK_PIN, DT_PIN);

volatile int v_value = 0;

boolean bool_CW;
int ENA = 6;
int IN1 = 4;
int IN2 = 5;
boolean run;

double sp, input, output;

double Kp = 35;
double Ki = 1;
double Kd = 2;
PID myPID(&input, &output, &sp, Kp, Ki, Kd, DIRECT);


int outputPWM;
float t;

void ISR_rotaryEncoder(){
  switch (g_rotaryEncoder.getRotation()){
    case KY040::CLOCKWISE:
      v_value++;
      break;
    case KY040::COUNTERCLOCKWISE:
      v_value--;
      break;
  }
}

void setup() {
  Serial.begin (9600);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), ISR_rotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), ISR_rotaryEncoder, CHANGE);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(50);
  myPID.SetMode(AUTOMATIC);
  sp = 0;

  delay(5000);
}

void loop() {
  static int lastValue = 0;
  int value;
  char data;

  t = millis()/1000.0;

  // cli();
  value = v_value;
  input = value;
  // sei();

  if(Serial.available() > 0){
    data = Serial.read();

    if(data == 'o'){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      sp += 36;
    } else if(data == 'c'){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      sp -= 36;
    }
    run = true;
  }

  myPID.Compute();

  if(lastValue != value){
    lastValue = value;
  }

  Serial.print(t);
  Serial.print("\t");
  Serial.print(sp);
  Serial.print("\t");
  Serial.print(input);
  Serial.print("\t");
  Serial.println(output);

  if(output > 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, output);
  } else if(output < 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(output));
  }
}