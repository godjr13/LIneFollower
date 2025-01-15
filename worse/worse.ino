#define enA 3
#define enB 5
#define in1 8
#define in2 9
#define in3 10
#define in4 11
#define irLeft A0
#define irRight A1
#define irRight1 A2
#define irLeft1 A3

//pid related variables
float kp= 0.3;
float kd = 0.12;
float ki = 0.00;
float speed = 100;
float integral, previousTime, prevError;
int correction, speedLeft, speedRight;
float left = analogRead(irLeft);
float right = analogRead(irRight);

float calculatePID(float er){
  float error = er;

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  previousTime = currentTime;

  integral += error * deltaTime; 
  float pid = kp * error + ki * integral + kd * (error - prevError)/deltaTime;

  prevError = error;
  return pid;
}

void setup() {
  pinMode(A0, INPUT);
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  pinMode(irLeft1, INPUT);
  pinMode(irRight1, INPUT);
}

void loop() {
  float x = analogRead(A0);
  float left1 = analogRead(irLeft1);
  float right1 = analogRead(irRight1);

  float error = left - right; 

  // Line following logic
  if(left < 100 && right < 100){ 
    if(left1 > 500) { 
      analogWrite(enA, speed); 
    } else if(left1 < 500) { 
      analogWrite(enA, 0); 
      analogWrite(enB, 0); 
    } 
    if(right1 > 500) { 
      analogWrite(enB, speed); 
    } else if(right1 < 500) { 
      analogWrite(enA, 0); 
      analogWrite(enB, 0); 
    } 
  } else { // Normal line following
    correction = calculatePID(error); 

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    speedLeft = speed + correction;
    speedRight = speed - correction;
    speedLeft = constrain(speedLeft,0,255);
    speedRight = constrain(speedRight,0,255);

    if(correction > 1000) { 
      analogWrite(enA, 0);
      analogWrite(enB, 0); 
    } else {
      analogWrite(enA, speedLeft);
      analogWrite(enB, speedRight);
    }
  }

  Serial.print("\t");
  Serial.print(correction);
  Serial.print("\t");
  Serial.print(irLeft);
  Serial.print("\t");
  Serial.print(irRight);
  Serial.print("\t");
  Serial.print(irRight1);
  Serial.print("\t");
  Serial.println(irLeft1);
}