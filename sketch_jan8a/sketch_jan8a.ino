#define enA 3
#define enB 5
#define in1 8
#define in2 9
#define in3 10
#define in4 11
#define irLeft A0
#define irRight A1

//pid related variables
float kp= 1.5;
float kd = 0.2;
float ki = 0;
float speed = 127;
float integral, previousTime, prevError;
int correction, speedLeft, speedRight;

float getError(){
  float error;
  float left = analogRead(irLeft);
  float right = analogRead(irRight);

  Serial.print(left);
  Serial.print("\t");
  Serial.print(right);
  
  if(left < 900){
    error = left - right;
  }else if(right < 900){
    error = left - right;
  }else if(left < 900 && right < 900){
    Serial.print("No line found.");
    error = -9999;
  }else {
    error = 0;
  }

  return error;
}

float calculatePID(){
  float error = getError();

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  previousTime = currentTime;

  integral += error * deltaTime; 
  float pid = kp * error + ki * integral + kd * (error - prevError)/deltaTime;

  prevError = error;
  return pid;
}


void setup() {
  // put your setup code here, to run once:
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

}

void loop() {
  // put your main code here, to run repeatedly:
  float x = analogRead(A0);

  correction = calculatePID();

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  speedLeft = speed - correction;
  speedRight = speed + correction;
  speedLeft = constrain(speedLeft,0,255);
  speedRight = constrain(speedRight,0,255);

  analogWrite(enA, speedLeft);
  analogWrite(enB, speedRight);
  Serial.print("\t");
  Serial.print(correction);
  Serial.print("\t");
  //Serial.print("\tSpeed Left: ");
  Serial.print(irLeft);
  Serial.print("\t");
  //Serial.print("\tSpeed Right: ");
  Serial.println(irRight);
  Serial.print("\t");
}


