// Pin Definitions
int ENA = 9;  // PWM pin to control speed of Motor A
int IN1 = 2;  // Motor A IN1
int IN2 = 3;  // Motor A IN2
int ENB = 10; // PWM pin to control speed of Motor B
int IN3 = 4;  // Motor B IN3
int IN4 = 5;  // Motor B IN4
int servoPin = 6;


char command;
void setup() {
  Serial.begin(9600);
  // Set all the motor control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Set initial speed for both motors
  analogWrite(ENA, 150); // Speed for Motor A (0-255)
  analogWrite(ENB, 100); // Speed for Motor B (0-255)
  

  

}

void loop() {
  if (Serial.available()){
    command = Serial.read();
    switch (command){
      case 'f':
        GoForward();
        break;
      case 's':
        Stop();
        break;
      case 'l':
        GoLeft();
        break;
      case 'r':
        GoRight();
        break;
      case 'b':
        TurnAround();
        break;
    }
  }
}

void Stop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
}
void GoForward(){
  analogWrite(ENA, 180); // Speed for Motor A (0-255)
  analogWrite(ENB, 100); // Speed for Motor B (0-255) 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void GoBackward(){
//  analogWrite(ENA, 180); // Speed for Motor A (0-255)
//  analogWrite(ENB, 100); // Speed for Motor B (0-255)
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);  
}

void GoLeft(){
//  analogWrite(ENA, 250);
//  analogWrite(ENB, 100); 

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  
}
  
void GoRight(){
 analogWrite(ENA, 100);
 analogWrite(ENB, 200);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void TurnAround(){
//  analogWrite(ENA, 180);
//  analogWrite(ENB, 100);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(4000); // Time to make a 180-degree turn
}
