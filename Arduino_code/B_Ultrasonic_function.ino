#include <Servo.h>

// Define pin connections
const int trigPin = A1;    // Ultrasonic sensor trigger pin
const int echoPin = A2;    // Ultrasonic sensor echo pin
const int servoPin = 10;    // Servo motor pin
int ENA = 6;  // PWM pin to control speed of Motor A
int IN1 = 2;  // Motor A IN1
int IN2 = 3;  // Motor A IN2
int ENB = 10; // PWM pin to control speed of Motor B
int IN3 = 4;  // Motor B IN3
int IN4 = 5;  // Motor B IN4

// Distance threshold for detecting obstacles
const int distanceThreshold = 20; // Distance in cm to trigger obstacle avoidance

Servo myServo; // Create servo object

void setup() {
  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
//  
//  // Attach the servo to the specified pin
  myServo.attach(servoPin);
  myServo.write(90);

  // Set initial speed for both motors
  analogWrite(ENA, 180); // Speed for Motor A (0-255)
  analogWrite(ENB, 150); // Speed for Motor B (0-255)
  
  // Begin serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  int distance = getDistance();  // Get the distance from the ultrasonic sensor
  
  // Print the distance to the Serial Monitor
  Serial.print("Current Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > distanceThreshold) {
    // No obstacle detected, move forward
    GoForward();
  } else {
    // Obstacle detected, stop and change direction
    avoidObstacle();
  }

  delay(300);  // Small delay for sensor stability
//GoForward();
}

// Function to get distance from the ultrasonic sensor
int getDistance() {
  // Send a 10us pulse to the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the pulse duration on the echo pin
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in cm
  int distance = duration * 0.034 / 2;

  // Return the calculated distance
  return distance;
}

// Function to avoid obstacles
void avoidObstacle() {
  Stop();   // Stop the car
  myServo.write(20);   // Look to the right
  delay(1000);
  int distanceRight = getDistance();
  
  myServo.write(160);    // Look to the left
  delay(1000);
  int distanceLeft = getDistance();
  
  // Compare distances and turn in the direction with more space
  if (distanceLeft >= distanceRight) {
    GoLeft();  // Turn left if left has more space
  } else {
    GoRight(); // Turn right if right has more space
  }
  
  myServo.write(90); // Reset the servo to the front
}

// Placeholder movement functions (implement these as needed)
void GoForward() {
  // Your code to move the car forward
  Serial.println("Go forward");
  analogWrite(ENA, 180); // Speed for Motor A (0-255)
  analogWrite(ENB, 150); // Speed for Motor B (0-255) 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void GoLeft() {
  Serial.println("Go left");
  analogWrite(ENA, 180); // Set speed for Motor A
  analogWrite(ENB, 180); // Set speed for Motor B

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); // Motor A backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);  // Motor B forward
}


void GoRight() {
  Serial.println("Go right");
  // Your code to turn the car right
  analogWrite(ENA, 100);
  analogWrite(ENB, 200); 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Stop() {
  Serial.println("Stop");
  // Your code to stop the car
  analogWrite(ENA, 0); // Stop Motor A
  analogWrite(ENB, 0); // Stop Motor B
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
