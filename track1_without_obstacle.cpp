// IR Sensor pins
const int leftSensor = A5;
const int middleSensor = 4;
const int rightSensor = A4;

// Motor driver pins
const int enAPin = 10;
const int in1Pin = 7;
const int in2Pin = 8;
const int in3Pin = 9;
const int in4Pin = 2;
const int enBPin = 3;

// Speed settings
const int baseSpeed = 180;
const int turnSpeed = 140;
const int curveAdjust = 40;
const int stopSpeed = 0;

// Thresholds
int thresholdLeft = 300;
int thresholdRight = 300;

// Line recovery for dashed lines
unsigned long lastLineTime = 0;
const unsigned long lineTimeout = 600;  // Continue forward for 600ms when line lost
int lastDirection = 0;  // -1 = left, 0 = center, 1 = right

// Dead end detection
unsigned long allBlackStartTime = 0;
const unsigned long deadEndDelay = 150;

// Intersection handling
bool intersectionDetected = false;
unsigned long intersectionTime = 0;
const unsigned long intersectionDelay = 300;

void setup() {
  Serial.begin(9600);
  
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enAPin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  
  pinMode(leftSensor, INPUT);
  pinMode(middleSensor, INPUT);
  pinMode(rightSensor, INPUT);
  
  Serial.println("=== Line Following Robot - Track 1 ===");
  Serial.println("Handles: Curves, Dashed Lines, Intersections, Dead Ends");
  delay(2000);
}

void setMotor(int leftSpeed, int rightSpeed) {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  analogWrite(enAPin, leftSpeed);
  analogWrite(enBPin, rightSpeed);
}

void stopRobot() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
  analogWrite(enAPin, stopSpeed);
  analogWrite(enBPin, stopSpeed);
}

void turnAround() {
  Serial.println(">>> DEAD END - Turning 180°!");
  
  // Back up
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  analogWrite(enAPin, 150);
  analogWrite(enBPin, 150);
  delay(300);
  
  // Spin 180 degrees
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  analogWrite(enAPin, 180);
  analogWrite(enBPin, 180);
  delay(800);
  
  lastDirection = 0;
}

void spinLeft() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  analogWrite(enAPin, 150);
  analogWrite(enBPin, 150);
}

void spinRight() {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  analogWrite(enAPin, 150);
  analogWrite(enBPin, 150);
}

void loop() {
  // Read sensors
  int leftValue = analogRead(leftSensor);
  int rightValue = analogRead(rightSensor);
  int middleValue = digitalRead(middleSensor);
  
  Serial.print("L:");
  Serial.print(leftValue);
  Serial.print(" M:");
  Serial.print(middleValue);
  Serial.print(" R:");
  Serial.print(rightValue);
  Serial.print(" | ");
  
  // Determine line detection
  bool leftOnLine = leftValue > thresholdLeft;
  bool rightOnLine = rightValue > thresholdRight;
  bool middleOnLine = (middleValue == HIGH);
  
  bool anyLineDetected = leftOnLine || middleOnLine || rightOnLine;
  
  if (anyLineDetected) {
    lastLineTime = millis();
  }
  
  // === DEAD END DETECTION ===
  // All three sensors see black = dead end
  if (leftOnLine && middleOnLine && rightOnLine) {
    if (allBlackStartTime == 0) {
      allBlackStartTime = millis();
    }
    
    if (millis() - allBlackStartTime > deadEndDelay) {
      turnAround();
      allBlackStartTime = 0;
      delay(100);
      return;
    } else {
      // Keep moving to confirm dead end
      setMotor(baseSpeed - 50, baseSpeed - 50);
      Serial.println("Checking dead end...");
      return;
    }
  } else {
    allBlackStartTime = 0;
  }
  
  // === INTERSECTION HANDLING ===
  // Left AND right see line (but not all three) = T or + intersection
  if (leftOnLine && rightOnLine && !middleOnLine) {
    if (!intersectionDetected) {
      intersectionDetected = true;
      intersectionTime = millis();
      Serial.println(">>> INTERSECTION - Going Forward");
    }
    
    if (millis() - intersectionTime < intersectionDelay) {
      setMotor(baseSpeed, baseSpeed);
      return;
    }
  } else {
    intersectionDetected = false;
  }
  
  // === SMOOTH LINE FOLLOWING ===
  if (middleOnLine && !leftOnLine && !rightOnLine) {
    // Perfect - straight forward
    setMotor(baseSpeed, baseSpeed);
    lastDirection = 0;
    Serial.println("Forward");
  } 
  else if (leftOnLine && middleOnLine && !rightOnLine) {
    // Gentle curve right
    setMotor(baseSpeed + curveAdjust, baseSpeed - curveAdjust);
    lastDirection = -1;
    Serial.println("Curve Right");
  }
  else if (rightOnLine && middleOnLine && !leftOnLine) {
    // Gentle curve left
    setMotor(baseSpeed - curveAdjust, baseSpeed + curveAdjust);
    lastDirection = 1;
    Serial.println("Curve Left");
  }
  else if (leftOnLine && !middleOnLine && !rightOnLine) {
    // Sharp right turn
    setMotor(turnSpeed + 40, turnSpeed - 40);
    lastDirection = -1;
    Serial.println("Sharp Right");
  } 
  else if (rightOnLine && !middleOnLine && !leftOnLine) {
    // Sharp left turn
    setMotor(turnSpeed - 40, turnSpeed + 40);
    lastDirection = 1;
    Serial.println("Sharp Left");
  }
  // === LINE LOST - DASHED LINE HANDLING ===
  else if (!anyLineDetected) {
    if (millis() - lastLineTime < lineTimeout) {
      // Continue forward to bridge gaps
      setMotor(baseSpeed, baseSpeed);
      Serial.println("Bridging gap...");
    } else {
      // Completely lost - recover based on last direction
      if (lastDirection == -1) {
        spinRight();
        Serial.println("Recovery: Spin Right");
      } else if (lastDirection == 1) {
        spinLeft();
        Serial.println("Recovery: Spin Left");
      } else {
        spinRight();
        Serial.println("Recovery: Spin Right (default)");
      }
    }
  }
  
  delay(10);
} 