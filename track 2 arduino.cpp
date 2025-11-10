const int leftSensor = A5;
const int middleSensor = 4;
const int rightSensor = A4;

const int trigPin = 5;
const int echoPin = 6;

const int enAPin = 10;
const int in1Pin = 7;
const int in2Pin = 8;
const int in3Pin = 9;
const int in4Pin = 2;
const int enBPin = 3;

const int baseSpeed = 180;
const int turnSpeed = 140;
const int curveAdjust = 40;
const int avoidSpeed = 160;  // Speed during obstacle avoidance

int thresholdLeft = 300;
int thresholdRight = 300;

const int obstacleDistance = 200;  // Detect obstacles at 20cm
const int safeDistance = 250;      // Safe distance after avoiding

unsigned long lastLineTime = 0;
const unsigned long lineTimeout = 600;
int lastDirection = 0;

unsigned long allBlackStartTime = 0;
const unsigned long deadEndDelay = 150;

bool intersectionDetected = false;
unsigned long intersectionTime = 0;
const unsigned long intersectionDelay = 300;

enum RobotState {
  FOLLOWING_LINE,
  AVOIDING_OBSTACLE,
  SEARCHING_LINE
};
RobotState currentState = FOLLOWING_LINE;
unsigned long avoidStartTime = 0;
int avoidStep = 0;

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
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  
  Serial.println("=== Line Following + Obstacle Avoidance - Track 2 ===");
  Serial.println("Features: Line following + Ultrasonic obstacle detection");
  delay(2000);
}

unsigned int readDistance() {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long period = pulseIn(echoPin, HIGH, 30000);  
  if (period == 0) return 9999;  
  
  return period * 343 / 2000;  
}

void setMotor(int leftSpeed, int rightSpeed) {
  // Forward direction
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  analogWrite(enAPin, leftSpeed);
  analogWrite(enBPin, rightSpeed);
}

void setMotorBackward(int leftSpeed, int rightSpeed) {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  
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
  analogWrite(enAPin, 0);
  analogWrite(enBPin, 0);
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

void turnAround() {
  Serial.println(">>> DEAD END - Turning 180°!");
  setMotorBackward(150, 150);
  delay(300);
  spinRight();
  delay(800);
  lastDirection = 0;
}

void avoidObstacle() {
  unsigned int distance = readDistance();
  
  switch(avoidStep) {
    case 0:  
      Serial.println("AVOID Step 1: Turn right");
      spinRight();
      delay(500);  
      avoidStep = 1;
      avoidStartTime = millis();
      break;
      
    case 1:  
      Serial.println("AVOID Step 2: Forward past obstacle");
      setMotor(avoidSpeed, avoidSpeed);
      if (millis() - avoidStartTime > 800) {  
        avoidStep = 2;
        avoidStartTime = millis();
      }
      break;
      
    case 2:  
      Serial.println("AVOID Step 3: Turn left");
      spinLeft();
      delay(500);  
      avoidStep = 3;
      avoidStartTime = millis();
      break;
      
    case 3:  
      Serial.println("AVOID Step 4: Forward alongside");
      setMotor(avoidSpeed, avoidSpeed);
      if (millis() - avoidStartTime > 1000) {  
        avoidStep = 4;
        avoidStartTime = millis();
      }
      break;
      
    case 4: 
      Serial.println("AVOID Step 5: Turn left toward line");
      spinLeft();
      delay(500);  
      avoidStep = 5;
      currentState = SEARCHING_LINE;
      break;
  }
}

void loop() {
  int leftValue = analogRead(leftSensor);
  int rightValue = analogRead(rightSensor);
  int middleValue = digitalRead(middleSensor);
  unsigned int distance = readDistance();
  
  Serial.print("Dist:");
  Serial.print(distance);
  Serial.print("mm L:");
  Serial.print(leftValue);
  Serial.print(" M:");
  Serial.print(middleValue);
  Serial.print(" R:");
  Serial.print(rightValue);
  Serial.print(" | State:");
  Serial.print(currentState);
  Serial.print(" | ");
  
  bool leftOnLine = leftValue > thresholdLeft;
  bool rightOnLine = rightValue > thresholdRight;
  bool middleOnLine = (middleValue == HIGH);
  bool anyLineDetected = leftOnLine || middleOnLine || rightOnLine;
  
  if (anyLineDetected) {
    lastLineTime = millis();
  }
  
  
  if (currentState == FOLLOWING_LINE && distance < obstacleDistance) {
    Serial.println("\n>>> OBSTACLE DETECTED! Starting avoidance...");
    currentState = AVOIDING_OBSTACLE;
    avoidStep = 0;
    return;
  }
  
  if (currentState == AVOIDING_OBSTACLE) {
    avoidObstacle();
    return;
  }
  
  if (currentState == SEARCHING_LINE) {
    if (anyLineDetected) {
      Serial.println("\n>>> LINE FOUND! Resuming line following");
      currentState = FOLLOWING_LINE;
      avoidStep = 0;
    } else {
      setMotor(120, 140);  
      Serial.println("Searching for line...");
      return;
    }
  }
  
  if (currentState == FOLLOWING_LINE) {
    
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
        setMotor(baseSpeed - 50, baseSpeed - 50);
        Serial.println("Checking dead end...");
        return;
      }
    } else {
      allBlackStartTime = 0;
    }
    
    if (leftOnLine && rightOnLine && !middleOnLine) {
      if (!intersectionDetected) {
        intersectionDetected = true;
        intersectionTime = millis();
        Serial.println(">>> INTERSECTION");
      }
      
      if (millis() - intersectionTime < intersectionDelay) {
        setMotor(baseSpeed, baseSpeed);
        return;
      }
    } else {
      intersectionDetected = false;
    }
    
    if (middleOnLine && !leftOnLine && !rightOnLine) {
      setMotor(baseSpeed, baseSpeed);
      lastDirection = 0;
      Serial.println("Forward");
    } 
    else if (leftOnLine && middleOnLine && !rightOnLine) {
      setMotor(baseSpeed + curveAdjust, baseSpeed - curveAdjust);
      lastDirection = -1;
      Serial.println("Curve Right");
    }
    else if (rightOnLine && middleOnLine && !leftOnLine) {
      setMotor(baseSpeed - curveAdjust, baseSpeed + curveAdjust);
      lastDirection = 1;
      Serial.println("Curve Left");
    }
    else if (leftOnLine && !middleOnLine && !rightOnLine) {
      setMotor(turnSpeed + 40, turnSpeed - 40);
      lastDirection = -1;
      Serial.println("Sharp Right");
    } 
    else if (rightOnLine && !middleOnLine && !leftOnLine) {
      setMotor(turnSpeed - 40, turnSpeed + 40);
      lastDirection = 1;
      Serial.println("Sharp Left");
    }
    else if (!anyLineDetected) {
      if (millis() - lastLineTime < lineTimeout) {
        setMotor(baseSpeed, baseSpeed);
        Serial.println("Bridging gap...");
      } else {
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
  }
  
  delay(20);  
}