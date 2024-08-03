volatile long temp, counter = 0;

double previousError = 0.0;
double integral = 0.0;

// Define the pins
const int encoderPin_A = 2;
const int encoderPin_B = 3;

const int MotorE_ENA = 10;  // Enable pin for motor E
const int MotorE_IN1 = 9;   // Input pin 1 for motor E
const int MotorE_IN2 = 8;   // Input pin 2 for motor E

const int MotorB_ENA = 5;  // Enable pin for motor B
const int MotorB_IN1 = 6;  // Input pin 1 for motor B
const int MotorB_IN2 = 7;  // Input pin 2 for motor B

double setpoint = 400;  // The desired motor speed or position
double kp = 1.0;          // Proportional gain
double ki = 0.1;          // Integral gain
double kd = 0.5;          // Derivative gain

int motorSpeed = 200;


void setup() {
  // Set the pinMode for the pins
  pinMode(MotorE_ENA, OUTPUT);
  pinMode(MotorE_IN1, OUTPUT);
  pinMode(MotorE_IN2, OUTPUT);
  pinMode(MotorB_ENA, OUTPUT);
  pinMode(MotorB_IN1, OUTPUT);
  pinMode(MotorB_IN2, OUTPUT);
  Serial.begin(9600);

  pinMode(encoderPin_A, INPUT_PULLUP);  // internal pullup input pin 2

  pinMode(encoderPin_B, INPUT_PULLUP);  // internalเป็น pullup input pin 3

  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(encoderPin_A), ai0, RISING);

  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(encoderPin_B), ai1, RISING);
}

void loop() {
  double currentValue = counter;
  double motorSpeed = calculatePID(counter);
  MotorE_moveUp(motorSpeed);
  Serial.println(counter);
}


void stop() {
  analogWrite(MotorE_ENA, 0);
  digitalWrite(MotorE_IN1, LOW);
  digitalWrite(MotorE_IN2, LOW);
}

void MotorE_moveUp(int motorSpeed) {
  if (motorSpeed >= 0) {
    digitalWrite(MotorE_IN1, HIGH);
    digitalWrite(MotorE_IN2, LOW);
  } else {
    digitalWrite(MotorE_IN1, HIGH);
    digitalWrite(MotorE_IN2, LOW);
  }
  
  digitalWrite(MotorE_ENA, abs(motorSpeed));
  delay(10);
  // Serial.println(counter);

  stop();
}

// void MotorE_moveDown(int t) {
//   while (counter > stopCounterDown) {
//     digitalWrite(MotorE_ENA, motorSpeed);
//     digitalWrite(MotorE_IN1, LOW);
//     digitalWrite(MotorE_IN2, HIGH);
//     delay(10);
//     Serial.println(counter);
//   }
//   stop();
// }

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(encoderPin_B) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(encoderPin_A) == LOW) {
    counter--;
  } else {
    counter++;
  }
}



double calculatePID(double counter) {
  double error = setpoint - counter;
  integral += error;
  double derivative = error - previousError;
  double output = (kp * error) + (ki * integral) + (kd * derivative);
  previousError = error;
  return output;
}