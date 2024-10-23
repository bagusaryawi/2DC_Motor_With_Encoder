// Dua Motor Dengan PID
// Motor direction
#define CW  0
#define CCW 1

// Motor 1 control pins
const int motor1Pin1 = 13;  // Pin motor 1 forward
const int motor1Pin2 = 14;  // Pin motor 1 backward
const int pwm1Pin = 26;     // Pin PWM motor 1

// Motor 2 control pins
const int motor2Pin1 = 12;  // Pin motor 2 forward
const int motor2Pin2 = 27;  // Pin motor 2 backward
const int pwm2Pin = 25;     // Pin PWM motor 2

// Encoder 1 pins
const int encoder1PinA = 4; // Pin encoder 1 A
const int encoder1PinB = 5; // Pin encoder 1 B

// Encoder 2 pins
const int encoder2PinA = 19; // Pin encoder 2 A
const int encoder2PinB = 21; // Pin encoder 2 B

// Encoder 1 variables
volatile int encoder1Pos = 0; // variable for interrupt
int encoder1PosRead = 0;
int prevEncoder1PosRead = 0;
int deltaPos1;
float speedMotor1;

// Encoder 2 variables
volatile int encoder2Pos = 0; // variable for interrupt
int encoder2PosRead = 0;
int prevEncoder2PosRead = 0;
int deltaPos2;
float speedMotor2;

// PID control constants (can be the same or different for each motor)
float Kp = 0.002;
float Ki = 15;
float Kd = 0.05;

// PID calculation variables for motor 1
int targetSpeed1 = 100; // Desired speed for motor 1
int error1;
int prevError1;
float integral1;
float derivative1;
float control1;

// PID calculation variables for motor 2
int targetSpeed2 = 100; // Desired speed for motor 2
int error2;
int prevError2;
float integral2;
float derivative2;
float control2;

// Motor 1 time variables
float dt1 = 0.060; // in seconds
int dtMillis1 = dt1 * 1000;
long int currTime1;
long int prevTime1;
long int delTime1;

// Motor 2 time variables
float dt2 = 0.060; // in seconds
int dtMillis2 = dt2 * 1000;
long int currTime2;
long int prevTime2;
long int delTime2;

// Motor speed
int velocity1;
int velocity2;

// Critical section variables for both encoders
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux2 = portMUX_INITIALIZER_UNLOCKED;

// Function for handling encoder 1 interrupt
void IRAM_ATTR doEncoder1A() {
  portENTER_CRITICAL_ISR(&mux);  // Enter critical section for interrupt
  digitalRead(encoder1PinB) ? encoder1Pos++ : encoder1Pos--;
  portEXIT_CRITICAL_ISR(&mux);   // Exit critical section for interrupt
}

// Function for handling encoder 2 interrupt
void IRAM_ATTR doEncoder2A() {
  portENTER_CRITICAL_ISR(&mux2);  // Enter critical section for interrupt
  digitalRead(encoder2PinB) ? encoder2Pos++ : encoder2Pos--;
  portEXIT_CRITICAL_ISR(&mux2);   // Exit critical section for interrupt
}

void setup() {
  // Setup encoder 1 interrupt
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, RISING);

  // Setup encoder 2 interrupt
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder2A, RISING);

  // Setup motor driver for both motors
  pinMode(motor1Pin1, OUTPUT);  // Forward pin motor 1
  pinMode(motor1Pin2, OUTPUT);  // Backward pin motor 1
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);

  pinMode(motor2Pin1, OUTPUT);  // Forward pin motor 2
  pinMode(motor2Pin2, OUTPUT);  // Backward pin motor 2
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);

  Serial.begin(9600); // Setup serial communication

  prevTime1 = millis(); // Initialize time tracking for motor 1
  prevTime2 = millis(); // Initialize time tracking for motor 2
}

void loop() {
  // Read encoder 1 position with critical section
  portENTER_CRITICAL(&mux);  // Protect access to encoder1Pos
  encoder1PosRead = encoder1Pos; // Read position from interrupt
  portEXIT_CRITICAL(&mux);

  // Read encoder 2 position with critical section
  portENTER_CRITICAL(&mux2);  // Protect access to encoder2Pos
  encoder2PosRead = encoder2Pos; // Read position from interrupt
  portEXIT_CRITICAL(&mux2);

  // Calculate motor 1 speed (assuming 100 steps per revolution)
  deltaPos1 = encoder1PosRead - prevEncoder1PosRead;
  prevEncoder1PosRead = encoder1PosRead;
  speedMotor1 = deltaPos1 / dt1 / 100; // Motor 1 speed in rotations per second (RPS)

  // PID control for motor 1 speed
  error1 = targetSpeed1 - speedMotor1;
  integral1 += error1 * dt1;
  derivative1 = (error1 - prevError1) / dt1;
  control1 = (Kp * error1) + (Ki * integral1) + (Kd * derivative1);
  velocity1 = min(max(control1, -255.0f), 255.0f);

  // Calculate motor 2 speed (assuming 100 steps per revolution)
  deltaPos2 = encoder2PosRead - prevEncoder2PosRead;
  prevEncoder2PosRead = encoder2PosRead;
  speedMotor2 = deltaPos2 / dt2 / 100; // Motor 2 speed in rotations per second (RPS)

  // PID control for motor 2 speed
  error2 = targetSpeed2 - speedMotor2;
  integral2 += error2 * dt2;
  derivative2 = (error2 - prevError2) / dt2;
  control2 = (Kp * error2) + (Ki * integral2) + (Kd * derivative2);
  velocity2 = min(max(control2, -255.0f), 255.0f);

  // Set motor 1 direction and PWM signal
  if (velocity1 >= 0) {
    digitalWrite(motor1Pin1, HIGH);  // Motor 1 forward
    digitalWrite(motor1Pin2, LOW);
    analogWrite(pwm1Pin, velocity1);
  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);  // Motor 1 backward
    analogWrite(pwm1Pin, 255 + velocity1); // Adjust for negative PWM
  }

  // Set motor 2 direction and PWM signal
  if (velocity2 >= 0) {
    digitalWrite(motor2Pin1, HIGH);  // Motor 2 forward
    digitalWrite(motor2Pin2, LOW);
    analogWrite(pwm2Pin, velocity2);
  } else {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);  // Motor 2 backward
    analogWrite(pwm2Pin, 255 + velocity2); // Adjust for negative PWM
  }

  // Print motor speeds in CSV format
  Serial.print(millis());  // Print timestamp in milliseconds
  Serial.print(",");       // CSV separator
  Serial.print(speedMotor1); // Print motor 1 speed
  Serial.print(",");       // CSV separator
  Serial.println(speedMotor2); // Print motor 2 speed and move to the next line

  // Delay for serial print
  delay(1000);  // Add a 1 second delay for the serial output

  // Time management for consistent loop time (Motor 1)
  do {
    currTime1 = millis();
    delTime1 = currTime1 - prevTime1;
  } while (delTime1 < dtMillis1);

  prevTime1 = currTime1;
  prevError1 = error1;

  // Time management for consistent loop time (Motor 2)
  do {
    currTime2 = millis();
    delTime2 = currTime2 - prevTime2;
  } while (delTime2 < dtMillis2);

  prevTime2 = currTime2;
  prevError2 = error2;
}
