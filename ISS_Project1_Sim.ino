#define IN1 A0
#define IN2 A1
#define IN3 A2
#define IN4 A3
#define ENA 5
#define ENB 6
 
 
#define WHEEL_L 2
#define WHEEL_R 3
 
 
double wheel_diameter = 6.5;       
double wheel_circ = wheel_diameter * M_PI;
int resolution = 20;               
double wheelbase = 9.0;           
 
 
const int EDGES_PER_SLOT = 2;      
const int pulses_per_rev = resolution * EDGES_PER_SLOT;
double distance_per_tick = wheel_circ / (double)pulses_per_rev;
 
volatile long left_ticks  = 0;
volatile long right_ticks = 0;
 
 
volatile int8_t dirL = 0;
volatile int8_t dirR = 0;
 
int default_speed = 150;
int SAMPLE_RATE = 10;
 
 
enum Direction { FORWARD, BACKWARD, LEFT, RIGHT };
 
void setup() {
  Serial.begin(9600);
 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
 
  pinMode(WHEEL_L, INPUT);
  pinMode(WHEEL_R, INPUT);
 
 
  attachInterrupt(digitalPinToInterrupt(WHEEL_L), isrLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(WHEEL_R), isrRight, CHANGE);
 
}
 
void loop() {
  odomReset();
 
  moveBy(20);  
  delay(500);
 
  moveBy(-20);  
  delay(500);
 
  double l, r;
  odomRead(l, r);
 
  Serial.print("L(cm): "); Serial.print(l, 2);
  Serial.print("  R(cm): "); Serial.print(r, 2);
  Serial.print("  Linear(cm): "); Serial.println(odomLinearCm(), 2);
 
  delay(2000);
 
  rotate(180);  
  delay(500);
 
  moveRightMotorBy(10);
  delay(500);
  moveLeftMotorBy(-10);
 
  odomRead(l, r);
Serial.print(l, 2);
Serial.println(r, 2);
 
  delay(3000);
 
  // if (Serial.available() > 0) {
  //   char readByte = Serial.read();  
 
  //   if (readByte == 'r') {           
  //     int value = Serial.parseInt();
  //     moveRightMotorBy(value);
 
  //   } else if (readByte == 'l') {    
  //     int value = Serial.parseInt();
  //     moveLeftMotorBy(value);
 
  //   } else if (readByte == 'f') {    
  //     int value = Serial.parseInt();
  //     moveBy(value);
 
  //   } else if (readByte == 'b') {    
  //     int value = Serial.parseInt();
  //     moveBy(-value);
  //   }
  //   else if (readByte == 't') {   
  //     int value = Serial.parseInt();
  //     rotate(value);
  //   }
  // }
}
 
 
void moveBy(int distance_cm) {
  if (distance_cm == 0) return;
  setDirection(distance_cm > 0 ? FORWARD : BACKWARD);
 
  setMotorSpeed(ENA, default_speed);
  setMotorSpeed(ENB, default_speed);
 
  int pulses_needed = abs(distance_cm) / distance_per_tick;
 
  long startL, startR;
  noInterrupts();
  startL = left_ticks;
  startR = right_ticks;
  interrupts();
 
  while ( (labs(left_ticks - startL) < pulses_needed) &&
          (labs(right_ticks - startR) < pulses_needed)) {
 
  }
 
  stopRobot();
}
 
 
void rotate(int angle) {
 
  noInterrupts();
  left_ticks = 0;
  right_ticks = 0;
  interrupts();
 
 
  double arc = PI * wheelbase / 360.0 * abs(angle);
  double target = arc / distance_per_tick;
 
 
  Direction right_direction = (angle > 0) ? FORWARD : BACKWARD;
 
 
  if (right_direction == FORWARD) {
 
    digitalWrite(IN1, HIGH);  
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);  
    digitalWrite(IN4, LOW);
    dirL = -1;
    dirR = +1;
  } else {
 
    digitalWrite(IN1, LOW);   
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);   
    digitalWrite(IN4, HIGH);
    dirL = +1;
    dirR = -1;
  }
 
 
  setMotorSpeed(ENA, default_speed);
  setMotorSpeed(ENB, default_speed);
 
 
  while ( (labs(left_ticks) < target) || (labs(right_ticks) < target) ) {
    delay(2);
  }
 
  stopRobot();
}
 
 
void moveLeftMotorBy(int distance_cm) {
  if (distance_cm == 0) return;
 
  if (distance_cm > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    dirL = +1;
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    dirL = -1;
  }
  dirR = 0;
 
  setMotorSpeed(ENA, default_speed);
  setMotorSpeed(ENB, 0);
 
  int pulses_needed = abs(distance_cm) / distance_per_tick;
  long startL;
  noInterrupts();
  startL = left_ticks;
  interrupts();
 
  while (labs(left_ticks - startL) < pulses_needed) { }
 
  stopRobot();
}
 
 
void moveRightMotorBy(int distance_cm) {
  if (distance_cm == 0) return;
 
  if (distance_cm > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    dirR = +1;
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    dirR = -1;
  }
  dirL = 0;
 
  setMotorSpeed(ENA, 0);
  setMotorSpeed(ENB, default_speed);
 
  int pulses_needed = abs(distance_cm) / distance_per_tick;
  long startR;
  noInterrupts();
  startR = right_ticks;
  interrupts();
 
  while (labs(right_ticks - startR) < pulses_needed) { }
 
  stopRobot();
}
 
 
void odomReset() {
  noInterrupts();
  left_ticks = 0;
  right_ticks = 0;
  interrupts();
}
 
void odomRead(double &left_cm, double &right_cm) {
  long l, r;
  noInterrupts();
  l = left_ticks;
  r = right_ticks;
  interrupts();
  left_cm  = l * distance_per_tick;
  right_cm = r * distance_per_tick;
}
 
double odomLinearCm() {
  double l, r;
  odomRead(l, r);
  return (l + r) * 0.5;
}
 
double odomPathLengthCm() {
  double l, r;
  odomRead(l, r);
  return (fabs(l) + fabs(r)) * 0.5;
}
 
 
void isrLeft() {
  left_ticks += (long)dirL;
}
 
void isrRight() {
  right_ticks += (long)dirR;
}
 
 
void setDirection(Direction dir) {
  switch (dir) {
    case FORWARD:
      dirL = +1; dirR = +1;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      break;
 
    case BACKWARD:
      dirL = -1; dirR = -1;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      break;
 
    case LEFT:
      dirL = -1; dirR = +1;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      break;
 
    case RIGHT:
      dirL = +1; dirR = -1;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
 
    default:
      dirL = 0; dirR = 0;
    
      break;
  }
}
 
void setMotorSpeed(int pin, int speed) {
  analogWrite(pin, constrain(speed, 0, 255));
}
 
void stopRobot() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  dirL = 0;
  dirR = 0;
}