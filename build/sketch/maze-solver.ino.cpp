#include <Arduino.h>
#line 1 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
#include <Wire.h>
#include <MPU6050_light.h>
#include <LiquidCrystal_I2C.h>

// Motors
#define leftMotorSpeed 10 // Use PWM to control speed of motor
#define leftMotorA  2 // Forward motor
#define leftMotorB  3 // Backward motor
#define rightMotorSpeed 9
#define rightMotorA 6
#define rightMotorB 7
// Ultrasonics
#define frontEcho A0 // Sends duration of wave (input)
#define frontTrig 5 // Sends wave (output)
#define leftEcho  A2
#define leftTrig 4
#define rightEcho A1
#define rightTrig 8
// Sound sensor
#define sound 11
#define buzzer 12

enum moveCommands
{
  FORWARD, /*000*/ BACK, /*001*/ 
  RIGHT, /*010*/ LEFT = 4, /*100*/
  STOP = 7 /*111*/
};

MPU6050 mpu(Wire);
LiquidCrystal_I2C display(0x3F, 16, 2);

const int safeDist = 13, junctionDist = 30, robotLength = 25;
const int motorSpeed = 140, speedDecrRate = 5;
const int stopDelay = 100, turnDelay = 850;
int lastDir = -1;
bool started = false, seenJunction = false; // Maze contains only one open junction

void test_bot(bool checkMove = false);
void buzz(int count = 1);
int get_dist(int sens);

#line 43 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
void setup();
#line 80 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
void loop();
#line 162 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
void print_dir(int dir);
#line 203 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
void restore_motor_speed();
#line 209 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
void waitTill90();
#line 218 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
void move(int dir);
#line 367 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
void align_bot();
#line 396 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
void navigate_maze();
#line 43 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
void setup()
{
  // Serial.begin(9600);
  Wire.begin();

  pinMode(sound, INPUT);
  pinMode(buzzer, OUTPUT);
  // Ultrasonic
  pinMode(frontEcho, INPUT);
  pinMode(frontTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(rightEcho, INPUT);
  pinMode(rightTrig, OUTPUT);
  // Motors
  pinMode(leftMotorSpeed, OUTPUT);
  pinMode(leftMotorA, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(rightMotorSpeed, OUTPUT);
  pinMode(rightMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);

  display.begin();
  display.clear();
  // Start from centre (based on length of text) of first row
  display.setCursor(4, 0);
  display.print("KUKU BOT");
  display.setCursor(0, 1);
  display.print("Chrisland School");

  // Set default speed of motors
  analogWrite(leftMotorSpeed, motorSpeed);
  analogWrite(rightMotorSpeed, motorSpeed);
  // Ensure motors aren't on
  move(STOP);
}

void loop()
{
  // Check if hand is in front of robot
  if (!started)
  {
    // Check for clap
    bool clapped = (digitalRead(sound) == LOW);
    if (clapped)
    {
      started = true;
      // Serial.println("Go!!!");
      
      display.clear();
      display.setCursor(0, 0);

      // Start gyro
      if (mpu.begin() != 0)
      {
        // Error with gyro
        display.print("MPU6050 error");
        buzz(4);
        started = false;
        return; 
      }
      
      display.print("Calibrating gyro");

      delay(1000);
      mpu.calcOffsets();

      // move(RIGHT);
    }
  }
  
  else
  {
    mpu.update();
    navigate_maze();
  }
}

void test_bot(bool checkMove = false)
{
  if (!checkMove)
  {
    delay(2000);
    get_dist(FORWARD);
    delay(2000);
    get_dist(LEFT);
    delay(2000);
    get_dist(RIGHT);
  }

  else
  {    
    move(FORWARD);
    delay(1500);
    move(STOP);
    move(LEFT);
    delay(1500);
    move(STOP);
    move(RIGHT);
    delay(1500);
    move(STOP);
    move(BACK);
    delay(1500);
    move(STOP);
  }
}

void buzz(int count = 1)
{
  for (int i = 0; i < count; i++)
  {
    tone(buzzer, 400);
    delay(200);
    noTone(buzzer);
    if (i != count - 1) delay(100);
  }
}

// Print direction on LCD
void print_dir(int dir)
{
  if (dir == lastDir) return;

  lastDir = dir;
  
  display.clear();
  
  // Print at centre
  display.setCursor(4, 0);
  display.print("KUKU BOT");

  String textToPrint = "Move ";
  textToPrint += (dir == FORWARD)
  ? "forward"
  : (dir == BACK)
    ? "backward"
    : (dir == STOP)
      ? "stop"
      : (dir == LEFT)
        ? "left"
        : "right";

  // Set cursor to middle depending on text length
  int cursPos = (16 - textToPrint.length()) / 2;
  // Print "move right/left" with arrow
  if (dir == LEFT || dir == RIGHT)
  {
    // Move cursor back to accommodate one more char
    display.setCursor(cursPos - 1, 1);
    display.write((dir == LEFT) ? 126 : 127); // arrow
  }

  // Print "move forward/backward"
  else
  {
    display.setCursor(cursPos, 1);
    display.print(textToPrint);
  }
}

void restore_motor_speed()
{
  analogWrite(leftMotorSpeed, motorSpeed);
  analogWrite(rightMotorSpeed, motorSpeed);
}

void waitTill90()
{
  float desiredAngle = abs(mpu.getAngleZ()) + 87.0;
  while (abs(mpu.getAngleZ()) < desiredAngle)
  {
    mpu.update();
  }
}

void move(int dir)
{
  // Don't move until [started] is true
  // Don't move if new direction is the same as previous one
  if (!started) return;
  
  print_dir(dir);

  // Check if direction is forward by checking if last bit is zero
  bool forward = !(dir & BACK);

  // If motor A is on, that means motor is moving forward
  // If motor B is on, that means motor is moving backward
  
  // Forward or backward only
  if (dir == FORWARD || dir == BACK)
  {
    digitalWrite(leftMotorA, forward);
    digitalWrite(leftMotorB, !forward);
    digitalWrite(rightMotorA, forward);
    digitalWrite(rightMotorB, !forward);
  }

  // Stop (put all motors off)
  else if (dir == STOP)
  {
    digitalWrite(leftMotorA, LOW);
    digitalWrite(leftMotorB, LOW);
    digitalWrite(rightMotorA, LOW);
    digitalWrite(rightMotorB, LOW);

    delay(stopDelay);
  }

  // Left or right
  else
  {
    // Turn off all motors
    move(STOP);
    mpu.update();
    
    if (dir & LEFT)
    {
      // Turning left requires enabling right motors 
      digitalWrite(rightMotorA, forward);
		  digitalWrite(leftMotorB, forward);
      // Set right motors to move back to make turning smoother
      digitalWrite(rightMotorB, !forward);
			digitalWrite(leftMotorA, !forward);
    }

    else
    {
      // Turning right requires enabling left motors
      digitalWrite(leftMotorA, forward);
		  digitalWrite(rightMotorB, forward);
      // Set left motors to move back to make turning smoother
      digitalWrite(leftMotorB, !forward);
			digitalWrite(rightMotorA, !forward);						
		}

    // Wait for a while to fully turn
    // delay(turnDelay);
    waitTill90();
    move(STOP);
  }
}

// sens -> FORWARD, LEFT, RIGHT
int get_dist(int sens) 
{
  delay(35); // Warm up sensor
  
  // Get echo pin (front, left or right)
  int echoPin = (sens == FORWARD)
    ? frontEcho
    : (sens == LEFT)
      ? leftEcho
      : rightEcho;
      
  // Get trig pin (front, left or right)
  int trigPin = (sens == FORWARD)
    ? frontTrig
    : (sens == LEFT)
      ? leftTrig
      : rightTrig;
		
  // Warm up trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);

  // Send out 8 cycle ultrasonic burst
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Get time taken for echo pin to go from HIGH to LOW
  long t = pulseIn(echoPin, HIGH);

  // v = 2x / t
  // x = vt / 2
  // x (cm) = 0.034 (cm/us) * t (us) / 2
  float x /* distance */ = 0.034 * (float) t / 2;

  // display.clear();
  // display.setCursor(0, 0);
  // display.print((sens == LEFT)
  //    ? "Left"
  //    : (sens == RIGHT)
  //      ? "Right" : "Front");
  // display.print(": ");
  // display.print(x);
  // display.print("cm");

  // Sensor not working
  if (x <= 0)
  {
    move(STOP);
    started = false;

    display.clear();
    display.setCursor(2, 0);
    
    // Error log
    if (sens == FORWARD)
    {
      display.print("Front");
      buzz();
    }

    else if (sens == LEFT)
    {
      display.print("Left");
      buzz(2);
    }

    else
    {
      display.print("Right");
      buzz(3);
    }
    display.print(" sensor");
    display.setCursor(2, 1);
    display.print("not working");
  }

  return x;
}

void align_bot()
{
  float currAngle = mpu.getAngleZ();
  int distFrom90 = ((int) abs(currAngle)) % 90;
  // Get angle to move by from 90 degrees
  int rotateBy = (distFrom90 < 45)
    ? distFrom90
    : 90 - distFrom90;

  // Serial.print("Rotate by ");
  // Serial.println(rotateBy);
  // delay(10);

  if (rotateBy >= 5)
  {
    // If the robot is facing the right, make it shift to the left
    // Shifting is done by reducing the speed of one motor
    if (currAngle < 0)
      analogWrite(rightMotorSpeed, motorSpeed - (rotateBy * speedDecrRate));          
    // If the robot is facing the left, make it shift to the right
    else
      analogWrite(leftMotorSpeed, motorSpeed - (rotateBy * speedDecrRate));
  }
  
  // Else, robot is moving straight enough
  else
    restore_motor_speed();
}

void navigate_maze()
{
  check:
    int frontDist = get_dist(FORWARD), leftDist = get_dist(LEFT), rightDist = get_dist(RIGHT);

    int greatestDist = frontDist, dir = FORWARD;
    if (leftDist > greatestDist)
    {
      greatestDist = leftDist;
      dir = LEFT;
    }

    if (rightDist > greatestDist)
    {
      greatestDist = rightDist;
      dir = RIGHT;
    }

    if (greatestDist < robotLength)
    {
      move(STOP);
      move(BACK);
      goto check;
    }

    delay(150);
    if (dir != lastDir)
      move(STOP);

    move(dir);
}

