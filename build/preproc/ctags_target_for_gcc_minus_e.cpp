# 1 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino"
# 2 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino" 2
# 3 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino" 2
# 4 "/home/kingo/Documents/Arduino Projects/maze-solver/maze-solver.ino" 2

// Motors






// Ultrasonics






// Sound sensor



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

void setup()
{
  // Serial.begin(9600);
  Wire.begin();

  pinMode(11, 0x0);
  pinMode(12, 0x1);
  // Ultrasonic
  pinMode(A0 /* Sends duration of wave (input)*/, 0x0);
  pinMode(5 /* Sends wave (output)*/, 0x1);
  pinMode(A2, 0x0);
  pinMode(4, 0x1);
  pinMode(A1, 0x0);
  pinMode(8, 0x1);
  // Motors
  pinMode(10 /* Use PWM to control speed of motor*/, 0x1);
  pinMode(2 /* Forward motor*/, 0x1);
  pinMode(3 /* Backward motor*/, 0x1);
  pinMode(9, 0x1);
  pinMode(6, 0x1);
  pinMode(7, 0x1);

  display.begin();
  display.clear();
  // Start from centre (based on length of text) of first row
  display.setCursor(4, 0);
  display.print("KUKU BOT");
  display.setCursor(0, 1);
  display.print("Chrisland School");

  // Set default speed of motors
  analogWrite(10 /* Use PWM to control speed of motor*/, motorSpeed);
  analogWrite(9, motorSpeed);
  // Ensure motors aren't on
  move(STOP);
}

void loop()
{
  // Check if hand is in front of robot
  if (!started)
  {
    // Check for clap
    bool clapped = (digitalRead(11) == 0x0);
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
    tone(12, 400);
    delay(200);
    noTone(12);
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
  analogWrite(10 /* Use PWM to control speed of motor*/, motorSpeed);
  analogWrite(9, motorSpeed);
}

void waitTill90()
{
  float desiredAngle = ((mpu.getAngleZ())>0?(mpu.getAngleZ()):-(mpu.getAngleZ())) + 87.0;
  while (((mpu.getAngleZ())>0?(mpu.getAngleZ()):-(mpu.getAngleZ())) < desiredAngle)
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
    digitalWrite(2 /* Forward motor*/, forward);
    digitalWrite(3 /* Backward motor*/, !forward);
    digitalWrite(6, forward);
    digitalWrite(7, !forward);
  }

  // Stop (put all motors off)
  else if (dir == STOP)
  {
    digitalWrite(2 /* Forward motor*/, 0x0);
    digitalWrite(3 /* Backward motor*/, 0x0);
    digitalWrite(6, 0x0);
    digitalWrite(7, 0x0);

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
      digitalWrite(6, forward);
    digitalWrite(3 /* Backward motor*/, forward);
      // Set right motors to move back to make turning smoother
      digitalWrite(7, !forward);
   digitalWrite(2 /* Forward motor*/, !forward);
    }

    else
    {
      // Turning right requires enabling left motors
      digitalWrite(2 /* Forward motor*/, forward);
    digitalWrite(7, forward);
      // Set left motors to move back to make turning smoother
      digitalWrite(3 /* Backward motor*/, !forward);
   digitalWrite(6, !forward);
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
    ? A0 /* Sends duration of wave (input)*/
    : (sens == LEFT)
      ? A2
      : A1;

  // Get trig pin (front, left or right)
  int trigPin = (sens == FORWARD)
    ? 5 /* Sends wave (output)*/
    : (sens == LEFT)
      ? 4
      : 8;

  // Warm up trigger
  digitalWrite(trigPin, 0x0);
  delayMicroseconds(4);

  // Send out 8 cycle ultrasonic burst
  digitalWrite(trigPin, 0x1);
  delayMicroseconds(10);
  digitalWrite(trigPin, 0x0);

  // Get time taken for echo pin to go from HIGH to LOW
  long t = pulseIn(echoPin, 0x1);

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
  int distFrom90 = ((int) ((currAngle)>0?(currAngle):-(currAngle))) % 90;
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
      analogWrite(9, motorSpeed - (rotateBy * speedDecrRate));
    // If the robot is facing the left, make it shift to the right
    else
      analogWrite(10 /* Use PWM to control speed of motor*/, motorSpeed - (rotateBy * speedDecrRate));
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
