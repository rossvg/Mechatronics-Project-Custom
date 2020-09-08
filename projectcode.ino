//**********************************************************************************************************************************************************************************************************
// ELEC 299 MECHATRONICS FINAL PROJECT            April 4, 2015
//**********************************************************************************************************************************************************************************************************
// Group #: 36
//
//  Final Project Code
//
// Group Members: 
// Ross Vrana-Godwin
// Raphael-Alexander Nelson
// Sean Hope
//
// Description:
  /* Robot Slam dunk competition. The objective of the program is to control
   *  the robot on a playing board. The robot will identify a lit beacon,
   *  go towards it and pick up a ball, and go the the dunk tube to 
   *  release the ball. Repeat.
   */

 //---------------------------------------------------------------
 //  GLOBAL VARIABLE AND CONSTANT DECLARATION,
 //--------------------------------------------------------------
 // declare all variables needed for the program. Defining all program constants

#define leftLight A0 // left line sensor as analog pin A0
#define centerLight A1 // centre line sensor as analog pin A1
#define rightLight A2 // right line sensor as analog pin A2

// Light threshold to identify whether on black tape or not.
// Threshold was calibrated based on line sensors
#define LTHRESH 400
#define CTHRESH 400
#define RTHRESH 400

// bumper pins
#define leftBumper 11
#define rightBumper 2

// constants for speed
// constant speed allows for line adjustment
#define rspeed 75
#define lspeed 75

#define buttonPin 3 // for pushbutton circuit on robot

#define rightDir  7 // right motor direction
#define rightSpeed  6 // right motor speed
#define leftDir  4 // left motor direction
#define leftSpeed  5 // left motor speed

#define adjust 30 // line adjustment speeds

# define panCentre 97 // centres the pan servo for the gripper
# define gripOpen 40 // opens the gripper all the way
# define tiltVertical 140 // puts tilt servo all the way up
# define tiltHoriz 65 // sets tilt at horizontal
# define gripBall 112 // grips the ball. Calibrated to grip the ball after grip sensing tests

#include <Servo.h>   // include servo library
Servo tilt, pan, grip; // setting the tilt, pan, and grip servos as servo objects

int check; // check variable for bluetooth validation
int station; // storagefor bluetooth number, 


//........................................................................SETUP...........................................................................................................................

void setup() {
  
  Serial.begin(115200); // open serial monitor to correspond with bluetooth
  // setting all motor servos as outputs
  pinMode(rightDir, OUTPUT); 
  pinMode(rightSpeed, OUTPUT);
  pinMode(leftDir, OUTPUT);
  pinMode(leftSpeed, OUTPUT);

// associating the grip servos with hardware pins
   pan.attach(8); 
   tilt.attach(9); 
   grip.attach(10); 

 pan.write(panCentre); // centres the gripper
 tilt.write(tiltVertical); // gets grippper in vertical position
 grip.write(gripOpen); // opens gripper completely

 WaitButton(); // wait for pushbutton
 BTwait(); // wait for bluetooth connection
}
//.....................................................................MAIN LOOP..........................................................................................................................

void loop() 
{
  check=Serial.available();//check's if there is a BT character received
  if (check==1)
  {
    station=(Serial.read()) - 48; // returns a byte character after reading bluetooth character
    
    Serial.println(station); // debug what station
    
    pickup(station); // main function to pickup and dunk ball
  }
}

//*****************************FUNCTIONS************************************************************************
// PICKUP
//function for ball retreival from either station starting from center field, AND BALL DUNKING

void pickup (int station) //depending on what variable recieved, rotates will designated
{
  if (station==0) // left beacon
  {
    Serial.println("Rotate"); // debug statement

    RightRotate90();//rotates from default position facing bucket towards '0' beacon at robot's 3 o'clock
    
    while(((analogRead(centerLight) < CTHRESH) && (analogRead(leftLight) < LTHRESH) && (analogRead(rightLight) < RTHRESH))!= 1) // run loop until all light sensor are on the black tape
    {
      Forward(); //drives forward until hits crossbeam for ball pickup
    }
    Stop(); // turn motors off to prepare to get the ball
    delay(20);
    GripBall(); //pick up the ball function
    BackUp(); //after picking up, backs up a tad to allow for rotating
    LeftRotate90(); //reverses direction 180 degrees. Function will rotate until centre light is on black tape 
    
    while(((analogRead(centerLight) < CTHRESH) && (analogRead(leftLight) < LTHRESH) && (analogRead(rightLight) < RTHRESH))!= 1) 
    {
      Forward(); //drives back towards center
    }
    
    TipToe(); //drives the nose past the crossline to allow for rotating
    delay(100);
    LeftRotate90(); //turns towards dunk net
    delay(100);
    score(); //takes ball to dunk and score
  }
  
  else if (station==1) // front beacon without black tape. Use bumpers instead
  {
    while (true) // run loop until if condition breaks it
    {
    Forward(); // drive forward following black tape
   
      if ((digitalRead(rightBumper) == LOW) && (digitalRead(leftBumper) == LOW)) // indicates bumper has been pressed, robot has hit wall 
      {
        break; // break out of loop to stop going forward
      }
    }
    Stop(); // stop robot
    // this sequence backs up the robot slightly to pickup the ball
     delay(50);
  digitalWrite(leftDir, LOW);
  delay(50);
  digitalWrite(rightDir, LOW);
  delay(50);

  analogWrite (leftSpeed, lspeed);
  analogWrite (rightSpeed, rspeed);
  delay(750);
  Stop(); // stop and prepare for grip
    GripBall(); //pick up the ball
    BackUp(); //after picking up, backs up a tad to allow for rotating
    LeftRotate90(); //reverses direction 180 degrees

    // while loop drives until all light sensors are on black tape meaning robot has hit the centre
    while(((analogRead(centerLight) < CTHRESH) && (analogRead(leftLight) < LTHRESH) && (analogRead(rightLight) < RTHRESH))!= 1) 
    {
      Forward(); //drives back towards center
    }
    
    TipToe(); //drives the nose past the crossline to allow for rotating
    score(); //takes ball to dunk and scores
  }
  
  else if (station==2) // left beacon
  {
    LeftRotate90(); //rotates from default position

    // drive until robot his black tape ,indicating it has reacyhed the beacon
    while(((analogRead(centerLight) < CTHRESH) && (analogRead(leftLight) < LTHRESH) && (analogRead(rightLight) < RTHRESH))!= 1)
    {
      Forward(); //drives forward until hits crossbeam for ball pickup
    }
    Stop(); // stop motors
    GripBall(); //pick up the ball
    BackUp(); //after picking up, backs up a tad to allow for rotating
    LeftRotate90(); //reverses direction 180 degrees

    //driving back towards the centre
    while(((analogRead(centerLight) < CTHRESH) && (analogRead(leftLight) < LTHRESH) && (analogRead(rightLight) < RTHRESH))!= 1)
    {
      Forward(); //drives back towards center
    }
    
    TipToe(); //drives the nose past the crossline to allow for rotating
    RightRotate90(); //turns towards dunk net
    score(); //takes ball to dunk and scores
  }
}

//.......................................................................................................................................................................................................
//function for ball delivery from center field

void score () 
{
    while(((analogRead(centerLight) < CTHRESH) && (analogRead(leftLight) < LTHRESH) && (analogRead(rightLight) < RTHRESH))!= 1)
    {
      Forward(); //drives forward towards dunk zone
    }
    DropBall(); //drop the ball in the net
    BackUp();
    BackUp();
    LeftRotate90();

    while(((analogRead(centerLight) < CTHRESH) && (analogRead(leftLight) < LTHRESH) && (analogRead(rightLight) < RTHRESH))!= 1)
    {
      Forward(); //drives back towards center
    }
   TipToe();
   LeftRotate90(); //after this will be facing station '0'
   LeftRotate90(); //rotate again to face net for default position
}

//.......................................................................................................................................................................................................
//function for grabbing ball
void GripBall() 
{
  grip.write(gripOpen); // opens gripper completely
  delay(1000);
  tilt.write(tiltHoriz); // make gripper horizontal
  delay(1000);
  grip.write(gripBall);
  delay(1000);
  tilt.write(tiltVertical);
}

//.......................................................................................................................................................................................................
//function for dropping ball

void DropBall() 
{
  
  grip.write(gripOpen);
  delay(1000);
  tilt.write(tiltVertical);
}

//.......................................................................................................................................................................................................
//drives bot forwards, staying aligned on guidelines darker than threshold

void Forward() 
{ digitalWrite(rightDir, HIGH);
  digitalWrite(leftDir, HIGH);

  if (analogRead(centerLight) < CTHRESH && analogRead(leftLight) > LTHRESH && analogRead(rightLight) > RTHRESH) // centre light on black line , drive sraight forward
  {
    analogWrite (leftSpeed, lspeed);
    analogWrite (rightSpeed, rspeed);
  }
  if (analogRead(centerLight) < CTHRESH && analogRead(leftLight) < LTHRESH && analogRead(rightLight) > RTHRESH) // centre and left light on tape ,robot is veering right
  {
    analogWrite (leftSpeed, lspeed  - adjust); // decrease eft whel speed and incease right wheel speed
    analogWrite (rightSpeed, rspeed + adjust); // increaase right wheel speed
  }
  if (analogRead(centerLight) > CTHRESH && analogRead(leftLight) < LTHRESH && analogRead(rightLight) > RTHRESH) // only left light is on ape , robot is  veering strongly right
  {
    analogWrite (leftSpeed, lspeed - adjust); // decrease left
    analogWrite (rightSpeed, rspeed + adjust);  // increase right
  }
  if (analogRead(centerLight) < CTHRESH && analogRead(leftLight) > LTHRESH && analogRead(rightLight) < RTHRESH) // centre is on tape, right is on the tape,robot is veering left
  {
    analogWrite (leftSpeed, lspeed + adjust); // increase left wheel speed
    analogWrite (rightSpeed, rspeed - adjust); // decrase right wheel speed
  }
  if (analogRead(centerLight) > CTHRESH && analogRead(leftLight) > LTHRESH && analogRead(rightLight) < RTHRESH) //  centre light is of tape and only right light is on tape, robot is veering hard left
  {
    analogWrite (leftSpeed, lspeed + adjust);
    analogWrite (rightSpeed, rspeed - adjust);
  }
  if (analogRead(centerLight) < CTHRESH && analogRead(leftLight) < LTHRESH && analogRead(rightLight) < RTHRESH) // one a horizontal tapeline is reached stop
  {
    Stop();
  }
}


//.......................................................................................................................................................................................................
//rotates bot left to realign on next black line

void LeftRotate90() 
{
  delay(50);
  digitalWrite(leftDir, LOW);
  delay(50);
  digitalWrite(rightDir, HIGH);
  delay(100);

  while (analogRead(centerLight) > CTHRESH)
  {
    analogWrite (leftSpeed, lspeed);
    analogWrite (rightSpeed, rspeed);
  }
  delay(50);
  
while (analogRead(centerLight) < CTHRESH )
  {
    analogWrite (leftSpeed, lspeed);
    analogWrite (rightSpeed, rspeed);
  }
delay(50);
  while (analogRead(centerLight) > CTHRESH)
  {
    analogWrite (leftSpeed, lspeed);
    analogWrite (rightSpeed, rspeed);
  }
  
 Stop();

}

//.......................................................................................................................................................................................................
//rotates bot right to realign on next black line

void RightRotate90() 
{
  delay(50);
  digitalWrite(leftDir, HIGH);
  delay(50);
  digitalWrite(rightDir, LOW);
  delay(100);
  while (analogRead(centerLight) > CTHRESH)
  {
    analogWrite (leftSpeed, lspeed);
    analogWrite (rightSpeed, rspeed);
  }
  delay(50);
  
  while (analogRead(centerLight) < CTHRESH )
  {
    analogWrite (leftSpeed, lspeed);
    analogWrite (rightSpeed, rspeed);
  }
delay(50);
  while (analogRead(centerLight) > CTHRESH)
  {
    analogWrite (leftSpeed, lspeed);
    analogWrite (rightSpeed, rspeed);
  }
 Stop();
delay(50);
}

//.......................................................................................................................................................................................................
//allows for start of operation after button press for testing

void WaitButton () 
{
  while (digitalRead(buttonPin) == HIGH)
  {

  }
  while (digitalRead(buttonPin) == LOW)
  {

  }
}

//.......................................................................................................................................................................................................
//Stops both motors and Halts movement

void Stop () 
{
  analogWrite (leftSpeed, 0);
  analogWrite (rightSpeed, 0);
}

void BackUp () //subroutine moves robot just a touch backward to allow for turning
{
  delay(50);
  digitalWrite(leftDir, LOW);
  delay(50);
  digitalWrite(rightDir, LOW);
  delay(50);

  analogWrite (leftSpeed, lspeed);
  analogWrite (rightSpeed, rspeed);

  delay(1000);
  Stop();

}

//.......................................................................................................................................................................................................
//subroutine moves robot just a touch forward to allow for turning

void TipToe ()
{
  delay(50);
  digitalWrite(leftDir, HIGH);
  delay(50);
  digitalWrite(rightDir, HIGH);
  delay(50);

  analogWrite (leftSpeed, lspeed);
  analogWrite (rightSpeed, rspeed);

  delay(500);
  Stop();

}

//.......................................................................................................................................................................................................
//subroutine recenters robot after backing up

void ReCenter () 
{
  delay(50);
  digitalWrite(leftDir, HIGH);
  delay(50);
  digitalWrite(rightDir, HIGH);
  delay(50);
  
  analogWrite (leftSpeed, lspeed);
  analogWrite (rightSpeed, rspeed);

  delay(1550);
  Stop();
 
}


//.......................................................................................................................................................................................................
//bluetooth wait
// wait for signal from bluetooth

void BTwait()
{
  while (Serial.available()==0)
  {
    Serial.println("waiting for BlueTooth"); 
    Serial.print(analogRead(rightLight));
    Serial.print(' ');
    Serial.print(analogRead(centerLight));
    Serial.print(' ');
    Serial.print(analogRead(leftLight));
    
    delay(1000); 
  }
   Serial.println("Connected"); 
  tilt.write(tiltHoriz);
  delay(500);
  tilt.write(tiltVertical);
  delay(500);
  tilt.write(tiltHoriz);
  delay(500);
  tilt.write(tiltVertical);
 
 
}

