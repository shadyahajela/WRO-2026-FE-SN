/*==============================================================================
  WRO Future Engineers 2026
  Arduino Nano Firmware

  Purpose:
  Low-level hardware controller for the robot.

  Receives commands from Raspberry Pi:

      steering speed direction lineCount state

  Example:

      100 200 1 4 STR
      120 160 1 5 TUR

  Raspberry Pi ---> Arduino
      steering angle
      motor speed
      driving direction
      detected line count
      competition state

  Arduino:
      - Controls steering servo
      - Controls DC motor through L298N
      - Displays telemetry on OLED
      - Controls NeoPixel status LEDs

==============================================================================*/

//------------------------------------------------------------------------------
// LIBRARIES
//------------------------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <Servo.h>
#include <stdio.h>
#include <string.h>

//------------------------------------------------------------------------------
// HARDWARE PIN DEFINITIONS
//------------------------------------------------------------------------------

// Steering servo
#define SERVO_PIN 2

// L298N motor driver
#define ENABLEMOT 3
#define MOTOR1 10
#define MOTOR2 11

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

//------------------------------------------------------------------------------
// SERVO LIMITS
//------------------------------------------------------------------------------

#define SERVO_CENTER 108
#define SERVO_RANGE 60

//------------------------------------------------------------------------------
// SERIAL BUFFER
//------------------------------------------------------------------------------

#define BUFFER_SIZE 16

char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;

int servo_turn = SERVO_CENTER;
int speed = 0;
int dir = 0;
int line_count = 0;
char state[5];

//------------------------------------------------------------------------------
// HARDWARE OBJECTS
//------------------------------------------------------------------------------

Adafruit_SH1106G screen(
 SCREEN_WIDTH,
 SCREEN_HEIGHT,
 &Wire,
 OLED_RESET
);

Servo myservo;

//------------------------------------------------------------------------------
// SETUP
//------------------------------------------------------------------------------

void setup() {

  // Motor pins
  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(ENABLEMOT, OUTPUT);

  // Make sure motor is stopped at startup
  analogWrite(ENABLEMOT, 0);
  digitalWrite(MOTOR1, LOW);
  digitalWrite(MOTOR2, LOW);

  // Serial communication with Raspberry Pi 
  // Must match Raspberry Pi baud rate.
  Serial.begin(19200);  
  
  // Attach steering servo
  myservo.attach(SERVO_PIN);

  //--------------------------------------------------------------------------
  // OLED INITIALIZATION
  //--------------------------------------------------------------------------

  screen.begin(0x3C, true);

  screen.clearDisplay();
  screen.setTextColor(SH110X_WHITE);
  screen.setTextSize(3);
  screen.setCursor(0, 24);
  screen.print("BOOTING");
  screen.display();

  //--------------------------------------------------------------------------
  // SERVO SELF-TEST AND CENTER
  //--------------------------------------------------------------------------

  myservo.write(SERVO_CENTER-45);
  delay(500);

  myservo.write(SERVO_CENTER);
  delay(300);

  //--------------------------------------------------------------------------
  // READY SCREEN
  //--------------------------------------------------------------------------

  screen.clearDisplay();
  screen.setTextColor(SH110X_WHITE);
  screen.setTextSize(3);
  screen.setCursor(0, 24);
  screen.print("-READY-");
  screen.display();

}


//------------------------------------------------------------------------------
// MAIN LOOP
//------------------------------------------------------------------------------

void loop() {

  /*
     Receive serial data one character at a time.

     The Raspberry Pi sends:

        100 200 1 4 STR\n

     We keep collecting characters until '\n' is received.

     This is more reliable for a real-time robot than
     Serial.readBytesUntil().
  */


  while (Serial.available() > 0) {
  
      char c = Serial.read();
  
      if (c == '\n') {
  
          inputBuffer[bufferIndex] = '\0';
          bufferIndex = 0;
  
          processData(inputBuffer);
      }
      else if (bufferIndex < BUFFER_SIZE - 1) {
  
          inputBuffer[bufferIndex++] = c;
      }
  }
}


//------------------------------------------------------------------------------
// PROCESS SERIAL DATA
//------------------------------------------------------------------------------

void processData(char receivedData[]) {

  /*
     Expected message:

        steering speed direction lineCount state

     Example:

        100 200 1 4 STR

     Values:

        steering
            Servo angle.
            Normally 60-140 for your robot.

        speed
            Motor PWM value.
            0-255.

        direction
            0 = stop
            1 = forward
            2 = reverse

        lineCount
            Number of detected lines.

        state
            STR = STRAIGHT
            TUR = TURNING
  */


  char header;

  //----------------------------------------------------------------------
  // PARSE HEADER
  //----------------------------------------------------------------------
  
  int parsed = sscanf(
      receivedData,
      " %c",
      &header
  );
  
  
  //----------------------------------------------------------------------
  // INVALID HEADER
  //----------------------------------------------------------------------
  
  if (parsed != 1) {
      return;
  }
  
  
  //----------------------------------------------------------------------
  // HANDLE MESSAGE TYPE
  //----------------------------------------------------------------------
  
  switch (header) {
  
      //==================================================================
      // $ = FULL MESSAGE
      // Expected:
      // $ servo_turn speed dir line_count state
      //==================================================================
  
      case '$':

          // parsed = sscanf(
          //     receivedData,
          //     " $ %d %d %d",
          //     &servo_turn, 
          //     &speed,
          //     &dir
          //     );
  
         parsed = sscanf(
             receivedData,
             " $ %d %d %d %d %4s",
             &servo_turn, &speed
             &speed,
             &dir,
             &line_count,
             state
         );
  
          // Make sure ALL 5 values were received
          if (parsed != 5) {
              return;
          }
  
  
          //------------------------------------------------------------------
          // SAFETY LIMITS
          //------------------------------------------------------------------
  
          servo_turn = constrain(
              servo_turn,
              SERVO_CENTER - SERVO_RANGE,
              SERVO_CENTER + SERVO_RANGE
          );
  
          speed = constrain(
              speed,
              0,
              255
          );
   
          break;
  
      default:
          return;
  }
  
  
  //--------------------------------------------------------------------------
  // OLED TELEMETRY
  //--------------------------------------------------------------------------

  myservo.write(servo_turn);

  driveMotor(speed,dir);
 
  screen.clearDisplay();
 
  screen.setTextColor(SH110X_WHITE);
  
  // Servo
  screen.setTextSize(2);
  screen.setCursor(2, 2);
  screen.print(servo_turn);
  
   //Separator
  screen.drawFastVLine(
    40,
    0,
    20,
    SH110X_WHITE
  );
 
  // Line count
  screen.setCursor(44, 2);
  screen.print(line_count);
 
  //Separator
  screen.drawFastVLine(
    70,
    0,
    20,
    SH110X_WHITE
  );
 
  // Speed
  screen.setCursor(75, 2);
  screen.print(speed);
  
  //Horizontal separator
  screen.drawFastHLine(
    0,
    20,
    128,
    SH110X_WHITE
  );
 
  // State
  screen.setTextSize(3);
  screen.setCursor(0, 30);
  screen.print(state);
 
  //Motor direction
  screen.drawFastVLine(
    80,
    20,
    44,
    SH110X_WHITE
  );
 
  //--------------------------------------------------------------------------
  // UPDATE OLED ONCE
  //--------------------------------------------------------------------------
 
  screen.display();
  
}


//------------------------------------------------------------------------------
// MOTOR CONTROL
//------------------------------------------------------------------------------

void driveMotor(int speed, int dir) {

  /*
     L298N motor control:

        dir = 0
            STOP

        dir = 1
            FORWARD

        dir = 2
            REVERSE
  */

  screen.setTextSize(3);
  screen.setCursor(90, 30);

  //--------------------------------------------------------------------------
  // FORWARD
  //--------------------------------------------------------------------------

  if (dir == 1) {

    analogWrite(
      ENABLEMOT,
      speed
    );

    digitalWrite(
      MOTOR1,
      LOW
    );

    digitalWrite(
      MOTOR2,
      HIGH
    );

    screen.print(">>");
  }


  //--------------------------------------------------------------------------
  // REVERSE
  //--------------------------------------------------------------------------

  else if (dir == 2) {

    analogWrite(
      ENABLEMOT,
      speed
    );

    digitalWrite(
      MOTOR1,
      HIGH
    );

    digitalWrite(
      MOTOR2,
      LOW
    );

    screen.print("<<");
  }


  //--------------------------------------------------------------------------
  // STOP
  //--------------------------------------------------------------------------

  else {

    analogWrite(
      ENABLEMOT,
      0
    );

    digitalWrite(
      MOTOR1,
      LOW
    );

    digitalWrite(
      MOTOR2,
      LOW
    );

    screen.print("XX");
  }

  // IMPORTANT:
  // OLED is NOT updated here.
  //
  // processData() calls screen.display()
  // once after all telemetry is drawn.
}
