/*
  ME 4405 Final Project Code

  Uses 2 Line sensors, 4 ultrasonic sensors, 2 encoders, and 2 motors to run a sumobot.

*/

//defining pins
int pushButton1 = PUSH1;
int pushButton2 = PUSH2;
int led1 = LED1;
int redled = RED_LED;
int greenled = GREEN_LED;
int blueled = BLUE_LED;
int lineFront = 29;//5.4
int lineBack = 30;//5.4
int ultraPulse = 34;//2.3
int ultraLL = 38;//2.4
int ultraLC = 19;//2.5
int ultraRC = 39;//2.6
int ultraRR = 40;//2.7
int encRight = 3;//3.2
int encLeft = 4;//3.3
int lMotor1 = 37;//5.6
int lMotor2 = 36;//6.6
int lMotor3 = 35;//6.7
int rMotor1 = 32;//3.5
int rMotor2 = 11;//3.6
int rMotor3 = 31;//3.7

//definibg variables
int lineFval; //Front Line Sensor Value
int lineBval; //Back "
int lineBlack = 256; // [choose this one] highest value that can be considered black
int lineGray = 512; // [choose this one] highest value that can be considered gray
int lineWhite = 1023; // [choose this one] highest value that can be considered white

long ultraLLval; // Leftmost Ultrasonic Sensor distance
long ultraLCval; // Centre-Left "
long ultraRCval; // Rentre-Right "
long ultraRRval; // Rightmost "
int inRange = 30; // [choose this one] distance at which object is considered important
int inFront = 2; // [choose this one] distance at which object is considered immediately in front
int alignmentDiff = 5; // [choose this one] max allowable difference between 2 sensors

int encRval; // Right Encoder ticks
int encRdeg; // " degrees
int encRdir; // " direction

int encLval; // Left encoder ticks
int encLdeg; // " degrees
int encLdir; // " direction

void setup() {
  // UART at 9600
  Serial.begin(9600);
  //setup pins
  pinMode(pushButton1, INPUT_PULLUP);
  pinMode(pushButton2, INPUT_PULLUP);
  pinMode(led1, OUTPUT);
  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(blueled, OUTPUT);
  pinMode(ultraPulse, OUTPUT);
  pinMode(ultraLL, INPUT);
  pinMode(ultraLC, INPUT);
  pinMode(ultraRC, INPUT);
  pinMode(ultraRR, INPUT);
  pinMode(encRight, INPUT);
  pinMode(encLeft, INPUT);
}

void loop() {
  // check state of line sensors
  lineFval = analogRead(lineFront);
  lineBval = analogRead(lineBack);
  //  Serial.print("Line 1: ");
  //  Serial.println(lineFval);
  //  Serial.print("Line 2: ");
  //  Serial.println(lineBval);

  if (lineFval > lineBlack && lineBval > lineBlack) { // neither sense black

    // Measuring distances
    ultraLLval = SonarSensor(ultraPulse, ultraLL);
    delayMicroseconds(10);
    ultraLCval = SonarSensor(ultraPulse, ultraLC);
    delayMicroseconds(10);
    ultraRCval = SonarSensor(ultraPulse, ultraRC);
    delayMicroseconds(10);
    ultraRRval = SonarSensor(ultraPulse, ultraRR);

    if (ultraLCval <= inFront && ultraRCval <= inFront) { //check if target is right in front and aligned
      //push forward
    }
    else if (ultraLCval <= inFront && (ultraRCval > inFront && ultraRCval <= inRange)) { //check if target is right in front, but misaligned
      //turn left slowly
    }
    else if ((ultraLCval > inFront && ultraLCval <= inRange) && ultraRCval <= inFront) { //check if target is right in front, but misaligned
      //turn right slowly
    }
    else if (ultraLCval <= inRange && ultraRCval <= inRange) { //check if object is somewhere in front
      long valDiff = abs(ultraLCval - ultraRCval);
      if (valDiff <= alignmentDiff) { //if directly in front, but far
        //go forward
      }
      else if (valDiff > alignmentDiff && ultraLCval > ultraRCval) { // if in front-right
        //turn slighly right while going forward
      }
      else if (valDiff > alignmentDiff && ultraRCval > ultraLCval) { // if in front left
        //turn slighly left while going forward
      }
    }
    else if (ultraLCval <= inRange && ultraRCval > inRange) { //seen by LC, not by RC
      //turn left
    }
    else if (ultraLCval > inRange && ultraRCval <= inRange) { //seen by RC, not by LC
      //turn right
    }
    else if (ultraLCval > inRange && ultraRCval > inRange) { // unseen by LC and RC
      if (ultraLLval <= inRange) { //found, to the left
        //turn left on the spot
      }
      else if (ultraRRval <= inRange) { //found, to the right
        //turn right on the spot
      }
      else if (ultraLLval > inRange && ultraRRval > inRange) { //completely unseen
        //search algorithm
      }
    }
  }

  else if (lineFval <= lineBlack && lineBval > lineBlack) { // front sensor on black
    // go backwards until X happens
  }

  else if (lineFval > lineBlack && lineBval <= lineBlack) { // back sensor on black
    // go forwards until X happens
  }

  else if (lineFval <= lineBlack && lineBval <= lineBlack) { //both sensors on black
    // TBD
    Serial.println("Damn, Daniel. Back at it again with the horrible positioning");
  }

}

long SonarSensor(int trigPin, int echoPin) { // measure sensor value
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration / 2) / 29.1;
  return distance;
}