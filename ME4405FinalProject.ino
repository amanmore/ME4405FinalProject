/*
 ME 4405 Final Project Code
 
 Uses 2 Line sensors, 4 ultrasonic sensors, 2 encoders, and 2 motors to run a sumobot.
 
 */

//defining pins
int pushButton1 = PUSH1;
int pushButton2 = PUSH2;

int led1 = 78;
int redled = RED_LED;
int greenled = GREEN_LED;
int blueled = BLUE_LED;

int lineFront = 29;//5.4
int lineBack = 30;//5.4

int ultraLLPulse = 34;//2.3
int ultraLL = 38;//2.4

int ultraLCPulse = 31;//3.7
int ultraLC = 19;//2.5

int ultraRCPulse = 35;//6.7
int ultraRC = 39;//2.6

int ultraRRPulse = 28;//4.7
int ultraRR = 40;//2.7

int encRight = 3;//3.2
int encLeft = 4;//3.3

int lMotor1 = 37;//5.6
int lMotor2 = 36;//6.6

int rMotor1 = 32;//3.5
int rMotor2 = 11;//3.6

//defining variables
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
int inFront = 15; // [choose this one] distance at which object is considered immediately in front
int alignmentDiff = 5; // [choose this one] max allowable difference between 2 sensors

int encRval; // Right Encoder ticks
int encRdir; // " direction

int encLval; // Left encoder ticks
int encLdir; // " direction

bool hasMoved = false;
bool rMoved = false;
bool lMoved = false;
int moveLPin;
int moveRPin;

long SonarSensor(int trigPin, int echoPin) { // measure sensor value
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    long distance = (duration / 2) / 29.1;
    if(distance>=1 && distance<=60){
        return distance;
    } else return 999;
}

void moveMotors(int speedL, int speedR) {
    if (speedL<0) {
        speedL = abs(speedL);
        encLdir = 0;
        moveLPin = lMotor1;
    } else if (speedL>=0) {
        speedL = abs(speedL);
        encLdir = 1;
        moveLPin = lMotor2;
    }
    if (speedR<0) {
        speedR = abs(speedR);
        encRdir = 0;
        moveRPin = rMotor1;
    } else if (speedR>=0) {
        speedR = abs(speedR);
        encRdir = 1;
        moveRPin = rMotor2;
    }
    analogWrite(moveRPin, speedR);
    analogWrite(moveLPin, speedL);
}

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
    pinMode(ultraLLPulse, OUTPUT);
    pinMode(ultraLCPulse, OUTPUT);
    pinMode(ultraRCPulse, OUTPUT);
    pinMode(ultraRRPulse, OUTPUT);
    pinMode(ultraLL, INPUT);
    pinMode(ultraLC, INPUT);
    pinMode(ultraRC, INPUT);
    pinMode(ultraRR, INPUT);
    pinMode(encRight, INPUT);
    pinMode(encLeft, INPUT);
}

void loop() {
    // check state of line sensors
    lineFval = 1023;//analogRead(lineFront);
    lineBval = 1023;//analogRead(lineBack);
    //  Serial.print("Line 1: ");
    //  Serial.println(lineFval);
    //  Serial.print("Line 2: ");
    //  Serial.println(lineBval);
    
    if (lineFval > lineBlack && lineBval > lineBlack) { // neither sense black
        
        // Measuring distances
        ultraLCval = SonarSensor(ultraLCPulse, ultraLC);
        delayMicroseconds(33000);
        //ultraRRval = SonarSensor(ultraRRPulse, ultraRR);
        delayMicroseconds(33000);
        //ultraLLval = SonarSensor(ultraLLPulse, ultraLL);
        delayMicroseconds(33000);
        ultraRCval = SonarSensor(ultraRCPulse, ultraRC);
        delayMicroseconds(33000);
        
        if (ultraLCval == 999 && ultraRCval != 999) {
            ultraLCval = ultraRCval;
        }
        if (ultraRCval == 999 && ultraLCval != 999) {
            ultraRCval = ultraLCval;
        }
        
        if (ultraRCval != 999 && ultraLCval != 999) {
            
            //Serial.print("LC: ");
            //Serial.print(ultraLCval);
            //Serial.print("; RC: ");
            //Serial.println(ultraRCval);
            
            if (ultraLCval <= inFront && ultraRCval <= inFront) { //check if target is right in front and aligned
                //push forward
                Serial.println("Full Forward");
            }
            else if (ultraLCval <= inFront && (ultraRCval > inFront && ultraRCval <= inRange)) { //check if target is right in front, but misaligned
                //turn left slowly
                Serial.println("left");
            }
            else if ((ultraLCval > inFront && ultraLCval <= inRange) && ultraRCval <= inFront) { //check if target is right in front, but misaligned
                //turn right slowly
                Serial.println("right");
            }
            else if (ultraLCval <= inRange && ultraRCval <= inRange) { //check if object is somewhere in front
                long valDiff = abs(ultraLCval - ultraRCval);
                if (valDiff <= alignmentDiff) { //if directly in front, but far
                    //go forward
                    Serial.println("Forward");
                }
                else if (valDiff > alignmentDiff && ultraLCval > ultraRCval) { // if in front-right
                    //turn slighly right while going forward
                    Serial.println("right Forward");
                }
                else if (valDiff > alignmentDiff && ultraRCval > ultraLCval) { // if in front left
                    //turn slighly left while going forward
                    Serial.println("left Forward");
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
    delay(50);
}

