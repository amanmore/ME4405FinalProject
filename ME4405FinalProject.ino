/*
 ME 4405 Final Project Code
 
 Uses 2 Line sensors, 4 ultrasonic sensors, 2 encoders, and 2 motors to run a sumobot.
 
 */

//defining pins
int pushButton1 = PUSH1;
int pushButton2 = PUSH2;

int attempt = 0;
int prevAttempt = 0;

int led1 = 78;
int redled = RED_LED;
int greenled = GREEN_LED;
int blueled = BLUE_LED;

int lineFront = 29;//5.4
int lineBack = 30;//5.5

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

int lMotor2 = 37;//5.6
int lMotor1 = 36;//6.6

int rMotor2 = 32;//3.5
int rMotor1 = 11;//3.6

//defining variables
int lineFval; //Front Line Sensor Value
int lineBval; //Back "

int line1Black = 480; // [choose this one] highest value that can be considered black
int line1Gray = 420; // [choose this one] highest value that can be considered gray
int line1White = 395; // [choose this one] highest value that can be considered white

int line2Black = 590; // [choose this one] highest value that can be considered black
int line2Gray = 570; // [choose this one] highest value that can be considered gray
int line2White = 565; // [choose this one] highest value that can be considered white

long ultraLLval; // Leftmost Ultrasonic Sensor distance
long ultraLCval; // Centre-Left "
long ultraRCval; // Rentre-Right "
long ultraRRval; // Rightmost "
int inRange = 60; // [choose this one] distance at which object is considered important
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
int stopLPin;
int stopRPin;

int alignBaseSpeed = 80;
int lastDiff = 0;
unsigned long now = 0;
unsigned long lastTime = 0;
long changeRate = 0;
int kP = 1;
int kD = 1;

long SonarSensor(int trigPin, int echoPin) { // measure sensor value
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    long distance = (duration / 2) / 29.1;
    if (distance >= 1 && distance <= inRange) {
        return distance;
    } else return 999;
}

void moveMotors(int speedL, int speedR) {
    if (speedL < 0) {
        speedL = abs(speedL);
        encLdir = 0;
        moveLPin = lMotor1;
        stopLPin = lMotor2;
    } else if (speedL >= 0) {
        speedL = abs(speedL);
        encLdir = 1;
        moveLPin = lMotor2;
        stopLPin = lMotor1;
    }
    if (speedR < 0) {
        speedR = abs(speedR);
        encRdir = 0;
        moveRPin = rMotor1;
        stopRPin = rMotor2;
    } else if (speedR >= 0) {
        speedR = abs(speedR);
        encRdir = 1;
        moveRPin = rMotor2;
        stopRPin = rMotor1;
    }
    analogWrite(stopRPin, 0);
    analogWrite(stopLPin, 0);
    analogWrite(moveRPin, speedR);
    analogWrite(moveLPin, speedL);
//    Serial.print("Left Motor: ");
//    Serial.print(speedL);
//    Serial.print("Right Motor: ");
//    Serial.println(speedR);
}

void alignTarget() {
    int difference = ultraLCval - ultraRCval;
    now = millis();
    changeRate = (difference - lastDiff) / (now - lastTime);
    lastDiff = difference;
    lastTime = now;
    int alignSpeedAdd = kP * (difference) - kD * changeRate;
    if (alignSpeedAdd > 105) {
        alignSpeedAdd = (alignSpeedAdd / 105) / abs(alignSpeedAdd / 105);
    }
    if (alignSpeedAdd >= 0) {
        moveMotors((alignBaseSpeed + alignSpeedAdd), alignBaseSpeed);
    } else if (alignSpeedAdd < 0) {
        moveMotors(alignBaseSpeed, (alignBaseSpeed + abs(alignSpeedAdd)));
    }
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

//void loop() {
//    lineFval = analogRead(lineFront);
//    lineBval = analogRead(lineBack);
//    Serial.print("Line 1: ");
//    Serial.println(lineFval);
//    Serial.print("Line 2: ");
//    Serial.println(lineBval);
//    delay(100);
//}

//void loop() {
//    moveMotors(200,200);
//    delay(500);
//    moveMotors(0,0);
//    delay(500);
////    moveMotors(-200,-200);
////    delay(500);
////    moveMotors(0,0);
////    delay(500);
//}

void loop() {
    // check state of line sensors
    lineFval = analogRead(lineFront);
    lineBval = analogRead(lineBack);
//    Serial.print("Line 1: ");
//    Serial.print(lineFval);
//    Serial.print("; Line 2: ");
//    Serial.println(lineBval);
//    digitalWrite(led1, LOW);
//    digitalWrite(redled, LOW);
//    digitalWrite(greenled, LOW);
//    digitalWrite(blueled, LOW);

    if (lineFval < line1Black && lineBval < line2Black) { // neither sense black

        // Measuring distances
        ultraLCval = SonarSensor(ultraLCPulse, ultraLC);
        delayMicroseconds(33000);
        ultraRRval = SonarSensor(ultraRRPulse, ultraRR);
        delayMicroseconds(33000);
        ultraLLval = SonarSensor(ultraLLPulse, ultraLL);
        delayMicroseconds(33000);
        ultraRCval = SonarSensor(ultraRCPulse, ultraRC);
        delayMicroseconds(33000);

        if (ultraLCval == 999 && ultraRCval != 999) {
            ultraLCval = ultraRCval;
        }
        if (ultraRCval == 999 && ultraLCval != 999) {
            ultraRCval = ultraLCval;
        }
        
//        Serial.print("LL: ");
//        Serial.print(ultraLLval);
//        Serial.print("; LC: ");
//        Serial.print(ultraLCval);
//        Serial.print("; RC: ");
//        Serial.print(ultraRCval);
//        Serial.print("; RR: ");
//        Serial.println(ultraRRval);

        if (ultraRCval <= inRange && ultraLCval <= inRange) {

            if ((ultraLCval <= inFront && ultraRCval <= inFront) || (abs(ultraLCval - ultraRCval) <= alignmentDiff)) { //check if target is right in front and aligned
                //push forward
                attempt = 1;
                //Serial.println("Full Forward");
                moveMotors(100, 100);
                digitalWrite(led1, HIGH);
                digitalWrite(redled, LOW);
                digitalWrite(greenled, LOW);
                digitalWrite(blueled, LOW);
            }
            else {
                attempt = 2;
//                Serial.println("Aligning");
                digitalWrite(led1, LOW);
                digitalWrite(redled, HIGH);
                digitalWrite(greenled, LOW);
                digitalWrite(blueled, LOW);
                alignTarget();
            }
        } else if (ultraLCval > inRange && ultraRCval > inRange) { // unseen by LC and RC
            if (ultraLLval <= inRange) { //found, to the left
                //turn left on the spot
                attempt = 3;
//                Serial.println("Left");
                digitalWrite(led1, LOW);
                digitalWrite(redled, LOW);
                digitalWrite(greenled, HIGH);
                digitalWrite(blueled, LOW);
                moveMotors(-150, 150);
            }
            else if (ultraRRval <= inRange) { //found, to the right
                //turn right on the spot
                attempt = 4;
//                Serial.println("Right");
                digitalWrite(led1, LOW);
                digitalWrite(redled, LOW);
                digitalWrite(greenled, HIGH);
                digitalWrite(blueled, LOW);
                moveMotors(150, -150);
            }
            else if (ultraLLval > inRange && ultraRRval > inRange) { //completely unseen
                //search algorithm
                attempt = 5;
//                Serial.println("Searching");
                digitalWrite(led1, LOW);
                digitalWrite(redled, LOW);
                digitalWrite(greenled, LOW);
                digitalWrite(blueled, HIGH);
                moveMotors(0, 0);
            }
        }
    }

    else if (lineFval >= line1Black && lineBval < line2Black) { // front sensor on black
        // go backwards until X happens
        attempt = 6;
//        Serial.println("Line in front");
        digitalWrite(led1, HIGH);
        digitalWrite(redled, HIGH);
        digitalWrite(greenled, LOW);
        digitalWrite(blueled, LOW);
        while (lineFval >= line1White) {
            moveMotors(-100, -100);
            delay(50);
            lineFval = analogRead(lineFront);
        }
    }

    else if (lineFval < line1Black && lineBval >= line2Black) { // back sensor on black
        // go forwards until X happens
        attempt = 7;
//        Serial.println("Line at back");
        digitalWrite(led1, HIGH);
        digitalWrite(redled, LOW);
        digitalWrite(greenled, HIGH);
        digitalWrite(blueled, LOW);
        while (lineBval >= line2White) {
            moveMotors(100, 100);
            delay(50);
            lineBval = analogRead(lineBack);
        }
    }

    else if (lineFval >= line1Black && lineBval >= line2Black) { //both sensors on black
        // Return to the center
        attempt = 8;
//        Serial.println("Damn, Daniel. Back at it again with the horrible positioning");
        digitalWrite(led1, HIGH);
        digitalWrite(redled, LOW);
        digitalWrite(greenled, LOW);
        digitalWrite(blueled, HIGH);
        while (lineFval >= line1Black && lineFval >= line2Black) {
            moveMotors(-50, 50);
            delay(50);
            lineFval = analogRead(lineFront);
            lineBval = analogRead(lineBack);
        }
        if (lineFval < line1Black && lineFval >= line2Gray) {
            while (lineFval >= line1Gray) {
                moveMotors(-100, -100);
                delay(50);
                lineFval = analogRead(lineFront);
            }
        } else if (lineBval < line1Black && lineBval >= line2Gray) {
            while (lineBval >= line2Gray) {
                moveMotors(-100, -100);
                delay(50);
                lineBval = analogRead(lineBack);
            }
        }
    }
    if (attempt != prevAttempt) {
        switch (attempt) {
            case 1:
                Serial.println("Full Forward");
                break;
            case 2:
                Serial.println("Aligning");
                break;
            case 3:
                Serial.println("Left");
                break;
            case 4:
                Serial.println("Right");
                break;
            case 5:
                Serial.println("Searching");
                break;
            case 6:
                Serial.println("Line Front");
                break;
            case 7:
                Serial.println("Line Back");
                break;
            case 8:
                Serial.println("Damn, Daniel. Back at it again with the horrible positioning");
                break;
        }
    }
    prevAttempt = attempt;
    delay(50);
}

