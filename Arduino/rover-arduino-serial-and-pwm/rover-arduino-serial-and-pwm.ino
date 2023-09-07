#include <Servo.h>

byte aPin = 40;
byte bPin = 42;
byte cPin = 46;
byte dPin = 44;

Servo motorA;
Servo motorB;
Servo motorC;
Servo motorD;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; //temporary array for use when parsing

//hold the parsed data
int aVal = 0;
int bVal = 0;
int cVal = 0;
int dVal = 0;

boolean newData = false;

void setup() {
    Serial.begin(115200);
    motorA.attach(aPin);
    motorB.attach(bPin);
    motorC.attach(cPin);
    motorD.attach(dPin);

    // Send "stop" signal
    motorA.writeMicroseconds(1500);
    motorB.writeMicroseconds(1500);
    motorC.writeMicroseconds(1500);
    motorD.writeMicroseconds(1500);

    delay(10000); // Allow ESCs to recognize signal and settle down
}

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        //showParsedData();
        sendData();
        newData = false;
    }
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// split the data into its parts
void parseData() {
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,","); // get the first part
    aVal = atoi(strtokIndx);
 
    strtokIndx = strtok(NULL, ","); // continue where it left off
    bVal = atoi(strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    cVal = atoi(strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    dVal = atoi(strtokIndx);

}

void sendData(){
  // Send signals to ESCs
  motorA.writeMicroseconds(aVal); 
  motorB.writeMicroseconds(bVal); 
  motorC.writeMicroseconds(cVal); 
  motorD.writeMicroseconds(dVal);
  Serial.print(aVal);
}

//void showParsedData() {
//    Serial.print("MotorA ");
//    Serial.println(motorA);
//    Serial.print("MotorB ");
//    Serial.println(motorB);
//    Serial.print("MotorC ");
//    Serial.println(motorC);
//    Serial.print("MotorD ");
//    Serial.println(motorD);
//}
