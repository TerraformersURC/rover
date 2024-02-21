const byte numChars = 50;
char receivedChars[numChars];
char tempChars[numChars]; //temporary array for use when parsing
String incoming_message;
boolean newData = false;

void setup() {
    Serial.begin(115200);
    delay(10000); // Allow ESCs to recognize signal and settle down
}

void loop() {
    recvWithStartEndMarkers();
    
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseDataSTRING();
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

        else if (rc == startMarker) { recvInProgress = true;}
    }
}

// split the data into its parts
void parseDataINTEGER() {
    char * strtokIndx; // this is used by strtok() as an index
    strtokIndx = strtok(tempChars, ",");
    incoming_message = atoi(strtokIndx); 

}

void parseDataSTRING() {
    incoming_message = receivedChars;
}


void sendData(){
  Serial.println(incoming_message);
}
