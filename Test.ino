
// Example 4 - Receive a number as text and convert it to an int
#include <Servo.h>
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;
int ar = 0;
int a = 0;
int b = 0;
int dataNumber = 0;             // new for this version
Servo servo1;
Servo servo2;
void setup() {
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");     // Setting up Arduino for serial communication
    servo1.attach(9);
    servo2.attach(11);
    pinMode(13,OUTPUT);
}

void loop() {
    recvWithEndMarker();
    showNewNumber();
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void showNewNumber() {
    if (newData == true) {
        dataNumber = 0;             // new for this version
        dataNumber = atoi(receivedChars);   // new for this version
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        Serial.print("Data as Number ... ");    // new for this version
        Serial.println(dataNumber);     // new for this version

        String RD = String(receivedChars);
        String Ar = RD.substring(0,1);
        String A = RD.substring(1,4);
        String B = RD.substring(4,7);

        int ar = Ar.toInt();
        int a = A.toInt();
        int b = B.toInt();

        Serial.println(ar);
        Serial.println(a);
        Serial.println(b);

        if (ar)
          digitalWrite(13,1);
        servo1.write(a);
        servo2.write(b);
        newData = false;
    }
}
