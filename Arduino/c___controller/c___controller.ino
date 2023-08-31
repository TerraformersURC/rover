#include <Wire.h>

const int I2C_ADDRESS = 8;  // I2C address of the Arduino

void setup()
{
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveData);
    // Initialize other components
}

void loop()
{
    // Loop logic
}

void receiveData(int byteCount)
{
    if (byteCount >= 2) {
        int receivedInt = Wire.read() << 8 | Wire.read();

        // Perform control actions based on receivedInt
        // For example, map receivedInt to motor control values
        int motorSpeed = map(receivedInt, 0, 1000, 0, 255);
        analogWrite(MOTOR_PIN, motorSpeed);

        // You can implement your own control logic here
    }
}
