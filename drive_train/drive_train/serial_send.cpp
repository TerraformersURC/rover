#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

int main() {
    const char* portName = "/dev/ttyACM0"; // Replace with the appropriate serial port
    int serialPort = open(portName, O_WRONLY | O_NOCTTY);

    if (serialPort == -1) {
        std::cerr << "Error opening serial port" << std::endl;
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serialPort, &tty) != 0) {
        std::cerr << "Error getting serial port attributes" << std::endl;
        close(serialPort);
        return 1;
    }

    cfsetospeed(&tty, B115200); // Set baud rate to match your Arduino configuration
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial port attributes" << std::endl;
        close(serialPort);
        return 1;
    }

    const char* data = "<25, 25, 50, 50>";
    int bytesWritten = write(serialPort, data, strlen(data));
    

    if (bytesWritten == -1) {
        std::cerr << "Error writing to serial port" << std::endl;
        close(serialPort);
        return 1;
    }

    close(serialPort);

    std::cout << "Data sent successfully!" << std::endl;

    return 0;
}