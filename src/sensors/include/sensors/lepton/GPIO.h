#ifndef _GPIO_
#define _GPIO_
#include <fcntl.h>
#include <unistd.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fstream>

#define GPIO_EXPORT_PATH "/sys/class/gpio/export"
#define GPIO_PATH "/sys/class/gpio/gpio"

class GPIO
{
public:
    GPIO(const int PIN, int mode = 2);
    ~GPIO();
    int Initialize();
    int Write(int val);
    int Read();

private:
    int modes;
    std::string DirectionPath;
    std::string valuePath;
    int GPIOPin;
    int readData;
    std::ofstream exportFile;
    std::ofstream valueFile;
    std::ofstream directionFile;
};
#endif