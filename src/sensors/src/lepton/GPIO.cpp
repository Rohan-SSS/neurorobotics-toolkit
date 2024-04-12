#include "sensors/lepton/GPIO.h"

using namespace std;

GPIO::GPIO(const int PIN, int mode)
{
    GPIOPin = 328 + PIN;
    modes = mode;
    DirectionPath = GPIO_PATH + to_string(GPIOPin) + "/direction";
    valuePath = GPIO_PATH + to_string(GPIOPin) + "/value";
}
GPIO::~GPIO()
{
}
int GPIO::Initialize()
{
    exportFile.open(GPIO_EXPORT_PATH);
    if (!exportFile)
        return -1;
    exportFile << GPIOPin;
    exportFile.close();
    directionFile.open(DirectionPath);
    if (!directionFile)
        return -1;
    switch (modes)
    {
    case 0:
        directionFile << "in";
        break;
    default:
        directionFile << "out";
        break;
    }
    directionFile.close();

    return 0;
}

int GPIO::Write(int val)
{
    valueFile.open(valuePath);
    if (!valueFile)
        return -1;
    valueFile << val;
    valueFile.close();
    return 0;
}

int GPIO::Read()
{
    ifstream temp(valuePath, std::ios::app);
    if (!temp)
        return -1;
    temp >> readData;
    return readData;
}
