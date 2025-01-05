#ifndef __GPIO_H__
#define __GPIO_H__

#include <JetsonGPIO.h>
#include <iostream>
#include <atomic>

class Gpio
{
private:
public:
    // const unsigned short SwitchPort = 443;
    std::string SwitchPort = "GPIO07";

    std::atomic<int> SwitchVal; // gpio7为 gpiochip1 PR.00 443
    Gpio(/* args */) {};
    ~Gpio() {};
    void GpioInit();
    int gpio_read(const unsigned short channel)
    {
        return GPIO::input(channel);
    }
    int gpio_read(std::string channel)
    {
        return GPIO::input(channel);
    }
};

#endif /* __GPIO_H__ */
