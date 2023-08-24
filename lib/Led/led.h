#ifndef LED_H
#define LED_H

class Led{
private:
    int pin_led;
    int nombreFlash;
    int tempsFlash;
    int tempsEteint;
    bool attendEteint;
public:
    Led(int pin_led);
    void setPinLed(int pin_led);
    bool getPinLed();
    void on();
    void off();
    void toggle();
    void flashLumineux(int nombreFlash, long dureeTotaleCycle);
    void impulsion(bool attendEteint);
};

#endif
