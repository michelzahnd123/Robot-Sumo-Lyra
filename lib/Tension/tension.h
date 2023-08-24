#ifndef TENSION_H
#define TENSION_H

class Tension{
private:
    int pin_tension;                    // mesure tension
    float valeurTension;
    float coefTension;
    float limiteTension;
    float limiteMilliVolt;
public:
    Tension(int pin_tension);
    void setPinTension(int pin_tension);
    float mesureTension();
    bool basseTension();
};

#endif
