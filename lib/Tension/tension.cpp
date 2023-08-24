#include <Arduino.h>
#include "tension.h"

// constructeur de Tension
Tension::Tension(int pin_tension){
    this->pin_tension=pin_tension;
    this->coefTension=3.79;                      // Ratio = (10k+3k56)/3k56
    this->limiteTension=10.00;                   // LiPo 3S
    pinMode(this->pin_tension, INPUT_PULLDOWN);
}

// initialisation
void Tension::setPinTension(int pin_tension){
    pinMode(this->pin_tension, INPUT_PULLDOWN);
}

// tension mesurée directement en mV
float Tension::mesureTension(){
    valeurTension=analogReadMilliVolts(this->pin_tension);
    return (valeurTension*coefTension/1000.00);
}

// alerte tension si inférieure à la limite
bool Tension::basseTension(){
    limiteMilliVolt=1000.00*limiteTension/coefTension;
    if(analogReadMilliVolts(this->pin_tension)<limiteMilliVolt){
        return true;}
    else{
        return false;}
}
