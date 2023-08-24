#include <Arduino.h>
#include "noirblanc.h"

// constructeur de NoirBlanc
NoirBlanc::NoirBlanc(int pin_noirblanc){
    this->pin_noirblanc=pin_noirblanc;
    this->etatCNY=etatCNY;
    pinMode(this->pin_noirblanc, INPUT);
}

// initialisation
void NoirBlanc::setPinNoirBlanc(int pin_noirblanc){
    pinMode(this->pin_noirblanc, INPUT);
}

// entrée valeur "noir=2.7 V" ou "blanc=0.2 V"
// trigger-inverseur : noir = 0 (tout va bien) et blanc = 1 (alerte)
bool NoirBlanc::getEtatNoirBlanc(){              // alerte si BLANC
    etatCNY=digitalRead(this->pin_noirblanc);
    delayMicroseconds(250);                      // durant 250 us
    etatCNY=etatCNY&&digitalRead(this->pin_noirblanc);
    return etatCNY;
}
