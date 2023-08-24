#include <Arduino.h>
#include "bouton.h"

// constructeur de Bouton
Bouton::Bouton(int pin_bouton){
    this->pin_bouton=pin_bouton;
    pinMode(this->pin_bouton, INPUT_PULLUP);
}

// initialisation
void Bouton::setPinBouton(int pin_bouton){
    pinMode(this->pin_bouton,INPUT_PULLUP);
}

// entrée valeur (inversion par trigger cd40106)
bool Bouton::getEtatBouton(){
    return digitalRead(this->pin_bouton);
}
