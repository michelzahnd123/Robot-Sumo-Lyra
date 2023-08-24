# include <Arduino.h>
# include "led.h"

// Constructeur de LED
Led::Led(int pin_led){
    this->pin_led=pin_led;
    this->tempsFlash=50;
    pinMode(this->pin_led, OUTPUT);
    digitalWrite(this->pin_led, LOW);
}

// Initialisation en sortie
void Led::setPinLed(int pin_led){
    pinMode(this->pin_led, OUTPUT);
}

// Récupération de l'état de la LED
bool Led::getPinLed(){
    return digitalRead(this->pin_led);
}

// Allumage
void Led::on(){
    digitalWrite(this->pin_led, HIGH);
}

// Extinction
void Led::off(){
    digitalWrite(this->pin_led, LOW);
}

// Changement d'état
void Led::toggle(){
    digitalWrite(this->pin_led, !getPinLed());
}

// Flashes lumineux pendant 1 seconde
void Led::flashLumineux(int nombreFlashes, long dureeTotaleCycle){
    this->nombreFlash=nombreFlashes;
    if(nombreFlashes<1){nombreFlashes=1;}
    if(nombreFlashes>10){nombreFlashes=10;}
    
    for(int iFlash=0; iFlash<nombreFlashes; iFlash++){
        digitalWrite(this->pin_led, HIGH);
        delay(tempsFlash);
        digitalWrite(this->pin_led, LOW);
        delay(tempsFlash);
    }
    tempsEteint=int(dureeTotaleCycle-2*nombreFlashes*tempsFlash);
    delay(tempsEteint);
}

// Impulsion 100 ms (50ms : allumé + 50 ms : éteint)
void Led::impulsion(bool attendEteint){
    this->attendEteint=attendEteint;
    digitalWrite(this->pin_led, HIGH);
    delay(tempsFlash);
    digitalWrite(this->pin_led, LOW);
    if(attendEteint){delay(tempsFlash);}
}
