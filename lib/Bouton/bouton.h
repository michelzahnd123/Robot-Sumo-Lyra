#ifndef BOUTON_H
#define BOUTON_H

class Bouton{
private:
    int pin_bouton;                   // pin d'entrée
public:
    Bouton(int pin_bouton);
    void setPinBouton(int pin_bouton);
    bool getEtatBouton();
};

#endif
