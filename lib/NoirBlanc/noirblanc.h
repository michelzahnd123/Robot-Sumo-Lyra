#ifndef NOIRBLANC_H
#define NOIRBLANC_H

class NoirBlanc{
private:
    int pin_noirblanc;                      // pin d'entrée
    bool etatCNY;
public:
    NoirBlanc(int pin_noirblanc);
    void setPinNoirBlanc(int pin_noirblanc);
    bool getEtatNoirBlanc();
};

#endif
