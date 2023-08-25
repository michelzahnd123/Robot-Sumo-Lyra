//  -------------------------------------------------------
//  --------- programme robot SUMO 2023 -------------------
//  ------------------- LYRA version 1 --------------------
//  -------------------------------------------------------

/* liste de courses
  inversion des pins CNY70 (MC14490 non inverseur)
  inversion des boutons bGO et tON (MC14490 non inverseur)
  mesurer le temps réel de traitement interruption -> suppression ?
*/



// 25/8/2023 - REACTION si ligne blanche ARRIERE verifie_noirblanc(), reactionLigneBlanche()
// 24/8/2023 - GitHub remote https://github.com/michelzahnd123/Robot-Sumo-Lyra
//             non inversion des pins capteurs POLOLU avec filtre MC14490
// 22/8/2023 - réaffectation des pins /nouveau PCB - inversion des pins moteurs (PCB moteur)
//             suppression : capteur presence AR, capteurs TOF, librairie AdaFruit
//             création du capteur YR (CNY70 arrière : pin 18) 
// 9/8/2023  - LYRA : création du programme & séparation de DAEMON-2023-V3 (IOREK)

// 11/7/2023 - IOREK : fin 1ère seconde -> limite tout_avant 3*duree 1ère seconde vitesse M6
//             Démontage de LYRA et reconstruction au Club Robor de St Sébastien s/ Loire
// 10/6/2023 - concours THIONVILLE (version 3) : 19 gagnés - 8 nuls - 14 perdus
//             combats LYRA<>IOREK - IOREK gagne -> démontage de LYRA
//             pb : sort tout seul (12fois) - arrêt subit (6fois) - survitesse avec charge LiPo
// 8/6/2023 -  TEST : 1 mètre (65cm/s en vitesse M3)
//             charge des robots : 11.8V (maxi lisible par diviseur de tension)
//             +--------------------------------------------------------------------------+
// last day ...!   ligne 205 -> durée de combat pour compétition : 60sec                  !
//             !   ligne 225 -> constantes de temps : 1ère seconde & boucle de combat     !
//             !   ligne 265 -> constantes de temps : blocage & ligne blanche             !
//             !   ligne 275 -> constantes de temps : homologation 1 mètre : 1.6sec       !
//             !   ligne 1040 -> borne d'arrêt pour calibrage de la 1ère seconde          !
//             +--------------------------------------------------------------------------+
// 7/6/2023    copie du 6/6/23 - 20h00 -> LYRA & IOREK
//             TESTS : calibrage 1ère seconde
//             programme "parcous 1 mètre" pour homologation
//             arrivée sur ligne blanche + Il Est Derrière !!!
//             réduction vitesse de réaction : arrivé seul sur ligne blanche
// 6/6/2023 -  màj des mouvements : "1ère seconde", "détecté" et "perte contact"
//             FàF -> face-à-face = adversaire proche + mouvement 31 + attente blocage 3 secondes
//             simplification : oldPresenceX, apparuPresenceX, disparuPresenceX
//             présence AVEC/SANS parasite -> sélection des capteurs actifs SANS parasite
//             durée boucle de combat 3ms en limitant les Serial.print ...
// 5/6/2023 -  définition des "mouvements" / "capteurs" (ligne 750)
// 4/6/2023 -  lecture 30 cycles de 50 us pour filter les parasites
//             LYRA -> vérification des combinaisons possibles de N capteurs : 1, 2 ou 3
//             recherche ADV à la seconde 3
// 3/6/2023 -  recherche élimination des parasites des capteurs par programme
// 1/6/2023 -  INPUT (sans pullup) - mode CHANGE - 1 seule INTERRUPTION par PIN
//             Boucle Combat : affichage des capteurs -> 30 ms - pas d'affichage -> 1 ms
//             capteurs ARR : RG et RD très perturbés : moteurs, WiFi/ESP32 ?
//             analyse réponse capteur / oscilloscope !!!
//             modification IOREK & LYRA : C 100nF, R 3k, écran mu-métal
// 31/5/2023 - reprendre la prog des capteurs en INTERRUPTION
//             variable "volatile" initilisée à "false"
//             abandon de l'objet PRESENCE .h et .cpp au profit de l'INTERRUPTION
//             suppression oldPresence, debutPresence
//             interruption en place ... ne fonctionne pas !!!
// 30/5/2023 - temps de réponse des capteurs -> 1 à 10 ms : total boucle de combat 33 ms
//             latence (après lecture) des capteurs : 2-3 ms -> lecture maxi 3 ou 4
//             limite : DEVANT (AG+A+AD -> 17 ms) - AUTOUR (G+RG+RD+D -> 19 ms)
//             mesure courant RZ60S : 25mA sous 5V
//             IOREK : soudage de 8 condensateurs 10uF sur alim 5V des capteurs -> Not OK
// 29/5/2023 - temps de la boucle de combat et compensation à 25 ms (distance 25mm)
//             formule de correction du temps de réaction en fonction de la tension LiPo
//             (1ère seconde- réaction boucle de combat - ligne blanche - face-à-face)
// 28/5/2023 - coefficient tension : 3.79 (Lyro & Iorek) - presence : PULL-UP (JS40F & RZ60S)
//             suite du dernier mouvement (oldMouvement) lorsqu'on perd le contact avec l'adversaire
// 27/5/2023 - gestion du blocage Face à face (après 3 sec -> reculade : M6/M7)
// 24/5/2023 - remontage électronique de IOREK (nouvelle carte verte) -> OK
//             pièce EtageHaut : IOREK - Capsules RZ60S envoyées par SUMOZADE
// 23/5/2023 - verifie_tension() -> LOOP : initialisation + boucle de combat
//             vitesse moteur 400 rpm : LYRA + IOREK -> SetUp 
//             verifie_presence() -> lecture des 8 capteurs de présence
//             PresenceLoinPres() -> calcul de la direction de l'adversaire
//                   directionAdversaire - nbCapteurActif - mouvement
//                   alertePresence - adversaireProche - faceAFace - attaqueArriere 
//             suppression de disTOF (capteur I2C : VL53L0X, SDA & SCL)
// 22/5/2023 - création des objets Presence et Tension
//             nouvelle version de la carte verte MOTEUR (suite erreur de pas)
//             nouvelle affectation des pins capteurs (tension et 8 presence)
//             suppression de disTOF (capteur I2C : VL53L0X, SDA & SCL)
// 19/5/2023 - création de la version 3 (après Champs s/ Marne & La Tour du Pin)
//             changement des capteurs : VL53L0X(TOF) -> RZ60S(4) et JS40F(3)
//             adoption des moteurs : JSUMO 6V-400rpm + pneus NOIRS
//             nouvelle cartes blanche (bouton) et verte(moteur)
//             adhésion à GitHub COPILOT (10$/mois)
//
// 29/4/2023 - CONCOURS à La Tour du Pin : robot OK seulement si appui bouton (giant battle)
// 13/4/2023 - mesure de la vitesse des moteurs -> création du cas 400 rpm
//             IOREK : moteur AliExpress 20000 R35 -> 600 rpm
//             LYRA : moteur JSUMO 6V -> 400rpm
//             Champ s/ Marne : moteurs N30 - 6V -> 200 rpm (pour les 2 robots)
// 11/4/2023 - new carte verte LYRA : contrôle impossible avec GM13*50 règlé à 200 -> passage 600 rpm
//            création de la vitesse M7 : adversaire devant tout près !!!
// 3/4/2023 - LYRA : soudure fils ald USB micro-B -> NOK - essai : pgm 400 rpm (moteurs 1ère génération)
//            le 27/3, LYRA avait toujours le pgm du 11/3/2023 -> USB micro B ne transmet plus les données :(
// 29/3/2023 - paramétrage 200 et 600 rpm (-(-(* ligne 250 *)-)-)
/* deplacement sans tenir compte des capteurs en vitesse à 200 rpm :
  . tout droit / 1 diamètre -> 1000ms : vit6 - vit6
  . 1 tour - diam 30 cm -> 2500ms : vit2 - vit6
  . 1 tour - diam 35 cm -> 3000ms : vit2 - vit5
  . 1 tour - diam 40 cm -> 4000ms : vit2 - vit4 */
// 27/3/2023 - changement des moteurs de IOREK (6V - 20000tr - r35:1 -> 600rpm)
// 20/3/2023 - fin 1ere seconde
// 19/3/2023 - ménage dans les paramètres - éviter la reculade expresse !!!
// 18/3/2023 - besoin de distinguer les paramètres pour changer la vitesse des moteurs
// 14/3/2023 - détection multiple en statique (mesure 4 fois 0246 et 1357)
// 13/3/2023 - après CHAMPS s/ MARNE : affinage des paramètres 1ère seconde
//             cibleAdversaire -> ledON ou led OFF si presenceAdversaire

// 11/3/2023 - CONCOURS à CHAMPS s/ MARNE : 17 débutants et 12 experts - LYRA & IOREK : 1 gagné - 3 perdus
// 10/3/2023 - simplification "temps de combat" + reprise du calcul de "alternance"
//             correction découmpte 5 secondes : arrêt au premier adversaire !
//             attaque directe à la 1ere seconde dans le cas "appui bouton"
//             temps de reaction specifiques : 1ere seconde
// 9/3/2023 - réduction vitesse 180->120 (adv détecté loin : 31 à 62) + durée réaction
// 8/3/2023 - Retour Arrière sur changement de mvt 5 sec (ne détecte plus la boîte)
// 7/3/2023 - changement de mouvement après 5 sec (si tourne en rond)
// 5/3/2023 - rotation alternée aléatoire Gauche ou Droite > OK seulement le 10/3/23
// 4/3/2023 - attaque -> défense si adversaire Proche (<5cm && >0)
// 2/3/2023 - reprise : concours Champs s/ Marne approche ! (11/3/2023)
// 20/12/2022 - construction du robot LYRA - changement pour IOREK roi des ours
// 7/12/2022 - début de construction/clonage : LYRA Belacqua ...
// 30/11/2022 - réaction avec présence adversaire &ligne blanche
// 21/11/2022 - TOF > temps maxi : 50 ms /capteur => 8 capteurs : 400 ms
// 18/11/2022 - changement de la carte VERTE (ESP32 grillé ... microUSB torturée ?)
// 30/10/2022 - test ligne blanche
// 19/10/2022 - moteurs 400 rpm >> 400 Hz : pb moteur mécanique moteur gauche 
// 18/10/2022 - premier cercle gauche & droite (moteurs 1000 rpm >> 1000 Hz)
// 17/10/2022 - inversion pins 14 et 27 moteur droit - fréquence 1000 Hz
// 16/10/2022 - assemblage OK : 8 capteurs A, AD, D, RD, R, RG, G, AG
// 15/8/2022 - préparation PCB : distance TOF + mot DAEMON
// 7/9/2022 - librairie ADAFRUIT
// 6/9/2022 - double capteur TOF : DFRobot library
// 4/9/2022 - création DISTOF (distance laser Time Of Flight)
// 3/9/2022 - re-création de MOTEUR (N30-20)
// 2/9/2022 - reprise de NOIR BLANC (gauche et droite)
// 31/8/2022 - reprise de BOUTON (bouton et télécommande)
// 30/8/2022 - reprise de LED (led work) à partir du programme MANGA-SUMO-2021

//  -------------------------------------------------------
//  ----- DEFINE -----
//  -------------------------------------------------------
#include <Arduino.h>
#include <ESP32Servo.h>
#include <cstdlib>
#include <ctime>
#include "Wire.h"

#include "led.h"
#define pin_ledWork 12
Led ledWork(pin_ledWork);

#include "bouton.h"
#define pin_boutonON 35                          // bouton GO
#define pin_telecomON 34                         // télécommande ON
#define remanenceBouton 1                        // 1 ms anti-rebond

Bouton boutonON(pin_boutonON);
Bouton telecomON(pin_telecomON);
volatile bool signalBoutonBouton;
volatile long debutBoutonBouton, debutBoutonTelecom;
volatile bool signalDeDepart, cestPartiPour3minutes;
volatile bool signalDepartTelecom, urgenceInitialisee, signalArretUrgence;

#include "noirblanc.h"
#define pin_cny70G 2                             // gauche
#define pin_cny70D 4                             // droite
#define pin_cny70R 18                            // arrière
#define remanenceNoirBlanc 2
//
NoirBlanc cny70G(pin_cny70G);
NoirBlanc cny70D(pin_cny70D);
NoirBlanc cny70R(pin_cny70R);
volatile bool alerteNoirBlancGauche;
volatile bool alerteNoirBlancDroite;
volatile bool alerteNoirBlancDerriere;
volatile long debutNoirBlancGauche, debutNoirBlancDroite, debutNoirBlancDerriere;
// réaction Noir & Blanc
volatile bool reactionLigneBlanche, finReactionLigneBlanche;
volatile bool reactionBlancGauche, reactionBlancDroite, reactionBlancDevant;
volatile bool finReactionBlancGauche, finReactionBlancDroite, finReactionBlancDevant;
volatile bool reactionBlancDerriere, finReactionBlancDerriere;
volatile long topEsquiveBlancGauche, tempsEsquiveBlancGauche;
volatile long topEsquiveBlancDroite, tempsEsquiveBlancDroite;
volatile long topEsquiveBlancDerriere, tempsEsquiveBlancDerriere;
volatile long topAvantBlanc, tempsAvantBlanc;
volatile int memoireLigneBlanche;

// micro-moteurs CC
#define pin_moteurGauche 33                      // moteur A (gauche)
#define pin_AvMotGauche 27                       // A1 (ald 25)
#define pin_ArrMotGauche 14                      // A2 (ald 26)
#define pin_moteurDroit 32                       // moteur B (droit)
#define pin_AvMotDroit 26                        // B1 (ald 14)
#define pin_ArrMotDroit 25                       // B2 (ald 27)
Servo moteurGauche;                              // creation moteurs
Servo moteurDroite;
volatile int oldVitGauche, oldVitDroite;
volatile int mouvement, oldMouvement;

volatile int8_t alternance;                      // nombre aléatoire (1-> Gauche - 0->Droite)

enum MOUVEMENT{TOUT_AVANT=0, TOURNE_DROITE, SUR_PLACE_DROITE, 
ESQUIVE_DROITE, TOUT_ARRIERE, ESQUIVE_GAUCHE, SUR_PLACE_GAUCHE, TOURNE_GAUCHE};
MOUVEMENT oldOrientation;
enum DIRECTION{AVANT=0, AVANT_DROITE, DROITE, ARRIERE_DROITE, ARRIERE, ARRIERE_GAUCHE, GAUCHE, AVANT_GAUCHE};
DIRECTION directionAdversaire;

// vitesse des moteurs
volatile int vitesseM0, vitesseM1, vitesseM2, vitesseM3, vitesseM4, vitesseM5, vitesseM6, vitesseM7;
// premiere seconde
volatile int duree1SecondeAvant, duree1SecondeAvantDroite, duree1SecondeAvantGauche, duree1SecondeDroite;
volatile int duree1SecondeGauche, duree1SecondeArriereDroite, duree1SecondeArriereGauche, duree1SecondeArriere;
// durée de réaction  présence
volatile int dureeReactionAvant, dureeReactionAvantDroite, dureeReactionAvantGauche, dureeReactionDroite;
volatile int dureeReactionGauche, dureeReactionArriereDroite, dureeReactionArriereGauche, dureeReactionArriere;

// timers des mouvements
volatile long reactionAvant, reactionAvantDroite, reactionDroite;
volatile long reactionArriereDroite, reactionArriere, reactionArriereGauche;
volatile long reactionGauche, reactionAvantGauche;
volatile bool continueMouvement;

// global combat
int cptPoint, cptExploration;
//#define dureeTotalCombat 10000                                // 10 secondes -> pour test
#define dureeTotalCombat 60000                                  // 1 minute -> durée du combat
#define dureeBoucleCombat 3                                     // 3 ms -> distance 3 mm
volatile unsigned long topDepartCombat, tempsDeCombat;
volatile unsigned long chrono5Secondes, avanceDuChrono;
volatile long debutBoucleCombat, memoBoucleCombat;
volatile long tempsBoucleCombat, interBoucleCombat;

// tension LiPo > 10V
#include "tension.h"
#define pin_vlipo 13                                            // pin analogique
#define remanenceTension 2
Tension vLiPo(pin_vlipo);
volatile bool alerteTension;
volatile long debutTension;
volatile float tensionLiPoMesuree;

#define tension1SecondeRef 11.45
#define duree1SecondeAvantRef 150
#define duree1SecondeAvantDroiteRef 50
#define duree1SecondeAvantGaucheRef 50
#define duree1SecondeDroiteRef 150
#define duree1SecondeGaucheRef 150
#define duree1SecondeArriereDroiteRef 200
#define duree1SecondeArriereGaucheRef 200
#define duree1SecondeArriereRef 250

#define tensionReactionRef 11.45
#define dureeReactionAvantRef 100
#define dureeReactionAvantDroiteRef 50
#define dureeReactionAvantGaucheRef 50
#define dureeReactionDroiteRef 100
#define dureeReactionGaucheRef 100
#define dureeReactionArriereDroiteRef 125
#define dureeReactionArriereGaucheRef 125
#define dureeReactionArriereRef 150

// capteurs "on/off" : POLOLU
#define pin_JS40F_A  15
#define pin_JS40F_AD 16
#define pin_JS40F_D  17
#define pin_JS40F_RD  5
#define pin_JS40F_RG 19
#define pin_JS40F_G  36
#define pin_JS40F_AG 39              
//#define latencePresence 2                                // stabilisation

volatile bool alertePresenceA, alertePresenceAD, alertePresenceD, alertePresenceRD;
volatile bool alertePresenceRG, alertePresenceG, alertePresenceAG;
volatile bool alertePresence, adversaireProche, lectureCapteur;
volatile bool presenceAvecParasiteA, presenceAvecParasiteAD, presenceAvecParasiteD, presenceAvecParasiteRD;
volatile bool presenceAvecParasiteRG, presenceAvecParasiteG, presenceAvecParasiteAG;
volatile bool ilEstDerriere;
volatile int indiceLecture, lecturePlus, lectureMoins, nbCapteurActif, nbAvecParasite;
volatile int nbApparitionA, nbApparitionAD, nbApparitionD, nbApparitionRD; 
volatile int nbApparitionRG, nbApparitionG, nbApparitionAG;
volatile int nbDisparitionA, nbDisparitionAD, nbDisparitionD, nbDisparitionRD;
volatile int nbDisparitionRG, nbDisparitionG, nbDisparitionAG;

// BFAF : blocage face à face
#define attenteBlocage 3000
#define dureeReactionBlocageRef 150
volatile long debutAnalyseBlocage;
volatile int dureeReactionBlocage;

// ligne blanche
#define dureeEsquiveBlancRef 150
#define dureeAvantBlancRef 225
volatile int dureeEsquiveBlanc, dureeAvantBlanc;

// homologation
#define distance1metre 1750

//  -------------------------------------------------------
//  ----- INTERRUPTION sur CAPTEURS -----
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuA()
{ 
  lecturePlus=0;lectureMoins=0;
  for(indiceLecture=0;indiceLecture<31;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_A);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    else{lectureMoins=lectureMoins+1;}
    delayMicroseconds(50);}
  //
  if(lecturePlus>lectureMoins){
    alertePresenceA=true;
    nbApparitionA=nbApparitionA+1;}
  else{
    alertePresenceA=false;
    nbDisparitionA=nbDisparitionA+1;}
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuAD()
{
  lecturePlus=0;lectureMoins=0;
  for(indiceLecture=0;indiceLecture<31;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_AD);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    else{lectureMoins=lectureMoins+1;}
    delayMicroseconds(50);}
  //
  if(lecturePlus>lectureMoins){
    alertePresenceAD=true;
    nbApparitionAD=nbApparitionAD+1;}
  else{
    alertePresenceAD=false;
    nbDisparitionAD=nbDisparitionAD+1;}
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuD()
{
  lecturePlus=0;lectureMoins=0;
  for(indiceLecture=0;indiceLecture<31;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_D);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    else{lectureMoins=lectureMoins+1;}
    delayMicroseconds(50);}
  //
  if(lecturePlus>lectureMoins){
    alertePresenceD=true;
    nbApparitionD=nbApparitionD+1;}
  else{
    alertePresenceD=false;
    nbDisparitionD=nbDisparitionD+1;}
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuRD()
{
  lecturePlus=0;lectureMoins=0;
  for(indiceLecture=0;indiceLecture<31;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_RD);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    else{lectureMoins=lectureMoins+1;}
    delayMicroseconds(50);}
  //
  if(lecturePlus>lectureMoins){
    alertePresenceRD=true;
    nbApparitionRD=nbApparitionRD+1;}
  else{
    alertePresenceRD=false;
    nbDisparitionRD=nbDisparitionRD+1;}
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuRG()
{
  lecturePlus=0;lectureMoins=0;
  for(indiceLecture=0;indiceLecture<31;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_RG);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    else{lectureMoins=lectureMoins+1;}
    delayMicroseconds(50);}
  //
  if(lecturePlus>lectureMoins){
    alertePresenceRG=true;
    nbApparitionRG=nbApparitionRG+1;}
  else{
    alertePresenceRG=false;
    nbDisparitionRG=nbDisparitionRG+1;}
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuG()
{ 
  lecturePlus=0;lectureMoins=0;
  for(indiceLecture=0;indiceLecture<31;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_G);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    else{lectureMoins=lectureMoins+1;}
    delayMicroseconds(50);}
  //
  if(lecturePlus>lectureMoins){
    alertePresenceG=true;
    nbApparitionG=nbApparitionG+1;}
  else{
    alertePresenceG=false;
    nbDisparitionG=nbDisparitionG+1;}
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuAG()
{
  lecturePlus=0;lectureMoins=0;
  for(indiceLecture=0;indiceLecture<31;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_AG);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    else{lectureMoins=lectureMoins+1;}
    delayMicroseconds(50);}
  //
  if(lecturePlus>lectureMoins){
    alertePresenceAG=true;
    nbApparitionAG=nbApparitionAG+1;}
  else{
    alertePresenceAG=false;
    nbDisparitionAG=nbDisparitionAG+1;}
}
//  -------------------------------------------------------
//  ----- SETUP -----
//  -------------------------------------------------------
void setup() 
{
  Serial.begin(115200);

// LED work
  ledWork.setPinLed(pin_ledWork);

// Bouton
  boutonON.setPinBouton(pin_boutonON);
  telecomON.setPinBouton(pin_telecomON);

// Noir Blanc
  cny70G.setPinNoirBlanc(pin_cny70G);
  cny70D.setPinNoirBlanc(pin_cny70D);
  cny70R.setPinNoirBlanc(pin_cny70R);
  alerteNoirBlancGauche=true;                    // false
  alerteNoirBlancDroite=true;                    // false
  alerteNoirBlancDerriere=true;                  // false
  reactionLigneBlanche=false;
  reactionBlancGauche=false;
  reactionBlancDroite=false;
  reactionBlancDevant=false;
  reactionBlancDerriere=false;
  finReactionLigneBlanche=false;
  finReactionBlancGauche=false;
  finReactionBlancDroite=false;
  finReactionBlancDevant=false;
  finReactionBlancDerriere=false;
  debutNoirBlancGauche=0;
  debutNoirBlancDroite=0;
  debutNoirBlancDerriere=0;
  topEsquiveBlancGauche=0;
  topEsquiveBlancDroite=0;
  topEsquiveBlancDerriere=0;
  topAvantBlanc=0;
  memoireLigneBlanche=0;

// MOTEURS (Servo) en PWM (all timers)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  moteurGauche.setPeriodHertz(400);                   // std 50 hz (20 ms) > 100 us
  moteurDroite.setPeriodHertz(400);                   // test OK : 400Hz/400rpm - (1000Hz/1000rpm)
  moteurGauche.attach(pin_moteurGauche,500,2500);     // mini à maxi (us)
  moteurDroite.attach(pin_moteurDroit,500,2500);
  oldVitGauche=0;
  oldVitDroite=0;

// MOTEUR sens (pont en H : TB6612)
  pinMode(pin_AvMotGauche, OUTPUT);                   // Avant Gauche
    digitalWrite(pin_AvMotGauche, LOW);
  pinMode(pin_ArrMotGauche, OUTPUT);                  // aRRiere Gauche
    digitalWrite(pin_ArrMotGauche, LOW);
  pinMode(pin_AvMotDroit, OUTPUT);                    // Avant Droit
    digitalWrite(pin_AvMotDroit, LOW);
  pinMode(pin_ArrMotDroit, OUTPUT);                   // aRRiere Droit
    digitalWrite(pin_ArrMotDroit, LOW);

// REACTIOn <-> VITESSE : LYRA et IOREK(JSUMO 6V-400) pour tournoi de THIONVILLE
// TENSION LiPo
  vLiPo.setPinTension(pin_vlipo);
  tensionLiPoMesuree=vLiPo.mesureTension();
  Serial.print("tension LiPo Mesuree : "); Serial.println(tensionLiPoMesuree);
  
// action 1ère seconde
  duree1SecondeAvant=int(duree1SecondeAvantRef*tension1SecondeRef/tensionLiPoMesuree);
  duree1SecondeAvantDroite=int(duree1SecondeAvantDroiteRef*tension1SecondeRef/tensionLiPoMesuree);
  duree1SecondeAvantGauche=int(duree1SecondeAvantGaucheRef*tension1SecondeRef/tensionLiPoMesuree);
  duree1SecondeDroite=int(duree1SecondeDroiteRef*tension1SecondeRef/tensionLiPoMesuree);
  duree1SecondeGauche=int(duree1SecondeGaucheRef*tension1SecondeRef/tensionLiPoMesuree);
  duree1SecondeArriereDroite=int(duree1SecondeArriereDroiteRef*tension1SecondeRef/tensionLiPoMesuree);
  duree1SecondeArriereGauche=int(duree1SecondeArriereGaucheRef*tension1SecondeRef/tensionLiPoMesuree);
  duree1SecondeArriere=int(duree1SecondeArriereRef*tension1SecondeRef/tensionLiPoMesuree);
  Serial.print("duree 1ere Seconde Avant          : "); Serial.println(duree1SecondeAvant);
  /*
  Serial.print("duree 1ere Seconde Avant Droite   : "); Serial.println(duree1SecondeAvantDroite);
  Serial.print("duree 1ere Seconde Avant Gauche   : "); Serial.println(duree1SecondeAvantGauche);
  Serial.print("duree 1ere Seconde Droite         : "); Serial.println(duree1SecondeDroite);
  Serial.print("duree 1ere Seconde Gauche         : "); Serial.println(duree1SecondeGauche);
  Serial.print("duree 1ere Seconde Arriere Droite : "); Serial.println(duree1SecondeArriereDroite);
  Serial.print("duree 1ere Seconde Arriere Gauche : "); Serial.println(duree1SecondeArriereGauche);
  Serial.print("duree 1ere Seconde Arriere        : "); Serial.println(duree1SecondeArriere);
  */
// durée de réaction si adversaire détecté
  dureeReactionAvant=int(dureeReactionAvantRef*tensionReactionRef/tensionLiPoMesuree);
  dureeReactionAvantDroite=int(dureeReactionAvantDroiteRef*tensionReactionRef/tensionLiPoMesuree);
  dureeReactionAvantGauche=int(dureeReactionAvantGaucheRef*tensionReactionRef/tensionLiPoMesuree);
  dureeReactionDroite=int(dureeReactionDroiteRef*tensionReactionRef/tensionLiPoMesuree);
  dureeReactionGauche=int(dureeReactionGaucheRef*tensionReactionRef/tensionLiPoMesuree);
  dureeReactionArriereDroite=int(dureeReactionArriereDroiteRef*tensionReactionRef/tensionLiPoMesuree);
  dureeReactionArriereGauche=int(dureeReactionArriereGaucheRef*tensionReactionRef/tensionLiPoMesuree);
  dureeReactionArriere=int(dureeReactionArriereRef*tensionReactionRef/tensionLiPoMesuree);
  Serial.print("duree Reaction Avant          : ");Serial.println(dureeReactionAvant);
  /*
  Serial.print("duree Reaction Avant Droite   : ");Serial.println(dureeReactionAvantDroite);
  Serial.print("duree Reaction Avant Gauche   : ");Serial.println(dureeReactionAvantGauche);
  Serial.print("duree Reaction Droite         : ");Serial.println(dureeReactionDroite);
  Serial.print("duree Reaction Gauche         : ");Serial.println(dureeReactionGauche);
  Serial.print("duree Reaction Arriere Droite : ");Serial.println(dureeReactionArriereDroite);
  Serial.print("duree Reaction Arriere Gauche : ");Serial.println(dureeReactionArriereGauche);
  Serial.print("duree Reaction Arriere        : ");Serial.println(dureeReactionArriere);
  */
// vitesse des moteurs      
  vitesseM0=0;
  vitesseM1=27;
  vitesseM2=55;
  vitesseM3=82;
  vitesseM4=110;
  vitesseM5=137;
  vitesseM6=165;
  vitesseM7=180;

// CAPTEURS de PRESENCE
pinMode(pin_JS40F_A, INPUT);
  attachInterrupt(pin_JS40F_A, adversaireApparuA, CHANGE);
pinMode(pin_JS40F_AD, INPUT);
  attachInterrupt(pin_JS40F_AD, adversaireApparuAD, CHANGE);
pinMode(pin_JS40F_D, INPUT);
  attachInterrupt(pin_JS40F_D, adversaireApparuD, CHANGE);
pinMode(pin_JS40F_RD, INPUT);
  attachInterrupt(pin_JS40F_RD, adversaireApparuRD, CHANGE);
pinMode(pin_JS40F_RG, INPUT);
  attachInterrupt(pin_JS40F_RG, adversaireApparuRG, CHANGE);
pinMode(pin_JS40F_G, INPUT);
  attachInterrupt(pin_JS40F_G, adversaireApparuG, CHANGE);
pinMode(pin_JS40F_AG, INPUT);
  attachInterrupt(pin_JS40F_AG, adversaireApparuAG, CHANGE);

  alertePresenceA=false;alertePresenceAD=false;alertePresenceD=false;alertePresenceRD=false;
  alertePresenceRG=false;alertePresenceG=false;alertePresenceAG=false;
  alertePresence=false;adversaireProche=false;lectureCapteur=false;
  presenceAvecParasiteA=false;presenceAvecParasiteAD=false;presenceAvecParasiteD=false;presenceAvecParasiteRD=false;
  presenceAvecParasiteRG=false;presenceAvecParasiteG=false;presenceAvecParasiteAG=false;
  ilEstDerriere=false;
  lecturePlus=0;lectureMoins=0;nbCapteurActif=0;nbAvecParasite=0;
  nbApparitionA=0;nbApparitionAD=0;nbApparitionD=0;nbApparitionRD=0;
  nbApparitionRG=0;nbApparitionG=0;nbApparitionAG=0;
  nbDisparitionA=0;nbDisparitionAD=0;nbDisparitionD=0;nbDisparitionRD=0;
  nbDisparitionRG=0;nbDisparitionG=0;nbDisparitionAG=0;

// REACTION : blocage - ligne blanche
  dureeReactionBlocage=dureeReactionBlocageRef*tensionReactionRef/tensionLiPoMesuree;
  Serial.print("duree Reaction Blocage : ");Serial.println(dureeReactionBlocage);
  dureeEsquiveBlanc=dureeEsquiveBlancRef*tensionReactionRef/tensionLiPoMesuree;
  Serial.print("duree Esquive Blanc : ");Serial.println(dureeEsquiveBlanc);
  dureeAvantBlanc=dureeAvantBlancRef*tensionReactionRef/tensionLiPoMesuree;
  Serial.print("duree Avant Blanc : ");Serial.println(dureeAvantBlanc);
}

//  -------------------------------------------------------
//  ----- BOUTON -----
//  -------------------------------------------------------
void verifie_bouton()
// Timer rémanence des boutons (1 ms)
{
if((millis()-debutBoutonBouton)>=remanenceBouton){
    signalBoutonBouton=boutonON.getEtatBouton();
    if(signalBoutonBouton==true){debutBoutonBouton=millis();}
    }
if((millis()-debutBoutonTelecom)>=remanenceBouton){
    signalDepartTelecom=!telecomON.getEtatBouton();
    if(signalDepartTelecom==true){urgenceInitialisee=true;}
    if(urgenceInitialisee==true){signalArretUrgence=!signalDepartTelecom;}
    }
}
//  -------------------------------------------------------
//  ----- TENSION LiPO -----
//  -------------------------------------------------------
bool verifie_tension()
// Timer rémanence de la tension (2 ms)
// alerte=true si tension < 10V
{
if((millis()-debutTension)>=remanenceTension){
    alerteTension=vLiPo.basseTension();
    if(alerteTension==true){debutTension=millis();}
    }
return alerteTension;
}
//  -------------------------------------------------------
//  ----- LIGNE BLANCHE-----
//  -------------------------------------------------------
bool verifie_noir_blanc()
// Timer avec rémanence Noir & Blanc (2 ms)
// Double lecture (250 us) définie dans l'objet NoirBlanc
// CNY70 : gauche, droite ou arrière
// alerte=true si contact ligne blanche
{
if((millis()-debutNoirBlancGauche)>=remanenceNoirBlanc){
  alerteNoirBlancGauche=cny70G.getEtatNoirBlanc();
  if(alerteNoirBlancGauche==true){
    debutNoirBlancGauche=millis();}
    }
if((millis()-debutNoirBlancDroite)>=remanenceNoirBlanc){
  alerteNoirBlancDroite=cny70D.getEtatNoirBlanc();
  if(alerteNoirBlancDroite==true){
    debutNoirBlancDroite=millis();}
    }
if((millis()-debutNoirBlancDerriere)>=remanenceNoirBlanc){
  alerteNoirBlancDerriere=cny70R.getEtatNoirBlanc();
  if(alerteNoirBlancDerriere==true){
    debutNoirBlancDerriere=millis();}
    }
return alerteNoirBlancGauche||alerteNoirBlancDroite||alerteNoirBlancDerriere;
}
//  -------------------------------------------------------
void ReactionLigneBlanche() 
// rémanence du contact ligne blanche G ou D (2 ms)
// durée de réaction selon la position robot / ligne blanche (250 ms)
// reactions sont exclusives Avant - Gauche - Droite - Derrière(arrière)
{
// ----- debut de réaction -----
  if(verifie_noir_blanc()==true){                          // contact ligne blanche
    if((alerteNoirBlancGauche==true)&&(alerteNoirBlancDroite==true)){
      if(reactionBlancDevant==false){
        topAvantBlanc=millis();
        reactionBlancDevant=true;
        finReactionBlancDevant=false;
        reactionBlancGauche=false;
        reactionBlancDroite=false;
        reactionBlancDerriere=false;
      }}
    if((alerteNoirBlancGauche==true)&&(alerteNoirBlancDroite==false)){
      if(reactionBlancGauche==false){
        topEsquiveBlancGauche=millis();
        reactionBlancGauche=true;
        finReactionBlancGauche=false;
        reactionBlancDevant=false;
        reactionBlancDroite=false;
        reactionBlancDerriere=false;
      }}
    if((alerteNoirBlancGauche==false)&&(alerteNoirBlancDroite==true)){
      if(reactionBlancDroite==false){
        topEsquiveBlancDroite=millis();
        reactionBlancDroite=true;
        finReactionBlancDroite=false;
        reactionBlancDevant=false;
        reactionBlancGauche=false;
        reactionBlancDerriere=false;
      }}
    if((alerteNoirBlancGauche==false)&&(alerteNoirBlancDroite==false)){
      if(alerteNoirBlancDerriere==true){
        if(reactionBlancDerriere==false){
          topEsquiveBlancDerriere=millis();
          reactionBlancDerriere=true;
          finReactionBlancDerriere=false;
          reactionBlancDevant=false;
          reactionBlancGauche=false;
          reactionBlancDroite=false;
  }}}}

// ----- fin de réaction -----
  if(reactionBlancDevant==true){                          // réaction CNY Gauche & Droite
    tempsAvantBlanc=millis()-topAvantBlanc;
    if(tempsAvantBlanc>dureeAvantBlanc){
      reactionBlancDevant=false;
      if(finReactionBlancDevant==false){finReactionBlancDevant=true;
      }}}
  if(reactionBlancGauche==true){                           // réaction CNY Gauche
    tempsEsquiveBlancGauche=millis()-topEsquiveBlancGauche;
    if(tempsEsquiveBlancGauche>dureeEsquiveBlanc){
      reactionBlancGauche=false;
      if(finReactionBlancGauche==false){finReactionBlancGauche=true;
      }}}
  if(reactionBlancDroite==true){                           // réaction CNY Droite
    tempsEsquiveBlancDroite=millis()-topEsquiveBlancDroite;
    if(tempsEsquiveBlancDroite>dureeEsquiveBlanc){
      reactionBlancDroite=false;
      if(finReactionBlancDroite==false){finReactionBlancDroite=true;
      }}}
  if(reactionBlancDerriere==true){                         // réaction CNY Derrière
    tempsEsquiveBlancDerriere=millis()-topEsquiveBlancDerriere;
    if(tempsEsquiveBlancDerriere>dureeEsquiveBlanc){
      reactionBlancDerriere=false;
      if(finReactionBlancDerriere==false){finReactionBlancDerriere=true;
      }}}
}
//  -------------------------------------------------------
//  ----- PRESENCE -----
//  -------------------------------------------------------
void verifie_presence(bool affichageVP)
/*
  présence à l'AVANT
  présence à l'AVANT DROITE (30°)
  présence à DROITE
  présence à l'ARRIERE DROITE (5°)
  présence à l'ARRIERE GAUCHE (5°)
  présence à GAUCHE
  présence à l'AVANT GAUCHE (30°)
*/
{
// ***** capteur actif & actif avec parasite *****
nbCapteurActif=0;
nbAvecParasite=0;
//
if((nbApparitionA>=1)||(nbDisparitionA>=1)){                                // parasite A
  if(alertePresenceA==true){
    presenceAvecParasiteA=true;
    nbCapteurActif=nbCapteurActif+1;
    nbAvecParasite=nbAvecParasite+1;}
  else{presenceAvecParasiteA=false;}}
else{
  presenceAvecParasiteA=false;
  alertePresenceA=!digitalRead(pin_JS40F_A);
  if(alertePresenceA==true){
    nbCapteurActif=nbCapteurActif+1;}}

if((nbApparitionAD>=1)||(nbDisparitionAD>=1)){                              // parasite AD
  if(alertePresenceAD==true){
    presenceAvecParasiteAD=true;
    nbCapteurActif=nbCapteurActif+1;
    nbAvecParasite=nbAvecParasite+1;}
  else{presenceAvecParasiteAD=false;}}
else{
  presenceAvecParasiteAD=false;
  alertePresenceAD=!digitalRead(pin_JS40F_AD);
  if(alertePresenceAD==true){
    nbCapteurActif=nbCapteurActif+1;}}

if((nbApparitionD>=1)||(nbDisparitionD>=1)){                                // parasite D
  if(alertePresenceD==true){
    presenceAvecParasiteD=true;
    nbCapteurActif=nbCapteurActif+1;
    nbAvecParasite=nbAvecParasite+1;}
  else{presenceAvecParasiteD=false;}}
else{
  presenceAvecParasiteD=false;
  alertePresenceD=!digitalRead(pin_JS40F_D);
  if(alertePresenceD==true){
    nbCapteurActif=nbCapteurActif+1;}}

if((nbApparitionRD>=1)||(nbDisparitionRD>=1)){                              // parasite RD
  if(alertePresenceRD==true){
    presenceAvecParasiteRD=true;
    nbCapteurActif=nbCapteurActif+1;
    nbAvecParasite=nbAvecParasite+1;}
  else{presenceAvecParasiteRD=false;}}
else{
  presenceAvecParasiteRD=false;
  alertePresenceRD=!digitalRead(pin_JS40F_RD);
  if(alertePresenceRD==true){
    nbCapteurActif=nbCapteurActif+1;}}

if((nbApparitionRG>=1)||(nbDisparitionRG>=1)){                              // parasite RG
  if(alertePresenceRG==true){
    presenceAvecParasiteRG=true;
    nbCapteurActif=nbCapteurActif+1;
    nbAvecParasite=nbAvecParasite+1;}
  else{presenceAvecParasiteRG=false;}}
else{
  presenceAvecParasiteRG=false;
  alertePresenceRG=!digitalRead(pin_JS40F_RG);
  if(alertePresenceRG==true){
    nbCapteurActif=nbCapteurActif+1;}}

if((nbApparitionG>=1)||(nbDisparitionG>=1)){                                // parasite G
  if(alertePresenceG==true){
    presenceAvecParasiteG=true;
    nbCapteurActif=nbCapteurActif+1;
    nbAvecParasite=nbAvecParasite+1;}
  else{presenceAvecParasiteG=false;}}
else{
  presenceAvecParasiteG=false;
  alertePresenceG=!digitalRead(pin_JS40F_G);
  if(alertePresenceG==true){
    nbCapteurActif=nbCapteurActif+1;}}

if((nbApparitionAG>=1)||(nbDisparitionAG>=1)){                              // parasite AG
  if(alertePresenceAG==true){
    presenceAvecParasiteAG=true;
    nbCapteurActif=nbCapteurActif+1;
    nbAvecParasite=nbAvecParasite+1;}
  else{presenceAvecParasiteAG=false;}}
else{
  presenceAvecParasiteAG=false;
  alertePresenceAG=!digitalRead(pin_JS40F_AG);
  if(alertePresenceAG==true){
    nbCapteurActif=nbCapteurActif+1;}}

// ***** affichage *****
if(affichageVP==true){
  Serial.print("JSu - AVANT          : ");
    if(alertePresenceA==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbApparitionA);
    Serial.print("  ->");Serial.println(nbDisparitionA);

  Serial.print("Zad - AVANT DROITE   : ");
    if(alertePresenceAD==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbApparitionAD);
    Serial.print("  ->");Serial.println(nbDisparitionAD);

  Serial.print("JSu - DROITE         : ");
    if(alertePresenceD==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbApparitionD);
    Serial.print("  ->");Serial.println(nbDisparitionD);

  Serial.print("Zad - ARRIERE DROITE : ");
    if(alertePresenceRD==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbApparitionRD);
    Serial.print("  ->");Serial.println(nbDisparitionRD);

  Serial.print("Zad - ARRIERE GAUCHE : ");
    if(alertePresenceRG==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbApparitionRG);
    Serial.print("  ->");Serial.println(nbDisparitionRG);

  Serial.print("JSu - GAUCHE         : ");
    if(alertePresenceG==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbApparitionG);
    Serial.print("  ->");Serial.println(nbDisparitionG);

  Serial.print("Zad - AVANT GAUCHE   : ");
    if(alertePresenceAG==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbApparitionAG);
    Serial.print("  ->");Serial.println(nbDisparitionAG);
}
  
// ***** ré-initialisation *****
nbApparitionA=0;nbDisparitionA=0;
nbApparitionAD=0;nbDisparitionAD=0;
nbApparitionD=0;nbDisparitionD=0;
nbApparitionRD=0;nbDisparitionRD=0;
nbApparitionRG=0;nbDisparitionRG=0;
nbApparitionG=0;nbDisparitionG=0;
nbApparitionAG=0;nbDisparitionAG=0;
}
//  -------------------------------------------------------
void PresenceLoinPres(bool affichagePLP)
// définition des mouvements
// nb capteurs : 1->loin . 2->près . 3->contact avant . 0->rien
{
  verifie_presence(false);                                     // affichage des capteurs
  adversaireProche=false;

  // ***** élimination des capteurs avec parasite *****
  if((nbCapteurActif>1)&&(nbAvecParasite>0)){                 
    if((alertePresenceA==true)&&(presenceAvecParasiteA==true)) {
      alertePresenceA=false;
      presenceAvecParasiteA=false;
      nbCapteurActif=nbCapteurActif-1;nbAvecParasite=nbAvecParasite-1;}

    if((alertePresenceAD==true)&&(presenceAvecParasiteAD==true)){
      alertePresenceAD=false;
      presenceAvecParasiteAD=false;
      nbCapteurActif=nbCapteurActif-1;nbAvecParasite=nbAvecParasite-1;}

    if((alertePresenceD==true)&&(presenceAvecParasiteD==true)) {
      alertePresenceD=false;
      presenceAvecParasiteD=false;
      nbCapteurActif=nbCapteurActif-1;nbAvecParasite=nbAvecParasite-1;}

    if((alertePresenceRD==true)&&(presenceAvecParasiteRD==true)){
      alertePresenceRD=false;
      presenceAvecParasiteRD=false;
      nbCapteurActif=nbCapteurActif-1;nbAvecParasite=nbAvecParasite-1;}

    if((alertePresenceRG==true)&&(presenceAvecParasiteRG==true)){
      alertePresenceRG=false;
      presenceAvecParasiteRG=false;
      nbCapteurActif=nbCapteurActif-1;nbAvecParasite=nbAvecParasite-1;}

    if((alertePresenceG==true)&&(presenceAvecParasiteG==true)) {
      alertePresenceG=false;
      presenceAvecParasiteG=false;
      nbCapteurActif=nbCapteurActif-1;nbAvecParasite=nbAvecParasite-1;}

    if((alertePresenceAG==true)&&(presenceAvecParasiteAG==true)){
      alertePresenceAG=false;
      presenceAvecParasiteAG=false;
      nbCapteurActif=nbCapteurActif-1;nbAvecParasite=nbAvecParasite-1;}
  }
  // ***** affichage *****
  if(affichagePLP==true){
    Serial.print("   Capteurs stables : ");Serial.print(nbCapteurActif);
    Serial.print(" -> ");
    if(alertePresenceA==true) {Serial.print("   A ");} else{Serial.print("   - ");}
    if(alertePresenceAD==true){Serial.print("  AD");} else{Serial.print("  - ");}
    if(alertePresenceD==true) {Serial.print("  D ");} else{Serial.print("  - ");}
    if(alertePresenceRD==true){Serial.print("  RD");} else{Serial.print("  - ");}
    if(alertePresenceRG==true){Serial.print("  RG");} else{Serial.print("  - ");}
    if(alertePresenceG==true) {Serial.print("  G ");} else{Serial.print("  - ");}
    if(alertePresenceAG==true){Serial.println("  AG");}else{Serial.println("  - ");}
    }
  // ***** définition des mouvements *****
  switch (nbCapteurActif){
    case 1:                                                    // 1 capteur actif
      alertePresence=true;
      adversaireProche=false;
      if(alertePresenceA==true){
        mouvement=31; directionAdversaire=AVANT;}
      if(alertePresenceAD==true){
        mouvement=33; directionAdversaire=AVANT_DROITE;}
      if(alertePresenceD==true){                               // près <-> loin            
        mouvement=41; directionAdversaire=DROITE;}
      if(alertePresenceRD==true){
        mouvement=43; directionAdversaire=ARRIERE_DROITE;}
      if(alertePresenceRG==true){
        mouvement=52; directionAdversaire=ARRIERE_GAUCHE;}
      if(alertePresenceG==true){                               // près <-> loin
        mouvement=61; directionAdversaire=GAUCHE;}
      if(alertePresenceAG==true){
        mouvement=63; directionAdversaire=AVANT_GAUCHE;}
      break;
    case 2:                                                    // 2 capteurs actifs
      alertePresence=true;
      adversaireProche=true;
      if((alertePresenceA==true)&&(alertePresenceAD==true)){
        mouvement=32; directionAdversaire=AVANT_DROITE;}
      if((alertePresenceAD==true)&&(alertePresenceD==true)){
        mouvement=34; directionAdversaire=AVANT_DROITE;}
      if((alertePresenceD==true)&&(alertePresenceRD==true)){
        mouvement=42; directionAdversaire=ARRIERE_DROITE;}
      if((alertePresenceRD==true)&&(alertePresenceRG==true)){
        mouvement=51; directionAdversaire=ARRIERE;}
      if((alertePresenceRG==true)&&(alertePresenceG==true)){
        mouvement=53; directionAdversaire=ARRIERE_GAUCHE;}
      if((alertePresenceG==true)&&(alertePresenceAG==true)){
        mouvement=62; directionAdversaire=AVANT_GAUCHE;}
      if((alertePresenceAG==true)&&(alertePresenceA==true)){
        mouvement=30; directionAdversaire=AVANT_GAUCHE;}
      if((alertePresenceAG==true)&&(alertePresenceAD==true)){
        mouvement=31; directionAdversaire=AVANT;}
      break;
    case 3:                                                    // 3 capteurs actifs 
      alertePresence=true;
      adversaireProche=true;
      if((alertePresenceA==true)&&(alertePresenceD==true)&&(alertePresenceAD==true)){
        mouvement=33; directionAdversaire=AVANT_DROITE;}
      if((alertePresenceA==true)&&(alertePresenceG==true)&&(alertePresenceAG==true)){
        mouvement=63; directionAdversaire=AVANT_GAUCHE;}
      if((alertePresenceA==true)&&(alertePresenceAD==true)&&(alertePresenceAG==true)){
        mouvement=31; directionAdversaire=AVANT;}
      break;
    default:                                                   // 0 capteur OU impossible !!!
      alertePresence=false;
      adversaireProche=false;
      break;
  } 
}
//  -------------------------------------------------------
//  ----- MOUVEMENT -----
//  -------------------------------------------------------
void SensEtDeplacement(MOUVEMENT orientation, int vitGauche, int vitDroite) 
// avant <> arriere - moteurs Gauche et Droite
{
  if((orientation!=oldOrientation)||(vitGauche!=oldVitGauche)){
    Serial.print("   ");Serial.print(vitGauche);Serial.print("  ");
    oldVitGauche=vitGauche;}
  //
  switch (orientation){
    case TOUT_AVANT :
        if(orientation!=oldOrientation){Serial.print("Tout Avant");}
        digitalWrite(pin_AvMotGauche, HIGH);digitalWrite(pin_ArrMotGauche, LOW);
        digitalWrite(pin_AvMotDroit, HIGH);digitalWrite(pin_ArrMotDroit, LOW);break;
    case TOURNE_DROITE :
        if(orientation!=oldOrientation){Serial.print("Tourne a droite");}
        digitalWrite(pin_AvMotGauche, HIGH);digitalWrite(pin_ArrMotGauche, LOW);
        digitalWrite(pin_AvMotDroit, LOW);digitalWrite(pin_ArrMotDroit, LOW);break;
    case SUR_PLACE_DROITE :
        if(orientation!=oldOrientation){Serial.print("Sur place groite");}
        digitalWrite(pin_AvMotGauche, HIGH);digitalWrite(pin_ArrMotGauche, LOW);
        digitalWrite(pin_AvMotDroit, LOW);digitalWrite(pin_ArrMotDroit, HIGH);break;
    case ESQUIVE_DROITE :
        if(orientation!=oldOrientation){Serial.print("Esquive droite");}
        digitalWrite(pin_AvMotGauche, LOW);digitalWrite(pin_ArrMotGauche, LOW);
        digitalWrite(pin_AvMotDroit, LOW);digitalWrite(pin_ArrMotDroit, HIGH);break;
    case TOUT_ARRIERE :
        if(orientation!=oldOrientation){Serial.print("Tout Arriere");}
        digitalWrite(pin_AvMotGauche, LOW);digitalWrite(pin_ArrMotGauche, HIGH);
        digitalWrite(pin_AvMotDroit, LOW);digitalWrite(pin_ArrMotDroit, HIGH);break;
    case ESQUIVE_GAUCHE :
        if(orientation!=oldOrientation){Serial.print("Esquive gauche");}
        digitalWrite(pin_AvMotGauche, LOW);digitalWrite(pin_ArrMotGauche, HIGH);
        digitalWrite(pin_AvMotDroit, LOW);digitalWrite(pin_ArrMotDroit, LOW);break;
    case SUR_PLACE_GAUCHE :
        if(orientation!=oldOrientation){Serial.print("Sur place gauche");}
        digitalWrite(pin_AvMotGauche, LOW);digitalWrite(pin_ArrMotGauche, HIGH);
        digitalWrite(pin_AvMotDroit, HIGH);digitalWrite(pin_ArrMotDroit, LOW);break;
    case TOURNE_GAUCHE :
        if(orientation!=oldOrientation){Serial.print("Tourne a gauche");}
        digitalWrite(pin_AvMotGauche, LOW);digitalWrite(pin_ArrMotGauche, LOW);
        digitalWrite(pin_AvMotDroit, HIGH);digitalWrite(pin_ArrMotDroit, LOW);break;}
  //
  if((orientation!=oldOrientation)||(vitDroite!=oldVitDroite)){
    Serial.print("  ");Serial.print(vitDroite);Serial.print("   ");
    oldVitDroite=vitDroite;}
  oldOrientation=orientation;  
  //
  moteurGauche.write(vitGauche);
  moteurDroite.write(vitDroite);
}
//  -------------------------------------------------------
void ArretOuFreinage(int urgence)
{
  if(urgence==1){
    Serial.print("Blocage");
    digitalWrite(pin_AvMotGauche, HIGH);
    digitalWrite(pin_ArrMotGauche, HIGH);
    digitalWrite(pin_AvMotDroit, HIGH);
    digitalWrite(pin_ArrMotDroit, HIGH);}
  else{
    Serial.print("Arret");
    digitalWrite(pin_AvMotGauche, LOW);
    digitalWrite(pin_ArrMotGauche, LOW);
    digitalWrite(pin_AvMotDroit, LOW);
    digitalWrite(pin_ArrMotDroit, LOW);}   
  //
  moteurGauche.write(0);
  moteurDroite.write(0);
}
//  -------------------------------------------------------
void decompte5secondes()
// démarrage après 5 secondes SI appui sur Bouton ON
// recherche de l'adversaire à la seconde 2, 3, 4, 5 -> 3ème seconde
// démarrage immédiat SI appui sur Télécomm ON
{
// temps 0 : décomptage
  chrono5Secondes=millis();                                     // horaire de départ en ms
  Serial.println("Décomptage des 5 secondes");                  // ledWork.impulsion(0);
  verifie_bouton();
  avanceDuChrono=millis()-chrono5Secondes;                      // durée écoulée
// seconde 1
  while((signalDepartTelecom==false)&&(avanceDuChrono<950)){
    avanceDuChrono=millis()-chrono5Secondes; 
    verifie_bouton();}
  ledWork.impulsion(0);
  Serial.print("   1 > chrono : ");Serial.println(millis()-chrono5Secondes);
// seconde 2
  while((signalDepartTelecom==false)&&(avanceDuChrono<1950)){
    avanceDuChrono=millis()-chrono5Secondes; 
    verifie_bouton();}
  ledWork.impulsion(0);
  Serial.print("   2 > chrono : ");Serial.println(millis()-chrono5Secondes);
// seconde 3
  while((signalDepartTelecom==false)&&(avanceDuChrono<2950)){
    avanceDuChrono=millis()-chrono5Secondes; 
    verifie_bouton();}
  ledWork.impulsion(0);
  Serial.print("   3 > chrono : ");Serial.println(millis()-chrono5Secondes);
  PresenceLoinPres(true);
// seconde 4
  while((signalDepartTelecom==false)&&(avanceDuChrono<3950)){
    avanceDuChrono=millis()-chrono5Secondes; 
    verifie_bouton();}
  ledWork.impulsion(0);  
  Serial.print("   4 > chrono : ");Serial.println(millis()-chrono5Secondes);
// seconde 5
  while((signalDepartTelecom==false)&&(avanceDuChrono<4950)){
    avanceDuChrono=millis()-chrono5Secondes; 
    verifie_bouton();}
  ledWork.impulsion(0);
  Serial.print("   5 > chrono : ");Serial.println(millis()-chrono5Secondes);
}
//  -------------------------------------------------------
void parcours1metre()
// homologation sur 1 metre tout droit + poussée sur balance
{
  Serial.print("HOMOLOGATION : 1 metre");
  SensEtDeplacement(TOUT_AVANT,vitesseM3,vitesseM3);
  delay(distance1metre);
  //
  ArretOuFreinage(0);
  ledWork.on();
  while(1){;};                                   // attente -> reset pour redémarrer
}
//  -------------------------------------------------------
//  ----- LOOP -----
//  -------------------------------------------------------
void loop() 
// codification des mouvements
/*
1 : arret d'urgence    > 10
2 : ligne blanche      > 21(G+D), 22(G), 23(D)

3 : présence AV, AvD   > 63(AG), 30(AG-A), 31(A), 32(A-AD), 33(AD)
4 : présence Dro, aRrD > 34(AD-D), 41(D), 42(D-RD)
5 : présence aRR, aRrG > 43(RD), 51(RD-RG), 52(RG)
6 : présence Gau, AvG  > 53(RG-G), 61(G), 62(G-AG)

7 : aucune détection   > 71(G), 72(D)
*/
{
// INITIALISATION
// ********** verification des conditions de démarrage ********************************************

// Vérification si (ROBOT sur NOIR) et (TENSION LiPo OK)
while((verifie_noir_blanc()==true)||(alerteTension==true)){
  Serial.print("Gauche : ");Serial.print(alerteNoirBlancGauche);
  Serial.print(" - Droite : ");Serial.print(alerteNoirBlancDroite);
  Serial.print("   * TENSION : ");Serial.print(vLiPo.mesureTension());
  Serial.println(" V ");
  ledWork.flashLumineux(7,1000);}
Serial.print("Gauche : ");
if(alerteNoirBlancGauche==false){Serial.print("NOIR");}else{Serial.print("BLANC");}
Serial.print(" - Droite : ");
if(alerteNoirBlancDroite==false){Serial.print("NOIR");}else{Serial.print("BLANC");}
Serial.print(" * TENSION : ");Serial.print(vLiPo.mesureTension());
Serial.println(" V ");

// READY !!! clin d'oeil ..
Serial.print("Appui Bouton ON ... ");
ledWork.impulsion(1);ledWork.impulsion(0);

// lecture capteurs de présence : au démarrage (même sans les 5 secondes)
PresenceLoinPres(true);

// appui Bouton (décomptage) ou Télécommande (départ direct)
signalDepartTelecom=false;
signalBoutonBouton=false;
verifie_bouton();
while(signalDepartTelecom==false&&signalBoutonBouton==false){
  if((cptPoint%100)==0){Serial.print(".");}
  cptPoint=cptPoint+1;delay(10);
  verifie_bouton();}

// si BOUTON : décomptage des 5 secondes
if(signalDepartTelecom==false&&signalBoutonBouton==true){decompte5secondes();}

// si TELECOM : parcours homologation -> 1m
if(signalDepartTelecom==true&&signalBoutonBouton==false){parcours1metre();}

// Dans tous les CAS : c'est parti !!!
topDepartCombat=millis();
tempsDeCombat=0;

// PREMIER MOUVEMENT - PREMIERE SECONDE
// ********** adversaire détécté dans le comptage des secondes ************************************
if(alertePresence==true){
  switch (directionAdversaire){
    case AVANT:                                  // 31
      SensEtDeplacement(TOUT_AVANT,vitesseM6,vitesseM6);
      delay(duree1SecondeAvant);
      break;
    case AVANT_DROITE:                           // 33
      SensEtDeplacement(SUR_PLACE_DROITE,vitesseM6,vitesseM4);
      delay(duree1SecondeAvantDroite);
      break;
    case DROITE:                                 // 41
      SensEtDeplacement(ESQUIVE_DROITE,vitesseM0,vitesseM6);
      delay(duree1SecondeDroite);
      break;
    case ARRIERE_DROITE:                         // 43
      SensEtDeplacement(SUR_PLACE_DROITE,vitesseM6,vitesseM4);
      delay(duree1SecondeArriereDroite);
      break;
    case ARRIERE:                                // 51
      if(alternance==1){
        SensEtDeplacement(SUR_PLACE_GAUCHE,vitesseM4,vitesseM6);}
      else{
        SensEtDeplacement(SUR_PLACE_DROITE,vitesseM6,vitesseM4);}
      delay(duree1SecondeArriere);
      break;
    case ARRIERE_GAUCHE:                         // 52
      SensEtDeplacement(SUR_PLACE_GAUCHE,vitesseM4,vitesseM6);
      delay(duree1SecondeArriereGauche);
      break;
    case GAUCHE:                                 // 61
      SensEtDeplacement(ESQUIVE_GAUCHE,vitesseM6,vitesseM0);
      delay(duree1SecondeGauche);
      break;
    case AVANT_GAUCHE:                           // 63
      SensEtDeplacement(SUR_PLACE_GAUCHE,vitesseM4,vitesseM6);
      delay(duree1SecondeAvantGauche);
      break;}
    
  PresenceLoinPres(true);                        // vérifie si on le retrouve à l'AVANT
  if(directionAdversaire==AVANT){
    SensEtDeplacement(TOUT_AVANT,vitesseM7,vitesseM7);
    delay(duree1SecondeAvant);}
}
/*
 // *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* 
  ArretOuFreinage(0);
  while(1){;};                                  // visualisation 1ère seconde
 // *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
*/
// BOUCLE de COMBAT
// ********** boucle 3 ms *************************************************************************

cptPoint=0; cptExploration=0;
alternance=int(topDepartCombat/1000)%2;          // rotation libre -> 1 : Gauche - 0 : Droite
verifie_bouton();                                // appui sur télécommande -> Arret d'Urgence
verifie_tension();                               // alerte tension LiPo -> plus de jus !

while((signalArretUrgence==false)&&(tempsDeCombat<=dureeTotalCombat)&&(alerteTension==false)){
// ----- boucle de combat : attaque et défense -----
  debutBoucleCombat=millis();
  tempsBoucleCombat=0;

// examen ligne blanche (priorité 1)
  ReactionLigneBlanche();
  reactionLigneBlanche=reactionBlancGauche||reactionBlancDroite||reactionBlancDevant||reactionBlancDerriere;
  finReactionLigneBlanche=finReactionBlancGauche||finReactionBlancDroite||finReactionBlancDevant||finReactionBlancDerriere;
  //
  if(reactionLigneBlanche==false){               // Sur ZONE NOIRE du DOJO
    if(finReactionLigneBlanche==false){          // cycle normal

// recherche présence adversaire (priorité 2) 
  PresenceLoinPres(false);                       // lecture des 7 capteurs sans affichage

// Adversaire détecté
    if(alertePresence==true){                    // adversaire DETECTE
      switch(mouvement){
        case 30:                                 // en avant sur la gauche
          if(mouvement!=oldMouvement){reactionAvant=millis();continueMouvement=true;}
          if((millis()-reactionAvant)<dureeReactionAvant){
            SensEtDeplacement(TOUT_AVANT,vitesseM6,vitesseM7);}                                    // ajustement G
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
          break;
        case 31:                                 // en avant !!!
          if(mouvement!=oldMouvement){
            reactionAvant=millis();
            continueMouvement=true;
            debutAnalyseBlocage=millis();
            oldMouvement=mouvement;}
          //
          if(adversaireProche==true){
            if((millis()-reactionAvant)<dureeReactionAvant){
              SensEtDeplacement(TOUT_AVANT,vitesseM7,vitesseM7);}                                  // attaque-défense
            else{continueMouvement=false;}
            if((millis()-debutAnalyseBlocage)>attenteBlocage){                                     // Blocage Face-à-Face
              if(alternance==1){                                                                   // recule à Gauche
                SensEtDeplacement(TOUT_ARRIERE,vitesseM6,vitesseM7);
                delay(dureeReactionBlocage);}
              else{                                                                                // recule à Droite
                SensEtDeplacement(TOUT_ARRIERE,vitesseM7,vitesseM6);
                delay(dureeReactionBlocage);}}}
          else{
            if((millis()-reactionAvant)<dureeReactionAvant){
              SensEtDeplacement(TOUT_AVANT,vitesseM4,vitesseM4);}                                  // poursuite
            else{continueMouvement=false;}}
          break;
        case 32:                                 // en avant sur la droite
          if(mouvement!=oldMouvement){reactionAvant=millis();continueMouvement=true;}
          if((millis()-reactionAvant)<dureeReactionAvant){
            SensEtDeplacement(TOUT_AVANT,vitesseM7,vitesseM6);}                                    // ajustement D
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
          break;
        case 33:                                 // tourne 45° (droite)
          if(mouvement!=oldMouvement){reactionAvantDroite=millis();continueMouvement=true;}
          if((millis()-reactionAvantDroite)<dureeReactionAvantDroite){
            if(adversaireProche==true){
                  SensEtDeplacement(SUR_PLACE_DROITE,vitesseM6,vitesseM4);}                        // attaque-défense
            else{ SensEtDeplacement(TOURNE_DROITE,vitesseM4,vitesseM0);}}                          // poursuite
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
        case 34:                                 // esquive Droite
          if(mouvement!=oldMouvement){reactionAvantDroite=millis();continueMouvement=true;}
          if((millis()-reactionAvantDroite)<dureeReactionAvantDroite){
            SensEtDeplacement(ESQUIVE_DROITE,vitesseM0,vitesseM6);}                                // défense
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
          break;
        case 41:                                 // esquive Droite
          if(mouvement!=oldMouvement){reactionDroite=millis();continueMouvement=true;}
          if((millis()-reactionDroite)<dureeReactionDroite){
            SensEtDeplacement(ESQUIVE_DROITE,vitesseM0,vitesseM5);}                                // poursuite
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
        case 42:                                 // esquive Droite
          if(mouvement!=oldMouvement){reactionArriereDroite=millis();continueMouvement=true;}
          if((millis()-reactionArriereDroite)<dureeReactionArriereDroite){
            SensEtDeplacement(ESQUIVE_DROITE,vitesseM0,vitesseM6);}                                // défense urgente
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
        case 43:                                 // sur place Droite
          if(mouvement!=oldMouvement){reactionArriereDroite=millis();continueMouvement=true;}
          if((millis()-reactionArriereDroite)<dureeReactionArriereDroite){
            SensEtDeplacement(SUR_PLACE_DROITE,vitesseM4,vitesseM4);}                              // poursuite
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
        case 51:                                 // demi-tour
          if(mouvement!=oldMouvement){reactionArriere=millis();continueMouvement=true;}
          if((millis()-reactionArriere)<dureeReactionArriere){
            if(alternance==1){
                  SensEtDeplacement(TOURNE_GAUCHE,vitesseM0,vitesseM6);}                           // défense
            else{ SensEtDeplacement(TOURNE_DROITE,vitesseM6,vitesseM0);}}                          // alternée
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
        case 52:                                 // sur place Gauche
          if(mouvement!=oldMouvement){reactionArriereGauche=millis();continueMouvement=true;}
          if((millis()-reactionArriereGauche)<dureeReactionArriereGauche){
            SensEtDeplacement(SUR_PLACE_GAUCHE,vitesseM4,vitesseM4);}                              // poursuite
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
        case 53:                                 // esquive Gauche
          if(mouvement!=oldMouvement){reactionArriereGauche=millis();continueMouvement=true;}
          if((millis()-reactionArriereGauche)<dureeReactionArriereGauche){
            SensEtDeplacement(ESQUIVE_GAUCHE,vitesseM6,vitesseM0);}                                // défense urgente
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
        case 61:                                 // esquive Gauche
          if(mouvement!=oldMouvement){reactionGauche=millis();continueMouvement=true;}
          if((millis()-reactionGauche)<dureeReactionGauche){
            SensEtDeplacement(ESQUIVE_GAUCHE,vitesseM5,vitesseM0);}                                // poursuite
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
        case 62:                                 // esquive Gauche
          if(mouvement!=oldMouvement){reactionAvantGauche=millis();continueMouvement=true;}
          if((millis()-reactionAvantGauche)<dureeReactionAvantGauche){
            SensEtDeplacement(ESQUIVE_GAUCHE,vitesseM6,vitesseM0);}                                // défense
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
        case 63:                                 // tourne -45° (gauche)
          if(mouvement!=oldMouvement){reactionAvantGauche=millis();continueMouvement=true;}
          if((millis()-reactionAvantGauche)<dureeReactionAvantGauche){
            if(adversaireProche==true){
                  SensEtDeplacement(SUR_PLACE_GAUCHE,vitesseM4,vitesseM6);}                        // attaque-défense
            else{ SensEtDeplacement(TOURNE_GAUCHE,vitesseM0,vitesseM4);}}                          // poursuite
          else{continueMouvement=false;}
          oldMouvement=mouvement;break;
        }
      Serial.print("   mvt : ");Serial.print(mouvement);Serial.print(" * ");
      Serial.print(tempsDeCombat);Serial.println(" ms");}

// Adversaire NON détecté
      else{                                      // adversaire NON DETECTE !!!
        if(continueMouvement==true){
        switch(oldMouvement){                    // * perte de CONTACT -> continue mouvement
          case 30:
            if((millis()-reactionAvant)<dureeReactionAvant){
              SensEtDeplacement(TOUT_AVANT,vitesseM6,vitesseM7);}                   // proche
            else{continueMouvement=false;}
            break;
          case 31:
            if((millis()-reactionAvant)<dureeReactionAvant){
              if(adversaireProche==true){
                    SensEtDeplacement(TOUT_AVANT,vitesseM7,vitesseM7);}             // proche
              else{ SensEtDeplacement(TOUT_AVANT,vitesseM4,vitesseM4);}}            // loin
            else{continueMouvement=false;}
            break;
          case 32:
            if((millis()-reactionAvant)<dureeReactionAvant){
              SensEtDeplacement(TOUT_AVANT,vitesseM7,vitesseM6);}                   // proche
            else{continueMouvement=false;}
            break;
          case 33:
            if((millis()-reactionAvantDroite)<dureeReactionAvantDroite){
              if(adversaireProche==true){
                    SensEtDeplacement(SUR_PLACE_DROITE,vitesseM6,vitesseM4);}       // proche
              else{ SensEtDeplacement(TOURNE_DROITE,vitesseM4,vitesseM0);}}         // loin
            else{continueMouvement=false;}
            break;
          case 34:
            if((millis()-reactionAvantDroite)<dureeReactionAvantDroite){
              SensEtDeplacement(ESQUIVE_DROITE,vitesseM0,vitesseM6);}               // proche
            else{continueMouvement=false;}
            break;
          case 41:
            if((millis()-reactionDroite)<dureeReactionDroite){
              SensEtDeplacement(ESQUIVE_DROITE,vitesseM0,vitesseM5);}               // loin <-> proche
            else{continueMouvement=false;}
            break;
          case 42:
            if((millis()-reactionArriereDroite)<dureeReactionArriereDroite){
              SensEtDeplacement(ESQUIVE_DROITE,vitesseM0,vitesseM6);}               // proche
            else{continueMouvement=false;}
            break;
          case 43:
            if((millis()-reactionArriereDroite)<dureeReactionArriereDroite){
              SensEtDeplacement(SUR_PLACE_DROITE,vitesseM4,vitesseM4);}             // loin
            else{continueMouvement=false;}
            break;
          case 51:
            if((millis()-reactionArriere)<dureeReactionArriere){
              if(alternance==1){
                SensEtDeplacement(TOURNE_GAUCHE,vitesseM0,vitesseM6);}              // défense
              else{ SensEtDeplacement(TOURNE_DROITE,vitesseM6,vitesseM0);}}         // alternée
            else{continueMouvement=false;}
            break;
          case 52:
            if((millis()-reactionArriereGauche)<dureeReactionArriereGauche){
              SensEtDeplacement(SUR_PLACE_GAUCHE,vitesseM4,vitesseM4);}             // loin
            else{continueMouvement=false;}
            break;
          case 53:
            if((millis()-reactionArriereGauche)<dureeReactionArriereGauche){
              SensEtDeplacement(ESQUIVE_GAUCHE,vitesseM6,vitesseM0);}               // proche
            else{continueMouvement=false;}
            break;
          case 61:
            if((millis()-reactionGauche)<dureeReactionGauche){
              SensEtDeplacement(ESQUIVE_GAUCHE,vitesseM5,vitesseM0);}               // loin <-> proche
            else{continueMouvement=false;}
            break;
          case 62:
            if((millis()-reactionAvantGauche)<dureeReactionAvantGauche){
              SensEtDeplacement(ESQUIVE_GAUCHE,vitesseM6,vitesseM0);}               // proche
            else{continueMouvement=false;}
            break;
          case 63:
            if((millis()-reactionAvantGauche)<dureeReactionAvantGauche){
              if(adversaireProche==true){
                    SensEtDeplacement(SUR_PLACE_GAUCHE,vitesseM4,vitesseM6);}       // proche
              else{ SensEtDeplacement(TOURNE_GAUCHE,vitesseM0,vitesseM4);}}         // loin
            else{continueMouvement=false;}
            break;}
            }
          else{                                // * adversaire hors champ -> recherche
            if(alternance==1){                   // <- légérement à Gauche
              mouvement=71;
              SensEtDeplacement(TOUT_AVANT,vitesseM1,vitesseM2);
              if(mouvement!=oldMouvement){
                Serial.print("OK > G ");
                Serial.print("   mvt : ");Serial.print(mouvement);Serial.print(" * ");
                Serial.print(tempsDeCombat);Serial.println(" ms");
                oldMouvement=mouvement;}}
            else{                                // légérement à Droite ->
              mouvement=72;
              SensEtDeplacement(TOUT_AVANT,vitesseM2,vitesseM1);
              if(mouvement!=oldMouvement){
                Serial.print("OK > D ");
                Serial.print("   mvt : ");Serial.print(mouvement);Serial.print(" * ");
                Serial.print(tempsDeCombat);Serial.println(" ms");
                oldMouvement=mouvement;}}
        }}}                                      // * mouvement libre *

// Réaction LIGNE BLANCHE
    else{                                        // dernier cycle de réaction
      Serial.println("Fin Reaction Ligne Blanche ... ");
      finReactionBlancDevant=false;
      finReactionBlancGauche=false;
      finReactionBlancDroite=false;
      finReactionBlancDerriere=false;
      }}
  else{                                          // REACTION Ligne Blanche activée
    ilEstDerriere=(!digitalRead(pin_JS40F_RG))||(!digitalRead(pin_JS40F_RD));
    if(reactionBlancDevant==true){               // ... réaction 2 cny : Avant (21)
      mouvement=21;
      if(mouvement!=oldMouvement){
        Serial.print("<G ");
        //
        if(ilEstDerriere==true){SensEtDeplacement(TOUT_ARRIERE,vitesseM7,vitesseM5);}
        else{                   SensEtDeplacement(TOUT_ARRIERE,vitesseM4,vitesseM5);}
        //
        Serial.print(" <D   mvt : ");Serial.print(mouvement);Serial.print(" * ");
        Serial.print(tempsDeCombat);Serial.println(" ms");
        memoireLigneBlanche=mouvement;
        oldMouvement=mouvement;}}
    if(reactionBlancGauche==true){               // ... réaction cny : Gauche (22)
      mouvement=22;
      if(mouvement!=oldMouvement){
        Serial.print("<G ");
        //
        if(ilEstDerriere==true){SensEtDeplacement(TOUT_AVANT,vitesseM7,vitesseM4);}
        else{                   SensEtDeplacement(TOUT_ARRIERE,vitesseM1,vitesseM5);}
        //
        Serial.print("   mvt : ");Serial.print(mouvement);Serial.print(" * ");
        Serial.print(tempsDeCombat);Serial.println(" ms");
        memoireLigneBlanche=mouvement;
        oldMouvement=mouvement;}}
    if(reactionBlancDroite==true){               // ... réaction cny : Droite (23)
      mouvement=23;
      if(mouvement!=oldMouvement){
        //
        if(ilEstDerriere==true){SensEtDeplacement(TOUT_AVANT,vitesseM4,vitesseM7);}
        else{                   SensEtDeplacement(TOUT_ARRIERE,vitesseM5,vitesseM1);} 
        //
        Serial.print(" <D   mvt : ");Serial.print(mouvement);Serial.print(" * ");
        Serial.print(tempsDeCombat);Serial.println(" ms");
        memoireLigneBlanche=mouvement;
        oldMouvement=mouvement;}}
    if(reactionBlancDerriere==true){             // ... réaction cny : Derriere (24)
      mouvement=24;
      if(mouvement!=oldMouvement){
        Serial.print(" <D ");
        //
        if(adversaireProche==true){SensEtDeplacement(TOUT_AVANT,vitesseM5,vitesseM7);}
        else{                      SensEtDeplacement(TOUT_AVANT,vitesseM5,vitesseM1);}
        //
        Serial.print(" <G   mvt : ");Serial.print(mouvement);Serial.print(" * ");
        Serial.print(tempsDeCombat);Serial.println(" ms");
        memoireLigneBlanche=mouvement;
        oldMouvement=mouvement;}}
    }
//
// ----- fin d'exécution des mouvements
//
// ***** complémentation : durée de boucle de combat -> 3ms *****
  tempsBoucleCombat=millis()-debutBoucleCombat;
  Serial.print("   Boucle Combat : ");
  while(tempsBoucleCombat<dureeBoucleCombat){
    if(cptPoint%100==0){Serial.print("-");}
    cptPoint=cptPoint+1;
    delayMicroseconds(5);
    tempsBoucleCombat=millis()-debutBoucleCombat;}
  Serial.print(tempsBoucleCombat);Serial.println(" ms");

// ********** réinitialisation des conditions *********  
  verifie_bouton();
  verifie_tension();
  tempsDeCombat=millis()-topDepartCombat;
  }                                              // ... fin de BOUCLE WHILE
// ************************************************************************************************ 
//  
// ----- temps écoulé ou appui sur arrêt d'urgence
//
  Serial.print("FIN de PARCOURS : ");
  mouvement=10;
  ArretOuFreinage(0);
  Serial.print("   mvt : ");
  Serial.print(mouvement);
  Serial.print(" * ");
  Serial.print(tempsDeCombat);
  Serial.println(" ms");
  ledWork.on();
  while(1){;};                                   // attente -> reset pour redémarrer
//
}
//  -------------------------------------------------------
//  -------------------------------------------------------
//  -------------------------------------------------------
