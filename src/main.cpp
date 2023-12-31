//  -------------------------------------------------------
//  -------------- programme robot ROBOT 2024 --------------
//  ------------------- LYRA version 1.1 ------------------
//  -------------------------------------------------------

/* liste de courses : .......
  limiter le temps de déplacement à 1/2 dohyo en vitesse M7
  calculer le décalage systématique de la vitesse des moteurs, pour aller tout droit
  vérifier la détection présence pendant décomptage des 5 secondes
  affinage du mouvement si 4 capteurs actifs (grande Bataille)
  test de direction 1ère seconde : pb capteur RG
  améliorer la directivité des capteurs "presence" : guide de rayonnement IR
  vérification des "durées" 1ère seconde
*/


// 4/11/2023 - calcul des vitesses en fonction de la tension et des moteurs : L = V * t


// 31/10/2023- modif. interruption : suppression signal capteur si bruité (limite de détection)
// 29/10/2023- tests 1ère seconde pb -> côté gauche sans mvt - détection bord si proche
//             A : ok - AD : ok - D : ok - RD : ok - RG : NOK(G) - G : ->NOK(rien) - AG : ok 
//             -> capteur RD : NOK
//             Mode PROG obligatoire pour 3.3V - indifférent pour VIN (5V) -> PCB vers 1.2 !
//             Pont en H : ZK-5AD peut fournir le 3.3V pour MC14490 et LED
// 25/10/2023- montage maquette PCB (version 1.2) : mode "ON" <-> node "PROG"
// 21/10/2023- test de téléchargement en ouvrant 3.3V et 5V (VIN) : OK ->faire new PCB vers. 1.2 !
// 20/10/2023- traitement des interruptions : 3 boucles x 25 us ald 31x50 -> 75 us : OK
//             visibilité adversaire : BLANC -> 30cm - NOIR MAt -> 12.5cm
// 16/10/2023- calcul de tension LiPo avec pont diviseur (10k+3k+100) : y=0.2371x-0.0137
// 15/10/2023- test de fonctionnement des capteurs #2578 -> OK sur dohyo NOIR, pb sur BLANC
// 12/10/2023- désactivation du WiFi et du Bluetooth (dans le setup)
// 11/10/2023- remontage version PCB moteur & bouton v1.1 : pin ESP 2.5 + header 3.5mm
//             . new ESP32 usbC - new TB6612 - MC14490 cms - connecteur JST
//             . démarre "sans capteurs" : CNY=1 (noir*3) - #2578=0 (présence*7)
//             # capteur ligne non branché -> alerte ligne blanche "faux"
//             # capteur présence non branché -> alerte présence "vrai"
//             les moteurs ne démarrent pas :(
//             Vérif -> affectation, connexions, câbles - test : 2 x 2.5 sec -> OK :)
// 12/9/2023 - désoudage ESP32, PH5 : pin header 5mm sur PCB moteur, ESP : pin mâle 4 mm
//             inversion acquisition des pins CNY70 (MC14490 non inverseur) dans objet "NoirBlanc"
//             inversion acquisition des boutons bGO et tON (MC14490 non inverseur) dans objet "Bouton"
//             inventaire de tous les paramètres du programme : temps, vitesse, distance, etc ...
// 8/9/2023 -  Dépôt DISTANT : GitHub - Dépôt LOCAL : PC ATELIER
// 25/8/2023 - REACTION si ligne blanche ARRIERE verifie_noirblanc(), reactionLigneBlanche()
// 24/8/2023 - GitHub remote https://github.com/michelzahnd123/Robot-Sumo-Lyra
//             non inversion des pins capteurs POLOLU avec filtre MC14490
// 22/8/2023 - réaffectation des pins /nouveau PCB - inversion des pins moteurs (PCB moteur)
//             suppression : capteur presence AR, capteurs TOF, librairie AdaFruit
//             création du capteur YR (CNY70 arrière : pin 18) 
// 9/8/2023  - LYRA : création du programme & séparation de DAEMON-2023-V3 (IOREK)

// 11/7/2023 - IOREK : fin 1ère seconde -> limite tout_avant 3*duree 1ère seconde vitesse M6
//             Démontage de LYRA et reconstruction au Club Robot de St Sébastien s/ Loire
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
#include <WiFi.h>
#include <esp_bt.h>

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

// micro-moteurs CC (pont en H : TB6612)
#define pin_moteurGauche 33                      // moteur A (gauche)
#define pin_ArrMotGauche 14                      // A2 (ald 26)
#define pin_AvMotGauche 27                       // A1 (ald 25)
#define pin_AvMotDroit 26                        // B1 (ald 14)
#define pin_ArrMotDroit 25                       // B2 (ald 27)
#define pin_moteurDroit 32                       // moteur B (droit)

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

// vitesse des moteurs : identique moteur gauche et droit
#define vitesseM0ref   0
#define vitesseM1ref  27
#define vitesseM2ref  55
#define vitesseM3ref  82
#define vitesseM4ref 110
#define vitesseM5ref 137
#define vitesseM6ref 165
#define vitesseM7ref 180
volatile int vitesseM0, vitesseM1, vitesseM2, vitesseM3, vitesseM4, vitesseM5, vitesseM6, vitesseM7;

// timers des mouvements
volatile long reactionAvant, reactionAvantDroite, reactionDroite;
volatile long reactionArriereDroite, reactionArriere, reactionArriereGauche;
volatile long reactionGauche, reactionAvantGauche;
volatile bool continueMouvement;

// global combat
int cptPoint, cptExploration;
#define dureeTotalCombat 2500                                   // 2.5 secondes -> pour test
//#define dureeTotalCombat 120000                               // 2 minutes -> durée du combat
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

#define tensionReference          11.45

#define duree1SecondeAvant          150
#define duree1SecondeAvantDroite     50
#define duree1SecondeAvantGauche     50
#define duree1SecondeDroite         150
#define duree1SecondeGauche         150
#define duree1SecondeArriereDroite  200
#define duree1SecondeArriereGauche  200
#define duree1SecondeArriere        250

#define dureeReactionAvant          100
#define dureeReactionAvantDroite     50
#define dureeReactionAvantGauche     50
#define dureeReactionDroite         100
#define dureeReactionGauche         100
#define dureeReactionArriereDroite  125
#define dureeReactionArriereGauche  125
#define dureeReactionArriere        150

// capteurs "on/off" : POLOLU
#define pin_JS40F_A  15
#define pin_JS40F_AD 16
#define pin_JS40F_D  17
#define pin_JS40F_RD  5
#define pin_JS40F_RG 19
#define pin_JS40F_G  36
#define pin_JS40F_AG 39 

//#define latencePresence 2
volatile bool alertePresenceA, alertePresenceAD, alertePresenceD, alertePresenceRD;
volatile bool alertePresenceRG, alertePresenceG, alertePresenceAG;
volatile bool alertePresence, adversaireProche, lectureCapteur;
volatile bool ilEstDerriere;
volatile int indiceLecture, lecturePlus, nbCapteurActif;
volatile int nbInterruptionA, nbInterruptionAD, nbInterruptionD, nbInterruptionRD; 
volatile int nbInterruptionRG, nbInterruptionG, nbInterruptionAG;

// BFAF : blocage face à face
#define attenteBlocage             3000
#define dureeReactionBlocage        150
volatile long debutAnalyseBlocage;

// Ligne Blanche : durée de réaction
#define dureeEsquiveBlanc           150
#define dureeAvantBlanc             225
#define dureeDerriereBlanc          250

// Homologation : parcours 1 mètre en vitesse M3
#define distance1metre             1750

//  -------------------------------------------------------
//  ----- INTERRUPTION sur CAPTEURS -----
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuA()
{ 
  lecturePlus=0;                                           // lecture 3 fois 25 micro-secondes
  for(indiceLecture=0;indiceLecture<3;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_A);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    delayMicroseconds(25);}
  //
  if(lecturePlus==3){alertePresenceA=true;}                // valeur "presence" stabilisée
  else{alertePresenceA=false;}                             // "rien" stabilisé ou "parasite"
  //
  nbInterruptionA=nbInterruptionA+1;                       // compteur d'interruption
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuAD()
{
  lecturePlus=0;
  for(indiceLecture=0;indiceLecture<3;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_AD);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    delayMicroseconds(25);}
  //
  if(lecturePlus==3){alertePresenceAD=true;}
  else{alertePresenceAD=false;}
  //
  nbInterruptionAD=nbInterruptionAD+1;
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuD()
{
  lecturePlus=0;
  for(indiceLecture=0;indiceLecture<3;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_D);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    delayMicroseconds(25);}
  //
  if(lecturePlus==3){alertePresenceD=true;}
  else{alertePresenceD=false;}
  //
  nbInterruptionD=nbInterruptionD+1;
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuRD()
{
  lecturePlus=0;
  for(indiceLecture=0;indiceLecture<3;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_RD);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    delayMicroseconds(25);}
  //
  if(lecturePlus==3){alertePresenceRD=true;}
  else{alertePresenceRD=false;}
  //
  nbInterruptionRD=nbInterruptionRD+1;
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuRG()
{
  lecturePlus=0;
  for(indiceLecture=0;indiceLecture<3;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_RG);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    delayMicroseconds(25);}
  //
  if(lecturePlus==3){alertePresenceRG=true;}
  else{alertePresenceRG=false;}
  //
  nbInterruptionRG=nbInterruptionRG+1;
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuG()
{ 
  lecturePlus=0;
  for(indiceLecture=0;indiceLecture<3;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_G);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    delayMicroseconds(25);}
  //
  if(lecturePlus==3){alertePresenceG=true;}
  else{alertePresenceG=false;}
  //
  nbInterruptionG=nbInterruptionG+1;
}
//  -------------------------------------------------------
void IRAM_ATTR adversaireApparuAG()
{
  lecturePlus=0;
  for(indiceLecture=0;indiceLecture<3;indiceLecture++){
    lectureCapteur=!digitalRead(pin_JS40F_AG);
    if(lectureCapteur==true){lecturePlus=lecturePlus+1;}
    delayMicroseconds(25);}
  //
  if(lecturePlus==3){alertePresenceAG=true;}
  else{alertePresenceAG=false;}
  //
  nbInterruptionAG=nbInterruptionAG+1;
}
//  -------------------------------------------------------
//  ----- SETUP -----
//  -------------------------------------------------------
void setup() 
{
// Communication série - Wifi - bluetooth
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF); Serial.println("WiFi désactivé");
  esp_bt_controller_disable();Serial.println("Bluetooth désactivé");

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
  Serial.print("tension LiPo Reference : "); Serial.println(tensionReference);
  
// action 1ère seconde
  Serial.print("duree 1ere Seconde Avant          : "); Serial.println(duree1SecondeAvant);
  Serial.print("duree 1ere Seconde Avant Droite   : "); Serial.println(duree1SecondeAvantDroite);
  Serial.print("duree 1ere Seconde Avant Gauche   : "); Serial.println(duree1SecondeAvantGauche);
  Serial.print("duree 1ere Seconde Droite         : "); Serial.println(duree1SecondeDroite);
  Serial.print("duree 1ere Seconde Gauche         : "); Serial.println(duree1SecondeGauche);
  Serial.print("duree 1ere Seconde Arriere Droite : "); Serial.println(duree1SecondeArriereDroite);
  Serial.print("duree 1ere Seconde Arriere Gauche : "); Serial.println(duree1SecondeArriereGauche);
  Serial.print("duree 1ere Seconde Arriere        : "); Serial.println(duree1SecondeArriere);

// durée de réaction si adversaire détecté
  Serial.print("duree Reaction Avant          : ");Serial.println(dureeReactionAvant);
  Serial.print("duree Reaction Avant Droite   : ");Serial.println(dureeReactionAvantDroite);
  Serial.print("duree Reaction Avant Gauche   : ");Serial.println(dureeReactionAvantGauche);
  Serial.print("duree Reaction Droite         : ");Serial.println(dureeReactionDroite);
  Serial.print("duree Reaction Gauche         : ");Serial.println(dureeReactionGauche);
  Serial.print("duree Reaction Arriere Droite : ");Serial.println(dureeReactionArriereDroite);
  Serial.print("duree Reaction Arriere Gauche : ");Serial.println(dureeReactionArriereGauche);
  Serial.print("duree Reaction Arriere        : ");Serial.println(dureeReactionArriere);

// vitesse des moteurs      
  vitesseM0=int(vitesseM0ref*tensionReference/tensionLiPoMesuree);
    if(vitesseM0>0){vitesseM0=0;}
  vitesseM1=int(vitesseM1ref*tensionReference/tensionLiPoMesuree);
    if(vitesseM1>30){vitesseM1=30;}
  vitesseM2=int(vitesseM2ref*tensionReference/tensionLiPoMesuree);
    if(vitesseM2>60){vitesseM2=60;}
  vitesseM3=int(vitesseM3ref*tensionReference/tensionLiPoMesuree);
    if(vitesseM3>90){vitesseM3=90;}
  vitesseM4=int(vitesseM4ref*tensionReference/tensionLiPoMesuree);
    if(vitesseM4>120){vitesseM4=120;}
  vitesseM5=int(vitesseM5ref*tensionReference/tensionLiPoMesuree);
    if(vitesseM5>150){vitesseM5=150;}
  vitesseM6=int(vitesseM6ref*tensionReference/tensionLiPoMesuree);
    if(vitesseM6>180){vitesseM6=180;}
  vitesseM7=int(vitesseM7ref*tensionReference/tensionLiPoMesuree);
    if(vitesseM7>180){vitesseM7=180;}
  //
  Serial.print("vitesse M0 : ");Serial.println(vitesseM0);
  Serial.print("vitesse M1 : ");Serial.println(vitesseM1);
  Serial.print("vitesse M2 : ");Serial.println(vitesseM2);
  Serial.print("vitesse M3 : ");Serial.println(vitesseM3);
  Serial.print("vitesse M4 : ");Serial.println(vitesseM4);
  Serial.print("vitesse M5 : ");Serial.println(vitesseM5);
  Serial.print("vitesse M6 : ");Serial.println(vitesseM6);
  Serial.print("vitesse M7 : ");Serial.println(vitesseM7);

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
  ilEstDerriere=false;
  lecturePlus=0;nbCapteurActif=0;
  nbInterruptionA=0;nbInterruptionAD=0;nbInterruptionD=0;nbInterruptionRD=0;
  nbInterruptionRG=0;nbInterruptionG=0;nbInterruptionAG=0;

// REACTION : blocage - ligne blanche
/*
  dureeReactionBlocage=dureeReactionBlocageRef*tensionReactionRef/tensionLiPoMesuree;
  dureeEsquiveBlanc=dureeEsquiveBlancRef*tensionReactionRef/tensionLiPoMesuree;
  dureeAvantBlanc=dureeAvantBlancRef*tensionReactionRef/tensionLiPoMesuree;
  dureeDerriereBlanc=dureeDerriereBlancRef*tensionReactionRef/tensionLiPoMesuree;
*/
  Serial.print("duree Reaction Blocage : ");Serial.println(dureeReactionBlocage);
  Serial.print("duree Esquive Blanc : ");Serial.println(dureeEsquiveBlanc);
  Serial.print("duree Avant Blanc : ");Serial.println(dureeAvantBlanc);
  Serial.print("duree Derriere Blanc : ");Serial.println(dureeDerriereBlanc);
  Serial.println("");
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
    if(tempsEsquiveBlancDerriere>dureeDerriereBlanc){
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
// ***** capteur actif & actif avec interruption *****
nbCapteurActif=0;
//
if(nbInterruptionA>=1){                                              // Interruption A
  alertePresenceA=(alertePresenceA)&&(!digitalRead(pin_JS40F_A));}
else{alertePresenceA=!digitalRead(pin_JS40F_A);}
if(alertePresenceA==true){nbCapteurActif=nbCapteurActif+1;}                                        
//
if(nbInterruptionAD>=1){                                             // Interruption AD
  alertePresenceAD=(alertePresenceAD)&&(!digitalRead(pin_JS40F_AD));}
else{alertePresenceAD=!digitalRead(pin_JS40F_AD);}
if(alertePresenceAD==true){nbCapteurActif=nbCapteurActif+1;}                                        
//
if(nbInterruptionD>=1){                                             // Interruption D
  alertePresenceD=(alertePresenceD)&&(!digitalRead(pin_JS40F_D));}
else{alertePresenceD=!digitalRead(pin_JS40F_D);}
if(alertePresenceD==true){nbCapteurActif=nbCapteurActif+1;}
//
if(nbInterruptionRD>=1){                                             // Interruption RD
  alertePresenceRD=(alertePresenceRD)&&(!digitalRead(pin_JS40F_RD));}
else{alertePresenceRD=!digitalRead(pin_JS40F_RD);}
if(alertePresenceRD==true){nbCapteurActif=nbCapteurActif+1;}
//
if(nbInterruptionRG>=1){                                             // Interruption RG
  alertePresenceRG=(alertePresenceRG)&&(!digitalRead(pin_JS40F_RG));}
else{alertePresenceRG=!digitalRead(pin_JS40F_RG);}
if(alertePresenceRG==true){nbCapteurActif=nbCapteurActif+1;}
//
if(nbInterruptionG>=1){                                             // Interruption G
  alertePresenceG=(alertePresenceG)&&(!digitalRead(pin_JS40F_G));}
else{alertePresenceG=!digitalRead(pin_JS40F_G);}
if(alertePresenceG==true){nbCapteurActif=nbCapteurActif+1;}
//
if(nbInterruptionAG>=1){                                             // Interruption AG
  alertePresenceAG=(alertePresenceAG)&&(!digitalRead(pin_JS40F_AG));}
else{alertePresenceAG=!digitalRead(pin_JS40F_AG);}
if(alertePresenceAG==true){nbCapteurActif=nbCapteurActif+1;}
//
// ***** affichage *****
if(affichageVP==true){
  Serial.print("AVANT          : ");
    if(alertePresenceA==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbInterruptionA);

  Serial.print("AVANT DROITE   : ");
    if(alertePresenceAD==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbInterruptionAD);

  Serial.print("DROITE         : ");
    if(alertePresenceD==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbInterruptionD);

  Serial.print("ARRIERE DROITE : ");
    if(alertePresenceRD==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbInterruptionRD);

  Serial.print("ARRIERE GAUCHE : ");
    if(alertePresenceRG==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbInterruptionRG);

  Serial.print("GAUCHE         : ");
    if(alertePresenceG==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbInterruptionG);

  Serial.print("AVANT GAUCHE   : ");
    if(alertePresenceAG==true){Serial.print("=ADV  ");}else{Serial.print("rien  ");}
    Serial.print("  +>");Serial.print(nbInterruptionAG);
}
  
// ***** ré-initialisation pour le prochain cycle de 3ms *****
  nbInterruptionA=0;
  nbInterruptionAD=0;
  nbInterruptionD=0;
  nbInterruptionRD=0;
  nbInterruptionRG=0;
  nbInterruptionG=0;
  nbInterruptionAG=0;

// ***** calibrage Nb Capteurs Actifs *****
if(nbCapteurActif<=0){nbCapteurActif=0;}
if(nbCapteurActif==1){;}                    // 1 seule direction
if(nbCapteurActif==2){;}                    // adversaire proche
if(nbCapteurActif==3){;}                    // 3 cas : AvantDroit, AVANT ou AvantGauche
if(nbCapteurActif>=4){nbCapteurActif=4;}    // cas de la Great Battle
}
//  -------------------------------------------------------
void PresenceLoinPres(bool affichagePLP)
// définition des mouvements
// nb capteurs : 1->loin . 2->près . 3->contact avant . 0->rien
{
  verifie_presence(false);                                     // affichage des capteurs
  adversaireProche=false;

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
    case 0:                                                    // adv non détecté
      alertePresence=false;
      adversaireProche=false;
      break;
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
    case 4:                                                   // Grande Bataille !!!
      alertePresence=true;
      adversaireProche=true;
      // priorité avant
      if(alertePresenceA==true){
        if((alertePresenceD==true)&&(alertePresenceAD==true)){
          mouvement=33; directionAdversaire=AVANT_DROITE;}
        if((alertePresenceG==true)&&(alertePresenceAG==true)){
          mouvement=63; directionAdversaire=AVANT_GAUCHE;}
        if((alertePresenceAD==true)&&(alertePresenceAG==true)){
          mouvement=31; directionAdversaire=AVANT;}}
      else{
        // priorité à droite
        if(alertePresenceD==true){
          if((alertePresenceAD==true)&&(alertePresenceRD==true)){
            mouvement=41; directionAdversaire=DROITE;}
          if((alertePresenceRD==true)&&(alertePresenceRG==true)){
            mouvement=43; directionAdversaire=ARRIERE_DROITE;}}
        else{
          // priorité à gauche
          if(alertePresenceG==true){
            if((alertePresenceAG==true)&&(alertePresenceRG==true)){
              mouvement=61; directionAdversaire=GAUCHE;}
            if((alertePresenceRG==true)&&(alertePresenceRD==true)){
              mouvement=52; directionAdversaire=ARRIERE_GAUCHE;}}
          else{
          if((alertePresenceRG==true)&&(alertePresenceRD==false)){
            mouvement=52; directionAdversaire=ARRIERE_GAUCHE;}
          if((alertePresenceRG==false)&&(alertePresenceRD==true)){
            mouvement=43; directionAdversaire=ARRIERE_DROITE;}
      }}}
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

// lecture capteurs de présence : au démarrage (attente bGO et/ou tON)
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
// *** COMPTAGE SUSPENDU PENDANT les TESTS ***
//if(signalDepartTelecom==false&&signalBoutonBouton==true){decompte5secondes();}

// si TELECOM : parcours homologation -> 1m
if(signalDepartTelecom==true&&signalBoutonBouton==false){parcours1metre();}

// Dans tous les CAS : c'est parti !!!
topDepartCombat=millis();
tempsDeCombat=0;
/*
// *** TEST FONCTIONNEMENT des MOTEURS SEULS ***
  SensEtDeplacement(TOUT_AVANT,vitesseM6,vitesseM3);delay(2500);     // tourne 1 tour à droite
  ledWork.impulsion(0);
  SensEtDeplacement(TOUT_AVANT,vitesseM3,vitesseM6);delay(2500));    // tourne 1 tour à gauche
  ArretOuFreinage(0);
  ledWork.impulsion(1);ledWork.impulsion(0);
  while(1){;};                                                       // arrêt à la fin du Grand "8"
*/
//
// *** TEST FONCTIONNEMENT des 7 CAPTEURS #2578 ***
delay(2000);
PresenceLoinPres(false);
  if(alertePresenceA==true){ledWork.flashLumineux(4,1000);}else{ledWork.flashLumineux(1,1000);}
  if(alertePresenceAD==true){ledWork.flashLumineux(4,1000);}else{ledWork.flashLumineux(1,1000);}
  if(alertePresenceD==true){ledWork.flashLumineux(4,1000);}else{ledWork.flashLumineux(1,1000);} 
  if(alertePresenceRD==true){ledWork.flashLumineux(4,1000);}else{ledWork.flashLumineux(1,1000);}
  if(alertePresenceRG==true){ledWork.flashLumineux(4,1000);}else{ledWork.flashLumineux(1,1000);}
  if(alertePresenceG==true){ledWork.flashLumineux(4,1000);}else{ledWork.flashLumineux(1,1000);}
  if(alertePresenceAG==true){ledWork.flashLumineux(4,1000);}else{ledWork.flashLumineux(1,1000);} 
//while(1){;};                                                       // arrêt après 7 capteurs
//
// PREMIER MOUVEMENT - PREMIERE SECONDE
// *** DETECTION SUSPENDUE PENDANT les TESTS ***
//PresenceLoinPres(false);                         // lecture des 7 capteurs sans affichage

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
}
//
// *** TEST de DIRECTION -> PREMIERE SECONDE ***
  SensEtDeplacement(TOUT_AVANT,vitesseM6,vitesseM6);
  delay(100);
  ArretOuFreinage(0);
  while(1){;};                                   // visualisation 1ère seconde
//

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
