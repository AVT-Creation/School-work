//---Ajout des librairies---//
#include <Arduino.h>           // pour les fonctions arduino
#include <EEPROM.h>            // pour manipuler l'EEPROM
#include<ChainableLED.h>       // pour la led
#include<Tiny_BME280.h>
#include <Wire.h>              // pour l'horloge RTC
#include <RTClib.h>            // pour l'horloge RTC
#include <SoftwareSerial.h>    // Pour le gps
#include <SdFat.h>             // Pour la carte SD

//---Définition des pins---//
#define PUSHRED 2                             // La broche digitale du bouton rouge 
#define PUSHGREEN 3                           // La broche digitale du bouton vert
#define lumi A0                               // La broche analogique pour la luminosité, les broches analogique sont toujours en INPUT
#define DATA_PIN 7                            // La broche qui reçois les données pour la led
#define CLOCK_PIN 8                           // La broche qui gère la partie clock pour la led
#define CARD 4                                // La broche utilisée par le shield de la carte SD pour communiquer

#define bleu 0,0,255,127                      // La couleur bleu choisie
#define jaune 0,255, 255,0                    // La couleur jaune choisie
#define rouge 0,255, 0, 0                     // La couleur rouge choisie
#define vert 0,34, 240, 10                    // La couleur vert choisie
#define orange 0,230, 27, 0                   // La couleur orange choisie
#define blanc 0,254, 200, 240                 // La couleur blanche choisie
#define NUM_LEDS 1                            // Nombre de LED dans la chaîne


#define SdFat_DEBUG 0  // Désactiver le mode debug




ChainableLED leds(DATA_PIN, CLOCK_PIN, NUM_LEDS);        // déclare une variable de type ChainableLED , les pins qui seront utilisé pour la led ainsi que le nombre de led dans la série : 1


////////////////////////////////////////
//// Structure sur les prélèvements ////
////////////////////////////////////////

typedef struct {
  char nom[10];   // L'horodatage du prélèvement qui va servir à donnéer le nom du prélèvement
  char time[6];    // L'horodatage du prélèvement (heure, minute)
  char lum[5];     // capteur de luminosité, on sait que lum fera au maximum 4 octets (+1 pour le \0)
  char gps[25];     // GPS
  float hygro;   // l'humiditée de l'air
  float thermo;  // la température de l'air
  float atmo;    // la pression atmosphérique
} prelevement;

prelevement prel;  // Declaring an instance of the struct


////////--Définitions des adresses pour stocker sur l'EEPROM, il faut prendre en compte la place occupée par toutes les variables avant notre variable--////////

#define LOG_INTERVAL_ADDR 0                                               // L'intervalle de temps entre 2 prises capteurs
#define FILE_MAX_SIZE_ADDR (LOG_INTERVAL_ADDR + sizeof(int))              // La taille maximale du fichier avant de le fermer                      
#define TIMEOUT_ADDR (FILE_MAX_SIZE_ADDR + sizeof(int))                   // La durée durant laquelle on attends que un capteur réponde, si elle est dépassé, on déclare une erreur

#define LUMIN_ADDR (TIMEOUT_ADDR + sizeof(int))                           // La variable qui définie si l'on prends les données du capteur de luminosité
#define LUMIN_LOW_ADDR (LUMIN_ADDR + sizeof(bool))                        // La variable qui définie la valeur du port analogique en dessous de laquelle la luminosité est considérée comme faible
#define LUMIN_HIGH_ADDR (LUMIN_LOW_ADDR + sizeof(int))                    // La variable qui définie la valeur du port analogique au dessus de laquelle la luminosité est considérée comme haute 

#define TEMP_AIR_ADDR (LUMIN_HIGH_ADDR + sizeof(int))                     // La variable qui définie si l'on prends les données pour la température de l'air
#define MIN_TEMP_AIR_ADDR (TEMP_AIR_ADDR + sizeof(int))                   // La variable qui définie la valeur basse à partir de laquelle on considère que le capteur est en erreur
#define MAX_TEMP_AIR_ADDR (MIN_TEMP_AIR_ADDR + sizeof(int))               // La variable qui définie la valeur haute à partir de laquelle on considère que le capteur est en erreur

#define HYGR_ADDR (MAX_TEMP_AIR_ADDR + sizeof(int))                       // La variable qui définie si l'on prends les données pour l'humidité
#define HYGR_MIN_ADDR (HYGR_ADDR + sizeof(bool))                          // La variable qui définie la valeur basse à partir de laquelle la température ne permet pas de prendre les données
#define HYGR_MAX_ADDR (HYGR_MIN_ADDR + sizeof(int))                       // La variable qui définie la valeur haute à partir de laquelle la température ne permet pas de prendre les données

#define PRESSURE_ADDR (HYGR_MAX_ADDR + sizeof(int))                       // La variable qui définie si l'on prens les données pour la pression atmosphérique
#define PRESSURE_MIN_ADDR (PRESSURE_ADDR + sizeof(bool))                  // La variable qui définie la valeur basse à partir de laquelle on considère que la prise du capteur pour la pression est erronée
#define PRESSURE_MAX_ADDR (PRESSURE_MIN_ADDR + sizeof(int))               // La variable qui définie la valeur haute à partir de laquelle on considère que la prise du capteur pour la pression est erronée






//---Définitions des paramètres de fonctionnement---//


//---Variables globales---//
volatile byte cntcapt;  //On initialise le timer de prélèvement
volatile int cntbouton = 5000; 		    // 5 secondes
volatile byte cntdemarrage = 5; 	    // 5 secondes au démarrage pour entrer en mode maintenance
volatile int cntconf = 1600; 		      // initialisé dans ModeConfig
volatile bool red_pushed = 0; 		    // Boutton rouge : 	0 = boutton relaché, 1 = boutton appuyé
volatile bool green_pushed = 0; 	    // Boutton vert : 	0 = boutton relaché, 1 = boutton appuyé
volatile byte current_mode = 0; 		  // 0 = standard, 1 = maintenance,  2 = eco, 3 = config
volatile bool tp_capt = 0;            // flag qui nous sert à savoir si un capteur ne répond pas           
volatile byte nb_er = 0;              // La variable qui compte le nombre d'erreur de capteurs, à partir de 2, on lance le script de l'erreur led
volatile bool prise = 0;
volatile byte TIMEOUT;

// On paramêtre notre liaison série émulée et notre ChainableLED





//---Déclaration des fonctions---//


void initialisationtimer1();  // Déclaration de la fonction qui initialise notre timer 1 et le démmare qui déclenche notre ISR1 chaque secondes, utilisé pour la gestion de quand prendre les capteurs 

void initialisationtimer2();  // Déclaration de la fonction qui initialise notre timer 2 qui déclenche notre ISR2 chaque milisecondes, utilisé pour la gestions des boutons

void interruption();          // Déclaration de la fonction qui attache nos boutons aux intérruptions

void ModeStandard();          // Déclaration de la fonction qui gère le mode standard

void ModeMaintenance();       // Déclaration de la fonction qui gère le mode maintenance

void ModeEconomie();          // Déclaration de la fonction qui gère le mode économie

void ModeConfig();            // Déclaration de la fonction qui gère le mode configuration

void breakTimer2();           // Déclaration de la fonction qui se chargera d'arrêter notre timer 2   

void Timer2();                // Déclaration de la fonction qui se chargera de démmarer notre timer 2

void changementRouge();       // Déclaration de la fonction qui s'occupe de

void changementVert();        // Déclaration de la fonction qui s'occupe de

void changementDemarrage();   // Déclaration de la fonction qui s'occupe de choisir le mode au démarrage de notre station météo

void RedChange();

void GreenChange();


void preleve(prelevement *prel);     // Déclaration de la fonction prélèvement

void luminosite(prelevement *prel);  // Déclaration de la fonction qui prends la luminositée

void hygrometrie(prelevement *prel); // Déclaration de la fonction qui prends les données liées aux BME280 (Température de l'air,pression atmosphérique, hygrométrie)

void time(prelevement *prel);        // Déclaration de la fonction qui prends les données liés à la RTC (heure,minutes,jours)

void GPS(prelevement *prel);         // Déclaration de la fonction qui prends la latitude et la longitude

void save(prelevement *prel);        // Déclaration de la fonction de sauvegarde des données



void stad ();                        // allumer la led pour le Mode standard

void cfg ();                         // allumer la led pour le Mode config

void eco ();                         // allumer la led pour le Mode economie

void mtn ();                         // allumer la led pour le Mode Maintenance 

void SetErrors(int er_code);         // Déclaration de la fonction qui allume les leds en fonction du paramêtre er_code

void clk_er(int er_code);                       // Erreur de la clock RTC

void gps_er(int er_code);                       // Erreur dU GPS

void sens_er(int er_code);                      // Erreur d'un / plusieurs capteur(s)

void data_er(int er_code);                      // Erreur de la validité des données reçues

void sdf_er(int er_code);                       // Capacité pleine sur la carte sd

void sda_er(int er_code);                       // erreur d'accès et ou d'écriture sur la carte sd



void setup() {
  Serial.begin(9600);
  Wire.begin();
  int log_interval_value;
  EEPROM.get(LOG_INTERVAL_ADDR, log_interval_value);
  cntcapt =log_interval_value;  //On initialise le timer de prélèvement
  initialisationtimer1();
  initialisationtimer2(); 
  interruption();               //On paramètre les pins 2 et 3 comme interruption	
  pinMode(PUSHGREEN, INPUT_PULLUP);   /// On définit notre pin lié au bouton vers comme un input avec une résistance de tirage
  pinMode(PUSHRED, INPUT_PULLUP);     /// On définit notre pin lié au bouton vers comme un input avec une résistance de tirage
  Serial.println("F_set");
  prelevement prel;                   /// On déclare la variable prel de type prélèvement
  
  ModeStandard();                     /// On débute toujours avec le mode standard (On peux bien évidemment changer de mode après et il y a quelques secondes pour passer au mode configuration)
}

void loop() {
///On met dans le loop toutes les fonction avec un while, les interruptions vont changer les flags, ce qui activera ses fonctions, car activer un while dans une fonction d'interruption est interdit
if(current_mode == 3){ModeConfig();} // Le mode config, la variable current mode peux être changé par l'ISR2 durant les premières secondes de fonctionnement
if(prise == 1){preleve(&prel);} // La fonction pour les prélèvement, elle appelle à plusieurs moments des while dans le débug en cas de capteur défaillant et pour récolter les données GPS dans le bon format
}



//---Gestion des modes---//
void ModeStandard(){
  stad();
  if (current_mode == 2){
    int log_interval;
    EEPROM.get(LOG_INTERVAL_ADDR,log_interval);
    cntcapt = log_interval;
  }
  current_mode = 0;
}

void ModeMaintenance(){
  current_mode = 1;        // On donne la valeur 2 pour indiquer que le mode actuel est le mode économie
	mtn();
	//désactiver les sauvegardes, prélèvements consultables en série
}

void ModeEconomie(){
  eco();
  current_mode = 2; // On donne la valeur 2 pour indiquer que le mode actuel est le mode économie
  int log_interval;
  EEPROM.get(LOG_INTERVAL_ADDR,log_interval);
  cntcapt = (2*log_interval);  // On met le compteur capteur comme étant 2 fois celui des autres modes, ce qui nous permet par la même occasion de reset le compteur capteur
}

void ModeConfig(){
	cfg();
  String input;                                                             /// On déclare la variable qui va prendre l'entrée de l'utilisateur
  Serial.println("Entrez commande");
  Serial.read();                                                            /// On vide le port série pour préparer la bonne réception de l'input
  if(!Serial.available()){while(!Serial.available()){}}
    if(Serial.available()){String input = Serial.readStringUntil('\n');
    input.toUpperCase();  // Mise à jour de la chaîne en majuscules pour plus de confort, l'utilisateur peux ainsi taper par exemple rEsET et donc celà deviendra RESET
    if(input == "RESET"){
      EEPROM.put(LOG_INTERVAL_ADDR,600);
      EEPROM.put(FILE_MAX_SIZE_ADDR,2048);
      EEPROM.put(TIMEOUT_ADDR,30);
      EEPROM.put(LUMIN_ADDR,1);
      EEPROM.put(LUMIN_LOW_ADDR,255);
      EEPROM.put(LUMIN_HIGH_ADDR,768);
      EEPROM.put(TEMP_AIR_ADDR,1);
      EEPROM.put(MIN_TEMP_AIR_ADDR,-10);
      EEPROM.put(MAX_TEMP_AIR_ADDR,60);
      EEPROM.put(HYGR_ADDR,1);
      EEPROM.put(HYGR_MIN_ADDR,0);
      EEPROM.put(HYGR_MAX_ADDR,50);
      EEPROM.put(PRESSURE_ADDR,1);
      EEPROM.put(PRESSURE_MIN_ADDR,850);
      EEPROM.put(PRESSURE_MAX_ADDR,1080);
      Serial.println("RESET réussi");
    }
    if(input == "VERSION"){
      Serial.println("Version 1.0");
    }
    // Séparation de la chaine de caractère
    int i = 0;
    String data[3];

    while (input.length() > 0 && i < 3){
      byte index = input.indexOf('=');  // On recherche le =
      if (index == -1) {
      // Si pas de "=", on prend la partie suivante et on quitte
      data[i] = input;
      break;
      }
      else{
        // Extraire la sous-chaine jusqu'au "="
        data[i] = input.substring(0, index);  // Partie avant le "=" qui est notre intruction
        input = input.substring(index + 1);   // Partie après le "=" qui est la valeur à mettre pour la variable
        
        // Extraire la partie après l'égalité et la stocker dans data[i+1]
        index = input.indexOf(' ');  // Chercher un espace (si besoin car celà donne plus de confort à la personne qui tape la commande)
        if (index == -1) {
          data[i + 1] = input;  // Si pas d'espace, prendre le reste de la chaîne
          input = "";
        }
        else {
          data[i + 1] = input.substring(0, index);
          input = input.substring(index + 1);  // Supprimer la partie extraite
        }
      }
    }

    //---Entrées à paramètres---//
    if(data[0] == "LOG_INTERVAL"){
      EEPROM.put(LOG_INTERVAL_ADDR,data[1].toInt());
    }
    if(data[0] == "FILE_MAX_SIZE"){
      EEPROM.put(FILE_MAX_SIZE_ADDR,data[1].toInt());
    }
    if(data[0] == "TIMEOUT"){
      EEPROM.put(TIMEOUT_ADDR,data[1].toInt());
    }
    if(data[0] == "LUMIN"){
      EEPROM.put(LUMIN_ADDR,data[1].toInt());
    }
    if(data[0] == "LUMIN_LOW"){
      EEPROM.put(LUMIN_LOW_ADDR,data[1].toInt());
    }
    if(data[0] == "LUMIN_HIGH"){
      EEPROM.put(LUMIN_HIGH_ADDR,data[1].toInt());
    }
    if(data[0] == "TEMP_AIR"){
      EEPROM.put(TEMP_AIR_ADDR,data[1].toInt());
    }
    if(data[0] == "MIN_TEMP_AIR"){
      EEPROM.put(MIN_TEMP_AIR_ADDR,data[1].toInt());
    }
    if(data[0] == "MAX_TEMP_AIR"){
      EEPROM.put(MAX_TEMP_AIR_ADDR,data[1].toInt());
    }
    if(data[0] == "HYGR"){
      EEPROM.put(HYGR_ADDR,data[1].toInt());
    }
    if(data[0] == "HYGR_MIN"){
      EEPROM.put(HYGR_MIN_ADDR,data[1].toInt());
    }
    if(data[0] == "HYGR_MAX"){
      EEPROM.put(HYGR_MAX_ADDR,data[1].toInt());
    }
    if(data[0] == "PRESSURE"){
      EEPROM.put(PRESSURE_ADDR,data[1].toInt());
    }
    if(data[0] == "PRESSURE_MIN"){
      EEPROM.put(PRESSURE_MIN_ADDR,data[1].toInt());
    }
    if(data[0] == "PRESSURE_MAX"){
      EEPROM.put(PRESSURE_MAX_ADDR,data[1].toInt());
    }
    }
    cntconf = 1600; // On remet le compteur de 30 minutes car l'utilisateur à donc été actif comme il a entré une commande
  //}
	//Quitte le mode après 30 min d'inactivité (interruption timer)  conformément au cahier des charges, c'est le seul moyen possible pour sortir du mode configuration 
}


//---Fonctions pour changer de mode---//

void RedChange(){
    if (red_pushed == 0){ //on a appuyé sur le boutton 
        red_pushed = 1;
		cntbouton = 5000;
        Timer2();
    }
    else{ //on a relaché le boutton
        red_pushed = 0;
        breakTimer2();
    }
}

void GreenChange(){
  if (green_pushed == 0){ //on a appuyé sur le boutton 
      green_pushed = 1;
  cntbouton = 5000;
      Timer2();
  }
  else{ //on a relaché le boutton
      green_pushed = 0;
      breakTimer2();
  }
}

void changementRouge(){ //changer de mode
    if (current_mode != 1){
        ModeMaintenance();
    }
    else{
        ModeStandard();
    }
}

void changementVert(){ //changer de mode
    if (current_mode != 2){
        ModeEconomie();
    }
    else{
        ModeStandard();
    }
}

void changementDemarrage(){
	ModeConfig();
}


//---Gestion des interruptions et timers---//
void interruption(){ /// On attaches nos pin digitales 2 et 3 à des interruptions
    attachInterrupt(digitalPinToInterrupt(PUSHRED),RedChange,CHANGE);
    attachInterrupt(digitalPinToInterrupt(PUSHGREEN),GreenChange,CHANGE);
}

ISR(TIMER2_COMPA_vect){///ISR pour les boutons
  if(cntbouton >=0 ){
    cntbouton -= 4;
  //Si le conter n'est pas à 0, on le réduit de 4 car notre ISR prends environ 3ms à s'exécuter
	if(cntbouton <= 0){ // Quand le compteur atteinds 0, on change de mode
    cntbouton = 5000;
		if (!(PIND & (1 << PD2))){changementRouge();} /// C'est le bouton rouge qui est pressé, alors on effectue le changement de mode en fonction du bouton rouge
		if (!(PIND & (1 << PD3))){changementVert();} /// C'est le bouton vert qui est pressé, alors on effectue le changement de mode en fonction du bouton vert
	}}
  if(cntdemarrage > 0){
    if(!(PIND & (1 << PD2))){current_mode = 3;}
    }
}

ISR(TIMER1_COMPA_vect){///ISR pour les capteurs
  ///Lié à quand effectuer un prélèvement
  if(cntcapt){
  cntcapt--;
  Serial.println(cntcapt);
  if(!cntcapt){prise = 1;} /// On active le booléen qui gère si la fonction de la prise des capteurs s'effectue
  }
  

  ////Lié à la sortie du mode config
  if(current_mode == 3){
  if(cntconf){
    cntconf--;
   if(!cntconf){ModeStandard();}
    }
  }

  // La partie qui en fonction de si le système viens de démarrer et que le bouton rouge est préssé, cntdemarrage n'est jamais réinitialisé, donc il ne peux s'exécuter que dans les toutes premières segondes du lancement
  if(cntdemarrage){cntdemarrage--;}

  ////lié à la non réponse d'un capteur
  if (tp_capt) {      // Si capteur ne capte pas les données
  if (TIMEOUT > 0) {TIMEOUT--; 
    }
   else {///Quand le capteur est à 0
     tp_capt = 0;    // Désactiver la capture après expiration du timer
     nb_er++;
      }}
  }

void initialisationtimer1(){///Le timer qui s'active toute les secondes, toujours fonctionnel
  cli(); // Désactiver les interruptions
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 62499; // Comparaison avec le Timer (1s)
  TCCR1B |= (1 << WGM12) | (1 << CS12);   // Mode CTC (Clear Timer on Compare) et Prescaler à 256   
  TIMSK1 |= (1<< OCIE1A);   // Active la comparaison
}

void initialisationtimer2() {//Timer des boutons chaque milisecondes
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 63; // Comparaison avec le Timer (1 ms)
  TCCR2B |= (1 << WGM21) | (1 << CS22);   // Mode CTC (Clear Timer on Compare) et Prescaler à 256  
  sei(); // Réactiver les interruptions
}

void Timer2(){
  TIMSK2 |= (1 << OCIE2A);//On active la comparaison(ce qui enclenche ISR2 chaque ms)
}

void breakTimer2(){
  TIMSK2 &= ~(1 << OCIE2A);//On supprime la comparison(ce qui dé-enclenche ISR chaque ms)
}



/////////////////////////////////////////
//// Fonction pour les prélèvements /////
/////////////////////////////////////////

void preleve(prelevement *prel) {
  luminosite(prel);  /// On appelle le capteur de luminosité

  hygrometrie(prel); /// On appelle le capteur d'hygrométrie

  time(prel);        /// On appelle la fonction qui prends les données de la RTC
  Serial.println(prel->lum);
  Serial.println(prel->hygro);
  Serial.println(prel->thermo);
  Serial.println(prel->atmo);
  Serial.println(prel->nom);
  Serial.println(prel->time);
  
  static bool prise_gps = false; // Variable statique pour le mode économie
  if (current_mode == 2) {
    if (prise_gps) {  // Si c'est la deuxième mesure, on ne prends pas les données
        strcpy(prel->gps , "NA"); // Donnée GPS non disponible, on met donc "NA"
    } else {           // Si c'est la première mesure
    GPS(prel);     // Prendre les données du GPS
    }
    prise_gps = !prise_gps; // Inverse le booléen pour la prochaine fois
   } else {
    GPS(prel); // Prendre les données GPS si l'on est dans un autre mode
    } 
    
   Serial.println(prel->gps);
  if(current_mode != 1){save(prel);  // Si l'on n'est pas dans le mode maintenance, on va sauvegarder les données
  }

  nb_er = 0;  ////Comme la parties des capteurs est finie, on réinitialise le compteurs d'erreur de capteurs
  prise = 0;  //// On réinitialise le timer pour la prise des capteurs
  int log_interval;
  EEPROM.get(LOG_INTERVAL_ADDR, log_interval); // On prends notre variable directement de l'EEPROM
  if(current_mode == 2){cntcapt = (2 * log_interval);} //On réinitialise le timer comme nous somme en mode écho, on prends les mesures 2 fois moins souvent
  else{cntcapt = log_interval;} // On réinitialise le timer avec sa valeur normale
}


//////////////////////////
// Fonction pour la lum //
//////////////////////////


void luminosite(prelevement *prel) {//  Marche comme voulu
  bool LUMIN;
  EEPROM.get(LUMIN_ADDR,LUMIN);
  if(LUMIN){
  if (analogRead(lumi) < 100 & !tp_capt) {/// Si on trouve que le capteur à un problème
    tp_capt = 1;
    byte TIMEOUT_init; // On réinitialise TIMEOUT
    EEPROM.get(TIMEOUT_ADDR, TIMEOUT_init);
    TIMEOUT = TIMEOUT_init;
    while(analogRead(lumi) < 100 && tp_capt) {}  // on bloque le programme, et on le relache quand le flag tp_capt change
    if(!tp_capt){///Si les 5 secondes sont passées, lum devient NA et on quite le programme
      strcpy(prel->lum, "NA");
      nb_er++; ///On augmente la varialbe du nombre d'erreur de 1
    }
  } 
  else {
    tp_capt = 0;  // si on a reçut des données du capteur de luminosité, on remet la variable à 0
    int lumino = analogRead(lumi);
    int LUMIN_LOW,LUMIN_HIGH;
    EEPROM.get(LUMIN_LOW_ADDR,LUMIN_LOW);
    EEPROM.get(LUMIN_HIGH_ADDR,LUMIN_HIGH);
    if (lumino < LUMIN_LOW) {
    strcpy(prel->lum, "LOW");  // Light is low
    }
    else if (lumino > LUMIN_HIGH) {
    strcpy(prel->lum, "HIGH"); // Light is high
    }
    else {
    strcpy(prel->lum, "MEDI");  // Light is average
    }
  }
  } else {strcpy(prel->lum, "NA");}
}

//////////////////////////
// Fonction pour l'hygro //
//////////////////////////


void hygrometrie(prelevement *prel) {
    // Initialize BME280
    Tiny_BME280 bme;

    // Try to initialize the BME280 sensor
    if (!bme.begin(0x76)) { // Si le capteur d'hygrométrie ne se lance pas
        tp_capt = 1; // Signal that the capture process is in a waiting state
        int TIMEOUT_initial; // On réinitialise TIMEOUT
        EEPROM.get(TIMEOUT_ADDR, TIMEOUT_initial);
        TIMEOUT = TIMEOUT_initial;

        // Retry until the sensor is available or a timeout occurs
        while (!bme.begin(0x76) && tp_capt) {}
        ///Lié au fait que les 2 capteurs n'ont pas répondu
       if(nb_er >= 2){SetErrors(3);}
        
        // Set values to zero if sensor initialization fails
        prel->atmo = 0.0;   // Valeur incohérente
        prel->thermo = 0.0; // Valeur incohérente
        prel->hygro = 0.0;  // Valeur incohérente
    } else {
        tp_capt = 0; // Reset the capture flag

        // Read data from the BME280 sensor
        float pression = bme.readPressure() / 100.0; // Convert pressure to hPa
        float temperature = bme.readTemperature(); // Temperature in Celsius
        float humidite = bme.readHumidity(); // Humidity in percentage

        // Process pressure
        bool PRESSURE;
        EEPROM.get(PRESSURE_ADDR, PRESSURE);
        if (PRESSURE) {
            int PRESSURE_MAX, PRESSURE_MIN;
            EEPROM.get(PRESSURE_MAX_ADDR, PRESSURE_MAX);
            EEPROM.get(PRESSURE_MIN_ADDR, PRESSURE_MIN);
            if (pression < PRESSURE_MAX && pression > PRESSURE_MIN) {
                prel->atmo = pression; // Store valid pressure value
            } else {
                SetErrors(4);  // donnée incohérente, on invoque l'erreur
            }
        } else {
            prel->atmo = 0.0; // Comme la prise est désactivée, on met la valeur à 0
        }

        // Process temperature
        bool TEMP_AIR;
        EEPROM.get(TEMP_AIR_ADDR, TEMP_AIR);
        if (TEMP_AIR) {
            int MIN_TEMP_AIR, MAX_TEMP_AIR;
            EEPROM.get(MIN_TEMP_AIR_ADDR, MIN_TEMP_AIR);
            EEPROM.get(MAX_TEMP_AIR_ADDR, MAX_TEMP_AIR);
            if (temperature < MAX_TEMP_AIR && temperature > MIN_TEMP_AIR) {
                prel->thermo = temperature; // Store valid temperature value
            } else {
                SetErrors(4);  //La valeur est incohérente... on invoque l'erreur
            }
        } else {
            prel->thermo = 0.0; // Valeur incohérente
        }

        // Process humidity
        bool HYGR;
        EEPROM.get(HYGR_ADDR, HYGR);
        if (HYGR) {
            int HYGR_MAX, HYGR_MIN;
            EEPROM.get(HYGR_MAX_ADDR, HYGR_MAX);
            EEPROM.get(HYGR_MIN_ADDR, HYGR_MIN);
            if (temperature < HYGR_MAX && temperature > HYGR_MIN) {
                prel->hygro = humidite; // Store valid humidity value
            } else {
                prel->hygro = 0.0; // Comme la température ne permet pas de prendre la donnée pour l'hygrométrie, on la met à 0
            }
        } else {
            prel->hygro = 0.0; // Comme le capteur est désactivé , on renvoie la valeur 0
        }
    }
}

//////////////////////////
// Fonction pour la rtc //
//////////////////////////

void time(prelevement *prel){ //lorsque nous enregistrons nos données, il nous faut appeller le fichier par annee mois jour _ num de révision
  RTC_DS1307 rtc;
  if (! rtc.begin()) { /// Si la rtc ne démarre pas,
    SetErrors(1); /// On appelle la fonction des leds
  }
  else{
  static bool init_clock = 1;
  if(init_clock){rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));init_clock = 0;} // On initialise l'heure lors du début du programme
  DateTime now = rtc.now(); /// On déclare la variable now qui stocke notre rtc
  // On stocke le nom du fichier On lui donne l'année, puis le mois, puis le jours, et le numéro de révision
   sprintf(prel->nom,"%04d%02d%02d_",now.year(),now.month(),now.day());
   ///L'horodatage de la mesure
   sprintf(prel->time, "%02d:%02d",now.hour(),now.minute());
  }
}

//////////////////////////
// Fonction pour le GPS //
//////////////////////////

void GPS(prelevement *prel) {
    SoftwareSerial ss(6, 7);  // Declare pins for software serial
    ss.begin(9600);
  
    // Check if the GPS is available
    if (!ss.read()) { 
        SetErrors(2); // on lance l'erreur
        return;
    }

    char gpsData[58]; // Buffer to store GPS data
    tp_capt = 1;  // Initialize capture flag

    // Attempt to read GPS data until a GPGGA sentence is found or timeout occurs
    while (tp_capt) {
        if (ss.available()) {
            int len = ss.readBytesUntil('\n', gpsData, sizeof(gpsData) - 1);
            gpsData[len] = '\0'; // Null-terminate the string
            if (strncmp(gpsData, "$GPGGA", 6) == 0) {
                break; // On a obtenu la trame voulue
            }
        }
    }

    tp_capt = 0;
    ss.flush();

    // Tokenize the GPS data
    char *token = strtok(gpsData, ","); // Tokenize the GPS data
    int i = 0;
    char *data[10]; // Array to store different parts of the GPS data

    while (token != NULL && i < 10) {
        data[i++] = token; // Add the token to the array
        token = strtok(NULL, ","); // Get the next token
    }

    // Ensure that enough data is available before formatting
    if (i > 5) { // Check if there are enough data parts
        // Format the GPS coordinates into the prel structure
        snprintf(prel->gps, sizeof(prel->gps), "%s%s  %s%s", data[2], data[3], data[4], data[5]);
    } else {
        strcpy(prel->gps, "NA"); // Indicate values are not available
    }
}



////////////////////////////////////////
//     Fonctions pour sauvegarder     //
////////////////////////////////////////


void save(prelevement *prel) {
    SdFat sd; // Variable de type Sd_FAT
    // Initialiser la liaison avec la carte SD. En cas d'erreur, invoquer l'erreur d'accès à la carte SD
    if (!sd.begin(CARD)) {
        SetErrors(6);
        return; // Sortir si l'initialisation échoue
    }

    // Récupérer l'indice en octets maximal pour un fichier dans l'EEPROM
    int max_size;
    EEPROM.get(FILE_MAX_SIZE_ADDR, max_size);

    // Déclaration de la variable du type SdFile pour ouvrir les fichiers
    SdFile file;

    // Essayer de trouver une place pour notre fichier
    char truename[20]; // Taille pour les noms de fichiers
    int size;

    // Trouver un nom de fichier valide
    for (byte i = 0; i < 250; i++) {
        sprintf(truename, "%s%d.txt", prel->nom, i);
        if (file.open(truename, O_READ)) {
            size = file.fileSize();
            file.close();
            if (size + 256 < max_size) {
                break; // On a trouvé un nom de fichier valide
            }
        } else {
            break; // Le fichier n'existe pas encore
        }
    }

    // Si le fichier n'existe pas encore, le créer et ajouter les en-têtes
    if (!sd.exists(truename)) {
        if (file.open(truename, O_WRONLY | O_CREAT | O_AT_END)) {
            file.print("  H            ,Pos                                 ,Temp             ,Lum            ,Atm        ,       Hum\n");
            file.close(); // Fermer le fichier après avoir écrit les en-têtes
        }
    }

    // Enregistrement des valeurs de nos prélèvements dans le fichier
    if (file.open(truename, O_WRONLY | O_AT_END)) {
        file.print(prel->time);
        file.print("       "); // Espacement entre les colonnes
        file.print(prel->gps);
        file.print("          ");
        if(prel->thermo == 0.0){file.print("NA");}else{file.print(prel->thermo);}
        file.print("        ");
        file.print(prel->lum);
        file.print("          ");
        if(prel->atmo == 0.0){file.print("NA");}else{file.print(prel->atmo);}
        file.print("         ");
        if(prel->hygro == 0.0){file.print("NA");}else{file.print(prel->hygro);} // Écrit une nouvelle ligne après les valeurs
        file.print("\n");
        file.close(); // Fermer le fichier après l'écriture
        Serial.println("Sauvegarde réussie");
    }
}




//---DEFINITION DES MODES---//

void stad () {///Mode standard

 leds.setColorRGB(vert);}

void cfg () {
  
  leds.setColorRGB(jaune);}

void eco () {
  
  leds.setColorRGB(bleu);}

void mtn () {///Mode Maintenance 

 leds.setColorRGB(orange);}


//---DEFINITION DES ERREURS---//

/*Les erreurs vont rester coincées dans une boucle while jusqu'à
la résolution de l'erreur, les prélèvements et toute autre interaction
vont être ignorées*/

void clk_er(int er_code) {/// Erreur de la clock RTC

    while (er_code == 1){
        leds.setColorRGB(rouge);
        delay(500);
        leds.setColorRGB(bleu);
        delay(500);
    }
}

void gps_er(int er_code) {/// Erreur dU GPS

    while (er_code == 2){
        leds.setColorRGB(rouge);
        delay(500);
        leds.setColorRGB(jaune);
        delay(500);
    }
}

void sens_er(int er_code) {///Erreur d'un / plusieurs capteur(s)

    while (er_code == 3){
        leds.setColorRGB(rouge);
    delay(500);
    leds.setColorRGB(vert);
    delay(500);
    }
}

void data_er(int er_code) {/// Erreur de la validité des données reçues

    while (er_code == 4){
        leds.setColorRGB(rouge);
        delay(333);
        leds.setColorRGB(vert);
        delay(666);
    }
}

void sdf_er(int er_code) {/// Capacité pleine

    while (er_code == 5){
    leds.setColorRGB(rouge);
    delay(500);
    leds.setColorRGB(blanc);
    delay(500);
    }
}

void sda_er(int er_code) {/// erreur d'accès et ou d'écriture sur la carte sd

    while (er_code == 6){
    leds.setColorRGB(rouge);
    delay(333);
    leds.setColorRGB(blanc);
    delay(666);
    }
}

void SetErrors(int er_code){
  Serial.println("Erreur");
	if (er_code == 1){
		clk_er(1);
	}
	else if (er_code == 2){
		gps_er(2);
	}
	else if (er_code == 3){
		sens_er(3);
	}
	else if (er_code==4){
        data_er(4);
    }
    else if (er_code==5){
        sdf_er(5);
    }
    else if (er_code==6){
        sda_er(6);
	}
}