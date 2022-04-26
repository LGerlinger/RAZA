#include <Servo.h>
#include <math.h>

bool modeManuel = true;

// Définition des numéro de pin
const int joyPin1 = A0;
const int joyPin2 = A1;
//const int joyBtnPin = ? Pin digital pour le bouton

const byte servoPin1 = 9;
const byte servoPin2 = 10;

const int LED1 = 7;

const byte interruptPin = 2;

//Def des servos
Servo servo1;
Servo servo2;
Servo servo3;

//-------------------- Definition des differents parametres geometriques de la patte----------------------

// Dimensions des segments de la patte en mm, issues du sujet de DS
float a = 20;
float b = 80;
float c = 80;

// Coordonnees du point E (coordonnees operationnelles)
float xe=-100;  // Coordonnees du point E (point de l'organe operationnel), que l'on impose pour avoir un mouvement plan
float ye = -140.;

void setup() {
  Serial.begin(9600);

  //Setup Bouton Interrupt pin
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt_function, FALLING); 

  //Setup Pin Moteurs
  pinMode(joyPin1,INPUT);
  pinMode(joyPin2,INPUT);  
  servo1.attach(servoPin1);  // attaches the servo on pin 9 to the servo object <=> GPIO.setup(11,GPIO.OUT)
  servo2.attach(servoPin2);
  //servo3.attach(servoPin3);

  servo1.write(1); //On place initialement les deux servos à 1° (pas 0 pour éviter les potentielles singularités)
  servo2.write(1);

  
  //Setup de la LED verte
  
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);
}

//------------------------------MAIN----------------------------------------
void loop(){
  // put your main code here, to run repeatedly:
  ModeManuel();
  
    servo1.write( -M_PI + atan2f(ye,(xe-a)) - acosf( (c*c-((xe-a)*(xe-a)+ye*ye+b*b)) /2/b/sqrt((xe-a)*(xe-a)+ye*ye) )* 180/M_PI );
    servo2.write(  M_PI + atan2f(ye,(xe+a)) + acosf( (c*c-((xe+a)*(xe+a)+ye*ye+b*b)) /2/b/sqrt((xe+a)*(xe+a)+ye*ye) )* 180/M_PI );
  
  delay(15);
}
//------------------------------------DELAY--------------------------
void delayFab(unsigned int duree){
  // Concretement, on ne fait que retarder d'une certaine duree en ms 
  unsigned int depart = millis();
  unsigned int tpsActuel;
  
  do{
    tpsActuel = millis();
  }while(tpsActuel-depart<duree);
  
}
//-------------------------------------MODE MANUEL-------------------------------------------------
void ModeManuel(){
  int Rx = analogRead(joyPin1); // On récupère les données des deux pins
  int Ry = analogRead(joyPin2);
  

  Rx = map(Rx, 0, 1023, -2, 2); // On converti la valeur (à gauche -1, à droite 1)
  Ry = map(Ry, 0, 1023, -2, 2);
  
  //Puis on voit ce qu'on fait :
  if(Rx == -2){
    //xe--
    xe--;
    Serial.println("HAUT");
    delayFab(200); //200millisecondes de "pause"
  }
  else if(Rx == 2){
    //xe++
    xe++;
    Serial.println("BAS");
    delayFab(200);
  }

  
  if(Ry == -2){
    ye--;
    Serial.println("GAUCHE");
    delayFab(200);
  }
  else if(Ry == 2){
    ye++;
    Serial.println("DROITE");
    delayFab(200);
  }
}
//-------------------------FONCTION INTERRUPT-----------------------
void interrupt_function(){
  Serial.println("Interrupt sur Button press");
  delay(100);
}
