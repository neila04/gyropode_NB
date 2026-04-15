#include <Arduino.h>
#include <String.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Encoder.h>
#include <BluetoothSerial.h>

#define CLK1 35 // CLK ENCODER mot G
#define DT1 34  // DT ENCODER mot G
#define CLK2 33 // CLK ENCODER mot D
#define DT2 32  // DT ENCODER mot D

BluetoothSerial SerialBT;

ESP32Encoder encoderG;
ESP32Encoder encoderD;

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp; // acceleration, vitesse gyro, temperature

char FlagCalcul = 0; // savoir quand on passe dans controle pour passer à affichage
float Te = 5;        // période d'échantillonage en ms
float Tau = 200;     // constante de temps du filtre angle en ms
float TauV = 115;    // constante de temps du filtre vitesse en ms

// mesure MPU
float tetaG, tetaW, tetaGF, teta; // angle gravité mesuré par l’accéléromètre, angle après filtrage éliminer le bruit haute fréquence,angel mesuree

// correction angle
float Kp = 3.195;
float Kd = 0.07;
float teta_offset = 0; // point d'équilibre mesuré
float ec;
float alpha1;
float alpha2;
float C0 = 0.134;
float teta_cons = 0.0;

// correction vitesse
long old_posG = 0;
long old_posD = 0;
long posD_actuelle, posG_actuelle;
float vitG, vitD, vitMoy;
float vitMoyF;

float Kpv = 0.7;
float Kdv = 0;
float erreur_vit;
float vit_cons = 0;
float derive_erreur;
float old_erreur_vit;

// coefficient du filtre angle
float A, B;

// coefficient du filtre vitesse
float Av, Bv;

// brochces
int pwmGp = 19; // IN1
int pwmGn = 18; // IN2
int pwmDp = 23; // IN3
int pwmDn = 17; // IN4

// caracteristiques PWM
int freq = 20000;
int canalGp = 0;
int canalGn = 1;
int canalDp = 2;
int canalDn = 3;
int resolution = 10;

void controle(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    posG_actuelle = encoderG.getCount();
    posD_actuelle = encoderD.getCount();

    // Calcul des dérivées
    // nb top /tour = 748 et Te en ms et R roue = 32.5mm
    vitG = (posG_actuelle - old_posG) * (2 * PI / 748) * (1000 / Te) * 0.0325;
    vitD = (posD_actuelle - old_posD) * (2 * PI / 748) * (1000 / Te) * 0.0325;

    // Mémorisation
    old_posG = posG_actuelle;
    old_posD = posD_actuelle;

    vitMoy = (vitG + vitD) / 2.0;

    vitMoyF = Av * vitMoy + Bv * vitMoyF; // vitmoyFiltré

    erreur_vit = vitMoyF - vit_cons;
    derive_erreur = erreur_vit - old_erreur_vit; // vitesse de variation
    old_erreur_vit = erreur_vit;

    // integrale erreur +=erreur supprimer erreur
    teta_cons = -(Kpv * erreur_vit) + (Kdv * derive_erreur);

    // Saturation de teta_cons 2°
    if (teta_cons > 0.15)
      teta_cons = 0.15;
    if (teta_cons < -0.15)
      teta_cons = -0.15;
    if (teta_cons == 0.0)
      teta_cons == 0.0;

    mpu.getEvent(&a, &g, &temp); // lecture des grandeurs

    tetaG = atan2(a.acceleration.y, a.acceleration.x); // angle de gravité

    tetaGF = A * tetaG + B * tetaGF; // filtre passe bas

    tetaW = A * Tau / 1000 * (-g.gyro.z) + B * tetaW;

    teta = tetaGF + tetaW + teta_offset;

    // PID
    ec = (Kp * (teta_cons - teta) + (Kd * g.gyro.z)); // calcul de la commande ec   normalement -(Kd*g.gyro) mais bon fonctionnement -g.gyro

    if (ec > 0) // compensation des trottements secs
      ec += C0;
    else if (ec < 0)
      ec -= C0;

    if (ec > 0.48) // saturation
      ec = 0.48;
    if (ec < -0.48)
      ec = -0.48;

    alpha1 = (0.5 + ec) * 1023;
    alpha2 = (0.5 - ec) * 1023;

    ledcWrite(canalGp, alpha1);
    ledcWrite(canalGn, alpha2);

    ledcWrite(canalDp, alpha2);
    ledcWrite(canalDn, alpha1);

    FlagCalcul = 1;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.printf("Bonjour \n\r");

  SerialBT.begin("Gyropode_NB"); // démarre le Bluetooth et nom gyropode

  // config canaux PWM
  ledcSetup(canalGp, freq, resolution);
  ledcSetup(canalGn, freq, resolution);
  ledcSetup(canalDp, freq, resolution);
  ledcSetup(canalDn, freq, resolution);

  // liaosn canaux PWM avec broches ESP32
  ledcAttachPin(pwmGp, canalGp);
  ledcAttachPin(pwmGn, canalGn);
  ledcAttachPin(pwmDp, canalDp);
  ledcAttachPin(pwmDn, canalDn);

  // encoder mot G
  encoderG.attachHalfQuad(DT1, CLK1);
  encoderG.setCount(0);

  // encoder mot D
  encoderD.attachHalfQuad(DT2, CLK2);
  encoderD.setCount(0);

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // calcul coeff filtre angle
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;

  // calcul coeff filtre vitesse
  Av = 1 / (1 + TauV / Te);
  Bv = TauV / Te * Av;

  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      10,         // tres haut niveau de priorite
      NULL        // descripteur
  );
}

void reception(char ch)
{

  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      // calcul coeff filtre
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }

    if (commande == "Te")
    {
      Te = valeur.toInt();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
      Av = 1 / (1 + TauV / Te);
      Bv = TauV / Te * Av;
    }

    if (commande == "Kp")
    {
      Kp = valeur.toFloat();
    }

    if (commande == "Kd")
    {
      Kd = valeur.toFloat();
    }

    if (commande == "C0")
    {
      C0 = valeur.toFloat();
    }
    if (commande == "TauV")
    {
      TauV = valeur.toFloat();
      Av = 1 / (1 + TauV / Te);
      Bv = TauV / Te * Av;
    }
    if (commande == "Kpv")
    {
      Kpv = valeur.toFloat();
    }
    if (commande == "Kdv")
    {
      Kdv = valeur.toFloat();
    }
    if (commande == "vit")
    {
      vit_cons = valeur.toFloat();
    }

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void loop()
{

  // Lecture des octets reçus via Bluetooth (tablette → ESP32)
  while (SerialBT.available() > 0) // tant qu'il y a des caractères en attente dans le buffer Bluetooth
    reception(SerialBT.read());    // lit un caractère et l'envoie à la fonction reception() pour parser la commande

  // Lecture des octets reçus via câble USB (PC → ESP32) pour debug
  while (Serial.available() > 0) // tant qu'il y a des caractères en attente dans le buffer USB
    reception(Serial.read());    // lit un caractère et l'envoie à la même fonction reception()

  if (FlagCalcul == 1)
  {
    // test hyperterminal

    // Serial.printf("%lf %lf %lf \n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
    // Serial.printf(" %.2f  %.2f %.2f %.2f \n", tetaG, tetaGF, tetaW, tetaW + tetaGF); // valeurs des angles de gravité et filtré en rad
    // Serial.printf(" %.2f  %.2f %.2f %.2f \n", tetaG, tetaGF, tetaW, g.gyro.z); // valeurs des angles de gravité et filtré en deg
    // Serial.printf("%lf %lf %lf %lf \n", ec, teta * 180 / M_PI, C0, tetaG * 180 / M_PI);
    // Serial.printf("%lf %lf %lf %lf \n", ec, teta, Kp, Kd);
    // Serial.printf("%lf %lf %lf %lf", ec, teta * 180 / M_PI, Kp * (0 - teta) , Kd * g.gyro.z);
    // Serial.printf("%lf %lf %lf %lf \n", vit_cons, vitMoyF, Av, Bv);
    // Serial.printf("%lf %lf \n", vitMoyF, teta_cons);
    // Serial.printf("%f %f\n", vitMoyF, teta_cons);
    // Serial.printf("%f %f %f %d\n", vitMoyF, teta_cons, vit_cons, posG_actuelle);
    // Serial.printf("%f %f %f\n", vitMoyF, teta_cons, vit_cons);
    // SerialBT.printf("%.2f %.2f\n", vitMoyF, teta * 180 / M_PI); // envoie vitesse filtrée et angle en degrés vers la tablette via Bluetooth

    static int compteur = 0;
    compteur++;

    if (compteur >= 40)
    {
      // SerialBT.printf("%lf \n", vitMoyF);
      compteur = 0;
    }

    FlagCalcul = 0;
  }
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}