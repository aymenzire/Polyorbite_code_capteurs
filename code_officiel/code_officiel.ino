/*
Hello guys, voici le main code pour nos tests de capteurs
VERSION : 1.2                                                 DERNIER CHANGEMENT PAR : Luka
Le code pour les photorésistances devrait fonctionné, il a été testé. J'ai modifié la pin du PIN_PHOTO. Jsp comment appeler les variables pour les photorésistances(Aymen va faire le tableau) Les "TODO" c'est des choses à compléter.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

//  Change si tu veux
const uint8_t PIN_PHOTO = A4;
float PR0 = A0;
float PR1 = A1;
float PR2 = A2;
float PR3 = A3;
float Res10k = 10000.0;

// Structure de données (une ligne du tableau)
struct DataRow {
  uint32_t t_ms = 0;
  float angle_deg = NAN;
  int light_raw = -1;     // photodiode brute (0-1023)
  float normalized_light0 = 0;
  float normalized_light1 = 0;
  float normalized_light2 = 0;
  float normalized_light3 = 0;
  float pressure_hpa = NAN; // réservé
  float humidity_pct = NAN; // réservé
};

DataRow row;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(PIN_PHOTO, INPUT);

  if (!accel.begin()) {
    Serial.println("ADXL345 not detected");
    while (1) {}
  }
  accel.setRange(ADXL345_RANGE_2_G);

  // Header CSV tableau
  Serial.println("t_ms,angle_deg,light_raw,pressure_hpa,humidity_pct");
}

void loop() {
  acquireAndLog();
  delay(20); // environ 50 Hz
}

// FUNCTIONS 

// Fait une “acquisition” complète et log la ligne
void acquireAndLog() {
  row.t_ms = millis();

  updateAccelerometer(row);
  updateLight(row);
  updatePressure(row);
  updateHumidity(row);

  printRowCSV(row);
}

// Lit l'accéléromètre et met à jour row.angle_deg
void updateAccelerometer(DataRow &r) {
  sensors_event_t event;
  accel.getEvent(&event);

  float ax = event.acceleration.x;
  float az = event.acceleration.z;

  float angleRad = atan2(ax, az);
  r.angle_deg = angleRad * 180.0 / PI;
}

// Lit la photodiode et met à jour row.light_raw
void updateLight(DataRow &r) {
//  r.light_raw = analogRead(PIN_PHOTO); //je ne sais pas si je dois utiliser light raw ou non
  float I0 = analogRead(PR0);
  float V0 = map(I0, 0, 1023, 0 , 10000) / Res10k; //on divise par 10000.0, car map() donne un nombre entier, mais on veut + de précision
  r.normalized_light0 = V0;

  float I1 = analogRead(PR1);
  float V1 = map(I1, 0, 1023, 0 , 10000) / Res10k;
  r.normalized_light1 = V1;

  float I2 = analogRead(PR2);
  float V2 = map(I2, 0, 1023, 0 , 10000) / Res10k;
  r.normalized_light2 = V2;

  float I3 = analogRead(PR3);
  float V3 = map(I3, 0, 1023, 0 , 10000) / Res10k;
  r.normalized_light3 = V3;
}

// Placeholder pression
void updatePressure(DataRow &r) {
  // TODO: remplacer par notre capteur 
  r.pressure_hpa = NAN;
}

// Placeholder humidité
void updateHumidity(DataRow &r) {
  // TODO: remplacer par notre capteur 

  r.humidity_pct = NAN;
}
//TODO : LES AUTRES CAPTEURS REGARDER DATASHEET 

// Imprime une ligne CSV: temps, angle, lumière, pression, humidité
void printRowCSV(const DataRow &r) {
  Serial.print(r.t_ms);
  Serial.print(",");

  Serial.print(r.angle_deg, 3);
  Serial.print(",");

  Serial.print(r.light_raw);
  Serial.print(",");

  // pression
  if (isnan(r.pressure_hpa)) Serial.print("");
  else Serial.print(r.pressure_hpa, 2);
  Serial.print(",");

  // humidité
  if (isnan(r.humidity_pct)) Serial.print("");
  else Serial.print(r.humidity_pct, 2);

  Serial.println();
}

