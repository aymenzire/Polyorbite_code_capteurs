/*
Hello guys, voici le main code pour nos tests de capteurs
VERSION : 1.1                                                 DERNIER CHANGEMENT PAR : Luka
J'ai implémenter les photorésistances, je ne sais pas si ça fonctionne, il me faudrait de l'aide pour test. Les "TODO" c'est des choses à compléter.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

//  Change si tu veux
const uint8_t PIN_PHOTO0 = A0;
const uint8_t PIN_PHOTO1 = A1;
const uint8_t PIN_PHOTO2 = A2;
const uint8_t PIN_PHOTO3 = A3;

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
  Serial.begin(115200);
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
  r.light_raw = analogRead(PIN_PHOTO0);
  r.normalized_light0 = r.light_raw / 1023;
  r.light_raw = analogRead(PIN_PHOTO1);
  r.normalized_light1 = r.light_raw / 1023;
  r.light_raw = analogRead(PIN_PHOTO2);
  r.normalized_light2 = r.light_raw / 1023;
  r.light_raw = analogRead(PIN_PHOTO3);
  r.normalized_light3 = r.light_raw / 1023;
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

