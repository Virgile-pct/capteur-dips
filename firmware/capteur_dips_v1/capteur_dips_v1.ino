// ============================================================================
// Capteur de dips V1 — enregistreur brut ESP32 + MPU6050 (GY-521)
//
// Rôle volontairement minimal (palier 1 de la doc projet) : lire l'IMU à
// 100 Hz, calibrer le biais gyro au démarrage, et streamer les mesures en CSV
// sur le port série. Tout le calcul de vitesse se fait côté PC en Python
// (analyse/analyse_reps.py) — on portera l'algo en C++ ici quand il sera figé.
//
// Câblage GY-521 -> ESP32 DevKit :
//   VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22 (AD0 laissé en l'air)
//
// Sortie série (115200 bauds) : t_ms,ax,ay,az,gx,gy,gz
//   accéléro en m/s², gyro en °/s, lignes de contexte préfixées par '#'
//
// Protocole de mesure : poser/tenir le capteur IMMOBILE pendant les 2 s de
// calibration (LED bleue allumée), puis faire la série. L'immobilité entre les
// reps (verrouillage en haut) sert d'ancre ZUPT à l'algo Python.
// ============================================================================

#include <Wire.h>

// ---- Réglages ---------------------------------------------------------------
#define BROCHE_SDA        21
#define BROCHE_SCL        22
#define BROCHE_LED        2        // LED bleue intégrée du DevKit
#define FREQ_ECH_HZ       100      // fréquence d'échantillonnage
#define N_CALIBRATION     200      // 2 s de calibration à 100 Hz
// #define SORTIE_PLOTTER          // décommenter pour le Serial Plotter Arduino

// ---- Registres MPU6050 (datasheet "MPU-6000/6050 Register Map") -------------
#define MPU_ADRESSE       0x68     // AD0 à la masse
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

// Échelles : ±8 g -> 4096 LSB/g ; ±500 °/s -> 65.5 LSB/(°/s)
const float ACCEL_M_S2_PAR_LSB = 9.80665f / 4096.0f;
const float GYRO_DPS_PAR_LSB   = 1.0f / 65.5f;

const uint32_t PERIODE_US = 1000000UL / FREQ_ECH_HZ;

float biaisGyro[3] = {0, 0, 0};
uint32_t prochainEchantillonUs = 0;
uint32_t t0Ms = 0;

// ---- Accès I2C bas niveau ---------------------------------------------------
void ecrireRegistre(uint8_t reg, uint8_t valeur) {
  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(reg);
  Wire.write(valeur);
  Wire.endTransmission();
}

uint8_t lireRegistre(uint8_t reg) {
  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(reg);
  Wire.endTransmission(false);          // restart, pas de stop
  Wire.requestFrom(MPU_ADRESSE, (uint8_t)1);
  return Wire.read();
}

// Lecture en rafale des 14 octets accel+temp+gyro (big-endian, int16 signés)
void lireIMU(float accel[3], float gyro[3]) {
  uint8_t brut[14];
  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADRESSE, (uint8_t)14);
  for (int i = 0; i < 14; i++) brut[i] = Wire.read();

  for (int i = 0; i < 3; i++) {
    int16_t a = (int16_t)((brut[2 * i] << 8) | brut[2 * i + 1]);
    int16_t g = (int16_t)((brut[8 + 2 * i] << 8) | brut[9 + 2 * i]);
    accel[i] = a * ACCEL_M_S2_PAR_LSB;
    gyro[i]  = g * GYRO_DPS_PAR_LSB;
  }
}

// ---- Initialisation ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(BROCHE_LED, OUTPUT);
  Wire.begin(BROCHE_SDA, BROCHE_SCL, 400000);   // I2C rapide 400 kHz

  delay(100);
  uint8_t whoami = lireRegistre(REG_WHO_AM_I);
  if (whoami != 0x68) {
    Serial.printf("# ERREUR : MPU6050 introuvable (WHO_AM_I=0x%02X). Verifier cablage SDA=21 SCL=22.\n", whoami);
    while (true) {                        // clignote vite = panne
      digitalWrite(BROCHE_LED, !digitalRead(BROCHE_LED));
      delay(100);
    }
  }

  ecrireRegistre(REG_PWR_MGMT_1, 0x80);   // reset
  delay(100);
  ecrireRegistre(REG_PWR_MGMT_1, 0x01);   // réveil, horloge PLL gyro X (plus stable que l'oscillo interne)
  ecrireRegistre(REG_CONFIG, 0x03);       // filtre passe-bas interne : 44 Hz accel / 42 Hz gyro
  ecrireRegistre(REG_SMPLRT_DIV, 9);      // 1 kHz / (1+9) = 100 Hz
  ecrireRegistre(REG_GYRO_CONFIG, 0x08);  // ±500 °/s
  ecrireRegistre(REG_ACCEL_CONFIG, 0x10); // ±8 g (marge large, un dip reste < 2 g)
  delay(100);

  // Calibration du biais gyro : NE PAS BOUGER pendant que la LED est allumée
  Serial.println("# Calibration gyro : immobile 2 s...");
  digitalWrite(BROCHE_LED, HIGH);
  float accel[3], gyro[3], sommeG[3] = {0, 0, 0}, sommeNormeA = 0;
  for (int n = 0; n < N_CALIBRATION; n++) {
    lireIMU(accel, gyro);
    for (int i = 0; i < 3; i++) sommeG[i] += gyro[i];
    sommeNormeA += sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    delay(1000 / FREQ_ECH_HZ);
  }
  for (int i = 0; i < 3; i++) biaisGyro[i] = sommeG[i] / N_CALIBRATION;
  digitalWrite(BROCHE_LED, LOW);

  Serial.printf("# biais gyro (deg/s) : %.3f %.3f %.3f | norme accel au repos : %.3f m/s2\n",
                biaisGyro[0], biaisGyro[1], biaisGyro[2], sommeNormeA / N_CALIBRATION);
  Serial.println("t_ms,ax,ay,az,gx,gy,gz");

  t0Ms = millis();
  prochainEchantillonUs = micros();
}

// ---- Boucle : échantillonnage cadencé à 100 Hz ------------------------------
void loop() {
  // attente active jusqu'au prochain tick (précis, sans dérive cumulative)
  while ((int32_t)(micros() - prochainEchantillonUs) < 0) {}
  prochainEchantillonUs += PERIODE_US;

  float accel[3], gyro[3];
  lireIMU(accel, gyro);
  for (int i = 0; i < 3; i++) gyro[i] -= biaisGyro[i];

#ifdef SORTIE_PLOTTER
  // Vue temps réel dans le Serial Plotter : norme accel - g, et norme gyro
  float normeA = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
  float normeG = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
  Serial.printf("accel_lin:%.3f,gyro:%.2f\n", normeA - 9.81f, normeG);
#else
  Serial.printf("%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n",
                (unsigned long)(millis() - t0Ms),
                accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
#endif
}
