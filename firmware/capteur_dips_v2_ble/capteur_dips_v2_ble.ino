// ============================================================================
// Capteur de dips V2 — enregistreur ESP32 + MPU6050 avec streaming BLE
//
// Étape 1 du chantier « couper le câble » : mêmes mesures brutes que la V1
// (100 Hz), mais diffusées AUSSI en Bluetooth Low Energy vers un PC ou un
// téléphone. La chaîne d'analyse Python reste inchangée : capture_ble.py
// reconstitue exactement le même CSV que la liaison série. L'algo embarqué
// et le vibreur viendront en V3, une fois le sans-fil validé.
//
// Double sortie simultanée :
//   - USB série 115200 : CSV identique à la V1 (le mode filaire marche toujours)
//   - BLE : paquets binaires de 5 échantillons (voir FORMAT ci-dessous)
//
// FORMAT d'un échantillon BLE (16 octets, little-endian) :
//   uint32  t_ms                  horodatage carte
//   int16   ax, ay, az            accélération × 500  (±65 m/s² de plage)
//   int16   gx, gy, gz            gyroscope × 10      (°/s, biais retiré)
// Un paquet notify = 5 échantillons = 80 octets, ~20 notifications/s.
//
// LED : fixe pendant la calibration (2 s, NE PAS BOUGER), clignotement lent
// en attente de connexion BLE, allumée fixe quand un client est connecté,
// clignotement rapide = MPU6050 introuvable.
// ============================================================================

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---- Réglages ---------------------------------------------------------------
#define BROCHE_SDA        21
#define BROCHE_SCL        22
#define BROCHE_LED        2
#define FREQ_ECH_HZ       100
#define N_CALIBRATION     200
#define ECH_PAR_PAQUET    5

#define NOM_BLE           "CapteurDips"
#define UUID_SERVICE      "b5f0a1c0-9d6b-4b7a-8e1f-3c2a7d9e0001"
#define UUID_CARAC_DATA   "b5f0a1c0-9d6b-4b7a-8e1f-3c2a7d9e0002"

// ---- Registres MPU6050 ------------------------------------------------------
#define MPU_ADRESSE       0x68
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

const float ACCEL_M_S2_PAR_LSB = 9.80665f / 4096.0f;   // ±8 g
const float GYRO_DPS_PAR_LSB   = 1.0f / 65.5f;         // ±500 °/s
const uint32_t PERIODE_US = 1000000UL / FREQ_ECH_HZ;

float biaisGyro[3] = {0, 0, 0};
uint32_t prochainEchantillonUs = 0;
uint32_t t0Ms = 0;

BLECharacteristic *caracData = nullptr;
volatile bool clientConnecte = false;
uint8_t paquet[16 * ECH_PAR_PAQUET];
int idxPaquet = 0;

// ---- BLE : gestion de la connexion ------------------------------------------
class Callbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { clientConnecte = true; }
  void onDisconnect(BLEServer *srv) override {
    clientConnecte = false;
    srv->getAdvertising()->start();       // redevenir visible aussitôt
  }
};

// ---- Accès I2C bas niveau (identique V1) ------------------------------------
void ecrireRegistre(uint8_t reg, uint8_t valeur) {
  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(reg);
  Wire.write(valeur);
  Wire.endTransmission();
}

uint8_t lireRegistre(uint8_t reg) {
  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADRESSE, (uint8_t)1);
  return Wire.read();
}

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

// ---- Encodage d'un échantillon dans le paquet BLE ---------------------------
void empiler(uint32_t tMs, const float accel[3], const float gyro[3]) {
  uint8_t *p = paquet + 16 * idxPaquet;
  memcpy(p, &tMs, 4);
  for (int i = 0; i < 3; i++) {
    int16_t v = (int16_t)lroundf(accel[i] * 500.0f);
    memcpy(p + 4 + 2 * i, &v, 2);
  }
  for (int i = 0; i < 3; i++) {
    int16_t v = (int16_t)lroundf(gyro[i] * 10.0f);
    memcpy(p + 10 + 2 * i, &v, 2);
  }
  if (++idxPaquet >= ECH_PAR_PAQUET) {
    idxPaquet = 0;
    if (clientConnecte && caracData) {
      caracData->setValue(paquet, sizeof(paquet));
      caracData->notify();
    }
  }
}

// ---- Initialisation ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(BROCHE_LED, OUTPUT);
  Wire.begin(BROCHE_SDA, BROCHE_SCL, 400000);

  delay(100);
  if (lireRegistre(REG_WHO_AM_I) != 0x68) {
    Serial.println("# ERREUR : MPU6050 introuvable. Verifier cablage SDA=21 SCL=22.");
    while (true) {
      digitalWrite(BROCHE_LED, !digitalRead(BROCHE_LED));
      delay(100);
    }
  }

  ecrireRegistre(REG_PWR_MGMT_1, 0x80);
  delay(100);
  ecrireRegistre(REG_PWR_MGMT_1, 0x01);
  ecrireRegistre(REG_CONFIG, 0x03);
  ecrireRegistre(REG_SMPLRT_DIV, 9);
  ecrireRegistre(REG_GYRO_CONFIG, 0x08);
  ecrireRegistre(REG_ACCEL_CONFIG, 0x10);
  delay(100);

  // Calibration gyro : immobile, LED fixe
  Serial.println("# Calibration gyro : immobile 2 s...");
  digitalWrite(BROCHE_LED, HIGH);
  float accel[3], gyro[3], somme[3] = {0, 0, 0};
  for (int n = 0; n < N_CALIBRATION; n++) {
    lireIMU(accel, gyro);
    for (int i = 0; i < 3; i++) somme[i] += gyro[i];
    delay(1000 / FREQ_ECH_HZ);
  }
  for (int i = 0; i < 3; i++) biaisGyro[i] = somme[i] / N_CALIBRATION;
  digitalWrite(BROCHE_LED, LOW);

  // BLE : serveur + service + caractéristique notify
  BLEDevice::init(NOM_BLE);
  BLEDevice::setMTU(185);
  BLEServer *serveur = BLEDevice::createServer();
  serveur->setCallbacks(new Callbacks());
  BLEService *service = serveur->createService(UUID_SERVICE);
  caracData = service->createCharacteristic(UUID_CARAC_DATA,
                                            BLECharacteristic::PROPERTY_NOTIFY);
  caracData->addDescriptor(new BLE2902());
  service->start();
  BLEAdvertising *pub = BLEDevice::getAdvertising();
  pub->addServiceUUID(UUID_SERVICE);
  pub->start();

  Serial.println("# BLE pret : advertising sous le nom CapteurDips");
  Serial.println("t_ms,ax,ay,az,gx,gy,gz");
  t0Ms = millis();
  prochainEchantillonUs = micros();
}

// ---- Boucle 100 Hz : série + BLE, LED d'état --------------------------------
void loop() {
  while ((int32_t)(micros() - prochainEchantillonUs) < 0) {}
  prochainEchantillonUs += PERIODE_US;

  float accel[3], gyro[3];
  lireIMU(accel, gyro);
  for (int i = 0; i < 3; i++) gyro[i] -= biaisGyro[i];

  uint32_t tMs = millis() - t0Ms;
  Serial.printf("%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n", (unsigned long)tMs,
                accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
  empiler(tMs, accel, gyro);

  // LED : fixe si connecté, battement lent sinon (visible sans être un phare)
  digitalWrite(BROCHE_LED, clientConnecte ? HIGH : ((tMs % 2000) < 100));
}
