#include <Arduino.h>

#include <Wire.h>
#include <MPU6050.h>


MPU6050 mpu;


void setup() {
  Wire.begin();
  Serial.begin(9600);

  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}

void loop() {

  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
  int16_t temp;

  mpu.getAcceleration(&accelX, &accelY, &accelZ);
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);

  // Extraction des données d'état du véhicule
  float accelX = accelX / 16384.0;  // Convertit les valeurs en m/s^2
  float accelY = accelY / 16384.0;
  float accelZ = accelZ / 16384.0;

  float gyroX = gyroX / 131.0;  // Convertit les valeurs en degrés par seconde
  float gyroY = gyroY / 131.0;
  float gyroZ = gyroZ / 131.0;

  // Calcul de l'accélération linéaire
  float linearAccelX = accelX - gyroY * 0.01; // 0.01 est un facteur d'échelle pour la correction
  float linearAccelY = accelY - gyroX * 0.01;
  float linearAccelZ = accelZ - gyroZ * 0.01;

  float totalAccel = sqrt(linearAccelX * linearAccelX + linearAccelY * linearAccelY + linearAccelZ * linearAccelZ);

  //angle d'inclinaison
  float angleX = atan2(accelY, accelZ) * (180.0 / M_PI);
  float angleY = atan2(-accelX, accelZ) * (180.0 / M_PI);
  float angleZ = atan2(-accelX, accelY) * (180.0 / M_PI);

  // Analyse de l'état du véhicule
  bool enMontee = (linearAccelY > 0.5); // Seuil d'accélération pour détecter une montée
  bool enDescente = (linearAccelY < -0.5); // Seuil d'accélération pour détecter une descente
  bool enRotation = (abs(gyroZ) > 50); // Seuil de vitesse angulaire pour détecter une rotation
  bool freinBrusque = (linearAccelX < -2.0); // Seuil d'accélération pour détecter un freinage brusque
  bool chocDetecte = (totalAccel > 5.0);//Seuil d'accélération pour détecter un choc

  // Affichage de l'état du véhicule
  SerialUSB.print("En montée : ");
  SerialUSB.println(enMontee);

  SerialUSB.print("En descente : ");
  SerialUSB.println(enDescente);

  SerialUSB.print("En rotation : ");
  SerialUSB.println(enRotation);

  SerialUSB.print("Frein brusque : ");
  SerialUSB.println(freinBrusque);

  SerialUSB.print("Choc détecté : ");
  SerialUSB.println(chocDetecte);

  SerialUSB.print("TEMPERATURE = " );
  SerialUSB.print((temp/340.0)+36.53);
  SerialUSB.println(" °C");

  delay(1000);
}