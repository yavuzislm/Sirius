#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Genel değişkenler
float verticalVelocity = 0.0;
float baseAccelerationZ = 0.0;
unsigned long previousTime = 0;

// Kalman filtresi parametreleri
float x_est = 0.0;
float P_est = 1.0;
const float Q = 0.0001;  // Daha düşük sistem gürültüsü
const float R = 10.0;    // Daha yüksek ölçüm gürültüsü

// Hız sıfırlama için değişkenler
const float VELOCITY_THRESHOLD = 0.03;  // Artırılmış hız eşiği
const unsigned long RESET_INTERVAL = 500;  // Daha sık sıfırlama kontrolü
unsigned long lastResetTime = 0;

// Hareketli ortalama filtresi için değişkenler
const int FILTER_WINDOW = 20;  // Daha büyük filtre penceresi
float velocityReadings[FILTER_WINDOW];
int readIndex = 0;
float velocityTotal = 0;

// Yeni: İvme için "ölü bölge" tanımı
const float ACCELERATION_DEADZONE = 0.02;  // g cinsinden

// Yeni: Hareket algılama için değişkenler
bool isMoving = false;
const int MOTION_THRESHOLD_COUNT = 5;
int motionCount = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("MPU6050 başlatılıyor...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 ile bağlantı kurulamadı!");
    while (1);
  }
  Serial.println("MPU6050 bağlandı.");

  // Kalibrasyon
  Serial.println("Kalibrasyon başlıyor. Lütfen sensörü sabit tutun...");
  float sumAz = 0;
  for (int i = 0; i < 2000; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAz += az / 16384.0;
    delay(1);
  }
  baseAccelerationZ = sumAz / 2000.0;
  Serial.println("Kalibrasyon tamamlandı.");
  Serial.print("Temel Z ivmesi: ");
  Serial.println(baseAccelerationZ);

  previousTime = millis();
  lastResetTime = previousTime;

  for (int i = 0; i < FILTER_WINDOW; i++) {
    velocityReadings[i] = 0;
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelerationZ = (az / 16384.0) - baseAccelerationZ;

  // Yeni: İvme "ölü bölge" kontrolü
  if (abs(accelerationZ) < ACCELERATION_DEADZONE) {
    accelerationZ = 0;
  }

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;

  // Kalman filtresi
  float x_pred = x_est + accelerationZ * 9.81 * deltaTime;
  float P_pred = P_est + Q;

  float K = P_pred / (P_pred + R);
  x_est = x_pred + K * (verticalVelocity - x_pred);
  P_est = (1 - K) * P_pred;

  verticalVelocity = x_est;

  // Yeni: Hareket algılama
  if (abs(accelerationZ) > ACCELERATION_DEADZONE) {
    motionCount++;
    if (motionCount > MOTION_THRESHOLD_COUNT) {
      isMoving = true;
    }
  } else {
    motionCount = 0;
    if (isMoving) {
      isMoving = false;
      // Hareket durduğunda hızı sıfırla
      verticalVelocity = 0;
      x_est = 0;
    }
  }

  // Düşük hızlarda sıfırlama (daha agresif)
  if (abs(verticalVelocity) < VELOCITY_THRESHOLD || 
      (currentTime - lastResetTime) > RESET_INTERVAL) {
    verticalVelocity = 0;
    x_est = 0;
    lastResetTime = currentTime;
  }

  // Hareketli ortalama filtresi
  velocityTotal = velocityTotal - velocityReadings[readIndex];
  velocityReadings[readIndex] = verticalVelocity;
  velocityTotal = velocityTotal + velocityReadings[readIndex];
  readIndex = (readIndex + 1) % FILTER_WINDOW;

  float averageVelocity = velocityTotal / FILTER_WINDOW;

  // Sonuçları yazdır
  Serial.print("İvme Z: ");
  Serial.print(accelerationZ);
  Serial.print(" g, Hız: ");
  Serial.print(verticalVelocity);
  Serial.print(" m/s, Ort. Hız: ");
  Serial.print(averageVelocity);
  Serial.print(" m/s, Hareket: ");
  Serial.println(isMoving ? "Evet" : "Hayır");

  previousTime = currentTime;
  delay(10);
}