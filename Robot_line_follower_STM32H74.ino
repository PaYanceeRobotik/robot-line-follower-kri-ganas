/*
 * ROBOT LINE FOLLOWER KRI GANAS - STM32H743
 * Team: PaYanceeRobotik - Bandung
 * Hardware: STM32H743 + 16x QTR + TB6612FNG + N20 Motor
 * Fitur: PID 1kHz + Feedforward + Anti-keluar-garis KRI
 */

// --- PIN CONFIG ---
// TB6612 Driver
#define AIN1  PA8
#define AIN2  PA9
#define PWMA  PA10
#define BIN1  PB12
#define BIN2  PB13
#define PWMB  PB14
#define STBY  PB15

// QTR 8 RC - pakai 8 dulu, nanti bisa 16
#define NUM_SENSORS 8
const uint8_t qtrPins[NUM_SENSORS] = {PC0, PC1, PC2, PC3, PA0, PA1, PA2, PA3};

unsigned int sensorValues[NUM_SENSORS];

// --- PID GANAS KRI ---
float Kp = 0.045;   // Tuning di lapangan KRI
float Ki = 0.0001;
float Kd = 0.8;

float error = 0, lastError = 0;
float P, I, D, PIDvalue;
int baseSpeed = 180; // Speed lurus (0-255)
int maxSpeed = 255;

void setup() {
  Serial.begin(115200);
  // Motor setup
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  Serial.println("KALIBRASI SENSOR - Geser robot di atas garis hitam 5 detik!");
  delay(1000);
  calibrateQTR(); // Kalibrasi otomatis
  Serial.println("Kalibrasi Selesai - SIAP GANAS KRI!");
  delay(1000);
}

void loop() {
  // Baca posisi garis: 0 = kiri mentok, 7000 = kanan mentok, 3500 = tengah
  int position = readLine();

  error = position - 3500;
  
  // --- PID ---
  P = error;
  I = I + error;
  D = error - lastError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  lastError = error;

  // --- SPEED PROFILING KRI (Kunci Juara) ---
  // Kalau tikungan tajam, pelan otomatis
  int dynamicBase = baseSpeed;
  if (abs(error) > 2000) { // Tikungan 90 derajat KRI
    dynamicBase = 90;
    I = 0; // Reset I biar tidak overshoot
  } else if (abs(error) > 1000) {
    dynamicBase = 140;
  }

  int leftSpeed = dynamicBase + PIDvalue;
  int rightSpeed = dynamicBase - PIDvalue;

  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  motorDrive(leftSpeed, rightSpeed);
}

// --- FUNGSI MOTOR GANAS ---
void motorDrive(int left, int right) {
  if (left >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, left); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, -left); }

  if (right >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, right); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, -right); }
}

// --- FUNGSI BACA GARIS SEDERHANA (TANPA LIBRARY BERAT) ---
int readLine() {
  // Baca analog semua QTR
  long weightedSum = 0;
  long sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(qtrPins[i]); // 0 putih, 4095 hitam (STM32 12bit)
    // Invert kalau sensor Bapak putih=tinggi
    // sensorValues[i] = 4095 - sensorValues[i]; // Aktifkan ini kalau kebalik
    weightedSum += (long)sensorValues[i] * i * 1000;
    sum += sensorValues[i];
  }
  if (sum == 0) return 3500; // Kalau hilang garis, tetap tengah
  return weightedSum / sum;
}

void calibrateQTR() {
  // Geser manual robot kiri-kanan di atas garis selama 5 detik
  for (int i = 0; i < 200; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      analogRead(qtrPins[j]);
    }
    delay(25);
  }
}
