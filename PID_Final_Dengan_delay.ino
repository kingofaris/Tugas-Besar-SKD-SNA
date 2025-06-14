#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>

//inisiasi LCD yang digunakan
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pin motor driver L298N
#define ENA 5  // Pompa BASA (pH UP)
#define IN1 7
#define ENB 6  // Pompa ASAM (pH DOWN)
#define IN3 8

// Sensor pH
#define PH_SENSOR A0

// Setpoint pH
double setpoint = 6.75;  // Nilai tengah antara 6.5-7.0
double input, output;

// Parameter PID (sesuaikan dengan sistem)
double Kp = 6.0;
double Ki = 0.3;
double Kd = 0.5;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Batas pH dan deadzone
const float pH_min = 6.0;
const float pH_max = 7.0;
const float deadzone = 0.1; // Deadzone 0.1 pH

// Kecepatan minimum pompa (agar bisa berputar)
const int MIN_PUMP_SPEED = 80;  // Minimum PWM untuk pompa bisa berputar
const int MAX_PUMP_SPEED = 255; // Maximum PWM

// Variabel untuk delay pompa
const unsigned long PUMP_ON_TIME = 500;   // 2 detik pompa aktif memberikan cairan
const unsigned long PUMP_DELAY = 20000;     // 3.5 detik delay untuk pembacaan sensor
unsigned long pumpStartTime = 0;
bool pumpIsRunning = false;
bool pumpInDelay = false;

// Variabel kalibrasi pH
int buf[10];

void setup() {
  Serial.begin(9600);

  // Inisialisasi pin motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  
  // Set arah pompa
  digitalWrite(IN1, HIGH);  // Arah pompa basa
  digitalWrite(IN3, HIGH);  // Arah pompa asam
  
  // Inisialisasi PID
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);  // Output negatif untuk pH UP, positif untuk pH DOWN
  
  // Stop pompa
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  //Inisiasi LCD & First Message
  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("Wong Liyo");
  lcd.setCursor(2, 1);
  lcd.print("Ngerti OPO");
  delay(3000);
  lcd.clear(); 
  lcd.setCursor(0, 0);
  lcd.print("LumiaTech X SAS");
  lcd.setCursor(1, 1);
  lcd.print("Hidroponik PID");
  delay(2000);
  lcd.clear();
  
  Serial.println("=== pH Monitor System Started ===");
  Serial.println("ENA = Pompa BASA (pH UP)");
  Serial.println("ENB = Pompa ASAM (pH DOWN)");
}

void loop() {
  input = read_pH();  // Baca pH
  pid.Compute();
  
  // Kontrol pompa dengan delay
  control_pumps_with_delay();
  
  // Debugging yang diperbaiki
  double error = setpoint - input;
  Serial.print("pH: ");
  Serial.print(input, 2);
  Serial.print(" | Setpoint: ");
  Serial.print(setpoint, 2);
  Serial.print(" | Error: ");
  Serial.print(error, 2);
  Serial.print(" | PID Output: ");
  Serial.print(output, 1);
  

  
  //Grafil Plot
  Serial.print(input);
  Serial.print(" ");
  Serial.println(setpoint);

  // Status pompa yang benar
  if (pumpIsRunning) {
    unsigned long remainingTime = PUMP_ON_TIME - (millis() - pumpStartTime);
    if (input < pH_min) {
      Serial.print(" | Status: POMPA BASA AKTIF - ");
      Serial.print(remainingTime / 1000.0, 1);
      Serial.println("s");
    } else if (input > pH_max) {
      Serial.print(" | Status: POMPA ASAM AKTIF - ");
      Serial.print(remainingTime / 1000.0, 1);
      Serial.println("s");
    }
  } else if (pumpInDelay) {
    unsigned long remainingDelay = PUMP_DELAY - (millis() - pumpStartTime - PUMP_ON_TIME);
    Serial.print(" | Status: DELAY PEMBACAAN - ");
    Serial.print(remainingDelay / 1000.0, 1);
    Serial.println("s");
  } else if (abs(error) <= deadzone) {
    Serial.println(" | Status: IDLE - Dalam range");
  } else if (input < pH_min) {
    Serial.println(" | Status: pH RENDAH - Siap aktifkan pompa BASA");
  } else if (input > pH_max) {
    Serial.println(" | Status: pH TINGGI - Siap aktifkan pompa ASAM");
  }

//Nilai yang digunakan untuk diagram plotter
  Serial.print("pH_Actual:");
  Serial.print(input, 2);
  Serial.print(",Setpoint:");
  Serial.println(setpoint, 2);

  // Tampilan pada LCD
  display_lcd_status();
  
  delay(500); // Sample time
}
 
float read_pH(){
  // Baca sensor dengan averaging
  for(int i = 0; i < 10; i++){
    buf[i] = analogRead(PH_SENSOR);
    delay(10);
  }

  float avgValue = 0;
  for(int i = 0; i < 10; i++){
    avgValue += buf[i];
  }
  
  float pHVol = (float)avgValue * 5.0 / 1024 / 10;
  float phValue = (-5.70 * pHVol + 21.34);
  
  // Batasi nilai pH dalam range wajar
  if (phValue < 0) phValue = 0;
  if (phValue > 14) phValue = 14;
  
  return phValue;
}

void control_pumps_with_delay() {
  double error = setpoint - input;
  unsigned long currentTime = millis();
  
  // Jika pompa sedang aktif memberikan cairan
  if (pumpIsRunning) {
    if (currentTime - pumpStartTime >= PUMP_ON_TIME) {
      // Waktu pemberian cairan selesai, matikan pompa dan mulai delay
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      pumpIsRunning = false;
      pumpInDelay = true;
    }
    return; // Tetap dalam mode pemberian cairan
  }
  
  // Jika pompa sedang dalam delay pembacaan
  if (pumpInDelay) {
    if (currentTime - pumpStartTime >= (PUMP_ON_TIME + PUMP_DELAY)) {
      // Delay selesai, reset status
      pumpInDelay = false;
    }
    return; // Masih dalam periode delay
  }
  
  // Cek apakah perlu mengaktifkan pompa
  if (abs(error) <= deadzone) {
    // Dalam range target, tidak perlu pompa
    return;
  }
  
  // Hitung kecepatan pompa
  int pump_speed = map(constrain(abs(output), 0, 255), 0, 255, MIN_PUMP_SPEED, MAX_PUMP_SPEED);
  
  if (input < pH_min) {  
    // pH TERLALU RENDAH - Butuh BASA untuk menaikkan pH
    analogWrite(ENA, pump_speed);
    analogWrite(ENB, 0);
    start_pump_cycle();
    
  } else if (input > pH_max) {  
    // pH TERLALU TINGGI - Butuh ASAM untuk menurunkan pH
    analogWrite(ENA, 0);
    analogWrite(ENB, pump_speed);
    start_pump_cycle();
  }
  return pump_speed;
}

void start_pump_cycle() {
  pumpStartTime = millis();
  pumpIsRunning = true;
  pumpInDelay = false;
}

void display_lcd_status() {
  lcd.clear();
  
  if (pumpIsRunning) {
    unsigned long remainingTime = PUMP_ON_TIME - (millis() - pumpStartTime);
    lcd.setCursor(0, 0);
    if (input < pH_min) {
      lcd.print("POMPA BASA: ");
      lcd.print(remainingTime / 1000.0, 1);
      lcd.print("s");
    } else if (input > pH_max) {
      lcd.print("POMPA ASAM: ");
      lcd.print(remainingTime / 1000.0, 1);
      lcd.print("s");
    }
    lcd.setCursor(3, 1);
    lcd.print("pH -> ");
    lcd.print(input, 2);
    
  } else if (pumpInDelay) {
    unsigned long remainingDelay = PUMP_DELAY - (millis() - pumpStartTime - PUMP_ON_TIME);
    lcd.setCursor(0, 0);
    lcd.print("Delay: ");
    lcd.print(remainingDelay / 1000.0, 1);
    lcd.print("s");
    lcd.setCursor(3, 1);
    lcd.print("pH -> ");
    lcd.print(input, 2);
    
  } else {
    lcd.setCursor(2, 0);
    lcd.print("Kandungan pH");
    lcd.setCursor(3, 1);
    lcd.print("pH -> ");
    lcd.print(input, 2);
  }
}

// Fungsi untuk testing manual pompa (panggil di setup() jika perlu)
// void test_pumps() {
//   Serial.println("=== Testing Pompa ===");
  
//   Serial.println("Testing Pompa BASA (ENA)...");
//   analogWrite(ENA, 150);  // Test dengan kecepatan sedang
//   analogWrite(ENB, 0);
//   delay(3000);
  
//   Serial.println("Testing Pompa ASAM (ENB)...");
//   analogWrite(ENA, 0);
//   analogWrite(ENB, 150);  // Test dengan kecepatan sedang
//   delay(3000);
  
//   Serial.println("Stop semua pompa");
//   analogWrite(ENA, 0);
//   analogWrite(ENB, 0);
//   delay(1000);
// }