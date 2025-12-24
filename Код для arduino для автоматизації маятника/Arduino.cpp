#ЗАПУСК ПРОВОДИТИ В ARDUINO IDE

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

const byte SENSOR1_PIN = A0;
const byte SENSOR2_PIN = A1;

// Наскільки повинно змінитись значення, щоб вважати промінь перекритим
const int DIFF_THRESHOLD = 80;
const float g = 9.81;

float h = 0.50;  // висота падіння (м)
float r = 0.02;  // радіус диску (м)
float m = 0.10;  // маса вантажу (кг)
float m1 = 0.05;  // маса тягарців на хрестовині (кг)
float m2 = 0.20;  // маса стержня без тягарців (кг)
float R = 0.10;  // відстань до тягарців (м)
float l = 0.40;  // довжина стержня (м)

// t-критерій Стьюдента для довірчого інтервалу
float t_a = 4.303;

const byte NUM_RUNS = 3;
float t_values[NUM_RUNS];
byte runIndex = 0;

// Значення калібровки
int base1 = 0, base2 = 0;

// Стан променів
bool beam1Broken = false;
bool beam2Broken = false;
bool prevBeam1Broken = false;
bool prevBeam2Broken = false;

// Логіка секундоміра
bool timerRunning = false;
bool waitingForStart = true;
bool waitingForFinish = false;
bool beamsMustBeClear = false;

unsigned long startTimeMs = 0;
unsigned long endTimeMs = 0;

void setup() {
    pinMode(SENSOR1_PIN, INPUT);
    pinMode(SENSOR2_PIN, INPUT);

    Wire.begin();
    lcd.init();
    lcd.backlight();

    Serial.begin(9600);

    lcd.setCursor(0, 0);
    lcd.print("Kalibraciya...");
    lcd.setCursor(0, 1);
    lcd.print("Ne zakryvat'!");

    calibrateSensors();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gotovo: vymir 1");
    lcd.setCursor(0, 1);
    lcd.print("Zakryy S1 potim");
}

void calibrateSensors() {
    const int samples = 100;
    long sum1 = 0;
    long sum2 = 0;

    for (int i = 0; i < samples; i++) {
        sum1 += analogRead(SENSOR1_PIN);
        sum2 += analogRead(SENSOR2_PIN);
        delay(10);
    }

    base1 = sum1 / samples;
    base2 = sum2 / samples;

    Serial.println("=== Calibration ===");
    Serial.print("Base1 = "); Serial.println(base1);
    Serial.print("Base2 = "); Serial.println(base2);
}

// Перевірка, чи промінь перекритий
bool isBeamBroken(int value, int base) {
    return abs(value - base) > DIFF_THRESHOLD;
}

void loop() {
    if (runIndex >= NUM_RUNS) return;

    int val1 = analogRead(SENSOR1_PIN);
    int val2 = analogRead(SENSOR2_PIN);

    beam1Broken = isBeamBroken(val1, base1);
    beam2Broken = isBeamBroken(val2, base2);

    if (beamsMustBeClear) {
        if (!beam1Broken && !beam2Broken) {
            beamsMustBeClear = false;
            waitingForStart = true;

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Gotovo: vymir ");
            lcd.print(runIndex + 1);

            lcd.setCursor(0, 1);
            lcd.print("Zakryy S1 potim");
        }
        prevBeam1Broken = beam1Broken;
        prevBeam2Broken = beam2Broken;
        return;
    }

    // --- СТАРТ секундоміра ---
    if (waitingForStart && !prevBeam1Broken && beam1Broken) {
        startTimeMs = millis();
        timerRunning = true;
        waitingForStart = false;
        waitingForFinish = true;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Vymir ");
        lcd.print(runIndex + 1);
        lcd.print(" start");

        lcd.setCursor(0, 1);
        lcd.print("Chekaemo S2...");
    }

    // --- ФІНІШ ---
    if (waitingForFinish && timerRunning && !prevBeam2Broken && beam2Broken) {
        endTimeMs = millis();
        timerRunning = false;
        waitingForFinish = false;

        float t = (endTimeMs - startTimeMs) / 1000.0;
        t_values[runIndex] = t;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Vymir ");
        lcd.print(runIndex + 1);
        lcd.print(" gotov");

        lcd.setCursor(0, 1);
        lcd.print("t=");
        lcd.print(t, 3);
        lcd.print(" s");

        runIndex++;

        if (runIndex >= NUM_RUNS) {
            calculateAndShowResults();
        }
        else {
            beamsMustBeClear = true;
        }
    }

    prevBeam1Broken = beam1Broken;
    prevBeam2Broken = beam2Broken;
}

void calculateAndShowResults() {
    float I_values[NUM_RUNS];

    for (int i = 0; i < NUM_RUNS; i++) {
        float t = t_values[i];

        float a = 2 * h / (t * t);
        float eps = a / r;
        float M = m * (g - a) * r;
        float I = M / eps;

        I_values[i] = I;
    }


    float sumI = 0;
    for (int i = 0; i < NUM_RUNS; i++) sumI += I_values[i];
    float I_mean = sumI / NUM_RUNS;

    float sumSq = 0;
    for (int i = 0; i < NUM_RUNS; i++) {
        float d = I_values[i] - I_mean;
        sumSq += d * d;
    }
    float s = sqrt(sumSq / (NUM_RUNS - 1));

    // 4. Стандартна похибка
    float standartError = s / sqrt(NUM_RUNS);

    // 5. Довірчий інтервал
    float delta = SE * t_a;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("I=");
    lcd.print(I_mean, 3);

    lcd.setCursor(0, 1);
    lcd.print("+/-");
    lcd.print(delta, 3);
}