#include <Arduino.h>

// Pines
const int sensorPin = 34;   // Entrada del termistor
const int pwmHeater = 13;   // Salida PWM al transistor
const int pwmLed = 12;      // Salida PWM al LED
const int potPin = 35;      // Potenciómetro para setpoint

// Parámetros del termistor
const float referenciaV = 3.3;
const float resistenciaTermistor = 10000.0; // 10k Ohm
const int adcMaxValue = 4095;
const float BETA = 3950.0;
const float T0_K = 25.0 + 273.15; // Kelvin

// PID
float Kp = 2.5;
float Ki = 0.4;
float Kd = 3.5;

float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float salidaPID = 0.0;

// Tiempo PID
unsigned long lastTime = 0;
float deltaTime = 0.1;  // Valor inicial seguro

// Muestreo
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 100; // cada 100 ms

// Temperatura
float temperaturaC = 0.0;
float resistenciaMedida = 0.0;
float voltajeTermistor = 0.0;

// ----------------------------
// Función para lectura serial PID
void checkSerialCommands() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("K ")) {
      float newKp, newKi, newKd;
      int matched = sscanf(input.c_str(), "K %f %f %f", &newKp, &newKi, &newKd);
      if (matched == 3) {
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
        Serial.printf("PID actualizado: Kp=%.2f Ki=%.2f Kd=%.2f\n", Kp, Ki, Kd);
      }
    }
  }
}
// ----------------------------

void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT);
  pinMode(pwmHeater, OUTPUT);
  pinMode(pwmLed, OUTPUT);
  pinMode(potPin, INPUT);
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();

  // Leer comandos del puerto serial
  checkSerialCommands();

  // Ejecutar solo cada 100 ms
  if (now - lastSampleTime >= sampleInterval) {
    lastSampleTime = now;

    // Leer temperatura
    int valorADC = analogRead(sensorPin);
    voltajeTermistor = (valorADC / float(adcMaxValue)) * referenciaV;
    resistenciaMedida = (resistenciaTermistor * (referenciaV - voltajeTermistor)) / voltajeTermistor;
    float tempK = 1.0 / ((log(resistenciaMedida / resistenciaTermistor) / BETA) + (1.0 / T0_K));
    temperaturaC = tempK - 273.15;

    // Leer setpoint desde potenciómetro
    int potValue = analogRead(potPin);
    float tempSetpoint = map(potValue, 0, 4095, 25, 80);

    // Calcular deltaTime
    deltaTime = (now - lastTime) / 1000.0;
    if (deltaTime <= 0) deltaTime = 0.01;
    lastTime = now;

    // PID
    error = tempSetpoint - temperaturaC;
    integral += error * deltaTime;

    // Anti-windup
    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;

    derivative = (error - lastError) / deltaTime;
    salidaPID = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // PWM limitado (0 a 28)
    int pwmOut = constrain((int)salidaPID, 0, 28);
    analogWrite(pwmHeater, pwmOut);
    analogWrite(pwmLed, pwmOut);

    // Error en porcentaje
    float porcentajeError = (error / tempSetpoint) * 100.0;

    // Serial tabulado para Python
    Serial.printf("Setpoint: %5.1f\tTemp: %5.1f\tPWM: %2d\tError: %6.2f\n",
                  tempSetpoint, temperaturaC, pwmOut, porcentajeError);
  }
}
