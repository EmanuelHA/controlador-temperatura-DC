#include <Arduino.h>
#include <PID_v1.h>
#include <TinyI2CMaster.h>
#define  TINY4KOLED_QUICK_BEGIN // Optimizacion de espacio en OLED blanca
#include <Tiny4kOLED.h>

#if defined(__AVR_ATtiny88__)
#define        PIN_MOSFET 9     // PWM (PB1)(ID 9)
#define   PIN_THERMISTOR 22     // ADC (PC3)(ID A3/22)
#endif

#if defined(__AVR_ATtiny85__)
#define       PIN_MOSFET PB1    // PWM
#define   PIN_THERMISTOR PB4    // ADC
#endif

// Retardo en la actualizacion de la pantalla OLED
#define PRINT_DELAY 500
// Temporizadores de actualizacion
unsigned long timer = 0, timerPrev = 0;
// Resistencia fija del divisor de tension
const float R1 = 100000;
// Coeficientes de S & H (http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm)
const float A = 2.114990448e-03, B = 0.3832381228e-04, C = 5.228061052e-07;
// Referencias del controlador PID
double setpoint, input, output;
// Variables de PID
double kp = 2.0, ki = 5.0, kd = 1.0;
const int DIRECTION = REVERSE;
// Declaracion del controlador PID
PID PIDcontroller(&input, &output, &setpoint, kp, ki, kd, DIRECTION);
// Logica de programa
struct Parametros {
  float temperatura;
  float humedad = 55.0f;
  int8_t eclosion;
};

float readTemperature() {
  int Vo = analogRead(PIN_THERMISTOR);			// lectura de A0
  float R2 = R1 / (1023.0 / (double)Vo - 1.0);	// conversion de tension a resistencia
  float lnR2 = log(R2);			// logaritmo de R2 necesario para ecuacion
  float temperatura = (1.0 / (A + (B + C*lnR2*lnR2)*lnR2)); 	// ecuacion S-H
  temperatura -= 273.15;   // Kelvin a Centigrados (Celsius)
  return temperatura;
}

void updateDisplay() {
  oled.clear();
  oled.setCursor(0,1);
  oled.print("TEMPERATURA:");
  oled.setCursor(0,2);
  oled.print(input);
  oled.print(" C");
  oled.switchFrame();
}

void setup() {
  // Inicializacion pines
  pinMode(PIN_MOSFET, OUTPUT);
  pinMode(PIN_THERMISTOR, INPUT);
  // Inicializacion OLED
  oled.begin();
  oled.setFont(FONT6X8);
  oled.on();
  oled.switchRenderFrame();
  // Inicializacion del control PID
  setpoint = 240.0f;
  input = static_cast <double> (readTemperature());
  PIDcontroller.SetMode(AUTOMATIC);
}

void loop() {
  timer = millis();
  PIDcontroller.Compute();
  analogWrite(PIN_MOSFET, output);
  input = readTemperature();
  if ((timer - timerPrev) > PRINT_DELAY) {
    updateDisplay();
    timerPrev = timer;
  }
}
