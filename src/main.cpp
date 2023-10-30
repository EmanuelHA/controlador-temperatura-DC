#include <Arduino.h>
#include <PID_v1.h>

#if defined(__AVR_ATtiny85__)
#define     PIN_MOSFET 1      // PIN PWM (PB1)
#define  PIN_TERMISTOR 2      // PIN ADC (PB2)
#endif

#if defined(__AVR_ATtiny88__)
#define     PIN_MOSFET 9      // PIN PWM (PB1)(ID 9)
#define  PIN_TERMISTOR 22     // PIN ADC (PC3)(ID A3)
#endif

// Referencias del controlador PID
double setpoint, input, output;
// Variables de PID
double kp = 2.0, ki = 5.0, kd = 1.0;
// Resistencia fija del divisor de tension
const float R1 = 100000;
// Coeficientes de S-H (http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm)
const float A = 2.114990448e-03, B = 0.3832381228e-04, C = 5.228061052e-07;
// Declaracion del controlador PID
PID PIDcontroller(&input, &output, &setpoint, kp, ki, kd, REVERSE);
// Declaracion de prototipos de fucion
float ln(float);
float readTemperature();

void setup() {
  // Inicializacion del control PID
  setpoint = 220.0f;
  input = readTemperature();
  PIDcontroller.SetMode(AUTOMATIC);
}

void loop() {
  PIDcontroller.Compute();
  analogWrite(PIN_MOSFET, output);
  input = readTemperature();
  delay(100);
}

float ln (float x) {
  unsigned int bx = * (unsigned int *) (&x);
  unsigned int ex = bx >> 23;
  signed int t = (signed int)ex-(signed int)127;
  unsigned int s = (t < 0) ? (-t) : t;
  bx = 1065353216 | (bx & 8388607);
  x = * (float *) (&bx);
  return -1.7417939+(2.8212026+(-1.4699568+(0.44717955-0.056570851*x)*x)*x)*x
+0.6931471806*t;
}

float readTemperature() {
  float R2, lnR2, temperature;
  // Conversion de tension a resistencia
  R2 = R1 * (1023.0 / (float)analogRead(PIN_TERMISTOR) - 1.0);
  // Logaritmo de R2 (necesario para la ecuacion)
  lnR2 = ln(R2);
  // Ecuacion SteinHart & Hart
  temperature = (1.0 / (A + lnR2*(B + C*lnR2*lnR2)));
  // Conversion y retorno de grados Kelvin a Celsius
  return (temperature - 273.15);
}