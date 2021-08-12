#include <Adafruit_INA219.h>//sensor corriente

Adafruit_INA219 sensorMotorP (0x40); //direccionamiento i2c a sensores
Adafruit_INA219 sensorMotorG (0x41);
Adafruit_INA219 sensorGeneradorG (0x44);
Adafruit_INA219 sensorGeneradorP (0x45);

// Pines Pwm
int motorP = 6; // Pin 6 Arduino Mega (para motor Pololu)
int motorG = 7; // Pin 7 Arduino Mega (para motor generico)

// Pines para interrupcion de encoder
int encoderMotorP = 2;
int encoderGeneradorP = 3;
int encoderMotorG = 18;
int encoderGeneradorG = 19;

int resistencia = 10;
int numeroDato = 0; // Conteo de datos
int forTamano = 0; // Tamaño del for dependiendo del sobreimpulso
int unicoSobreImpulso = 0; // Limite de for a un solo ciclo para el sobre impulso
int rangoPwm = 0; // rango de pwm para obtener datos de ciclos utiles bajos, medios y altos

// Variables encoder motor Pololu
unsigned int rpmMotorP = 0;            // Revoluciones por minuto calculadas.
unsigned int rpmGeneradorP = 0;        // Revoluciones por minuto calculadas.
volatile byte pulsosMotorP = 0;        // Número de pulsos leidos por el Arduino en un segundo
volatile byte pulsosGeneradorP = 0;    // Número de pulsos leidos por el Arduino en un segundo
unsigned long tiempoMotorP = 0;        // Tiempo
unsigned long tiempoGeneradorP = 0;    // Tiempo
static volatile unsigned long tiempoReboteMotorP = 0;     // Tiempo del rebote.
static volatile unsigned long tiempoReboteGeneradorP = 0; // Tiempo del rebote.
unsigned int numeroMuescasP = 12; // Número de muescas que tiene el disco del encoder.

// Variables encoder motor Generico
unsigned int rpmMotorG = 0;            // Revoluciones por minuto calculadas.
unsigned int rpmGeneradorG = 0;        // Revoluciones por minuto calculadas.
volatile byte pulsosMotorG = 0;        // Número de pulsos leidos por el Arduino en un segundo
volatile byte pulsosGeneradorG = 0;    // Número de pulsos leidos por el Arduino en un segundo
unsigned long tiempoMotorG = 0;        // Tiempo
unsigned long tiempoGeneradorG = 0;    // Tiempo
static volatile unsigned long tiempoReboteMotorG = 0;     // Tiempo del rebote.
static volatile unsigned long tiempoReboteGeneradorG = 0; // Tiempo del rebote.
unsigned int numeroMuescasG = 20; // Número de muescas que tiene el disco del encoder.

void setup() {

  Serial.begin (9600); // Establecer velocidad puerto serial

  // configuracion pines de encoder
  pinMode(encoderMotorP, INPUT);
  pinMode(encoderGeneradorP, INPUT);
  pinMode(encoderMotorG, INPUT);
  pinMode(encoderGeneradorG, INPUT);

  attachInterrupt(0, interrupcionMotorP, RISING); // interrupcion 1 - pin 2
  attachInterrupt(1, interrupcionGeneradorP, RISING); // interrupcion 2 - pin 3
  attachInterrupt(4, interrupcionMotorG, RISING); // interrupcion 3 - pin 18
  attachInterrupt(5, interrupcionGeneradorG, RISING); // interrupcion 4 - pin 19

  // Configuracion pines PWM de motores
  pinMode(motorP, OUTPUT); // Para motor Pololu
  pinMode(motorG, OUTPUT); // Para motor Generico

  // Establecer transmision de datos con sensores INA219
  sensorMotorP.begin();
  sensorGeneradorP.begin();
  sensorMotorG.begin();
  sensorGeneradorG.begin();
}

void loop(void) {

  // Valor random para ciclo util
  float pwmMotorP; // Para motor Pololu
  float pwmMotorG; // Para motor Generico

  // Variables sensor INA219 motor Pololu
  float svMotorP = 0; // Shuntvoltage
  float faMotorP = 0; // Fuente Alimentacion
  float vMotorP = 0; // Voltaje de la carga en mV
  float cMotorP = 0; // Corriente de la carga en mA
  float svGeneradorP = 0; // Shuntvoltage
  float faGeneradorP = 0; // Fuente Alimentacion
  float vGeneradorP = 0; // Voltaje de la carga en mV
  float cGeneradorP = 0; // Corriente de la carga en mA

  // Variables sensor INA219 motor Generico
  float svMotorG = 0; // Shuntvoltage
  float faMotorG = 0; // Fuente Alimentacion
  float vMotorG = 0; // Voltaje de la carga en mV
  float cMotorG = 0; // Corriente de la carga en mA
  float svGeneradorG = 0; // Shuntvoltage
  float faGeneradorG = 0; // Fuente Alimentacion
  float vGeneradorG = 0; // Voltaje de la carga en mV
  float cGeneradorG = 0; // Corriente de la carga en mA

  if (unicoSobreImpulso == 0)
  {
    pwmMotorP = pwmMotorG = 200;// Sobre impulso
    forTamano = 0;
  }
  else
  {
    forTamano = 150;
    if (rangoPwm == 1 || rangoPwm == 2) // PWM rango bajo
    {
      pwmMotorP = random(30, 105);
      pwmMotorG = random(30, 105);
    }
    else if (rangoPwm == 3 || rangoPwm == 4) // PWM rango medio
    {
      pwmMotorP = random(106, 180);
      pwmMotorG = random(106, 180);
    }
    else if (rangoPwm == 5 || rangoPwm == 6) // PWM rango alto
    {
      pwmMotorP = random(181, 255);
      pwmMotorG = random(181, 255);
      if (rangoPwm == 6) // reset del rando PWM
      {
        rangoPwm = 0;
      }
    }
  }

  for (int i = 1; i <= forTamano; i++) {

    // Valores de los valores de carga en los generadores
    if (numeroDato <= 24)
    {
      resistencia = 5;
    }
    else if (25 <= numeroDato && numeroDato <= 48)
    {
      resistencia = 10;
    }
    else if (49 <= numeroDato && numeroDato <= 72)
    {
      resistencia = 18;
    }
    else if (73 <= numeroDato && numeroDato <= 96)
    {
      resistencia = 24;
    }
    else if (97 <= numeroDato && numeroDato <= 120)
    {
      resistencia = 0;
    }

    // cambio de ciclo util en los motores
    analogWrite(motorP, pwmMotorP);
    analogWrite(motorG, pwmMotorG);

    if (millis() - tiempoMotorP >= 1000) { // Se actualiza cada medio segundo
      noInterrupts(); // Desconectamos las interrupciones para que no actué en esta parte del programa.
      rpmMotorP = ((60000) / numeroMuescasP) / (millis() - tiempoMotorP) * pulsosMotorP; // Calculamos las revoluciones por minuto
      tiempoMotorP = millis(); // Almacenamos el tiempo actual
      pulsosMotorP = 0;
      interrupts(); // Reiniciamos la interrupción
    }

    if (millis() - tiempoGeneradorP >= 1000) { // Se actualiza cada medio segundo
      noInterrupts();
      rpmGeneradorP = ((60000) / numeroMuescasP) / (millis() - tiempoGeneradorP) * pulsosGeneradorP; // Calculamos las revoluciones por minuto
      tiempoGeneradorP = millis(); // Almacenamos el tiempo actual
      pulsosGeneradorP = 0;
      interrupts(); // Reiniciamos la interrupción
    }

    if (millis() - tiempoMotorG >= 1000) { // Se actualiza cada medio segundo
      noInterrupts();
      rpmMotorG = ((60000) / numeroMuescasG) / (millis() - tiempoMotorG) * pulsosMotorG; // Calculamos las revoluciones por minuto
      tiempoMotorG = millis(); // Almacenamos el tiempo actual
      pulsosMotorG = 0;
      interrupts(); // Reiniciamos la interrupción
    }

    if (millis() - tiempoGeneradorG >= 1000) { // Se actualiza cada medio segundo
      noInterrupts(); // Desconectamos la interrupción para que no actué en esta parte del programa.
      rpmGeneradorG = ((60000) / numeroMuescasG) / (millis() - tiempoGeneradorG) * pulsosGeneradorG; // Calculamos las revoluciones por minuto
      tiempoGeneradorG = millis(); // Almacenamos el tiempo actual
      pulsosGeneradorG = 0;
      interrupts(); // Reiniciamos la interrupción
    }

    svMotorP = sensorMotorP.getShuntVoltage_mV();
    faMotorP = sensorMotorP.getBusVoltage_V();
    cMotorP = (sensorMotorP.getCurrent_mA());
    vMotorP = (faMotorP * 1000) + svMotorP;

    svGeneradorP = sensorGeneradorP.getShuntVoltage_mV();
    faGeneradorP = sensorGeneradorP.getBusVoltage_V();
    cGeneradorP = (sensorGeneradorP.getCurrent_mA());
    vGeneradorP = (faGeneradorP * 1000) + svGeneradorP ;

    svMotorG = sensorMotorG.getShuntVoltage_mV();
    faMotorG = sensorMotorG.getBusVoltage_V();
    cMotorG = (sensorMotorG.getCurrent_mA());
    vMotorG = (faMotorG * 1000) + svMotorG;

    svGeneradorG = sensorGeneradorG.getShuntVoltage_mV();
    faGeneradorG = sensorGeneradorG.getBusVoltage_V();
    cGeneradorG = (sensorGeneradorG.getCurrent_mA());
    vGeneradorG = (faGeneradorG * 1000) + svGeneradorG ;

    Serial.println( (String) pwmMotorP + "," + pwmMotorG + "," + rpmMotorP + "," + rpmGeneradorP + "," + rpmMotorG + "," + rpmGeneradorG + "," + cMotorP + "," + cGeneradorP + "," + cMotorG + "," + cGeneradorG + "," + vMotorP + "," + vGeneradorP + "," + vMotorG + "," + vGeneradorG + "," + resistencia + "," + resistencia + "," + numeroDato);

    delay(5);
  }
  pwmMotorP = pwmMotorG = rpmMotorP = rpmGeneradorP = rpmMotorG = rpmGeneradorG = 0; //para cambiar de pwm
  unicoSobreImpulso = 1; //para que el for sea solo 1 y no se quede en bucle del mismo pwm
  numeroDato++;
  rangoPwm++;

}

void interrupcionMotorP()
{
  if (  digitalRead (encoderMotorP) && (micros() - tiempoReboteMotorP > 500) && digitalRead (encoderMotorP) ) {
    //  Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
    tiempoReboteMotorP = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
    pulsosMotorP++;  // Suma el pulso bueno que entra.
  }
}
void interrupcionGeneradorP()
{
  if (  digitalRead (encoderGeneradorP) && (micros() - tiempoReboteGeneradorP > 500) && digitalRead (encoderGeneradorP) ) {
    // Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
    tiempoReboteGeneradorP = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
    pulsosGeneradorP++;  // Suma el pulso bueno que entra.
  }
}
void interrupcionMotorG()
{
  if (  digitalRead (encoderMotorG) && (micros() - tiempoReboteMotorG > 500) && digitalRead (encoderMotorG) ) {
    // Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
    tiempoReboteMotorG = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
    pulsosMotorG++;  // Suma el pulso bueno que entra.
  }
}
void interrupcionGeneradorG()
{
  if (  digitalRead (encoderGeneradorG) && (micros() - tiempoReboteGeneradorG > 500) && digitalRead (encoderGeneradorG) ) {
    //    Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
    tiempoReboteGeneradorG = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
    pulsosGeneradorG++;  // Suma el pulso bueno que entra.
  }
}
