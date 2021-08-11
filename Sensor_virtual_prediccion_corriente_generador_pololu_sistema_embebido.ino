#include <Adafruit_INA219.h>//sensor corriente

Adafruit_INA219 sensorMotorP (0x40);
Adafruit_INA219 sensorGeneradorP (0x45);

// Pines Pwm
int motorP = 6; // Pin 6 Arduino Mega (para motor Pololu)

// Pines para interrupcion de encoder
int encoderMotor = 2;
int encoderGenerador = 3;

int resistencia = 10;
int numeroDato = 0; // Conteo de datos por ciclo util
int forSobreImpulso = 0; // Limite de for a un solo ciclo para el sobre impulso

unsigned int rpmMotor = 0;            // Revoluciones por minuto calculadas.
unsigned int rpmGenerador = 0;        // Revoluciones por minuto calculadas.
volatile byte pulsosMotor = 0;        // Número de pulsos leidos por el Arduino en un segundo
volatile byte pulsosGenerador = 0;    // Número de pulsos leidos por el Arduino en un segundo
unsigned long tiempoMotor = 0;        // Tiempo
unsigned long tiempoGenerador = 0;    // Tiempo
static volatile unsigned long tiempoReboteMotor = 0;     // Tiempo del rebote.
static volatile unsigned long tiempoReboteGenerador = 0; // Tiempo del rebote.
unsigned int numeroMuescas = 12; // Número de muescas que tiene el disco del encoder.

// Valor random para ciclo util
float pwmMotor; // Para motor Pololu

float svMotor = 0; // Shuntvoltage
float faMotor = 0; // Fuente Alimentacion
float vMotor = 0; // Voltaje de la carga en mV
float cMotor = 0; // Corriente de la carga en mA
float svGenerador = 0; // Shuntvoltage
float faGenerador = 0; // Fuente Alimentacion
float vGenerador = 0; // Voltaje de la carga en mV
float cGenerador = 0; // Corriente de la carga en mA

// Variables autorregresoras
float corriente1 = 0; // Autorregresor uno variable de salida corriente generador
float corriente2 = 0; // Autorregresor dos variable de salida corriente generador
float corriente3 = 0; // Autorregresor tres variable de salida corriente generador
float autorreresoresCorriente[4] = {0, 0, 0, 0}; // Arreglo donde se realiza el corriemiento de autorregresores

// Peso de coeficientes variables de entrada
float pesoEntrada[10][6] = {{ 0.10433859,  0.07615146,  0.30752148, -0.37842551, -0.28603359,     0.26281427},
  { 0.02611759, -0.0442557 ,  0.27074476,  0.21024687,  0.34993859,   -0.24395077},
  { 0.21613436, -0.17935778, -0.01416489,  0.05304785, -0.11315   ,    0.03063525},
  { -0.20926019,  0.16096659,  0.06224134,  0.08137274, -0.14171729,    0.37267934},
  { 0.26710852,  0.05678658, -0.22075162, -0.1269945 ,  0.15589049,   -0.19908153},
  { -0.32469995,  0.18027679,  0.07767284,  0.46223136, -0.23437649,   -0.26049152},
  { 0.07638786, -0.01203665,  0.18285504,  0.38721025, -0.11642481,    0.36318586},
  { -0.27976162, -0.0887764 ,  0.85897466, -0.71028213,  0.17607334,    0.46972637},
  { 0.31047586, -0.43326161,  0.2170249 , -0.25924324,  0.3417878 ,    0.45916226},
  { 0.02029994, -0.68222511,  0.41072451, -0.35316739,  0.34135534,    0.11827475}
};
// Peso de coeficientes primera capa oculta
float pesoSalida[6] = {0.15030452, -0.52454721,  0.67321959, -0.97381235,  0.16827032, 0.60860765};
// Peso de coeficientes neuronas vacias
float pesoVacioEntrada[6] = { -0.28368652, -0.08180608,  0.04067154,  0.3984017 ,  0.34561398, 0.23757913};
float pesoVacioSalida = 0.04162424;

// Variables redes neuronales
float salidaCapaUno[4];
float prediccionCapaSalida;

void setup() {

  Serial.begin (9600); // Establecer velocidad puerto serial

  // configuracion pines de encoder
  pinMode(encoderMotor, INPUT);
  pinMode(encoderGenerador, INPUT);

  // Configuracion pines PWM de motores
  pinMode(motorP, OUTPUT); // Para motor Pololu

  attachInterrupt(0, interrupcionMotor, RISING); //interrupcion 1 - pin 2
  attachInterrupt(1, interrupcionGenerador, RISING); //interrupcion 2 - pin3

  // Establecer transmision de datos con sensores INA219
  sensorMotorP.begin(); //sensores de corriente
  sensorGeneradorP.begin();

}

void loop(void) {

  pwmMotor = random(30, 255);

  for (int i = 1; i <= 300; i++) {

    analogWrite(motorP, pwmMotor); //valor 0-255

    if (millis() - tiempoMotor >= 1000) { // Se actualiza cada medio segundo
      noInterrupts(); // Desconectamos las interrupciones para que no actué en esta parte del programa.
      rpmMotor = ((60000) / numeroMuescas) / (millis() - tiempoMotor) * pulsosMotor; // Calculamos las revoluciones por minuto
      tiempoMotor = millis(); // Almacenamos el tiempo actual
      pulsosMotor = 0;
      interrupts(); // Reiniciamos la interrupción
    }

    if (millis() - tiempoGenerador >= 1000) { // Se actualiza cada medio segundo
      noInterrupts();
      rpmGenerador = ((60000) / numeroMuescas) / (millis() - tiempoGenerador) * pulsosGenerador; // Calculamos las revoluciones por minuto
      tiempoGenerador = millis(); // Almacenamos el tiempo actual
      pulsosGenerador = 0;
      interrupts(); // Reiniciamos la interrupción
    }

    svMotor = sensorMotorP.getShuntVoltage_mV();
    faMotor = sensorMotorP.getBusVoltage_V();
    cMotor = (sensorMotorP.getCurrent_mA());
    vMotor = (faMotor * 1000) + svMotor ;

    svGenerador = sensorGeneradorP.getShuntVoltage_mV();
    faGenerador = sensorGeneradorP.getBusVoltage_V();
    cGenerador = (sensorGeneradorP.getCurrent_mA());
    vGenerador = (faGenerador * 1000) + svGenerador;

    corriente1 = autorreresoresCorriente[2];
    corriente2 = autorreresoresCorriente[1];
    corriente3 = autorreresoresCorriente[0];

    red();

    autorreresoresCorriente[3] = prediccionCapaSalida;

    Serial.println( (String) prediccionCapaSalida + "," + cGenerador );

    //Serial.println( (String) pwmMotor + ",  " + rpmMotor + "," + rpmGenerador + ",  " + cMotor + "," + cGenerador + ",  " + vMotor + "," + vGenerador + ",  " + resistencia + ",  " + numeroDato  + ",  --- " + prediccionCapaSalida+ ",   " + corriente1 + "," + corriente2 + "," + corriente3 );
    delay(5);

    autorreresoresCorriente[0] = autorreresoresCorriente[1];
    autorreresoresCorriente[1] = autorreresoresCorriente[2];
    autorreresoresCorriente[2] = autorreresoresCorriente[3];
  }
  rpmMotor = rpmGenerador = 0; //para cambiar de pwm
  numeroDato++;
}

void interrupcionMotor()
{
  if (  digitalRead (encoderMotor) && (micros() - tiempoReboteMotor > 500) && digitalRead (encoderMotor) ) {
    //  Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
    tiempoReboteMotor = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
    pulsosMotor++;  // Suma el pulso bueno que entra.
  }
}
void interrupcionGenerador()
{
  if (  digitalRead (encoderGenerador) && (micros() - tiempoReboteGenerador > 500) && digitalRead (encoderGenerador) ) {
    // Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
    tiempoReboteGenerador = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
    pulsosGenerador++;  // Suma el pulso bueno que entra.
  }
}

float logic(float n)
{
  return (1.0 / (1.0 + (exp(-n))));
}

void red ()
{
  //valores normalizados
  float pwmMotorN = (pwmMotor - 30) / (251 - 30); // Variable normarlizada valor Pwm
  float rpmMotorN = ((float)rpmMotor / 1265.00); // Variable normarlizada valor RPM motor
  float rpmGeneradorN = ((float)rpmGenerador / 1265.00); // Variable normarlizada valor RPM generador
  float cMotorN = (cMotor - (-318.50)) / (1576.10 - (-318.50)); // Variable normarlizada valor Corriente motor
  float vMotorN = (vMotor / 12154.48); // Variable normarlizada valor Voltaje motor
  float vGeneradorN = (vGenerador - (-0.09)) / (9535.91 - (-0.09)); // Variable normarlizada valor Voltaje generador
  float resistenciaN = resistencia / 24; // Variable normarlizada valor resistencia de carga
  float corriente1N = (corriente1 - (-1.30)) / (648.80 - (-1.30)); // Variable normarlizada valor Corriente motor autorregresor 1
  float corriente2N = (corriente2 - (-1.30)) / (648.80 - (-1.30)); // Variable normarlizada valor Corriente motor autorregresor 2
  float corriente3N = (corriente3 - (-1.30)) / (648.80 - (-1.30)); // Variable normarlizada valor Corriente motor autorregresor 3

  //primera capa oculta
  for (int i = 0; i <= 5; i++)
  {
    salidaCapaUno[i] = logic( pesoVacioEntrada[i] + ((pwmMotorN * pesoEntrada[0][i]) + (rpmMotorN * pesoEntrada[1][i])+ (rpmGeneradorN * pesoEntrada[2][i]) + (cMotorN * pesoEntrada[3][i]) + (vMotorN * pesoEntrada[4][i])+ (vGeneradorN * pesoEntrada[5][i]) + (resistenciaN * pesoEntrada[6][i]) + (corriente1N * pesoEntrada[7][i])+ (corriente2N * pesoEntrada[8][i]) + (corriente3N * pesoEntrada[9][i])));
  }

  //salida
  prediccionCapaSalida = ( pesoVacioSalida + ((salidaCapaUno[0] * pesoSalida[0]) + (salidaCapaUno[1] * pesoSalida[1])+ (salidaCapaUno[2] * pesoSalida[2]) + (salidaCapaUno[3] * pesoSalida[3]) + (salidaCapaUno[4] * pesoSalida[4]) + (salidaCapaUno[5] * pesoSalida[5])));

  //quitar normalizacion
  prediccionCapaSalida = (prediccionCapaSalida * (648.80 - (-1.30))) + (-1.30);
}
