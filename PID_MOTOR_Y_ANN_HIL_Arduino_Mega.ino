#include <Adafruit_INA219.h>//sensor corriente
#include <Wire.h>

Adafruit_INA219 sensorGeneradorP (0x45);

// Variables PID

float Pv = 0; // Variable proceso, porcentaje voltaje
float Sp = 0; // Set point
int Cv = 0; // variable de control PWM

float recieveData; // Variable donde se guardan los valores recibidos por el maestro
double sendData; // Variable donde se guardan los valores para enviar al maestro

float error = 0; // Eerror actual
float error_1 = 0; // Error anterior

float Kp = 0.22; //Ganancia proporcional
float P = 0; // Termino P

float Ti = 0.13; // Tiempo integral
float I; // Termino I

// variables para asegurar tiempo de muestreo
unsigned long pasado = 0;
unsigned long actual;
float T = 3; // Tiempo de muestreo en ms

// Variables Red neuronal

// Pines Pwm
int motorP = 6; // Pin 6 Arduino Mega (para motor Pololu)
int motorG = 7; // Pin 7 Arduino Mega (para motor generico)

float porcentajePrediccion;

// Valor random para ciclo util
float pwmMotor; // Para motor Pololu
float pwmRed; // Para motor Generico

float svGenerador = 0; // Shuntvoltage
float faGenerador = 0; // Fuente Alimentacion
float vGenerador = 0; // Voltaje de la carga en mV

int resistencia = 10;
int numeroDato = 0; // Conteo de datos por ciclo util
int forSobreImpulso = 0; // Limite de for a un solo ciclo para el sobre impulso

// Variables autorregresoras
float voltaje1; // Un autorregresor variable voltaje
float voltaje2; // Dos autorregresores variable voltaje
float arregloVoltajes[3]; // Arreglo donde se realiza el corriemiento de autorregresores
float pwm1; // Un autorregresor variable PWM
float pwm2; // Dos autorregresores variable PWM
float pwm3; // Tres autorregresores variable PWM
float arregloPwm[4]; // Arreglo donde se realiza el corriemiento de autorregresores
float resistencia1; // Un autorregresor variable resistencia
float resistencia2; // Dos autorregresores variable resistencia
float resistencia3; // Tres autorregresores variable resistencia
float arregloResistencia[4]; // Arreglo donde se realiza el corriemiento de autorregresores

// Peso de coeficientes variables de entrada
float pesoEntrada[10][4] = {{ 0.0503495 ,  0.09787219,  0.230692  , -0.18955838},
  { 0.12414322, -0.24473884,  0.04695013, -0.25383953},
  { 0.14548225, -0.50778034,  0.26215501, -0.07860203},
  { 0.49524334, -0.01081944, -0.46197639, -0.68004976},
  { 0.06621314, -0.187868  , -0.22307271,  0.14026993},
  { 0.15347677,  0.18251413,  0.36365175,  0.09579585},
  { 0.35472423,  0.09908581,  0.08739848, -0.22009138},
  { -0.14705993, -0.03579321,  0.1715042 ,  0.16386781},
  { -1.60063049,  0.00878672,  0.48240163, -0.04605388},
  { -1.12231726, -0.38753033,  0.73385017, -0.33213966}
};
// Peso de coeficientes primera capa oculta
float pesoSalida[4] = { -0.92563619,  -0.34144565,  1.19496599,  -0.43680658};
// Peso de coeficientes neuronas vacias
float pesoVacioUno[4] = {  1.33780168,  0.19494858, -0.0168969 ,  0.1677481};
float  pesoVacioSalida = 0.56086622;

// Variables redes neuronales
float salidaCapaUno[4];
float prediccionCapaSalida;

float valorRecibido; // Valore recibido por el esclavo

void setup() {
  Serial.begin(9600); // Establecer velocidad puerto serial
  Wire.begin(); // Indica que funcionara como maestro

  // Configuracion pines PWM de motores
  pinMode(motorP, OUTPUT); // Para motor Pololu
  pinMode(motorG, OUTPUT); // Para motor Generico

  // Establecer transmision de datos con sensores INA219
  sensorGeneradorP.begin();

  analogWrite(motorP , 100); // Sobre impulso de motores
  analogWrite(motorG , 0); // Mantener el motor 2 apagado
}

void loop() {

  sistemaRedNeuronal();
  sistemaFisico();
  Serial.println((String) porcentajePrediccion + "," + Pv + "," + Sp);
}

void  sistemaRedNeuronal() {

  // Requerir al esclavo datos y el tamaño de ellos
  valorRecibido = 0;
  Wire.requestFrom(0x20, sizeof(valorRecibido));

  uint8_t index = 0;
  byte* pointer = (byte*) & valorRecibido;

  // Almacenar datos en datosRecibidos mientras los envie el esclavo
  while (Wire.available())
  {
    *(pointer + index) = (byte)Wire.read();
    index++;
  }

  pwmMotor = valorRecibido;

  // Asignar valores de arreglo de autorregresores a variables que seran usadas en la red neuronal  d
  // Aurotorresores de la variable de salida (Voltaje)
  voltaje1 = arregloVoltajes[1];
  voltaje2 = arregloVoltajes[0];
  // Autorregresores de la variable de resistencia
  resistencia1 = arregloResistencia[2];
  resistencia2 = arregloResistencia[1];
  resistencia3 = arregloResistencia[0];
  // Autorregresores de la variable Pwm
  pwm1 = arregloPwm[2];
  pwm2 = arregloPwm[1];
  pwm3 = arregloPwm[0];

  // Llamar la funcion de red neuronal
  red();

  Wire.beginTransmission(0x20); // Iniciar envio de datos con Esclavo
  Wire.write((byte*)& prediccionCapaSalida , sizeof(prediccionCapaSalida)); // Envio de datos con el tamaño
  Wire.endTransmission(); // Finalizacion envio de datos

  porcentajePrediccion = map(prediccionCapaSalida, 0, 9535.91, 0, 100); // Converir variable predecida en porcentaje

  // Asignar valor de variables en ultima posicion de arreglo de autorregresores
  arregloVoltajes[2] = prediccionCapaSalida;
  arregloPwm[3] = pwmRed;
  arregloResistencia[3] = resistencia;

  // Realizar corrimiento de valores dentro del arreglo de autorregresores para siguiente prediccion
  // Corrimiento autorregresores de variable de prediccion (Voltaje)
  arregloVoltajes[0] = arregloVoltajes[1];
  arregloVoltajes[1] = arregloVoltajes[2];

  // Corrimiento autorregresores de variables pwm
  arregloPwm[0] = arregloPwm[1];
  arregloPwm[1] = arregloPwm[2];
  arregloPwm[2] = arregloPwm[3];

  // Corrimiento autorregresores de variables resistencia
  arregloResistencia[0] = arregloResistencia[1];
  arregloResistencia[1] = arregloResistencia[2];
  arregloResistencia[2] = arregloResistencia[3];

}

float logic(float n)
{
  return (1.0 / (1.0 + (exp(-n))));
}

void red ()
{
  //valores normalizados
  float Npwm = (pwmMotor - 30) / (251 - 30); // Variable normarlizada valor Pwm
  float NpwmR1 = (pwm1 - 30) / (251 - 30); // Variable normalizada valor Pwm con un autorregresor
  float NpwmR2 = (pwm2 - 30) / (251 - 30); // Variable normalizada valor Pwm con dos autorregresores
  float NpwmR3 = (pwm3 - 30) / (251 - 30); // Variable normalizada valor Pwm con tres autorregresores
  float Nresistencia = resistencia / 24; // Variable normarlizada valor resistencia
  float NresistenciaR1 = resistencia1 / 24; // Variable normalizada valor resistenica con un autorregresor
  float NresistenciaR2 = resistencia2 / 24; // Variable normalizada valor resistencia con dos autorregresores
  float NresistenciaR3 = resistencia3 / 24; // Variable normalizada valor resistencia con dos autorregresores
  float NVoltajeR1 = (voltaje1 - (-0.09)) / (9535.91 - (-0.09)); // Variable normarlizada valor voltaje con un autorregresor
  float NVoltajeR2 = (voltaje2 - (-0.09)) / (9535.91 - (-0.09)); // Variable normarlizada valor voltaje con dos autorregresores

  // Primera capa oculta
  for (int i = 0 ; i <= 3 ; i++)
  {
    salidaCapaUno[i] = logic(pesoVacioUno[i] + ((Npwm * pesoEntrada[0][i]) + (NpwmR1 * pesoEntrada[1][i]) + (NpwmR2 * pesoEntrada[2][i]) + (NpwmR3 * pesoEntrada[3][i])
                             + (Nresistencia * pesoEntrada[4][i]) + (NresistenciaR1 * pesoEntrada[5][i]) + (NresistenciaR2 * pesoEntrada[6][i])
                             + (NresistenciaR3 * pesoEntrada[7][i]) + (NVoltajeR1 * pesoEntrada[8][i]) + (NVoltajeR2 * pesoEntrada[9][i])));
  }

  // salida
  prediccionCapaSalida = (pesoVacioSalida + ((salidaCapaUno[0] * pesoSalida[0]) + (salidaCapaUno[1] * pesoSalida[1]) + (salidaCapaUno[2] * pesoSalida[2])
                          + (salidaCapaUno[3] * pesoSalida[3])));

  // Qquitar normalizacion
  prediccionCapaSalida = (prediccionCapaSalida * (9535.91 - (-0.09))) + (-0.09);
}

void sistemaFisico() {
  float vGeneradorSuma = 0;
  for (int i = 0; i <= 100; i++)
  {
    // Obtener valores de sensor INA219 Generador Pololu
    svGenerador = sensorGeneradorP.getShuntVoltage_mV(); // Voltaje en la resistencia Shunt
    faGenerador = sensorGeneradorP.getBusVoltage_V(); // Voltaje en la fuente de alimentacion
    vGenerador = (faGenerador * 1000) + svGenerador; // Voltaje en la entrada de la carga
  }
  vGenerador = vGeneradorSuma / 100;

  Pv = abs(map(vGenerador, 0, 9535.91, 0, 100)); // Convertir valor de voltaje (prediccion) recibido por el maestro a porcentaje

  // Leer valor de potenciometro Set Point
  float setpoint = analogRead(A1);
  Sp = map(setpoint, 0, 1023, 0, 100); // Convertir valor de potenciometro a porcentaje

  actual = millis(); // Medir tiempo desde que se inicializa funcion
  unsigned long dt = actual - pasado; // Calcular tiempo transcurrido desde la ultima vez que fue llamada

  if (dt >= T) // El dt debe de ser mayor al tiempo de muestreo
  {
    pasado = actual; // Guardar valor actual en pasado para proxima vez que sea llamado

    error = Sp - Pv; // Calculo de error
    // Serial.print((String) "  error  " + error); // Imprimir error

    P = Kp * error; // Termino P

    I = I + (Kp / Ti) * error * (T); // Termino I

    error_1 = error; // Guardar error en tiempo pasado

    Cv = P + I; // Obtener variable de control

    Cv = constrain(Cv, 0, 255); // Restringir numero para que este dentro de rango de 0 a 255

    analogWrite(motorP, Cv); // Enviar valor de ciclo util a motor
  }
}
