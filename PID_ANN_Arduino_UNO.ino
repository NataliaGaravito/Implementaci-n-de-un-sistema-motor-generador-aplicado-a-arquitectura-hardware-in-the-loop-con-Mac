#include <Wire.h>

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


void setup() {
  Serial.begin(9600); // Establece velocidad de datos para puerto serial
  Wire.begin(0x20); // Indica que funcionara como esclavo
  Wire.onReceive(receiveEvent); // Se asocia funcion Callback para recibir datos desde el maestro
  Wire.onRequest(requestEvent); // Se asocia funcion Callback para enviar datos
}

void loop() {
  delay(0.01);
}

void receiveEvent(int bytes)
{
  recieveData = 0;
  uint8_t index = 0;

 // Almacenar datos en recieveData mientras los envie el maestro
  while (Wire.available())
  {
    byte* pointer = (byte*)& recieveData;
    *(pointer + index) = (byte)Wire.read(); 
    index++; 
  }
  
  Pv = abs(map(recieveData, 0, 9535.91, 0, 100)); // Convertir valor de voltaje (prediccion) recibido por el maestro a porcentaje
  Serial.print((String) "  Input " + Pv); // Imprimir valor de porcentaje

  // Leer valor de potenciometro Set Point
  float setpoint = analogRead(A1); 
  Sp = map(setpoint, 0, 1023, 0, 100); // Convertir valor de potenciometro a porcentaje
  Serial.print((String) "  setponit " + Sp);

}

void requestEvent() {
  actual = millis(); // Medir tiempo desde que se inicializa funcion
  unsigned long dt = actual - pasado; // Calcular tiempo transcurrido desde la ultima vez que fue llamada 

  if (dt >= T) // El dt debe de ser mayor al tiempo de muestreo 
  {
    pasado = actual; // Guardar valor actual en pasado para proxima vez que sea llamado

    error = Sp - Pv; // Calculo de error
    Serial.print((String) "  error  " + error); // Imprimir error

    P = Kp * error; // Termino P

    I = I + (Kp / Ti) * error;  // Termino I

    error_1 = error; // Guardar error en tiempo pasado

    Cv = P + I; // Obtener variable de control

    Cv = constrain(Cv, 0, 255); // Restringir numero para que este dentro de rango de 0 a 255
  }
  Serial.println((String) "  Output " + Cv); // Imprimir valor variable de control

  sendData = (double)Cv; // Convertir valor variable de control a double para poder enviarlo 
  Wire.write((byte*)& sendData, sizeof(sendData)); // Enviar variable de control y tamaño
  
}
