# Implementación de un sistema motor-generador aplicado a arquitectura “hardware in the loop” con “Machine Learning”

## Contribucion
Se desarrollo un articulo basado en el tema, el cual fue aceptado en IEEE 5th Colombian Conference on Automatic Control (CCAC). 19 al 22 de octubre del 2021. 

## Planteamiento del problema
Realizar el modelamiento de un sistema motor-generador por medio de redes neuronales artificiales del tipo perceptron multicapa, para ser implementado en un sistema embebido de bajo costo, el cual permita ser utilizado como dispositivo en una arquitectura hardware in the loop.

## Introduccion
El esquema hardware in the loop presenta varias ventajas, una de ellas radica en que el sistema embebido es operado en tiempo real, y no requiere de la manipulación directa sobre el sistema por parte de un operario, ya que cuenta con una plataforma o medio digital para su manipulación, lo cual resulta también en una reducción de costos. 
Por otra parte, la inteligencia artificial aplicada a redes neuronales, es importante ya que es capaz de predecir variables, a partir de un conjunto de datos tomados previamente del sistema. Otra importancia es que son tolerantes a fallos o cambios en las entradas, sin alterar drásticamente la veracidad de su predicción. Además, este sistema puede operar en tiempo real, por lo cual es apropiado para implementarlo con un sistema hardware in the loop.

## Explicacion de codigos

Cada uno de los codigos viene comentado para mejor interpretacion del usuario.

* Sensor_virtual_prediccion_corriente_generador_Pololu_entrenamiento_ANN.ipynb : Archivo el cual contiene el entrenamiento de la ANN implementada en el sensor virual para la prediccion de corriente del generador Pololu. este archivo esta importado desde la herramienta Google Colab.
* Sensor_virtual_prediccion_corriente_generador_pololu_sistema_embebido.ino : Archivo en el cual se implementa el sensor virtual en el sistema embebido, programado en Arduino Mega e importado desde el IDE de Arduino. 
* HIL_entrenamiento_ANN.ipynb : Archivo el cual contiene el entrenamiento de la ANN implementada en el HIL. este archivo esta importado desde la herramienta Google Colab.
* PID_ANN_Arduino_UNO.ino : Archivo el cual contiene el PID de la red neuronal, programado en Arduino UNO e importado desde el IDE del Arduino.
* PID_MOTOR_Y_ANN_HIL_Arduino_Mega.ino : Archivo el cual contiene la ANN del HIL y el PID del sistema fisico (estan programadas en la misma tarjeta pero funcionan por aparte), programado en Arduino Mega e importado desde el IDE de Arduino.
