/* 

Robot Sigue Linea:
  - Espera 2 segundos
  - Inicializa con una calibracion, girando 360 para determinar valores maximos, minimos y punto medio
  - Espera 3 segundos, y comienza el loop
  - En base al error de los sensores externos establece los valores del PID 
  - En base a los valores del PID establece cuanto gira cada motor

Tener en cuenta:
  - Los sensores miden que tan negro o que tan blanco es la superficie (no son binarios). 
  - Cuanto mas blanco mayor el error

*/

#include <SparkFun_TB6612.h>

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

// === Configuracion del motor ==================================================================
// Alinearlo segun corresponda.  Puede ser 1 o -1
const int offsetA = 1;
const int offsetB = 1;

// === Inicializacion de motores ================================================================
// The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


// === Inicializacion de variables ==============================================================
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;

float Kp = 0; // Inicializadas en cero por si son comentadas mas adelante tome este valor y no de error
float Kd = 0;
float Ki = 0;

int minValues[6], maxValues[6], threshold[6]; // Variables para calibrar los colores y establece el valor minimo y maximo


// === Setup ====================================================================================
void setup(){ // VER QUE MODIFICAR DE ACA
  Serial.begin(9600); 
  // pinMode(11, INPUT_PULLUP);
  // pinMode(12, INPUT_PULLUP);
}


/*
analogRead(1) & analogRead(4) son nuestros sensores extremos
analogRead(3) es nuestro sensor central
MODIFICAR SENSORES *analogRead(n)*
*/ 
// === Loop ====================================================================================
void loop(){
  
  // while (digitalRead(11)) {} // Boton
  delay(2000);
  calibrate();
  // while (digitalRead(12)) {} // Boton
  delay(3000);

  while (1){ // Este while es necesario si realizamos la calibracion
    
    if (*analogRead(3)* > threshold[3]){
      // MODIFICAR hasta encontrar valores optimos. Buscar Kp optimo, luego Kd y luego Ki
      Kp = 0.0006 * (1000 - *analogRead(3)*);
      //Kd = 10 * Kp;
      //Ki = 0.0001;
      
      linefollow();
        
    }
  }
}

//  === LineFollowFunction =======================================================================================
void linefollow(){
  int error = (*analogRead(2)* - *analogRead(4)*);

  // Calculo valores segun el errror
  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // Asigna velocidades a los motores | Se resta o suma dependiendo de cual restamos para el error
  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  // Establece valores maximos y minimos en caso de ser necesario
  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }

  // Asignamos velocidad a los motores
  motor1.drive(lsp);
  motor2.drive(rsp);

}

//  === CalibrateFunction =======================================================================================
void calibrate()
{
  for ( int i = 1; i < 3; i++)  // Inicializamos las variables 
  {
    minValues[i] = *analogRead*(i);
    maxValues[i] = *analogRead*(i);
  }
  
  for (int i = 0; i < 3000; i++)  // Gira para guardar valores maximos y minimos
  {
    // Giro 360 
    motor1.drive(50);
    motor2.drive(-50);

    for ( int i = 1; i < 3; i++) 
    {
      if (*analogRead*(i) < minValues[i])
      {
        minValues[i] = *analogRead*(i);
      }
      if (*analogRead*(i) > maxValues[i])
      {
        maxValues[i] = *analogRead*(i);
      }
    }
  }

  for ( int i = 1; i < 3; i++) // Establece el valor medio de cada sensor
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  motor1.drive(0);
  motor2.drive(0);
}
