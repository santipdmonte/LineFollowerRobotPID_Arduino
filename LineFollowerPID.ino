/*

Robot Sigue Linea:
  - En base al error de los sensores externos establece los valores del PID 
  - En base a los valores del PID establece cuanto gira cada motor

Tener en cuenta:
  - Los sensores miden que tan negro o que tan blanco es la superficie (no son binarios). 
  - Cuanto mas blanco mayor el error

*/

// === Establecemos Velocidades
int baseSpeed = 200;
int maxSpeed = 255;
int minSpeed = 75;

// FineTuning del PID
float Kp2 = 0.0008; //Kp = Kp2 * (1000 - leer_sensor(1));
float Kd2 = 0.1; //Kd = Kd2 * Kp;
float Ki2 = 0.0001; // Ki = Ki2

// === Desde aca es todo codigo, no se deberia tener que modificar nada ==========================

// Pin definitions
  //Motores
const int AIA = 3;
const int AIB = 11;
const int BIA = 10;
const int BIB = 6;
  // Sensores
const int irIzquierda = 9;
const int irDerecha = 8;
const int irCentro = 7;
const int ledTest = 13;

// === Sensor pins ==================================================================
// A0 -> Sensor Derecha
// A1 -> Sensor Medio
// A2 -> Sensor Izquierda
const int sensorPins[3] = {A0, A1, A2};
const int irSensor[3] = {irDerecha, irCentro, irIzquierda};


// === Inicializacion de variables ==============================================================
int P, D, I = 0, previousError = 0, PIDvalue, error;
int lsp, rsp;

float Kp = 0; // Inicializadas en cero por si son comentadas mas adelante tome este valor y no de error
float Kd = 0;
float Ki = 0;

int ref; // Para la funcion leer_sensor


// === Setup ====================================================================================
void setup(){ 

  pinMode(13, OUTPUT);  // ledTest
  digitalWrite(13, LOW);

  // Inicializamos los sensores
  pinMode(irIzquierda, OUTPUT); 
  pinMode(irDerecha, OUTPUT);
  pinMode(irCentro, OUTPUT);
  
  // Inicializamos los motores
  pinMode(AIA, OUTPUT);
  pinMode(AIB, OUTPUT);
  pinMode(BIA, OUTPUT);
  pinMode(BIB, OUTPUT);

  // Apagamos los sensores 
  digitalWrite(irIzquierda, LOW);
  digitalWrite(irDerecha, LOW);
  digitalWrite(irCentro, LOW);

  // Iniciamos los motores como apagados (para evitar movimientos no deseados)
  digitalWrite(AIA, LOW);
  digitalWrite(AIB, LOW);
  digitalWrite(BIA, LOW);
  digitalWrite(BIB, LOW);
  
  // Initial blink
  digitalWrite(ledTest, LOW);
  delay(500);
  digitalWrite(ledTest, HIGH);
  delay(500);
  digitalWrite(ledTest, LOW);
  delay(500);

  Serial.begin(9600);
}


// === Loop ====================================================================================
void loop(){

  if (leer_sensor(1) > 450){ // Si el sensor del medio es mayor a 450(Negro)
    
    /* Otra forma de hacerlo es establecer valores fijos en la declaracion de variables */
    Kp = Kp2 * (1000 - leer_sensor(1)); // Si esta centrado, kp sera casi 0
    Kd = Kd2 * Kp;
    Ki = Ki2;
    
    linefollow();
  } 
  
}

//  === LineFollowFunction =======================================================================================
void linefollow(){

  int error = ((leer_sensor(2)) - leer_sensor(0));
      
  // Calculo valores segun el error
  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // Asigna velocidades a los motores | Se resta o suma dependiendo de cual restamos para el error
  lsp = baseSpeed - PIDvalue;
  rsp = baseSpeed + PIDvalue;

  // Establece valores maximos y minimos en caso de ser necesario
  if (lsp > maxSpeed) {
    lsp = maxSpeed;
  }
  if (lsp < minSpeed) {
    lsp = minSpeed;
  }
  if (rsp > maxSpeed) {
    rsp = maxSpeed;
  }
  if (rsp < minSpeed) {
    rsp = minSpeed;
  }

  // Asignamos velocidad a los motores
  analogWrite(AIB, lsp);
  analogWrite(BIA, rsp);

}

//  === Leer Sensor =======================================================================================
int leer_sensor(int i){
  digitalWrite(irSensor[i], HIGH);
  ref = analogRead(sensorPins[i]);
  digitalWrite(irSensor[i], LOW);

  return ref;
}


