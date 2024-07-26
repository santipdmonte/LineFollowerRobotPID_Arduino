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
int baseSpeed = 200;

float Kp = 0; // Inicializadas en cero por si son comentadas mas adelante tome este valor y no de error
float Kd = 0;
float Ki = 0;

int minValues[3], maxValues[3], threshold[3]; // Variables para calibrar los colores y establece el valor minimo y maximo

int ref; // Para la funcion leer_sensor


// === Setup ====================================================================================
void setup(){ 

  // Additional setup from your original code
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

  Serial.begin(9600); // -> Antes estaba solo esto
}


// === Loop ====================================================================================
void loop(){

  // while (digitalRead(11)) {} // Boton
  delay(2000);
  calibrate();
  // while (digitalRead(12)) {} // Boton
  delay(2000);


  // imprimir_valores_sensores();


  while (1){ // Este while es necesario si realizamos la calibracion
    
    if (leer_sensor(1) > threshold[1]){
      
      // MODIFICAR hasta encontrar valores optimos. Buscar Kp optimo, luego Kd y luego Ki
      /* Otra forma de hacerlo es establecer valores fijos en la declaracion de variables */
      Kp = 0.0008 * (1000 - leer_sensor(1)); // Si esta centrado, kp sera casi 0
      Kd = 10 * Kp;
      Ki = 0.0001;
      
      linefollow();
        
    }
  }
}

//  === LineFollowFunction =======================================================================================
void linefollow(){

  int error = ((leer_sensor(2)) - leer_sensor(0));
      
  // Calculo valores segun el errror
  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // Asigna velocidades a los motores | Se resta o suma dependiendo de cual restamos para el error
  lsp = baseSpeed - PIDvalue;
  rsp = baseSpeed + PIDvalue;

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
  analogWrite(AIB, lsp);
  analogWrite(BIA, rsp);

}

//  === CalibrateFunction =======================================================================================
void calibrate()
{

  for ( int i = 0; i < 3; i++)  // Inicializamos las variables 
  {
    Serial.println("Inicializando Variables"); // BORAR
    minValues[i] = analogRead(sensorPins[i]);
    maxValues[i] = analogRead(sensorPins[i]);
  }
  
  for (int i = 0; i < 300; i++)  // Gira 360 para guardar valores maximos y minimos de la superficie
  {
    Serial.println("GIRANDO"); // BORAR
    // Giro 360 
    analogWrite(AIB, 200); // Motor A hacia ADELANTE vel.50
    analogWrite(BIB, 200); // Motor B hacia ATRAS vel.50

    for ( int i = 0; i < 3; i++) 
    {
      if (leer_sensor(i) < minValues[i])
      {
        minValues[i] = leer_sensor(i);
      }
      if (leer_sensor(i) > maxValues[i])
      {
        maxValues[i] = leer_sensor(i);
      }
    }
  }

  for ( int i = 0; i < 3; i++) // Asigna el valor medio de cada sensor
  {
    Serial.println("Asignando Valores"); // BORAR
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  // Parar
  Serial.println("PARAR"); // BORAR
  analogWrite(AIA, 0);
  analogWrite(AIB, 0);
  analogWrite(BIA, 0);
  analogWrite(BIB, 0);
}

//  === Leer Sensor =======================================================================================
int leer_sensor(int i){
  digitalWrite(irSensor[i], HIGH);
  ref = analogRead(sensorPins[i]);
  digitalWrite(irSensor[i], LOW);

  return ref;
}

void imprimir_valores_sensores(){
    // Imprime minValues[0], threshold[0], maxValues[0] en una línea
  Serial.print(minValues[0]);
  Serial.print(" ");
  Serial.print(threshold[0]);
  Serial.print(" ");
  Serial.println(maxValues[0]);

  // Imprime minValues[1], threshold[1], maxValues[1] en una línea
  Serial.print(minValues[1]);
  Serial.print(" ");
  Serial.print(threshold[1]);
  Serial.print(" ");
  Serial.println(maxValues[1]);

  // Imprime minValues[2], threshold[2], maxValues[2] en una línea
  Serial.print(minValues[2]);
  Serial.print(" ");
  Serial.print(threshold[2]);
  Serial.print(" ");
  Serial.println(maxValues[2]);
}


