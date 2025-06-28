// Ars Machina
// Microcontrolador: MegaPi

#include <MeMegaPi.h> // Librería del MegaPi
#include <Wire.h> // Para comunicación I2C (usado por el sensor de color)
#include "Adafruit_TCS34725.h" // Librería para el sensor de color TCS34725

// --- Definiciones de Pines y Constantes ---

// Pines ultrasonido
#define ULTRASONIDO_DERECHO_TRIG A12
#define ULTRASONIDO_DERECHO_ECHO A13
#define ULTRASONIDO_IZQUIERDO_TRIG A15
#define ULTRASONIDO_IZQUIERDO_ECHO A14
#define ULTRASONIDO_FRONTAL_TRIG A10
#define ULTRASONIDO_FRONTAL_ECHO A11

// Boton
#define BOTON_INICIO A7

//#define DISTANCIA_PARADA_FRONTAL 10 // Si la pared frontal está a 20 cm o menos, el robot se detiene

// Distancia para girar
#define DISTANCIA_UMBRAL_GIRO 72 

// Velocidad del motor cuando está en movimiento (RPM)
#define VELOCIDAD_ROBOT_RPM -115

#define TOLERANCIA_EQUIDISTANCIA_LATERAL 10 // Detectar si hay diferencia entre las mediciones ultrasonido

#define UMBRAL_ABERTURA_LATERAL 150  // umbral de deteccion para giros

// Ángulo de servo hacia adelante
#define ANGULO_SERVO_FRONTAL -10

// Ángulos de dirección para el servomotor (escala de servoCorregido: -90 a 90)
#define ANGULO_GIRO_IZQUIERDA -80   // Angulo direccion izquierda
#define ANGULO_GIRO_DERECHA 80   // Angulo direccion derecha


// Variables

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);

// Límites de color
int red_limit = 900;
int green_limit = 700;
int blue_limit = 800;

// Servomotor
Servo servomotor;

float Kp_servo = 2; // ajustable

// Variables para lecturas de distancia de los ultrasonidos
int distanciaDerecha = 0;
int distanciaIzquierda = 0;
int distanciaFrontal = 0;

float errorLateral;

int girando = 0;

// El SLOT cambia según donde tengan el driver conectado. PORT1 = SLOT1
MeEncoderOnBoard Encoder_1(SLOT1);

// Definición de los estados del robot
enum EstadoRobot {
  ESTADO_INICIAL,       // Robot esperando una señal para empezar a moverse
  ESTADO_MOVIENDO,      // Robot en movimiento, navegando por el pasillo
  ESTADO_CHOQUE_EMERGENCIA, // Robot muy cerca de la pared
  ESTADO_DECIDIR_GIRO, // Robot decide hacia donde girar
  ESTADO_CORRECCION_IZQUIERDA, // Robot corrige para girar
  ESTADO_CORRECCION_DERECHA, 
  ESTADO_GIRO_IZQUIERDA,// Robot gira
  ESTADO_GIRO_DERECHA,
  ESTADO_PARADO_FINAL,   // Robot ha detectado un obstáculo frontal y se ha detenido
  ESTADO_CORRECCION_IZQUIERDA_INTELIGENTE,
  ESTADO_RETROCESO_IZQUIERDO,
  ESTADO_CORRECCION_DERECHA_INTELIGENTE,
  ESTADO_RETROCESO_DERECHO
};

// Estado actual
EstadoRobot estadoActual = ESTADO_INICIAL;

// Variables para tiempo
unsigned long tiempoUltimaLecturaBoton = 0;
const long retardoDebounceBoton = 200; 
unsigned long tiempoInicioGiro = 0;
unsigned long tiempoChoqueEmergencia = 0;
unsigned long tiempoDeRonda = 0;

// Funcion interrupcion de encoder

void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}

void setup() {
  // Inicialización de interrupción para el encoder
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);

  // Inicialización de comunicación serial
  Serial.begin(115200); // Para el monitor serial
  Serial3.begin(9600); // Para comunicacion bluetooth

  // Configuración de temporizadores PWM a 8KHz para motores más suaves
  //Los motores van mas suave
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  servomotor.attach(A6);
  servoCorregido(ANGULO_SERVO_FRONTAL); // Servo hacia adelante

  // Configuración del encoder
  Encoder_1.setPulse(360);      // Resolución del encoder (360 pulsos por revolución)
  Encoder_1.setRatio(1);        // Relación de reducción (1 si el encoder mide la salida directa del motor)
  Encoder_1.setSpeedPid(10, 0, 0); // Constantes PID para el control de velocidad (Kp, Ki, Kd)

  // Configuración de los pines para los sensores ultrasónicos
  pinMode(ULTRASONIDO_DERECHO_TRIG, OUTPUT);
  pinMode(ULTRASONIDO_DERECHO_ECHO, INPUT);
  pinMode(ULTRASONIDO_IZQUIERDO_TRIG, OUTPUT);
  pinMode(ULTRASONIDO_IZQUIERDO_ECHO, INPUT);
  pinMode(ULTRASONIDO_FRONTAL_TRIG, OUTPUT);
  pinMode(ULTRASONIDO_FRONTAL_ECHO, INPUT);

  // Configuración del pin del botón de inicio
  pinMode(BOTON_INICIO, INPUT);

  // Inicialización del sensor de color TCS34725
  // Si esta parte falla, se detiene la ejecucion
  /*if (!tcs.begin()) {
    //Serial.println("Error TCS34725: Sensor no encontrado o problema de conexión I2C.");
    //Serial.println("REvisa conexiones I2C del MegaPi y la alimentación.");
    while (1) delay(1000); // Bucle infinito para detener el programa si el sensor no inicializa
  } else {
    //Serial.println("Sensor TCS34725 inicializado correctamente.");
  }
*/
  //Serial.println("Robot en ESTADO_INICIAL. Presiona el botón para comenzar la navegación.");
}

void loop() {
  leerSensores(); // Lee todos los sensores en cada ciclo de loop

  // Si el tiempo de la ronda se acaba, estado final
  /*if (tiempoDeRonda != 0 && millis - tiempoDeRonda >= 180000){
    estadoActual = ESTADO_PARADO_FINAL;
  }
  */
  // Máquina de estados
  switch (estadoActual) {
    case ESTADO_INICIAL:
      manejarEstadoInicial();
      break;
    case ESTADO_MOVIENDO:
      manejarEstadoMoviendo();
      break;
    case ESTADO_CHOQUE_EMERGENCIA:
      manejarChoqueEmergencia();
      break;
    case ESTADO_DECIDIR_GIRO:
      manejarEstadoDecidirGiro();
      break;
    case ESTADO_CORRECCION_IZQUIERDA:
      manejarEstadoCorreccionIzquierda();
      break;
    case ESTADO_CORRECCION_DERECHA:
      manejarEstadoCorreccionDerecha();
      break;
    case ESTADO_GIRO_IZQUIERDA:
      manejarEstadoGiroIzquierda();
      break;
    case ESTADO_GIRO_DERECHA:
      manejarEstadoGiroDerecha();
      break;
    case ESTADO_PARADO_FINAL:
      manejarEstadoParadoFinal();
      break;
    case ESTADO_CORRECCION_IZQUIERDA_INTELIGENTE:
      manejarCorreccionIzquierdaInteligente();
      break;
    case ESTADO_CORRECCION_DERECHA_INTELIGENTE:
      manejarCorreccionDerechaInteligente();
      break;
    case ESTADO_RETROCESO_IZQUIERDO:
      break;
    case ESTADO_RETROCESO_DERECHO:
      break;
  }

  // Actualiza la velocidad del motor PID
  Encoder_1.loop();

  // Imprimir informacion de sensores y estados
  imprimirInformacionDebug();

  delay(30); 
}

void servoCorregido(int anguloCorregido) {
  int angulo = map(anguloCorregido, -90, 90, 180, 0);

  if (angulo < 10) {
    angulo = 10;  // Si es menor que 20, lo ajusta a 20
  } else if (angulo > 170) {
    angulo = 170;  // Si es mayor que 160, lo ajusta a 160
  }

  servomotor.write(angulo);
  Serial.println(angulo);
}

int leerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Lee el tiempo de duración del pulso (con un timeout de 30ms)
  long duracion = pulseIn(echoPin, HIGH, 30000);

  if (duracion == 0) { // Si el pulso es 0, significa timeout o error.
    return 999; // Retorna un valor alto para indicar fuera de rango/no detección.
  }

  return duracion * 0.017; // Valor aproximado de la distancia en cm
}

void leerSensores() {
  distanciaIzquierda = leerDistancia(ULTRASONIDO_DERECHO_TRIG, ULTRASONIDO_DERECHO_ECHO);
  distanciaDerecha = leerDistancia(ULTRASONIDO_IZQUIERDO_TRIG, ULTRASONIDO_IZQUIERDO_ECHO);
  distanciaFrontal = leerDistancia(ULTRASONIDO_FRONTAL_TRIG, ULTRASONIDO_FRONTAL_ECHO);
}

void color_identification() {
  uint16_t r, g, b, c;
  // Obtiene los valores RGB y claro del sensor.
  tcs.getRawData(&r, &g, &b, &c);

  String paquete_color = String(r) + "," + String(g) + "," + String(b) + "," + String(c);
  Serial3.println("Color: " + paquete_color); // Imprime los valores del color
}

void manejarEstadoInicial() {
  Encoder_1.runSpeed(0); // Asegura que el motor esté parado
  servoCorregido(ANGULO_SERVO_FRONTAL); // Mantiene el servo en la posición inicial (hacia adelante)
  color_identification();

  // Verifica si el botón de inicio ha sido presionado 
  if (digitalRead(BOTON_INICIO) == LOW) {
    delay(100);
    //Serial.println("Botón presionado. Iniciando movimiento.");
    tiempoDeRonda = millis();
    estadoActual = ESTADO_MOVIENDO; // Cambia al estado de movimiento
  }
}

void manejarEstadoMoviendo() {
  //mueve el robot hacia adelante a velocidad constante
  Encoder_1.runSpeed(VELOCIDAD_ROBOT_RPM);

  // Calcula el error del robot para saber que tan al madio esta
  errorLateral = (float)distanciaDerecha - (float)distanciaIzquierda;

  // Término Proporcional (P)
  float p_term = Kp_servo * errorLateral;

  // Calcula correccion del angulo del servo
  float correccion_angulo_servo = ANGULO_SERVO_FRONTAL + p_term;

  // Aplica la corrección al servomotor
  servoCorregido((int)correccion_angulo_servo);

  if (distanciaFrontal < 40 && abs(errorLateral) > 100){ // chequear condiciones
  // Si la distancia frontal es pequeña y el error lateral es grande, gira
    estadoActual = ESTADO_DECIDIR_GIRO;
    //int absDiferenciaDistanciaLateral = abs(distanciaIzquierda - distanciaDerecha);
    /*if (absDiferenciaDistanciaLateral > 50) {
      //Encoder_1.runSpeed(0);
      estadoActual = ESTADO_DECIDIR_GIRO;
    }*/
  }
}

void manejarChoqueEmergencia(){
  Encoder_1.runSpeed(125);
  if (girando == 1){
    servoCorregido(80);
  }
  else {
  servoCorregido(-80);
  }
  if(millis() - tiempoInicioGiro >= 2000){
    estadoActual = ESTADO_MOVIENDO;
  }
}

void manejarEstadoDecidirGiro() {
  Encoder_1.runSpeed(0); // Asegura que el motor esté parado
  servoCorregido(ANGULO_SERVO_FRONTAL); // Asegura que la dirección esté recta para la decisión.

  // Logica para decidir el giro
  bool hayAperturaIzquierda = (distanciaIzquierda > UMBRAL_ABERTURA_LATERAL || distanciaIzquierda == 999);
  bool hayAperturaDerecha = (distanciaDerecha > UMBRAL_ABERTURA_LATERAL || distanciaDerecha == 999);

  String informacionUltrasonidos = "debug: DistI=" + String(distanciaIzquierda) + "cm, DistF=" + String(distanciaFrontal) + "cm, DistD=" + String(distanciaDerecha) + "cm";
  Serial3.println(informacionUltrasonidos);

  if (hayAperturaIzquierda && !hayAperturaDerecha) {
    Serial.println("Decidido: Girar a la Izquierda.");
    girando = 1;
    estadoActual = ESTADO_CORRECCION_IZQUIERDA_INTELIGENTE;
    //estadoActual = ESTADO_GIRO_IZQUIERDA;
    tiempoInicioGiro = millis(); // Guarda el tiempo para el giro temporizado
  } else if (hayAperturaDerecha && !hayAperturaIzquierda) {
    Serial.println("Decidido: Girar a la Derecha.");
    estadoActual = ESTADO_CORRECCION_DERECHA_INTELIGENTE;
    //estadoActual = ESTADO_GIRO_IZQUIERDA;
    tiempoInicioGiro = millis(); // Guarda el tiempo para el giro temporizado
  }
}

void manejarEstadoCorreccionIzquierda(){ //No se usa
  Encoder_1.runSpeed(125);
  servoCorregido(80);

  if (millis() - tiempoInicioGiro >= 6700){
    tiempoInicioGiro = millis();
    estadoActual = ESTADO_GIRO_IZQUIERDA;
  }
}

void manejarEstadoCorreccionDerecha(){ //No se usa
  Encoder_1.runSpeed(125);
  servoCorregido(-80);

  if (millis() - tiempoInicioGiro >= 6700){
    tiempoInicioGiro = millis();
    estadoActual = ESTADO_GIRO_DERECHA;
  }
}

void manejarEstadoGiroIzquierda() { //Giro
  servoCorregido(-90); // Dirección para girar a la izquierda
  Encoder_1.runSpeed(-150); // Mover el motor para ejecutar el giro
  if (distanciaFrontal < 35){
    estadoActual = ESTADO_CHOQUE_EMERGENCIA;
  }

  // Espera a que el tiempo de giro se cumpla 
  if (millis() - tiempoInicioGiro >= 4500) {
    //Serial.println("Giro a la Izquierda completado. Volviendo a ESTADO_MOVIENDO.");
    //Encoder_1.runSpeed(0); // Detiene el motor al finalizar el giro
    servoCorregido(ANGULO_SERVO_FRONTAL); // Vuelve la dirección a recta
    estadoActual = ESTADO_MOVIENDO; // Vuelve al estado de movimiento normal
  }
}

void manejarEstadoGiroDerecha() { //Giro
  servoCorregido(90); // Dirección para girar a la derecha
  Encoder_1.runSpeed(-150); // Mover el motor para ejecutar el giro
  if (distanciaFrontal < 35){
    estadoActual = ESTADO_CHOQUE_EMERGENCIA;
  }

  // Espera a que el tiempo de giro se cumpla
  if (millis() - tiempoInicioGiro >= 4500) {
    //Serial.println("Giro a la Derecha completado. Volviendo a ESTADO_MOVIENDO.");
    //Encoder_1.runSpeed(0); // Detiene el motor al finalizar el giro
    servoCorregido(ANGULO_SERVO_FRONTAL); // Vuelve la dirección a recta
    estadoActual = ESTADO_MOVIENDO; // Vuelve al estado de movimiento normal
  }
}

// Robot dormido
void manejarEstadoParadoFinal() {
  Encoder_1.runSpeed(0); // Asegura que el motor esté parado
  servomotor.write(ANGULO_SERVO_FRONTAL); // Mantiene el servo hacia adelante
  Serial.println("Robot detenido. (Pared frontal detectada)");
}

void manejarCorreccionIzquierdaInteligente() { //Retroceso
    Encoder_1.runSpeed(-VELOCIDAD_ROBOT_RPM);

  // Calcula el error del robot para saber que tan al madio esta
  float errorCorreccion = (float) 55 - (float) distanciaDerecha;

  // Término Proporcional (P)
  float p_term = Kp_servo * errorCorreccion;

  // Calcula correccion del angulo del servo
  float correccion_angulo_servo = ANGULO_SERVO_FRONTAL - p_term;

  // Aplica la corrección al servomotor
  servoCorregido((int)correccion_angulo_servo);

  if (millis() - tiempoInicioGiro >= 4800){
    tiempoInicioGiro = millis();
    unsigned long tiempoRetroceso = millis();
    estadoActual = ESTADO_GIRO_IZQUIERDA;
  }

  
  /if (distanciaFrontal < 70 /&& abs(errorCorreccion) > 100){ // chequear condiciones
  // Si la distancia frontal es pequeña y el error lateral es grande, gira
    estadoActual = ESTADO_PARADO_FINAL;
    //int absDiferenciaDistanciaLateral = abs(distanciaIzquierda - distanciaDerecha);
    if (absDiferenciaDistanciaLateral > 50) {
      //Encoder_1.runSpeed(0);
      estadoActual = ESTADO_DECIDIR_GIRO;
    }
  }*/
}

void manejarCorreccionDerechaInteligente() { //Retroceso
    Encoder_1.runSpeed(-VELOCIDAD_ROBOT_RPM);

  // Calcula el error del robot para saber que tan al madio esta
  float errorCorreccion = (float) 55 + (float) distanciaDerecha;

  // Término Proporcional (P)
  float p_term = Kp_servo * errorCorreccion;

  // Calcula correccion del angulo del servo
  float correccion_angulo_servo = ANGULO_SERVO_FRONTAL - p_term;

  // Aplica la corrección al servomotor
  servoCorregido((int)correccion_angulo_servo);

  if (millis() - tiempoInicioGiro >= 4800){
    tiempoInicioGiro = millis();
    unsigned long tiempoRetroceso = millis();
    estadoActual = ESTADO_GIRO_DERECHA;
  }
}

void imprimirInformacionDebug() {
  // Imprimir datos de los sensores y estado actual en Serial3 Bluetooth
  Serial3.print("Estado: ");
  switch (estadoActual) {
    case ESTADO_INICIAL: Serial3.print("INICIAL"); break;
    case ESTADO_MOVIENDO: Serial3.print("MOVIENDO"); break;
    case ESTADO_CHOQUE_EMERGENCIA: Serial3.print("CHOQUE EMERGENCIA"); break;
    case ESTADO_DECIDIR_GIRO: Serial3.print("DECIDIR GIRO"); break;
    case ESTADO_CORRECCION_IZQUIERDA: Serial3.print("CORRECION IZQUIERDA"); break;
    case ESTADO_CORRECCION_DERECHA: Serial3.print("CORRECION DERECHA"); break;
    case ESTADO_GIRO_DERECHA: Serial3.print("GIRO DERECHA"); break;
    case ESTADO_GIRO_IZQUIERDA: Serial3.print("GIRO IZQUIERDA"); break;
    case ESTADO_PARADO_FINAL: Serial3.print("PARADO_FINAL"); break;
    case ESTADO_CORRECCION_IZQUIERDA_INTELIGENTE: Serial3.print("CORRECCION INTELIGENTE IZQUIERDA"); break;
    case ESTADO_RETROCESO_IZQUIERDO: Serial3.print("RETROCESO IZQUIERDO"); break;
    case ESTADO_CORRECCION_DERECHA_INTELIGENTE: Serial3.print("CORRECCION INTELIGENTE DERECHA"); break;
    case ESTADO_RETROCESO_DERECHO: Serial3.print("RETROCESO IZQUIERDO"); break;
  }
  Serial3.println("");

  Serial3.print("Velocidad Motor: ");
  Serial3.print(Encoder_1.getCurrentSpeed());
  Serial3.println(" RPM");

  String informacionUltrasonidos = "DistI=" + String(distanciaIzquierda) + "cm, DistF=" + String(distanciaFrontal) + "cm, DistD=" + String(distanciaDerecha) + "cm";
  Serial3.println(informacionUltrasonidos);

  Serial3.print("Error lateral: "); Serial3.println(errorLateral);

  Serial3.print("tiempo: "); Serial3.println(millis()/1000);


  color_identification(); 

  Serial3.println("-------------");
}