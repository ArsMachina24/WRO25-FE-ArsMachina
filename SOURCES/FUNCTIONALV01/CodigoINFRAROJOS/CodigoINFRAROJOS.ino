// Ars Machina - Modificado para WRO "Futuros Ingenieros" - Rondas Abiertas
// Microcontrolador: MegaPi

#include <MeMegaPi.h>        // Librería del MegaPi
#include <Wire.h>            // Para comunicación I2C (usado por el sensor de color)
#include "Adafruit_TCS34725.h" // Librería para el sensor de color TCS34725

// --- Definiciones de Pines y Constantes ---

// Pines ultrasonido
#define ULTRASONIDO_DERECHO_TRIG A12
#define ULTRASONIDO_DERECHO_ECHO A13
#define ULTRASONIDO_IZQUIERDO_TRIG A15
#define ULTRASONIDO_IZQUIERDO_ECHO A14
#define ULTRASONIDO_FRONTAL_TRIG A10
#define ULTRASONIDO_FRONTAL_ECHO A11

// Pines de los sensores infrarrojos (NUEVOS)
#define IR_FRONTAL_DERECHO 5
#define IR_FRONTAL_IZQUIERDO 7
#define IR_TRASERO_IZQUIERDO 6
#define IR_TRASERO_DERECHO 4

// Definición para el estado de detección de los sensores IR
// Asumimos que el sensor devuelve LOW cuando detecta un obstáculo.
// Si tus sensores devuelven HIGH al detectar, cambia esto a HIGH.
#define IR_DETECTADO LOW

// Boton de Inicio
#define BOTON_INICIO A7

// Velocidad del motor (RPM) - AJUSTADO: El valor base ahora es POSITIVO
// Este es el valor de magnitud que usarás. El signo se aplica en las funciones de movimiento.
// Si "adelante" es con RPM negativos, aquí pones la magnitud, ej., 150.
#define VELOCIDAD_BASE_RPM 150

// Constantes PID para el control del servo (corrección de dirección)
// Kp: Ganancia proporcional (ajusta la respuesta a la diferencia lateral)
// Kd: Ganancia derivativa (ayuda a amortiguar las oscilaciones y mejorar la estabilidad)
#define KP_SERVO 0.8  // Aumenta si necesitas más corrección, disminuye si oscila
#define KD_SERVO 0.05 // Añadido para suavizar las oscilaciones y mejorar la estabilidad

// Umbrales de detección y giro
#define DISTANCIA_UMBRAL_GIRO_FRONTA 30 // Distancia frontal para iniciar la decisión de giro
#define UMBRAL_ABERTURA_LATERAL    100 // Distancia mínima para considerar una "abertura" lateral
#define TOLERANCIA_CENTRAL         15  // Tolerancia para considerarse "centrado" entre paredes

// Ángulo de servo "recto" o hacia adelante
#define ANGULO_SERVO_FRONTAL 18 // 0 grados = recto

// Ángulos de dirección para el servomotor para giros específicos (escala de servoCorregido: -90 a 90)
#define ANGULO_GIRO_IZQUIERDA_SUAVE -45 // Giro suave a la izquierda
#define ANGULO_GIRO_DERECHA_SUAVE   45  // Giro suave a la derecha
#define ANGULO_GIRO_IZQUIERDA_PRONUNCIADO -80 // Giro más pronunciado a la izquierda
#define ANGULO_GIRO_DERECHA_PRONUNCIADO   80  // Giro más pronunciado a la derecha

// Constantes para el sensor de color TCS34725
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X); // Ajustado para una respuesta más rápida

// Límites de color para la detección de azul (AJUSTAR SEGÚN CALIBRACIÓN EN AMBIENTE DE PRUEBA)
int red_limit_color = 300;     // Valor máximo para el componente rojo cuando es azul
int green_limit_color = 300;   // Valor máximo para el componente verde cuando es azul
int blue_detection_threshold = 1000; // Valor mínimo para el componente azul cuando es azul

// --- Variables Globales ---

Servo servomotor;

// Variables para lecturas de distancia de los ultrasonidos
int distanciaDerecha = 0;
int distanciaIzquierda = 0;
int distanciaFrontal = 0;

// Variables para el estado de los sensores infrarrojos (NUEVAS)
bool irFrontalDerechoDetectado = false;
bool irFrontalIzquierdoDetectado = false;
bool irTraseroIzquierdoDetectado = false;
bool irTraseroDerechoDetectado = false;

float errorLateral = 0;        // Error de posicionamiento lateral (Derecha - Izquierda)
float errorLateralAnterior = 0; // Para el término derivativo del control del servo

// Flag para indicar si se detectó una línea azul
bool detectoLineaAzul = false;

// El SLOT cambia según donde tengan el driver conectado. PORT1 = SLOT1
MeEncoderOnBoard Encoder_1(SLOT1); // Asume un solo motor para simplificar la dirección

// Definición de los estados del robot
enum EstadoRobot {
  ESTADO_INICIAL,            // Robot esperando una señal para empezar a moverse
  ESTADO_NAVEGANDO,          // Robot en movimiento, manteniendo el centro del pasillo
  ESTADO_DECIDIR_GIRO,       // Robot detecta obstáculo o línea y decide la dirección
  ESTADO_REALIZANDO_GIRO_IZQ, // Robot gira a la izquierda
  ESTADO_REALIZANDO_GIRO_DER, // Robot gira a la derecha
  ESTADO_CORRECCION_IR,      // NUEVO: Estado para manejar correcciones por sensores IR
  ESTADO_PARADO_FINAL        // Robot ha completado su tarea o no puede avanzar
};

// Estado actual
EstadoRobot estadoActual = ESTADO_INICIAL;

// Variables de tiempo para control de duración de giros (ajusta según tus necesidades)
unsigned long tiempoInicioGiro = 0;
const unsigned long DURACION_GIRO_NORMAL = 1500; // Duración de giro si no es por línea azul
const unsigned long DURACION_GIRO_LINEA_AZUL = 2500; // Duración de giro por línea azul

// Variables para el nuevo estado de corrección IR (NUEVAS)
unsigned long tiempoInicioCorreccionIR = 0;
const unsigned long DURACION_RETROCESO = 500; // Duración del retroceso en ms
const unsigned long DURACION_GIRO_CORRECCION_IR = 700; // Duración del giro de corrección en ms
// 'necesitaRetroceder' ya no es un flag, el retroceso siempre ocurre al entrar en CORRECCION_IR


// Variable para el debounce del botón de inicio
unsigned long tiempoUltimoBotonPresionado = 0;
const unsigned long DEBOUNCE_TIME = 50; // Tiempo en milisegundos para debounce

// --- Funciones de Utilidad ---

// Interrupción del encoder para lectura de velocidad
void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}

// Controla el ángulo del servo mapeando de -90 a 90 a 0 a 180
void servoCorregido(int anguloCorregido) {
  // Mapea el ángulo de -90 a 90 (nuestra escala) a 0 a 180 (escala del servo)
  int angulo = map(anguloCorregido, -90, 90, 0, 180);

  // Asegura que el ángulo esté dentro de los límites seguros para el servo
  if (angulo < 10) { // Límite inferior para evitar que el servo golpee
    angulo = 10;
  } else if (angulo > 170) { // Límite superior para evitar que el servo golpee
    angulo = 170;
  }
  servomotor.write(angulo);
}

// Función para leer la distancia de un sensor ultrasónico
int leerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH, 30000); // Timeout de 30ms

  if (duracion == 0) {
    return 999; // Valor alto para indicar fuera de rango o error
  }
  return duracion * 0.017; // Distancia en cm
}

// Lee todos los sensores de distancia ultrasónicos
void leerSensoresUltrasonido() {
  distanciaIzquierda = leerDistancia(ULTRASONIDO_IZQUIERDO_TRIG, ULTRASONIDO_IZQUIERDO_ECHO);
  distanciaDerecha = leerDistancia(ULTRASONIDO_DERECHO_TRIG, ULTRASONIDO_DERECHO_ECHO);
  distanciaFrontal = leerDistancia(ULTRASONIDO_FRONTAL_TRIG, ULTRASONIDO_FRONTAL_ECHO);
}

// NUEVA FUNCIÓN: Lee el estado de los 4 sensores infrarrojos
void leerSensoresIR() {
  irFrontalDerechoDetectado = (digitalRead(IR_FRONTAL_DERECHO) == IR_DETECTADO);
  irFrontalIzquierdoDetectado = (digitalRead(IR_FRONTAL_IZQUIERDO) == IR_DETECTADO);
  irTraseroIzquierdoDetectado = (digitalRead(IR_TRASERO_IZQUIERDO) == IR_DETECTADO);
  irTraseroDerechoDetectado = (digitalRead(IR_TRASERO_DERECHO) == IR_DETECTADO);
}

// Detecta si el color bajo el sensor es azul
bool detectarColorAzul() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Considera que es azul si 'b' es alto y 'r' y 'g' son relativamente bajos
  if (b > blue_detection_threshold && r < red_limit_color && g < green_limit_color) {
    Serial.print("Color detectado: AZUL (R:"); Serial.print(r); Serial.print(", G:"); Serial.print(g); Serial.print(", B:"); Serial.println(b);
    return true;
  }
  return false;
}

// Muestra los valores RGB del sensor de color para depuración
void color_identification() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  String paquete_color = String(r) + "," + String(g) + "," + String(b) + "," + String(c);
  Serial3.println("Color: " + paquete_color);
}

// Imprime información de depuración por Serial3 (Bluetooth)
void imprimirInformacionDebug() {
  Serial3.print("Estado: ");
  switch (estadoActual) {
    case ESTADO_INICIAL: Serial3.print("INICIAL"); break;
    case ESTADO_NAVEGANDO: Serial3.print("NAVEGANDO"); break;
    case ESTADO_DECIDIR_GIRO: Serial3.print("DECIDIR GIRO"); break;
    case ESTADO_REALIZANDO_GIRO_DER: Serial3.print("GIRO DERECHA"); break;
    case ESTADO_REALIZANDO_GIRO_IZQ: Serial3.print("GIRO IZQUIERDA"); break;
    case ESTADO_CORRECCION_IR: Serial3.print("CORRECCION IR"); break; // NUEVO
    case ESTADO_PARADO_FINAL: Serial3.print("PARADO_FINAL"); break;
  }
  Serial3.println("");

  Serial3.print("Velocidad Motor: ");
  Serial3.print(Encoder_1.getCurrentSpeed());
  Serial3.println(" RPM");

  String infoDistancias = "DistI=" + String(distanciaIzquierda) + "cm, DistF=" + String(distanciaFrontal) + "cm, DistD=" + String(distanciaDerecha) + "cm";
  Serial3.println(infoDistancias);

  // Información de los sensores IR (NUEVO)
  Serial3.print("IR FD:"); Serial3.print(irFrontalDerechoDetectado ? "DETECTADO" : "LIBRE");
  Serial3.print(", FI:"); Serial3.print(irFrontalIzquierdoDetectado ? "DETECTADO" : "LIBRE");
  Serial3.print(", TI:"); Serial3.print(irTraseroIzquierdoDetectado ? "DETECTADO" : "LIBRE");
  Serial3.print(", TD:"); Serial3.println(irTraseroDerechoDetectado ? "DETECTADO" : "LIBRE");


  Serial3.print("Error lateral: "); Serial3.println(errorLateral);

  color_identification();
  Serial3.println("-------------");
}

// --- Máquina de Estados ---

void manejarEstadoInicial() {
  Encoder_1.runSpeed(0); // Asegura que el motor esté parado
  servoCorregido(ANGULO_SERVO_FRONTAL); // Mantiene el servo en la posición inicial
  Serial.println("Esperando boton de inicio...");
  imprimirInformacionDebug(); // Muestra el estado inicial

  // Lee el estado actual del botón
  int estadoBoton = digitalRead(BOTON_INICIO);

  // Si el botón está presionado (LOW, asumiendo pull-up o cableado a GND al presionar)
  if (estadoBoton == LOW) {
    // Verifica si ha pasado suficiente tiempo desde la última lectura válida
    if (millis() - tiempoUltimoBotonPresionado > DEBOUNCE_TIME) {
      Serial.println("Boton presionado. Iniciando navegacion.");
      estadoActual = ESTADO_NAVEGANDO; // Transición al estado de movimiento
    }
  } else {
    // Si el botón no está presionado, actualiza el tiempo para el próximo chequeo
    tiempoUltimoBotonPresionado = millis();
  }
}

void manejarEstadoNavegando() {
  // AJUSTADO: Se usa -VELOCIDAD_BASE_RPM para que el robot avance
  Encoder_1.runSpeed(-VELOCIDAD_BASE_RPM);

  // Calcula el error de posicionamiento lateral (queremos que errorLateral sea 0)
  // Un valor positivo significa que la pared derecha está más lejos que la izquierda
  // Un valor negativo significa que la pared izquierda está más lejos que la derecha
  errorLateral = (float)distanciaDerecha - (float)distanciaIzquierda;

  // Cálculo del término derivativo
  float d_term = KD_SERVO * (errorLateral - errorLateralAnterior);

  // Cálculo del término proporcional
  float p_term = KP_SERVO * errorLateral;

  // Calcula la corrección del ángulo del servo con control PD
  // Sumamos p_term + d_term al ángulo frontal. Si errorLateral es positivo (más cerca de la izquierda),
  // el robot girará hacia la derecha (ángulo positivo) para alejarse de la izquierda.
  // Si errorLateral es negativo (más cerca de la derecha), girará a la izquierda (ángulo negativo).
  float correccion_angulo_servo = ANGULO_SERVO_FRONTAL + p_term + d_term;

  // Aplica la corrección al servomotor
  servoCorregido((int)correccion_angulo_servo);

  // Guarda el error actual para la siguiente iteración derivativa
  errorLateralAnterior = errorLateral;

  // --- Lógica de Transición de Estados ---

  // Prioridad 1: Detección de línea azul (indica giro o fin de sección)
  if (detectarColorAzul()) {
    Serial.println("Linea azul detectada. Decidiendo giro...");
    detectoLineaAzul = true;
    estadoActual = ESTADO_DECIDIR_GIRO;
    return;
  }

  // Prioridad 2: Detección de sensores infrarrojos (colisión inminente)
  // Si CUALQUIER sensor IR detecta, se inicia el estado de corrección con retroceso.
  if (irFrontalDerechoDetectado || irFrontalIzquierdoDetectado || irTraseroDerechoDetectado || irTraseroIzquierdoDetectado) {
    Serial.println("Sensor IR detectado. Iniciando correccion con retroceso.");
    estadoActual = ESTADO_CORRECCION_IR;
    tiempoInicioCorreccionIR = millis(); // Inicia el temporizador para la corrección
    return;
  }

  // Prioridad 3: Detección de obstáculo frontal por ultrasonido (requiere giro)
  if (distanciaFrontal < DISTANCIA_UMBRAL_GIRO_FRONTA) {
    Serial.println("Obstaculo frontal detectado por ultrasonido. Decidiendo giro...");
    detectoLineaAzul = false; // Resetear por si venía de un giro por línea azul no procesado
    estadoActual = ESTADO_DECIDIR_GIRO;
    return;
  }
}

// NUEVA FUNCIÓN: Maneja el estado de corrección por sensores infrarrojos
void manejarEstadoCorreccionIR() {
  // Paso 1: Retroceso
  if (millis() - tiempoInicioCorreccionIR < DURACION_RETROCESO) {
    Encoder_1.runSpeed(VELOCIDAD_BASE_RPM); // Velocidad positiva para retroceder
    servoCorregido(ANGULO_SERVO_FRONTAL); // Mantener el servo recto durante el retroceso
    Serial.println("Retrocediendo por deteccion IR.");
    return; // Permanece en este estado y sigue retrocediendo
  }

  // Paso 2: Giro de corrección después del retroceso
  // Reinicia el temporizador para el giro de corrección si el retroceso acaba de terminar
  if (millis() - tiempoInicioCorreccionIR == DURACION_RETROCESO) { // Solo la primera vez después del retroceso
      tiempoInicioCorreccionIR = millis(); // Reinicia para el temporizador del giro
  }

  if (millis() - tiempoInicioCorreccionIR < DURACION_GIRO_CORRECCION_IR) {
    // Determina la dirección de giro basada en qué sensor IR se activó
    // Prioriza girar lejos del obstáculo.
    // Si ambos frontales detectan, o si no hay una dirección clara, gira a la derecha por defecto.
    if (irFrontalDerechoDetectado || irTraseroDerechoDetectado) {
      Serial.println("Corrigiendo: Girando a la izquierda por IR derecho.");
      Encoder_1.runSpeed(-VELOCIDAD_BASE_RPM * 0.5); // Avanza lento mientras gira
      servoCorregido(ANGULO_GIRO_IZQUIERDA_PRONUNCIADO); // Giro más pronunciado
    } else if (irFrontalIzquierdoDetectado || irTraseroIzquierdoDetectado) {
      Serial.println("Corrigiendo: Girando a la derecha por IR izquierdo.");
      Encoder_1.runSpeed(-VELOCIDAD_BASE_RPM * 0.5); // Avanza lento mientras gira
      servoCorregido(ANGULO_GIRO_DERECHA_PRONUNCIADO); // Giro más pronunciado
    } else {
      // Si por alguna razón no se sabe cuál IR lo activó (ej. detección muy breve),
      // o si ambos frontales detectaron y ya retrocedió, se usa lógica de ultrasonido
      // o un giro por defecto si no hay ultrasonidos.
      // Para simplificar, si no hay una dirección clara de IR, gira a la derecha.
      Serial.println("Corrigiendo: Giro por defecto (derecha) despues de retroceso IR.");
      Encoder_1.runSpeed(-VELOCIDAD_BASE_RPM * 0.5);
      servoCorregido(ANGULO_GIRO_DERECHA_PRONUNCIADO);
    }
  } else {
    Serial.println("Correccion IR completada. Volviendo a navegacion.");
    estadoActual = ESTADO_NAVEGANDO; // Vuelve al estado de navegación normal
    // Resetear los flags de detección IR para evitar re-entrada inmediata
    irFrontalDerechoDetectado = false;
    irFrontalIzquierdoDetectado = false;
    irTraseroIzquierdoDetectado = false;
    irTraseroDerechoDetectado = false;
  }
}


void manejarEstadoDecidirGiro() {
  Encoder_1.runSpeed(0); // Detiene el robot para tomar una decisión clara
  servoCorregido(ANGULO_SERVO_FRONTAL); // Pone el servo recto

  bool hayAperturaIzquierda = (distanciaIzquierda > UMBRAL_ABERTURA_LATERAL || distanciaIzquierda == 999);
  bool hayAperturaDerecha = (distanciaDerecha > UMBRAL_ABERTURA_LATERAL || distanciaDerecha == 999);

  // Si detectó línea azul, el giro es más pronunciado y temporizado
  if (detectoLineaAzul) {
    Serial.println("Decidiendo giro por linea azul...");
    if (hayAperturaIzquierda) { // Prioriza la izquierda si hay espacio
      Serial.println("Giro por linea azul: IZQUIERDA.");
      estadoActual = ESTADO_REALIZANDO_GIRO_IZQ;
    } else if (hayAperturaDerecha) {
      Serial.println("Giro por linea azul: DERECHA.");
      estadoActual = ESTADO_REALIZANDO_GIRO_DER;
    } else {
      // Si la línea azul indica un final y no hay aperturas claras, se detiene
      Serial.println("Linea azul detectada pero sin aperturas claras. Deteniendose.");
      estadoActual = ESTADO_PARADO_FINAL;
    }
    tiempoInicioGiro = millis(); // Inicia temporizador para el giro
    return;
  }

  // Lógica para giros por obstáculo (sin línea azul)
  Serial.println("Decidiendo giro por obstaculo...");
  if (hayAperturaIzquierda && !hayAperturaDerecha) {
    Serial.println("Decidido: Girar a la Izquierda.");
    estadoActual = ESTADO_REALIZANDO_GIRO_IZQ;
  } else if (hayAperturaDerecha && !hayAperturaIzquierda) {
    Serial.println("Decidido: Girar a la Derecha.");
    estadoActual = ESTADO_REALIZANDO_GIRO_DER;
  } else if (hayAperturaIzquierda && hayAperturaDerecha) {
    // Si ambos lados están abiertos, prioriza el lado con más espacio
    if (distanciaIzquierda > distanciaDerecha) {
      Serial.println("Decidido: Ambos lados abiertos, girar a la Izquierda (mas espacio).");
      estadoActual = ESTADO_REALIZANDO_GIRO_IZQ;
    } else {
      Serial.println("Decidido: Ambos lados abiertos, girar a la Derecha (mas espacio).");
      estadoActual = ESTADO_REALIZANDO_GIRO_DER;
    }
  } else {
    // Sin aperturas claras (callejón sin salida o esquina cerrada), se detiene
    Serial.println("Decidido: Sin apertura clara. Deteniendose.");
    estadoActual = ESTADO_PARADO_FINAL;
  }
  tiempoInicioGiro = millis(); // Inicia temporizador para el giro
}

void manejarEstadoRealizandoGiroIzq() {
  // Ajusta la velocidad y el ángulo según si el giro es por línea azul o por obstáculo
  if (detectoLineaAzul) {
    // AJUSTADO: Se usa -VELOCIDAD_BASE_RPM para que el robot avance
    Encoder_1.runSpeed(-VELOCIDAD_BASE_RPM * 0.7); // Reduce velocidad para giro pronunciado
    servoCorregido(ANGULO_GIRO_IZQUIERDA_PRONUNCIADO);
    if (millis() - tiempoInicioGiro >= DURACION_GIRO_LINEA_AZUL) {
      Serial.println("Giro Izquierda por linea azul completado.");
      detectoLineaAzul = false; // Resetear el flag
      estadoActual = ESTADO_NAVEGANDO;
    }
  } else {
    // AJUSTADO: Se usa -VELOCIDAD_BASE_RPM para que el robot avance
    Encoder_1.runSpeed(-VELOCIDAD_BASE_RPM); // Velocidad normal para giro más suave
    servoCorregido(ANGULO_GIRO_IZQUIERDA_SUAVE);
    if (millis() - tiempoInicioGiro >= DURACION_GIRO_NORMAL) {
      Serial.println("Giro Izquierda normal completado.");
      estadoActual = ESTADO_NAVEGANDO;
    }
  }
}

void manejarEstadoRealizandoGiroDer() {
  // Ajusta la velocidad y el ángulo según si el giro es por línea azul o por obstáculo
  if (detectoLineaAzul) {
    // AJUSTADO: Se usa -VELOCIDAD_BASE_RPM para que el robot avance
    Encoder_1.runSpeed(-VELOCIDAD_BASE_RPM * 0.7); // Reduce velocidad para giro pronunciado
    servoCorregido(ANGULO_GIRO_DERECHA_PRONUNCIADO);
    if (millis() - tiempoInicioGiro >= DURACION_GIRO_LINEA_AZUL) {
      Serial.println("Giro Derecha por linea azul completado.");
      detectoLineaAzul = false; // Resetear el flag
      estadoActual = ESTADO_NAVEGANDO;
    }
  } else {
    // AJUSTADO: Se usa -VELOCIDAD_BASE_RPM para que el robot avance
    Encoder_1.runSpeed(-VELOCIDAD_BASE_RPM); // Velocidad normal para giro más suave
    servoCorregido(ANGULO_GIRO_DERECHA_SUAVE);
    if (millis() - tiempoInicioGiro >= DURACION_GIRO_NORMAL) {
      Serial.println("Giro Derecha normal completado.");
      estadoActual = ESTADO_NAVEGANDO;
    }
  }
}

void manejarEstadoParadoFinal() {
  Encoder_1.runSpeed(0); // Detiene el motor
  servoCorregido(ANGULO_SERVO_FRONTAL); // Servo recto
  Serial.println("Robot detenido. Fin de la ruta o sin salida.");
  // Podrías añadir lógica aquí para indicar el fin de la competencia (LED, sonido, etc.)
}

// --- Configuración Inicial (setup) ---
void setup() {
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  Serial.begin(115200);    // Para depuración por cable USB
  Serial3.begin(9600);     // Para comunicación Bluetooth

  TCCR1A = _BV(WGM10); // Configuración para PWM de motor más suave
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  servomotor.attach(A6);
  servoCorregido(ANGULO_SERVO_FRONTAL);

  Encoder_1.setPulse(360);       // Pulsos por revolución del encoder
  Encoder_1.setRatio(1);         // Relación de reducción
  // Los PID del encoder son cruciales para mantener la velocidad
  // Afina estos valores si el robot no mantiene una velocidad constante
  Encoder_1.setSpeedPid(10, 0, 0); // Kp, Ki, Kd para la velocidad del motor

  // Configuración de pines para ultrasonidos
  pinMode(ULTRASONIDO_DERECHO_TRIG, OUTPUT);
  pinMode(ULTRASONIDO_DERECHO_ECHO, INPUT);
  pinMode(ULTRASONIDO_IZQUIERDO_TRIG, OUTPUT);
  pinMode(ULTRASONIDO_IZQUIERDO_ECHO, INPUT);
  pinMode(ULTRASONIDO_FRONTAL_TRIG, OUTPUT);
  pinMode(ULTRASONIDO_FRONTAL_ECHO, INPUT);

  // Configuración de pines para sensores infrarrojos (NUEVO)
  pinMode(IR_FRONTAL_DERECHO, INPUT);
  pinMode(IR_FRONTAL_IZQUIERDO, INPUT);
  pinMode(IR_TRASERO_IZQUIERDO, INPUT);
  pinMode(IR_TRASERO_DERECHO, INPUT);


  // Configuración de pin del botón de inicio
  pinMode(BOTON_INICIO, INPUT); // Asegúrate de usar un resistor pull-up si no tienes uno externo

  // Inicialización del sensor de color
  if (!tcs.begin()) {
    Serial.println("Error TCS34725: Sensor no encontrado o problema de conexion I2C.");
    while (1) delay(1000); // Detiene el programa si el sensor no inicializa
  } else {
    Serial.println("Sensor TCS34725 inicializado correctamente.");
  }
}

// --- Bucle Principal (loop) ---
void loop() {
  leerSensoresUltrasonido(); // Siempre lee los sensores ultrasónicos
  leerSensoresIR();         // NUEVO: Siempre lee los sensores infrarrojos

  // La máquina de estados principal
  switch (estadoActual) {
    case ESTADO_INICIAL:
      manejarEstadoInicial();
      break;
    case ESTADO_NAVEGANDO:
      manejarEstadoNavegando();
      break;
    case ESTADO_DECIDIR_GIRO:
      manejarEstadoDecidirGiro();
      break;
    case ESTADO_REALIZANDO_GIRO_IZQ:
      manejarEstadoRealizandoGiroIzq();
      break;
    case ESTADO_REALIZANDO_GIRO_DER:
      manejarEstadoRealizandoGiroDer();
      break;
    case ESTADO_CORRECCION_IR: // NUEVO: Maneja el estado de corrección por IR
      manejarEstadoCorreccionIR();
      break;
    case ESTADO_PARADO_FINAL:
      manejarEstadoParadoFinal();
      break;
  }

  Encoder_1.loop(); // Actualiza el control PID del motor
  imprimirInformacionDebug(); // Envía información por Bluetooth para depuración

  delay(20); // Pequeña pausa para estabilización y lectura (ajusta si es necesario)
}
