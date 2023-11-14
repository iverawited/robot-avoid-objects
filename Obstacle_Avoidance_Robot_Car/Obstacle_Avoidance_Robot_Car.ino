#include <Servo.h>

Servo myservo;               // Inicialización de un objeto de servo motor llamado 'myservo'
int Echo_Pin = A0;           // Pin ECHO del módulo ultrasónico
int Trig_Pin = A1;           // Pin TRIG del módulo ultrasónico
#define Lpwm_pin 5            // Pin de control de velocidad ENA de la placa del controlador de motor
#define Rpwm_pin 6            // Pin de control de velocidad ENB de la placa del controlador de motor
int pinLB = 2;               // Pin de control de giro IN1 de la placa del controlador de motor
int pinLF = 4;               // Pin de control de giro IN2 de la placa del controlador de motor
int pinRB = 7;               // Pin de control de giro IN3 de la placa del controlador de motor
int pinRF = 8;               // Pin de control de giro IN4 de la placa del controlador de motor

volatile int Front_Distance;  // Variable para almacenar la distancia frontal medida por el sensor ultrasónico
volatile int Left_Distance;   // Variable para almacenar la distancia izquierda medida por el sensor ultrasónico
volatile int Right_Distance;  // Variable para almacenar la distancia derecha medida por el sensor ultrasónico

float checkdistance() {
  // Función para medir la distancia usando el sensor ultrasónico
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);
  float distance = pulseIn(Echo_Pin, HIGH) / 58.00;
  delay(10);
  return distance;
}

void Ultrasonic_obstacle_avoidance() {
  // Función principal para la evitación de obstáculos utilizando el sensor ultrasónico
  Front_Distance = checkdistance();  // Medir la distancia frontal
  if ((Front_Distance < 20) && (Front_Distance > 0)) {
    // Si la distancia frontal es menor que 20 cm y mayor que 0 cm
    stopp();               // Detener el movimiento
    delay(100);
    myservo.write(180);    // Girar el servo motor a la posición 180 grados (izquierda)
    delay(500);
    Left_Distance = checkdistance();   // Medir la distancia izquierda después de girar
    delay(100);
    myservo.write(0);      // Girar el servo motor a la posición 0 grados (derecha)
    delay(500);
    Right_Distance = checkdistance();  // Medir la distancia derecha después de girar
    delay(100);

    if (Left_Distance > Right_Distance) {
      // Si la distancia izquierda es mayor que la derecha
      go_backward(150);   // Retroceder con velocidad 150
      delay(1500);        // Retroceder durante 1.5 segundos
      rotate_left(150);   // Girar a la izquierda con velocidad 150
      myservo.write(90);  // Reposicionar el servo motor al centro
      delay(500);         // Girar a la izquierda durante 0.5 segundos
    } else {
      // Si la distancia derecha es mayor que la izquierda
      go_backward(150);   // Retroceder con velocidad 150
      delay(1500);        // Retroceder durante 1.5 segundos
      rotate_right(150);  // Girar a la derecha con velocidad 150
      myservo.write(90);  // Reposicionar el servo motor al centro
      delay(500);         // Girar a la derecha durante 0.5 segundos
    }
  } else {
    go_forward(200);      // Avanzar con velocidad 200 si no hay obstáculos
  }
}

void Obstacle_Avoidance_Main() {
  // Función principal para la evitación de obstáculos
  Ultrasonic_obstacle_avoidance();  // Llamar a la función de evitación de obstáculos
}

void setup() {
  myservo.attach(A2);     // Adjuntar el servo motor al pin A2
  Serial.begin(9600);     // Inicializar la comunicación serial a 9600 baudios
  pinMode(Echo_Pin, INPUT);     // Configurar el pin ECHO como entrada
  pinMode(Trig_Pin, OUTPUT);    // Configurar el pin TRIG como salida
  pinMode(pinLB, OUTPUT);       // Configurar pinLB como salida
  pinMode(pinLF, OUTPUT);       // Configurar pinLF como salida
  pinMode(pinRB, OUTPUT);       // Configurar pinRB como salida
  pinMode(pinRF, OUTPUT);       // Configurar pinRF como salida
  pinMode(Lpwm_pin, OUTPUT);    // Configurar pin Lpwm_pin como salida (PWM)
  pinMode(Rpwm_pin, OUTPUT);    // Configurar pin Rpwm_pin como salida (PWM)
}

void loop() {
  Obstacle_Avoidance_Main();  // Llamar a la función principal de evitación de obstáculos en el bucle principal
}

void go_forward(unsigned char speed_val) {
  // Función para mover el robot hacia adelante
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
}

void go_backward(unsigned char speed_val) {
  // Función para mover el robot hacia atrás
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
}

void rotate_left(unsigned char speed_val) {
  // Función para rotar el robot a la izquierda
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
}

void rotate_right(unsigned char speed_val) {
  // Función para rotar el robot a la derecha
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
}

void stopp() {
  // Función para detener todos los motores
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
  analogWrite(Lpwm_pin, 0);
  analogWrite(Rpwm_pin, 0);
}