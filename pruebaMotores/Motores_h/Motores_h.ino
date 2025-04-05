// Definición de pines para Motor A
#define IN1 15    // Dirección Motor A
#define IN2 2     // Dirección Motor A
#define ENA 4     // PWM Motor A (PWM control)

// Definición de pines para Motor B
#define IN3 16    // Dirección Motor B
#define IN4 17    // Dirección Motor B
#define ENB 18    // PWM Motor B (PWM control)

// Configuración para LEDC (PWM en ESP32)
const int freq = 5000;         // Frecuencia de PWM en Hz
const int resolution = 8;      // Resolución en bits (0-255)
const int channelA = 0;        // Canal LEDC para Motor A
const int channelB = 1;        // Canal LEDC para Motor B

void setup() {
  // Configurar pines de dirección como salidas
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Configurar canales LEDC para la señal PWM
  ledcSetup(channelA, freq, resolution);
  ledcAttachPin(ENA, channelA);
  
  ledcSetup(channelB, freq, resolution);
  ledcAttachPin(ENB, channelB);
  
  // Iniciar comunicación serial
  Serial.begin(115200);
  Serial.println("Comandos: a=Adelante, b=Atrás, c=Derecha, d=Izquierda");
}

void loop() {
  if (Serial.available() > 0) {
    char comando = Serial.read();
    
    // Ignorar saltos de línea y retorno de carro
    if (comando == '\n' || comando == '\r') {
      return;
    }
    
    switch(comando) {
      case 'a':  // Adelante: ambos motores hacia adelante
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(channelA, 255);  // Velocidad máxima
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        ledcWrite(channelB, 255);  // Velocidad máxima
        Serial.println("Ambos motores avanzando");
        break;
        
      case 'b':  // Atrás: ambos motores en reversa
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(channelA, 255);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(channelB, 255);
        Serial.println("Ambos motores en reversa");
        break;
        
      case 'c':  // Derecha: motor A hacia adelante y motor B en reversa
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(channelA, 255);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(channelB, 255);
        Serial.println("Giro a la derecha");
        break;
        
      case 'd':  // Izquierda: motor A en reversa y motor B hacia adelante
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(channelA, 255);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        ledcWrite(channelB, 255);
        Serial.println("Giro a la izquierda");
        break;
        
      default:  // Comando no reconocido: detener ambos motores
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        ledcWrite(channelA, 0);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        ledcWrite(channelB, 0);
        Serial.println("Comando no reconocido, motores detenidos");
        break;
    }
  }
}