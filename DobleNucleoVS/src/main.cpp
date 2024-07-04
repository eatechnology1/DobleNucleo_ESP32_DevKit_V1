#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

AS5600 as5600;  

void loop0(void *parameter);

// Pines del puente H BTS7960
const int PWM_L = 14;
const int PWM_R = 12;
const int EN_R = 26;
const int EN_L = 27;

// Media Movil Delta del angulo
#define MOVING_AVERAGE_WINDOW 100
float deltaAngleHistory[MOVING_AVERAGE_WINDOW]; // Array para almacenar el historial de deltaAngle
int deltaAngleIndex = 0; // Índice para actualizar el array circularmente

// Variables Globales
uint16_t rawAngle;
uint16_t pwm = 10; // valores 0 - 255
float rpm = 0;
float anguloGrados = 0;
float angAnterior = 0;
float deltaAngle = 0;
unsigned long prevTime = 0;

char buffer[500]; // Buffer para convertir los valores a cadena de caracteres

// Definiciones de timer
#define UPDATE_PERIOD_MS 15
#define MINUTE_INTERVAL_MS 60000 // 1 minuto en milisegundos
#define DELTA_TIME_CONSTANT 1.7e-5
float delta_tiempo;

// Variables compartidas
volatile bool timerFlag = false;
volatile unsigned long TiempoAhora = 0;
int crudo = 0;

TaskHandle_t Tarea0;


void setup() {
    Serial.begin(115200);
    pinMode(EN_L, OUTPUT);
    pinMode(EN_R, OUTPUT);
    pinMode(PWM_L, OUTPUT);
    pinMode(PWM_R, OUTPUT);
    digitalWrite(EN_L, HIGH);
    digitalWrite(EN_R, HIGH);
    ledcSetup(0, 5000, 8); // Canal 0, frecuencia de 5kHz, resolución de 8 bits
    ledcAttachPin(PWM_R, 0);
    ledcWrite(0, 0);
    // Iniciar la comunicación I2C
    Wire.begin();
    // Configuracion del encoder  
    as5600.begin(4);  //  set direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    Serial.println(as5600.getAddress());
    int b = as5600.isConnected();
    Serial.print("Connect: ");
    Serial.println(b);
    xTaskCreatePinnedToCore(loop0, "Tarea_0", 3000, NULL, 0, &Tarea0, 0);
}

void loop() {
    // Serial.print(" ");
    //     Serial.print(crudo);
    //     Serial.print(" ");
    //     Serial.println(rpm);    

    // Mostrar el ángulo crudo en la interfaz serial
    sprintf(buffer, "Time: %lu, PWM: %d, Crudo: %u, Grados: %.5f, deltaAngle: %.3f, deltaTime: %.2f, RPM: %.6f",
                    TiempoAhora, pwm, crudo, anguloGrados, deltaAngle, delta_tiempo, rpm);
    Serial.println(buffer);

        // Control de PWM
        if (TiempoAhora > 30000) pwm = 25;
        ledcWrite(0, pwm);
        delay(1);
    //
    
}

void loop0(void *parameter) {
    while (1) {
        crudo = as5600.readAngle();
        TiempoAhora = millis();
        delta_tiempo = (TiempoAhora - prevTime);     
        anguloGrados = (crudo * 360) / 4096.0; // convertir a grados
        // Calcular la diferencia de ángulo
        if (anguloGrados > angAnterior) {
            deltaAngle = anguloGrados - angAnterior;
        } else if (anguloGrados < angAnterior) {
            deltaAngle = (360 - angAnterior) + anguloGrados;
        }
        // Calcular RPM
        float deltaTime = delta_tiempo * DELTA_TIME_CONSTANT; // Convertir a minutos
        if (deltaTime > 0) {
            float omega = deltaAngle / deltaTime;
            rpm = omega * (1.0 / 360.0); // pulsos por revolución
            rpm /= 131;  
        }
        // if (rpm>500)rpm=0;  
        vTaskDelay(0);
        angAnterior = anguloGrados;
        prevTime = TiempoAhora;
    }
}