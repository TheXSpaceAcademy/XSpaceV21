/*
Este ejemplo demuestra cómo implementar un Controlador de Ángulo para un Balancing Robot.

Autor: PabloC
Fecha: 26/06/2024
Dependencias: <XSpaceV21.h>, <XSpaceIoT.h>, <XSControl.h>

                                                                            SpeedController TASK (10 ms)
                                                            +------------------------------------------------------+                   
                                                            |          SpeedController        DC Motor 1           |  
                                                            |           +----------+         +---------+           |  
                                         +----+             |   +       |      Ki  | voltage |    b    |           |  +------------------------+
                                     +-->| -1 |-------------|---->o---->| Kp + --- |-------->|  -----  |-----------|->|                        |
            AngleController          |   +----+ speed_m1_sp |     ^ -   |       s  |         |  s + a  |       |   |  |                        |
                  TASK               |                      |     |     +----------+         +---------+       |   |  |                        |
          +------------------+       |                      |     |         10ms                               |   |  |                        |
          |        Ki        | velx  |                      |     |                                            |   |  |                        |
0 --->o-->|  Kp + --- + Kd*s |-------+                      |     |                  speed_m1                  |   |  |        Dynamics of     |  Pitch Angle
      |   |        s         |       |                      |     +--------------------------------------------+   |  |   Self Balancig Robot  |---------->
      |   +------------------+       |                      |                                                      |  |                        |        |
      |           20 ms              |                      |          SpeedController        DC Motor 2           |  |                        |        |
      |                              |                      |           +----------+         +---------+           |  |                        |        |
      |                              |          speed_m2_sp |  +        |      Ki  | voltage |    b    |           |  |                        |        |
      |                              +----------------------|-->o------>| Kp + --- |-------->|  -----  |-----------|->|                        |        |
      |                                                     |   ^ -     |       s  |         |  s + a  |       |   |  |                        |        |
      |                                                     |   |       +----------+         +---------+       |   |  |                        |        |
      |                                                     |   |                      10ms                    |   |  |                        |        |
      |                                                     |   |                                              |   |  +------------------------+        |
      |                                                     |   |                     speed_m2                 |   |                                    |
      |                                                     |   +----------------------------------------------+   |                                    |
      |                                                     +------------------------------------------------------+                                    |
      |                                                                   +------------------+                                                          |
      +-------------------------------------------------------------------|   Filter TASK    |----------------------------------------------------------+
                                                                          +------------------+
                                                                                   1ms

Instala las dependencias de las bibliotecas:
- XSpaceV21
- XSpaceIoT
- XSControl

Este ejemplo utiliza el escudo XSQC-2S para carga rápida de batería.
*/

#include <Arduino.h>
#include <XSpaceV21.h>
#include <XSpaceIoT.h>
#include <XSControl.h>

// Definición de los pines para controlar los LEDs de estado
#define RED 21
#define GREEN 22
#define BLUE 17
#define OFF 0

// Prototipos de funciones para el estado del LED y la obtención del voltaje de la batería
void XSQC_2S_Shield_LedSTATUS(int color);
double XSQC_2S_Shield_GetBatteryVoltage();

// Instanciación de objetos para la interfaz con el hardware
XSpaceV21Board XSBoard;
XSEthernet XSnet;
XSFilter Filter_a, Filter_g;
XSController Controller1, Controller2, Controller3;

// Definición de constantes para la configuración del control del motor
#define PWM_FREQUENCY 20000 // Frecuencia PWM para control de motor a 20 kHz para evitar ruido audible.
#define ENCODER_RESOLUTION 960 // Resolución del codificador, típicamente el número de pasos por revolución.

// Variables para almacenar datos brutos del sensor
float gx, gy, gz; // Datos del giroscopio para el ángulo de inclinación
float pitchAccel; // Ángulo de inclinación calculado a partir del acelerómetro
float pitchAccel_filtered; // Ángulo de inclinación del acelerómetro filtrado
float pitchGyro; // Ángulo de inclinación calculado a partir del giroscopio
float pitchGyro_filtered; // Ángulo de inclinación del giroscopio filtrado

// Variables para el filtro complementario
float pitch = 0.0; // Ángulo de inclinación combinado
const float alpha = 0.65; // Coeficiente del filtro complementario
double equilibrium_angle = 0; // Ángulo de equilibrio del robot

// Variables para almacenar la velocidad y el punto de consigna de los motores
double speed_m1;
double speed_m2;
double speed_m1_sp = 0; // Setpoint (punto de consigna) para el motor 1
double speed_m2_sp = 0; // Setpoint (punto de consigna) para el motor 2

// Tarea de filtrado: combina datos de acelerómetro y giroscopio para calcular el ángulo de inclinación
void FilterTask(void *pv) {
    pitch = XSBoard.BMI088_GetPitch_Accel(); // Inicializa la inclinación a partir del acelerómetro

    while (1) {
        // Recupera los últimos datos del giroscopio
        XSBoard.BMI088_GetGyroData(&gx, &gy, &gz);
        // Calcula el ángulo de inclinación del acelerómetro
        float pitchAccel = XSBoard.BMI088_GetPitch_Accel();

        // Integra los datos del giroscopio para obtener el cambio de inclinación
        float pitchGyro = pitch + gy * 0.001; // gy en grados por segundo

        // Aplica filtros de paso bajo de segundo orden a los datos
        pitchAccel_filtered = Filter_a.SecondOrderLPF(pitchAccel, 35, 0.001);
        pitchGyro_filtered  = Filter_g.SecondOrderLPF(pitchGyro, 35, 0.001);

        // Filtro complementario para combinar datos del acelerómetro y giroscopio
        pitch = alpha * pitchGyro_filtered + (1 - alpha) * pitchAccel_filtered;

        vTaskDelay(1); // Retraso de 1 milisegundo (ajustar según la aplicación)
    }

    // Nunca se alcanza, pero se coloca por seguridad
    vTaskDelete(NULL);
}

// Tarea del controlador de velocidad: ajusta el voltaje de los motores para seguir las velocidades deseadas
void SpeedController(void *pv){
    while(1){
        // Obtiene la velocidad actual de los motores a partir de los codificadores
        speed_m1 = XSBoard.GetEncoderSpeed(E1, DEGREES_PER_SECOND);
        speed_m2 = XSBoard.GetEncoderSpeed(E2, DEGREES_PER_SECOND);
        
        // Aplica el controlador PI para ajustar el voltaje del motor
        XSBoard.DRV8837_Voltage(DRVx1, Controller1.PI_ControlLaw(speed_m1, speed_m1_sp, 0.0241, 0.4820, FORWARD_EULER, 0.01));
        XSBoard.DRV8837_Voltage(DRVx2, Controller2.PI_ControlLaw(speed_m2, speed_m2_sp, 0.0222, 0.4440, FORWARD_EULER, 0.01));

        vTaskDelay(10); // Pausa de 10 milisegundos entre mediciones de velocidad
    }

    // Nunca se alcanza, pero se coloca por seguridad
    vTaskDelete(NULL);
}

// Tarea del controlador de ángulo: ajusta la velocidad de los motores para mantener el ángulo deseado
void AngleController(void *pv){
    float velx; // Velocidad calculada para los motores

    double Kp = 50; // Ganancia proporcional
    double Kd = 0; // Ganancia derivativa
    double Ki = 120; // Ganancia integrativa

    while(1){
        // Calcula la velocidad deseada basada en el control PID
        velx = Controller3.PID_ControlLaw(pitch, equilibrium_angle, Kp, Ki, FORWARD_EULER, Kd, FORWARD_EULER, 9.73920144223375, 0.02);
        
        // Ajusta los puntos de consigna de velocidad para los motores
        speed_m1_sp = -velx;
        speed_m2_sp = velx;

        vTaskDelay(20); // Pausa de 20 milisegundos entre ajustes
    }

    // Nunca se alcanza, pero se coloca por seguridad
    vTaskDelete(NULL);
}

// Configuración inicial
void setup() {
    Serial.begin(1000000); // Inicializa la comunicación serial a 1 Mbps para transmisión rápida de datos
    
    // Obtiene el voltaje de la batería conectada al escudo XSQC-2S
    double VM = XSQC_2S_Shield_GetBatteryVoltage();

    // Inicializa la placa XSpace con la frecuencia PWM y resolución del codificador
    XSBoard.init(PWM_FREQUENCY, ENCODER_RESOLUTION, VM);
    XSBoard.DRV8837_Wake(DRVx1); // Despierta el controlador del motor 1
    XSBoard.DRV8837_Wake(DRVx2); // Despierta el controlador del motor 2

    // Inicializa WiFi y conexión UDP para monitoreo
    XSnet.Wifi_init("Delta", "c9aa28ba93");
    XSnet.UDP_Connect("192.168.31.150", 55000); // Usa la APP Aurora como monitor de datos inalámbrico

    // Crea la tarea para el filtrado y cálculo del ángulo de inclinación
    xTaskCreate(FilterTask, "FilterTask", 5000, NULL, 1, NULL);

    // Encuentra el ángulo de equilibrio inicial
    XSQC_2S_Shield_LedSTATUS(BLUE); // LED azul indica calibración
    for(int i = 0; i < 200; i++){
        equilibrium_angle = pitch; // El ángulo de equilibrio es el valor promedio de inclinación
        XSnet.println(equilibrium_angle); // Envía el ángulo de equilibrio a través de la red
        delay(10);
    }
    XSQC_2S_Shield_LedSTATUS(OFF); // Apaga el LED después de la calibración

    delay(1000); // Pausa para asegurar que todo esté listo
    XSQC_2S_Shield_LedSTATUS(GREEN); // LED verde indica que el robot está listo

    // Crea las tareas para el control de velocidad y ángulo
    xTaskCreate(SpeedController, "SpeedController", 5000, NULL, 1, NULL);
    xTaskCreate(AngleController, "AngleController", 5000, NULL, 1, NULL);
}

// Bucle principal: transmite datos de inclinación y velocidad
void loop() {
    XSnet.println(String(pitch) + " " + String(speed_m2_sp)); // Envia el ángulo de inclinación y la velocidad a través de la red

    // Seguridad: si la inclinación supera 40 grados, detiene los motores
    if(abs(pitch) > 40){
        XSQC_2S_Shield_LedSTATUS(RED); // LED rojo indica un error
        XSBoard.DRV8837_Sleep(DRVx1); // Pone a dormir el motor 1
        XSBoard.DRV8837_Sleep(DRVx2); // Pone a dormir el motor 2
        delay(1000000); // Espera indefinidamente para seguridad
    }
    delay(10); // Pausa de 10ms entre envíos de datos
}

double XSQC_2S_Shield_GetBatteryVoltage(){
    return (double)analogRead(36)/4096.0*3.3*4.0;
}

void XSQC_2S_Shield_LedSTATUS(int color){
    pinMode(22,OUTPUT); //STATUS GREEN LED
    pinMode(21,OUTPUT); //STATUS RED LED
    pinMode(17,OUTPUT); //STATUS BLUE LED

        if(color == RED){
            digitalWrite(RED,HIGH);
            digitalWrite(GREEN,LOW);
            digitalWrite(BLUE,LOW);
        } 
        if(color == GREEN){
            digitalWrite(RED,LOW);
            digitalWrite(GREEN,HIGH);
            digitalWrite(BLUE,LOW);
        }
        if(color == BLUE){
            digitalWrite(RED,LOW);
            digitalWrite(GREEN,LOW);
            digitalWrite(BLUE,HIGH);
        } 
        if(color == OFF){
            digitalWrite(RED,LOW);
            digitalWrite(GREEN,LOW);
            digitalWrite(BLUE,LOW);
        } 
}
