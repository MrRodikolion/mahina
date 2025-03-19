#include <Arduino.h>
#include <QTRSensors.h>
#include <ESP32Servo.h>

#define MOTOR_IN1 19
#define MOTOR_IN2 18
#define BASESPED 10
#define STOP 0

TaskHandle_t Task1;
TaskHandle_t Task2;

//------------ svet
bool is_red = true;
bool is_stop_lin = true;
int porog = 250;

//------------ irda
int sensor = 0;
int lastFilter = 0;

//------------ qtr sensor
QTRSensors qtr;

const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];
int sensetivity = 1000;
int pos = 0;
int posm = 0;

bool is_line_passing = false;
bool is_line_passed = false;

uint16_t position;

struct line_data
{
    uint32_t pos = 0;
    size_t start = 0, end = 0;

    int len()
    {
        return end - start;
    }
};

line_data lines[2];
line_data old_line;
line_data cur_line;

size_t line_n = 0;
int line_len = 0;

//------------ servo
Servo myServo;

//------------ motor
int speed = BASESPED;

//------------ pid
class PID
{
    float p, i, d;
    float old_error = 0;
    float integral = 0;

    float start_time;

public:
    PID(float kp, float ki, float kd)
    {
        p = kp;
        i = ki;
        d = kd;

        start_time = millis();
    }

    int get_angle(int centr)
    {
        int error = centr - 4500;

        int time = millis();
        // if (start_time - time > 30 * 1000) {
        //   start_time = millis();
        // }
        integral += error * i;
        float integ = integral / (start_time - time);

        int derivative = error - old_error;

        int angle = (p * error + integ + d * derivative) + 90;

        old_error = error;

        return min(max(angle, 57), 123);
    }
};

PID pid{0.015, 0.0025, 0.04}; // { 0.01, 0.004, 0.06 };  //{0.015, 0.0025, 0.04}; // {0.02, 0.003, 0.04}; //{0.015, 0.0025, 0.05}; // 0.015 0.003 0.03

int angle = 90;

void irDa();
line_data get_cool_line();
void find_centers();
void Task1code(void *pvParameters);
void Task2code(void *pvParameters);

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200);
    // setup pins
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    myServo.attach(5);

    // qtr setup
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){
                          34, 35, 32, 33, 25, 26, 27, 14, 12, 13},
                      SensorCount);
    // qtr calibration
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    for (uint16_t i = 0; i < 200; i++)
    {
        qtr.calibrate();
        // delay(10);
    }
    digitalWrite(LED_BUILTIN, LOW);

    xTaskCreatePinnedToCore(
        Task1code, /* Функция задачи. */
        "Task1",   /* Ее имя. */
        10000,     /* Размер стека функции */
        NULL,      /* Параметры */
        1,         /* Приоритет */
        &Task1,    /* Дескриптор задачи для отслеживания */
        0);        /* Указываем пин для данного ядра */
    delay(500);

    xTaskCreatePinnedToCore(
        Task2code, /* Функция задачи. */
        "Task2",   /* Имя задачи. */
        10000,     /* Размер стека */
        NULL,      /* Параметры задачи */
        1,         /* Приоритет */
        &Task2,    /* Дескриптор задачи для отслеживания */
        1);        /* Указываем пин для этой задачи */
    delay(500);
}

float calculateWeightedCenter(const u_int16_t *array, int size)
{
    float weightedSum = 0.0;
    float weightSum = 0.0;

    for (int i = cur_line.start; i < cur_line.end; i++)
    {
        // Умножаем индекс на 1000 для масштабирования до диапазона 0-10000
        weightedSum += (float)i * 1000 * array[i];
        weightSum += array[i];
    }

    return weightedSum / weightSum;
}

void Task1code(void *pvParameters)
{
    for (;;)
    {
        // get centr line
        // qtr.readCalibrated(sensorValues);
        pos = qtr.readLineBlack(sensorValues);

        for (int i = 0; i < SensorCount; i++)
        {
            if (sensorValues[i] > 500)
            {   
                cur_line.start = i;
                for (; i < SensorCount; i++)
                {
                    if (sensorValues[i] < 500) {
                        cur_line.end = i;
                        break;
                    }
                }
                break;
            }
        }

        posm = calculateWeightedCenter(sensorValues, SensorCount);

        // if (!is_line_passed)
        // {
        //     if (line_n <= 1 && is_line_passing)
        //     {
        //         is_line_passed = true;
        //     }
        //     else if (line_n > 1)
        //     {
        //         is_line_passing = true;
        //     }
        // }

        // Serial.print(pos);
        // Serial.print(" ");
        // Serial.println(posm);
        // for (int i = 0; i < SensorCount; i++){
        //     Serial.print(qtr.calibrationOn.initialized);
        //     Serial.print(" ");
        // }
        // Serial.println();
        Serial.print(pos);
        Serial.print(" ");
        Serial.println(posm);

        // get angle
        angle = pid.get_angle(pos);
        // if (is_stop_lin)
        // {
        //   angle = 90;
        // }

        // // write
        // myServo.write(angle);
        delay(5);
    }
}

void Task2code(void *pvParameters)
{
    for (;;)
    {
        irDa();
        speed = min(BASESPED, speed + 1);
        // if (analogRead(12) > porog && analogRead(13) > porog)
        // {
        //     is_stop_lin = true;
        // }
        // else
        // {
        //     is_stop_lin = false;
        // }

        if ((is_red && is_stop_lin))
        {
            speed = 0;
        }
        else if (is_red)
        {
            speed = BASESPED / 2;
        }
        analogWrite(MOTOR_IN1, speed);
        analogWrite(MOTOR_IN2, LOW);
    }
}

void loop()
{
}

void irDa()
{
    if (Serial2.available() > 0)
    {
        lastFilter = Serial2.read();
        if (lastFilter > -1 && lastFilter < 7)
        {
            sensor = lastFilter;
        }
        if (sensor == 0 || sensor == 1 || sensor == 4)
        {
            is_red = true;
        }
        else if (sensor == 2 || sensor == 3)
        {
            is_red = false;
        }
    }
    else
    {
        is_red = false;
    }
}
