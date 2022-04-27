#include <Arduino.h>
#include <Ticker.h>
#include <TM1620.h>
#include <ArduinoNvs.h>
#include <AccelStepper.h>
#include "BluetoothSerial.h"
#include "SerialHelper.h"

#define BUFFER 128    // serial cmd buffer
#define DIVIDE 400.0  // micro stepping of stepping motor
#define RATIO  50.0   // speed ratio
#define SPEED  4.0    // antenna rotation speed in r/min
#define RANGE  450.0  // max rotation degree

enum SerialCmd {
    Right = 'R',
    Left = 'L',
    Stop = 'S',
    Current = 'C',
    Goto = 'M',
    Position = 'P'
};

TM1620 led(4, 16, 17, 4);

AccelStepper motor(AccelStepper::DRIVER, 19, 18);

long getTicks(double angle) {
    return (long) (angle / 360 * DIVIDE * RATIO);
}

double getAngle(long ticks) {
    return ticks / (DIVIDE * RATIO) * 360;
}

double getCurrentAngle() {
    return getAngle(motor.currentPosition());
}

double getClosestAngle(double angle) {
    while (angle < 0 || angle > RANGE) {
        if (angle < 0)
            angle += 360;
        if (angle > RANGE)
            angle -= 360;
    }

    double opposite;
    if (angle < RANGE - 360) {
        opposite = angle + 360;
    } else if (angle > 360) {
        opposite = angle - 360;
    } else
        return angle;

    double current = getCurrentAngle();

    return abs(angle - current) < abs(opposite - current) ? angle : opposite;
}

BluetoothSerial __SerialBT;
SerialHelper SerialUsb(&Serial, BUFFER);
SerialHelper SerialBT(&__SerialBT, BUFFER);

void setup() {
    Serial.begin(9600);
    __SerialBT.begin("BY6DX Rotator");

    motor.setEnablePin(5);
    motor.setMinPulseWidth(5);
    motor.setMaxSpeed(DIVIDE * RATIO * SPEED / 60);
    motor.setAcceleration(DIVIDE * RATIO / 300);
    NVS.begin();
    ledcSetup(15, 800, 2);
    ledcAttachPin(23, 15);
    ledcWrite(15, 2);
}

void serialDecode(SerialHelper serial) {
    char *ptr = serial.Buffer;

    switch (serial.Buffer[0]) {
        case Current: {
            serial.Device->printf("+0%03d\r\n", (int) getCurrentAngle());
            led.setDisplayToString("6DX");
            break;
        }
        case Goto: {
            long position = strtol(serial.Buffer + 1, &ptr, 10);
            motor.moveTo(getTicks(getClosestAngle(position)));
            break;
        }
        case Stop: {
            motor.stop();
            break;
        }
        case Right: {
            motor.moveTo(getTicks(RANGE));
            break;
        }
        case Left: {
            motor.moveTo(0);
            break;
        }
        case Position: {
            long position = strtol(serial.Buffer + 1, &ptr, 10);
            if (position < 0 || position > RANGE) {
                serial.Device->println("OUT OF RANGE");
            }
            if (motor.isRunning())
                serial.Device->println("STOP FIRST");
            motor.setCurrentPosition(getTicks(position));
        }
        default: {
            serial.Device->println(serial.Buffer);
            break;
        }
    }
}

void loop() {
    if (SerialBT.Read())
        serialDecode(SerialBT);
    if (SerialUsb.Read())
        serialDecode(SerialUsb);
    motor.run();
}