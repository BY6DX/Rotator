#include <Arduino.h>
#include "Ticker.h"
#include "TM1620.h"
#include "EasyBuzzer.h"
#include "ArduinoNvs.h"
#include "AccelStepper.h"
#include "BluetoothSerial.h"
#include "SerialHelper.h"

#define BUFFER 128    // serial cmd buffer
#define DIVIDE 400.0  // micro stepping of stepping motor
#define RATIO  50.0   // speed ratio
#define SPEED  4.0    // antenna rotation speed in r/min
#define RANGE  450.0  // max rotation degree

enum SerialCmd {
    CW = 'R',
    CCW = 'L',
    Stop = 'S',
    Current = 'C',
    Goto = 'M',
    Position = 'P'
};

BluetoothSerial __SerialBT;
SerialHelper SerialUsb(&Serial, BUFFER);
SerialHelper SerialBT(&__SerialBT, BUFFER);

Ticker tickerLed, tickerMotor, tickerSave;

TM1620 led(4, 16, 17, 4);

AccelStepper motor(AccelStepper::DRIVER, 5, 18);

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


void loadPosition(){
    long ticks = (long)NVS.getInt("POSITION");
    double angle = getAngle(ticks);
    if (angle < 0 || angle > RANGE){
        led.setDisplayToString("ERRP");

        return;
    }
    motor.setCurrentPosition(ticks);
}

void savePosition(){
    NVS.setInt("POSITION", (int64_t)motor.currentPosition());
}

void savePositionOnRun(){
    EasyBuzzer.singleBeep(440, 100);
    savePosition();
    if (motor.isRunning()){
        tickerSave.once(1, savePositionOnRun);
    }
    else
    {
        motor.disableOutputs();
    }
}

void gotoAbsoluteAngle(double angle){
    motor.enableOutputs();
    motor.moveTo(getTicks(angle));
    savePositionOnRun();
}

void gotoAngle(double angle){
    gotoAbsoluteAngle(getClosestAngle(angle));
}

void displayPosition(){
    led.setDisplayToDecNumber((int)getCurrentAngle());
}

void setup() {
    Serial.begin(9600);
    __SerialBT.begin("BY6DX Rotator");
    NVS.begin();
    motor.setEnablePin(19);
    motor.setPinsInverted(false, false, true);
    motor.setMinPulseWidth(5);
    motor.setMaxSpeed(DIVIDE * RATIO * SPEED / 60);
    motor.setAcceleration(DIVIDE * RATIO / 100);
    loadPosition();
    EasyBuzzer.setPin(13);
    tickerLed.attach(1, displayPosition);
    pinMode(12, INPUT_PULLUP);
}

void serialDecode(SerialHelper serial) {
    char *ptr = serial.Buffer;

    switch (serial.Buffer[0]) {
        case Current: {
            serial.Device->printf("+0%03d\r\n", (int) getCurrentAngle());
            break;
        }
        case Goto: {
            long position = strtol(serial.Buffer + 1, &ptr, 10);
            gotoAngle(position);
            break;
        }
        case Stop: {
            motor.stop();
            break;
        }
        case CW: {
            gotoAbsoluteAngle(RANGE);
            break;
        }
        case CCW: {
            gotoAbsoluteAngle(0);
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
            savePosition();
            break;
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
    EasyBuzzer.update();
}