#include "SerialHelper.h"

SerialHelper::SerialHelper(class Stream *device, uint32_t bufferLength) {
    this->Device = device;
    this->bufferLength = bufferLength;
    this->Buffer = (char *)malloc(bufferLength);
    this->readPtr = this->Buffer;
}

bool SerialHelper::Read() {
    if (this->Device->available()) {
        char inChar = (char)this->Device->read();
        if (this->readComplete)
        {
            memset(this->Buffer, 0, this->bufferLength);
            this->readPtr = this->Buffer;
            this->readComplete = false;
        }

        *this->readPtr++ = inChar;

        if (inChar == '\r' || inChar == '\n' || this->readPtr >= this->Buffer + this->bufferLength) {
            this->readComplete = true;
            return true;
        }
    }
    return false;
}