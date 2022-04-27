#ifndef ROTATOR_SERIALHELPER_H
#define ROTATOR_SERIALHELPER_H

#include "Stream.h"

class SerialHelper {
private:
    uint32_t bufferLength;
    char *readPtr;
    bool readComplete = true;
public:
    SerialHelper(Stream *device, uint32_t bufferLength);
    Stream *Device;
    bool Read();
    char *Buffer;
    const uint32_t &BufferLength = bufferLength;
    const bool &ReadComplete = readComplete;
};


#endif //ROTATOR_SERIALHELPER_H
