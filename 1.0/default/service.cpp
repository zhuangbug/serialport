#define LOG_TAG "serialport1.0-service"

#include <android/log.h>
#include <hidl/HidlTransportSupport.h>
#include "SerialPort.h"

using android::sp;
using android::status_t;
using android::OK;
 
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;
using android::hardware::serialport::V1_0::ISerialPort;
using android::hardware::serialport::V1_0::implementation::SerialPort;

int main(int /* argc */, char* /* argv */ []) {
    android::sp<ISerialPort> service = new SerialPort();

    configureRpcThreadpool(4, true /*callerWillJoin*/);
    status_t status = service->registerAsService();

    if (status == OK) {
        LOGD("SerialPort HAL Ready.");
        service->initSerialPort("/dev/ttyS2", 115200, 0);
        service->startRead();
        joinRpcThreadpool();
    }

    LOGD("Cannot register Serialport HAL service");
    return 1;
}
