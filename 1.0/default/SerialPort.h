#ifndef ANDROID_HARDWARE_SERIALPORT_V1_0_SERIALPORT_H
#define ANDROID_HARDWARE_SERIALPORT_V1_0_SERIALPORT_H

#include <android/hardware/serialport/1.0/ISerialPort.h>
#include <hidl/MQDescriptor.h>
#include <hidl/Status.h>
#include "android/log.h"

static const char *TAG="serialport";
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, TAG, __VA_ARGS__) 
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG , TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO  , TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN  , TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR  , TAG, __VA_ARGS__)

namespace android {
namespace hardware {
namespace serialport {
namespace V1_0 {
namespace implementation {

using ::android::hardware::hidl_array;
using ::android::hardware::hidl_memory;
using ::android::hardware::hidl_string;
using ::android::hardware::hidl_vec;
using ::android::hardware::Return;
using ::android::hardware::Void;
using ::android::sp;

struct SerialPort : public ISerialPort {
    // Methods from ::android::hardware::serialport::V1_0::ISerialPort follow.
    Return<void> initSerialPort(const hidl_string& address, int32_t baudrate, int32_t flags) override;
    Return<void> startRead() override;
    Return<void> stopRead() override;
    Return<void> closeSerialPort() override;
    Return<bool> sendSerialPort(const hidl_vec<int32_t>& buf, int32_t size) override;
    Return<void> setCallback(int32_t type, const sp<::android::hardware::serialport::V1_0::ISerialPortCallback>& callback) override;
    Return<bool> isOpen() override;

    // Methods from ::android::hidl::base::V1_0::IBase follow.
    private:
        void run();

};

// FIXME: most likely delete, this is only for passthrough implementations
// extern "C" ISerialPort* HIDL_FETCH_ISerialPort(const char* name);

}  // namespace implementation
}  // namespace V1_0
}  // namespace serialport
}  // namespace hardware
}  // namespace android

#endif  // ANDROID_HARDWARE_SERIALPORT_V1_0_SERIALPORT_H
