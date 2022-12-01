#ifndef ANDROID_HARDWARE_SERIALPORT_V1_0_SERIALPORTCALLBACK_H
#define ANDROID_HARDWARE_SERIALPORT_V1_0_SERIALPORTCALLBACK_H

#include <android/hardware/serialport/1.0/ISerialPortCallback.h>
#include <hidl/MQDescriptor.h>
#include <hidl/Status.h>

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

struct SerialPortCallback : public ISerialPortCallback {
    // Methods from ::android::hardware::serialport::V1_0::ISerialPortCallback follow.
    Return<void> onDataChange(const ::android::hardware::serialport::V1_0::SerialPortEvent& event) override;

    // Methods from ::android::hidl::base::V1_0::IBase follow.

};

// FIXME: most likely delete, this is only for passthrough implementations
// extern "C" ISerialPortCallback* HIDL_FETCH_ISerialPortCallback(const char* name);

}  // namespace implementation
}  // namespace V1_0
}  // namespace serialport
}  // namespace hardware
}  // namespace android

#endif  // ANDROID_HARDWARE_SERIALPORT_V1_0_SERIALPORTCALLBACK_H
