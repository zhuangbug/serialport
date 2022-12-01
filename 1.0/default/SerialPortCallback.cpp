#include "SerialPortCallback.h"

namespace android {
namespace hardware {
namespace serialport {
namespace V1_0 {
namespace implementation {

// Methods from ::android::hardware::serialport::V1_0::ISerialPortCallback follow.
Return<void> SerialPortCallback::onDataChange(const ::android::hardware::serialport::V1_0::SerialPortEvent& event) {
    // TODO implement
    return Void();
}


// Methods from ::android::hidl::base::V1_0::IBase follow.

//ISerialPortCallback* HIDL_FETCH_ISerialPortCallback(const char* /* name */) {
    //return new SerialPortCallback();
//}
//
}  // namespace implementation
}  // namespace V1_0
}  // namespace serialport
}  // namespace hardware
}  // namespace android
