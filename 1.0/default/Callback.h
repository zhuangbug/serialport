#include <android/hardware/serialport/1.0/ISerialPortCallback.h>


namespace android {
namespace hardware {
namespace serialport {
namespace V1_0 {
namespace implementation {

class Callback {
    public:
        SerialPortType type;
        sp<ISerialPortCallback> callback;
};

}  // namespace implementation
}  // namespace V1_0
}  // namespace serialport
}  // namespace hardware
}  // namespace android
