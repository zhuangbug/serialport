namespace android {
namespace hardware {
namespace serialport {
namespace V1_0 {
namespace implementation {

class AckInfo {
    public:
        int data[256];//
        int msgId;//
        int msgType;//
        int cmd;//
        int length;//
        int index = 0;//发送次数
        bool isNeedAck = true;//是否需要应答

        bool operator==(const AckInfo& ack) {
            if (this->msgId == ack.msgId && this->msgType == ack.msgType && this->cmd == ack.cmd) {
                return true;
            }
            return false;
        }

        void addIndex() {
            index = index + 1;
        }
};

}  // namespace implementation
}  // namespace V1_0
}  // namespace serialport
}  // namespace hardware
}  // namespace android
