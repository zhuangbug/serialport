namespace android {
namespace hardware {
namespace serialport {
namespace V1_0 {
namespace implementation {

class Type {
    public:
        static const int PUBLIC            = 0x01;
        static const int KEY               = 0x02;//按键
        static const int CAR_VERSION       = 0x30;//版本信息
        static const int CAR_BUS_INFO      = 0x31;//原车信息
        static const int AUTOPILOT         = 0x32;//自动驾驶
        static const int VDR               = 0x33;//行车记录仪
        static const int REAR_ZONE_CTL     = 0x34;//后区控制
        static const int TOUCH_DATA        = 0x35;//触摸屏数据
        static const int AIRCONTRL         = 0x36;//空调控制（暂时保留）
        static const int CAR_BODY_DATA     = 0x37;//车身数据
        static const int SYSTEM_MODE       = 0x38;//系统模式
        static const int UUID              = 0x39;//UUID识别码
        static const int AIRCC             = 0x40;//空调控制
        static const int TPMS              = 0x42;//胎压监测
        static const int FACTORY_TEST      = 0xF0;//工厂测试
        static const int SYSCALIBRATION    = 0xF2;//标定参数
};

class Public {
    public:
        //MCU -> ARM
        static const int U_MIN   = 0x20;
        static const int U_ACK   = U_MIN;

        //ARM -> MCU
        static const int C_MIN   = 0x80;
        static const int C_ACK   =0xA0;
};

class Key {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_KEY = 0x21;
        static const int U_ACK = 0x22;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_KEY = 0x81;
        static const int C_ACK = 0x82;
};

class CarVersion {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_ACK = U_MIN + 10;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_ACK = C_MIN + 4;
};

class SystemMode {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_ACK = U_MIN + 2;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_ACK = C_MIN + 3;
};

class Factory {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_ACK = U_MIN + 3;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_ACK = C_MIN + 8;
};

class Vdr {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_ACK = U_MIN + 39;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_ACK = C_MIN + 22;
};

class CarBody {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_ACK = U_MIN + 10;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_ACK = C_MIN + 24;

};

class Air {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_ACK = U_MIN + 4;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_ACK = C_MIN + 3;
};

class SysCalibration {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_ACK = U_MIN + 6;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_ACK = C_MIN + 6;

};

class TouchData {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_ACK = U_MIN + 3;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_ACK = C_MIN + 3;
};

class Autopilot {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_ACK = U_MIN + 6;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_ACK = C_MIN + 5;
};

class TPMS {
    public:
        //MCU -> ARM
        static const int U_MIN = 0x20;
        static const int U_ACK = U_MIN + 3;

        //ARM -> MCU
        static const int C_MIN = 0x80;
        static const int C_ACK = C_MIN + 2;
};

}  // namespace implementation
}  // namespace V1_0
}  // namespace serialport
}  // namespace hardware
}  // namespace android
