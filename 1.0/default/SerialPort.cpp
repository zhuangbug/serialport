#define UNUSED(x) (void)(x)
#include "SerialPort.h"
#include "Callback.h"
#include "AckInfo.cpp"
#include "FinalData.cpp"

#include <bitset>
#include <cstring>
#include <fcntl.h>
#include <list>
#include <stdio.h>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sstream>
#include <vector>
#include <mutex>
//add for simulate data
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <cutils/properties.h>
#include <iomanip>
#define SIMULATE_DEV_NAME "/vendor/vdr"
#define PTMASTER "/dev/ptmx"
#define GPSPIPE "/data/misc/gps/gps-pipe"
//add for simulate data end
using namespace std;
typedef unsigned char byte;

namespace android {
namespace hardware {
namespace serialport {
namespace V1_0 {
namespace implementation {

bool DEBUG = false;
int fd = -1;
int gps_fd = -1;
sp<ISerialPortCallback> mCallback = nullptr;
list<Callback> mList;//回调
list<AckInfo> mSendList;//发送指令集合
list<AckInfo> mAckList;//发送应答指令集合
//std::thread mThread;
pthread_t mReadThread;//读取数据线程
pthread_t mWriteThread;//写入数据线程
pthread_t mSimulateThread;//模拟数据线程
mutex mtx;
bool mIsWaitAck = false;//是否等待MCU应答
bool mIsOpen;//串口文件是否打开
bool mPtsOpen;

//获取波特率
static int getBaudrate(int baudrate) {
    switch(baudrate) {
        case 0: return B0;
        case 50: return B50;
        case 75: return B75;
        case 110: return B110;
        case 134: return B134;
        case 150: return B150;
        case 200: return B200;
        case 300: return B300;
        case 600: return B600;
        case 1200: return B1200;
        case 1800: return B1800;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default: return -1;
     }
}

//读取串口数据
int32_t UART_recv(int len) {
    //LOGD("UART_recv");
    //定义读事件集合
    fd_set fdRead;
    int ret;
    struct timeval aTime;

    FD_ZERO(&fdRead);
    FD_SET(fd,&fdRead);

    aTime.tv_sec = 0;
    aTime.tv_usec = 300000; //300ms

    //ret = select(fd + 1, &fdRead, NULL, NULL, &aTime );
    ret = select(fd + 1, &fdRead, NULL, NULL, NULL);

    int32_t data[1];
    if (ret < 0) {
        //关闭串口
        close(fd);
    } else if (ret > 0) {
        //判断是否读事件
        if (FD_ISSET(fd,&fdRead)) {
            //data available, so get it!
            ret = read(fd, data, len);
            // 对接收的数据进行处理，这里为简单的数据回发
            return data[0];
        }
    }
    return -1;
}

//验证校验码
bool check(vector<int32_t> receiveBuffer) {
    int checkSum = receiveBuffer.back();
    int check = 0;
    int size = receiveBuffer.size();
    for (int i = 3; i < size -1; i++) {
        check += receiveBuffer[i];
    }

    bitset<sizeof(byte)*8> bs(check);
    int sum = (bs.flip().to_ulong() + 1) & 0xFF;
    if (DEBUG) LOGD("check --- checkSum = %d   sum = %d", checkSum, sum);
    if (checkSum == sum) {
        return true;
    }
    return false;

}

//收到mcu的回复，删除此消息队列
void removeAck(int msgId, int msgType, int cmd) {
    if (mIsWaitAck) {
        AckInfo info;
        info.msgId = msgId;
        info.msgType = msgType;
        info.cmd = cmd;
        list<AckInfo>::iterator ite = find(mSendList.begin(), mSendList.end(), info);
        if (ite != mSendList.end()) {//存在此应答
            mSendList.erase(ite);
            mIsWaitAck = false;
        }
    }
}

//添加应答队列，回复mcu收到消息
void addAck(int msgId, int msgType, int ackId, int cmd) {
    int data[9];
    data[0] = 0xFA;
    data[1] = 0x55;
    data[2] = msgId;
    data[3] = 5;
    data[4] = msgType;
    data[5] = ackId;
    data[6] = cmd;
    data[7] = 1;
    bitset<sizeof(byte)*8> bs(5 + msgType + ackId + cmd + 1);
    int sum = bs.flip().to_ulong() + 1;
    data[8] = sum;
    AckInfo ackInfo;
    memcpy(ackInfo.data, data, 9 * sizeof(int));
    ackInfo.msgId = msgId;
    ackInfo.msgType = msgType;
    ackInfo.cmd = cmd;
    ackInfo.length = 9;
    mAckList.push_back(ackInfo);
    if (DEBUG) LOGD("addack  data msgId = %d  msgType = %d   ackId = %d  cmd = %d", ackInfo.data[2], ackInfo.data[4], ackInfo.data[5], ackInfo.data[6]);
}

//分发数据
void dataReceive(vector<int32_t> receiveBuffer) {
    if (receiveBuffer.size() > 0) {
        int msgID = receiveBuffer[2];
        //int length = receiveBuffer[3];
        int msgType = receiveBuffer[4];
        int cmd = receiveBuffer[5];
        int CallbackType = -1;
        int start = 0;
        switch (msgType) {
            case Type::PUBLIC://
                if (cmd == Public::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != Public::C_ACK) {
                    addAck(msgID, msgType, Public::C_ACK, cmd);
                }
                CallbackType = 0;
                start = 4;
                break;
            case Type::KEY://按键
                if (cmd == Key::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != Key::C_ACK) {
                    addAck(msgID, msgType, Key::C_ACK, cmd);
                }
                CallbackType = 4;
                start = 5;
                break;
            case Type::CAR_VERSION://版本信息
                CallbackType = 0;
                if (cmd == CarVersion::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != CarVersion::C_ACK) {
                    addAck(msgID, msgType, CarVersion::C_ACK, cmd);
                }
                start = 4;
                break;
            case Type::CAR_BUS_INFO://原车信息
                CallbackType = 0;
                start = 4;
                break;
            case Type::AUTOPILOT://自动驾驶
                if (cmd == Autopilot::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != Autopilot::C_ACK) {
                    addAck(msgID, msgType, Autopilot::C_ACK, cmd);
                }
                CallbackType = 6;
                start = 5;
                break;
            case Type::REAR_ZONE_CTL://后区控制
                CallbackType = 0;
                start = 4;
                break;
            case Type::TOUCH_DATA://触摸屏数据
                if (cmd == TouchData::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != TouchData::C_ACK) {
                    addAck(msgID, msgType, TouchData::C_ACK, cmd);
                }
                CallbackType = 0;
                start = 4;
                break;
            case Type::AIRCONTRL://空调控制(暂时保留)
                CallbackType = 0;
                start = 4;
                break;
            case Type::SYSTEM_MODE://系统模式
                CallbackType = 0;
                start = 4;
                if (cmd == SystemMode::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != SystemMode::C_ACK) {
                    addAck(msgID, msgType, SystemMode::C_ACK, cmd);
                }
                break;
            case Type::UUID://UUID 识别码
                CallbackType = 0;
                start = 4;
                break;
            case Type::FACTORY_TEST://工厂测试模式
                CallbackType = 5;
                start = 5;
                if (cmd == Factory::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != Factory::C_ACK) {
                    addAck(msgID, msgType, Factory::C_ACK, cmd);
                }
                break;
            case Type::SYSCALIBRATION://标定参数
                CallbackType = 0;
                start = 4;
                if (cmd == SysCalibration::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != SysCalibration::C_ACK) {
                    addAck(msgID, msgType, SysCalibration::C_ACK, cmd);
                }
                break;
            case Type::VDR://行车记录仪
                CallbackType = 1;
                start = 5;
                if (cmd == Vdr::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != Vdr::C_ACK) {
                    addAck(msgID, msgType, Vdr::C_ACK, cmd);
                }
                break;
            case Type::CAR_BODY_DATA://车身数据
                CallbackType = 2;
                start = 5;
                if (cmd == CarBody::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != CarBody::C_ACK) {
                    addAck(msgID, msgType, CarBody::C_ACK, cmd);
                }
                break;
            case Type::AIRCC://空调控制
                CallbackType = 3;
                start = 5;
                if (cmd == Air::U_ACK) {//收到应答 删除发送指令
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != Air::C_ACK) {
                    addAck(msgID, msgType, Air::C_ACK, cmd);
                }
                break;
            case Type::TPMS://胎压监测
                CallbackType = 7;
                start = 5;
                if (cmd == TPMS::U_ACK) {
                    removeAck(msgID, msgType, receiveBuffer[6]);
                } else if (cmd != TPMS::C_ACK) {
                    addAck(msgID, msgType, TPMS::C_ACK, cmd);
                }
				break;
        }
        vector<int32_t> data;
        for(int i = start; i < ((int) receiveBuffer.size() -1); i++) {//去除前面几位和最后一位无效数据
            data.push_back(receiveBuffer[i]);
        }
        if(msgType == Type::SYSTEM_MODE && cmd == 0x21) {
            LOGD("dataReceive --- SYSTEM_MODE = %d", receiveBuffer[6]);
        }

        list <Callback>::iterator ita = mList.begin();
        for (; ita != mList.end(); ++ita) {
            SerialPortEvent event;
            if (CallbackType == 4) {
                event.what = (int32_t) msgID;
            } else {
                event.what = (int32_t) (*ita).type;
            }
            event.length = receiveBuffer.size() - start - 1;
            event.data = data;
            int type = (int32_t) (*ita).type;
            if (DEBUG) LOGD("dataReceive --- CallbackType = %d   what = %d", CallbackType, type);
            if ((*ita).callback != nullptr && type == CallbackType) {
                (*ita).callback->onDataChange(event);
            }
        }
    }
}

void *runRead(void* run) {
    long time = (long)run;
    LOGD("runRead %lu", time);
    vector<int32_t> receiveBuffer;
    int receiveLength;

    while(1) {//读取一包数据
        receiveBuffer.clear();
        receiveLength = 0;
        if (DEBUG) LOGD("runRead  while");
        int32_t data = UART_recv(1);//读取第一位
        if (data == 250) {
            receiveBuffer.push_back(data);
            data = UART_recv(1);//读取第二位
            if (data == 85) {//一包的数据开头是 250 85
                receiveBuffer.push_back(data);
                data = UART_recv(1);//读取弟三位   MsgID
                receiveBuffer.push_back(data);
                data = UART_recv(1);//读取第四位   数据Length
                receiveBuffer.push_back(data);


                int length = data;
                LOGD("runRead --- length = %d", length);
                for (int n = 0; n < length; n++) {
                    data = UART_recv(1);
                    receiveBuffer.push_back(data);
                }
                stringstream dataStr;
                for(int i = 0; i < (int) receiveBuffer.size(); i++) {
                    dataStr << " " << hex << receiveBuffer[i];
                }
                LOGD("runRead --- dataStr = %s", dataStr.str().c_str());
                if (check(receiveBuffer)) {
                    dataReceive(receiveBuffer);
                }
            }
        }
        usleep(10 * 1000);
    }
}
//add for simulate data
string readTxt(string file){
    ifstream infile; 
    infile.open(file.data());   //将文件流对象与文件连接起来 
    assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行 

    string s;
	string result;
    while(getline(infile,s))
    {
        result = result + s;
    }
    infile.close(); 
	return result;
}

int readBytes(string path, vector<int32_t> result) {
	fstream file;
    file.open(path.data(), ios::binary|ios::in); 
 
    assert(file.is_open());  
    char buffer[200];           //存储读取字符串
	file.read(buffer, 200);
	int len = file.gcount();
	for(int i =0;i<len;i++) {
                result.push_back((int32_t) (buffer[i]&0xFF));
	}
	file.close();
	return len;
}

void *runSimulate(void* run) {
	long time = (long)run;
    if (DEBUG) LOGD("runSimulate %lu", time);
    int fd;
	int wd;
	int len;
	int nread;
	char buf[BUFSIZ];
	struct inotify_event *event;
 
	fd = inotify_init();
	if (fd < 0)
	{
		if (DEBUG) LOGD("runSimulate inotify_init failed");
		return NULL;
	}
 
	wd = inotify_add_watch(fd, "vendor/simulate.enable", IN_MODIFY);
	if (wd < 0)
	{
		if (DEBUG) LOGD("runSimulate inotify_add_watch vendor/simulate.enable failed");
		return NULL;
	}
 
	buf[sizeof(buf) - 1] = 0;
	while ((len = read(fd, buf, sizeof(buf) - 1)) > 0)
	{
		nread = 0;
		if(len > 0)
		{
			event = (struct inotify_event *)&buf[nread];
			LOGD("runSimulate inotify event=%d",event->mask);
			vector<int32_t> buffer;
			int len = readBytes(SIMULATE_DEV_NAME, buffer);
			stringstream dataStr;
			for(int i = 0; i < (int) buffer.size(); i++) {
				dataStr << " " << hex << buffer[i];
			}
			if (DEBUG) LOGD("runSimulate --- len = %d, dataStr = %s", len, dataStr.str().c_str());
			dataReceive(buffer);
		}
	}
	return NULL;
}
//add for simulate data end

// Methods from ::android::hardware::serialport::V1_0::ISerialPort follow.
Return<void> SerialPort::initSerialPort(const hidl_string& address, int32_t baudrate, int32_t flags) {
    if (DEBUG) LOGD("initSerialPort");

    int speed;
    speed = getBaudrate(baudrate);
    if (speed == -1) {
        return Void();
    }

    char c[20];
    strcpy(c, address.c_str());
    if (flags == 0) {
        fd = open(c, O_RDWR|O_NOCTTY);
    }
    if(fd < 0) {
        if (DEBUG) LOGD("open uart device error\n");
        mIsOpen = false;
    } else {
        mIsOpen = true;
    }

    struct termios cfg;
    if (tcgetattr(fd, &cfg) < 0) {
        close(fd);
        return Void();
    }

    cfmakeraw(&cfg);
    cfsetispeed(&cfg, speed);
    cfsetospeed(&cfg, speed);

    //修改控制模式，保证程序不会占用串口？
    cfg.c_cflag |= CLOCAL;
    //printf("c_cflag |= CLOCAL => %x\r\n", new_opt.c_cflag);

    //修改控制模式，使得能够从串口读取输入数据
    cfg.c_cflag |= CREAD;
    //printf("c_cflag |= CREAD => %x\r\n", new_opt.c_cflag);

    if (tcsetattr(fd, TCSANOW, &cfg) < 0) {
        close(fd);
        return Void();
    }

    return Void();
}

void writeData(AckInfo info) {
    unsigned char data[info.length];
    stringstream dataStr;
    for(int i = 0; i < info.length; i++) {
        data[i] = (unsigned char) info.data[i];
        dataStr << " " << hex << info.data[i];
    }
    int len = write(fd, data, info.length);
    if (DEBUG) LOGD("writeData  data   = %s   len = %d", dataStr.str().c_str(), len);
}

void *runWrite(void* write) {
    long time = (long) write;
    if (DEBUG) LOGD("runWrite %lu", time);
    int wait = 0;
    while (1) {
        usleep(10 * 1000);
        mtx.lock();

        //向MCU 发送应答
        if (!mAckList.empty()) {
            AckInfo ackInfo = mAckList.front();
            writeData(ackInfo);
            list<AckInfo>::iterator ite = find(mAckList.begin(), mAckList.end(), ackInfo);
            if(ite != mAckList.end()) {
                mAckList.erase(ite);
            }
        }

        if (mIsWaitAck) {
            wait++;
            if (wait < 20) {//等待200ms
                mtx.unlock();
                continue;
            } else {
            }
            wait = 0;
        }
        if (!mSendList.empty()) {
            AckInfo ackInfo = mSendList.front();
            list<AckInfo>::iterator liter = mSendList.begin();
            writeData(ackInfo);
            if ((*liter).isNeedAck) {
                (*liter).index += 1;//发送次数加一
                if (DEBUG) LOGD("runWrite  index = %d", (*liter).index);
                if ((*liter).index > 3) {//发送4次后删除此消息
                    list<AckInfo>::iterator ite = find(mSendList.begin(), mSendList.end(), ackInfo);
                    if (ite != mSendList.end()) {//存在此应答
                        mSendList.erase(ite);
                    }
                    mIsWaitAck = false;
                } else {
                    mIsWaitAck = true;
                }
            } else {//不需要应答 删除此消息
                list<AckInfo>::iterator ite = find(mSendList.begin(), mSendList.end(), ackInfo);
                if (ite != mSendList.end()) {
                    mSendList.erase(ite);
                }
                mIsWaitAck = false;
            }
        }
        mtx.unlock();
    }
}

Return<void> SerialPort::startRead() {
    int read = 1;
    pthread_create(&mReadThread, nullptr, runRead, &read);
    int write = 2;
    pthread_create(&mWriteThread, nullptr, runWrite, &write);
	//add for simulate data
	string simulateFlag = readTxt("vendor/simulate.enable");
	if (DEBUG) LOGD("start read simulateFlag=%s", simulateFlag.c_str());
	if(simulateFlag == "1") {
		int simulate = 3;
		pthread_create(&mSimulateThread, nullptr, runSimulate, &simulate);
	}
	//add for simulate data end
    return Void();
}

Return<void> SerialPort::stopRead() {
    pthread_detach(mReadThread);
    pthread_detach(mWriteThread);
    return Void();
}

Return<void> SerialPort::closeSerialPort() {
    if (DEBUG) LOGD("closeSerialPort");
    close(fd);
    return Void();
}

Return<bool> SerialPort::sendSerialPort(const hidl_vec<int32_t>& buf, int32_t size) {
    if (DEBUG) LOGD("sendSerialPort  buf size = %d" , size);
    mtx.lock();
    int data[size];
    stringstream dataStr;
    for(int i = 0; i < size; i++) {
        data[i] = buf[i];
        dataStr << " " << hex << buf[i];
    }
    if (DEBUG) LOGD("sendSerialPort  data   = %s", dataStr.str().c_str());
    AckInfo ackInfo;
    memcpy(ackInfo.data, data, size*sizeof(int32_t));
    ackInfo.msgId = buf[2];
    ackInfo.msgType = buf[4];
    ackInfo.cmd = buf[5];
    ackInfo.length = size;
    mSendList.push_back(ackInfo);
    mtx.unlock();
    return bool {};
}

Return<void> SerialPort::setCallback(int32_t type, const sp<::android::hardware::serialport::V1_0::ISerialPortCallback>& callback) {
    mCallback = callback;
    Callback call;
    call.type = SerialPortType(type);
    call.callback = callback;
    mList.push_back(call);
    return Void();
}

Return<bool> SerialPort::isOpen() {
    return mIsOpen;
}


// Methods from ::android::hidl::base::V1_0::IBase follow.

//ISerialPort* HIDL_FETCH_ISerialPort(const char* /* name */) {
    //return new SerialPort();
//}
//
}  // namespace implementation
}  // namespace V1_0
}  // namespace serialport
}  // namespace hardware
}  // namespace android
