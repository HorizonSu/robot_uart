#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include "robot_uart.h"

#define DEBUG       0
#define SPEED       B115200             /* 串口波特率 */ 
#define TIMEOUT     20000               /* 读串口超时时间 */
#define BUFF_SIZE   64                  /* 缓冲区大小 */                      
#define FRAME_HEAD  0xAA                /* 帧头 */
#define FRAME_TAIL  0x55                /* 帧尾 */
#define FRAME_CTRL  0xA5                /* 转义字符 */

#pragma pack(push, 1)

/* 控制寄存器组 */
struct control_register {
    int lineVelocity;                   /* 线速度 */
    int angularVelocity;                /* 角速度 */
};

/* 状态寄存器组 */
struct status_register {
    unsigned short ultrasound[6];
    unsigned int timeStamp;             /* 时间戳 */
    unsigned char dropSensor;           /* 跌落传感器 */
    unsigned short irSensor;            /* 红外传感器 */
    unsigned char collisionSensor;      /* 碰撞传感器 */
    short angularPos;                   /* 当前航向角位置 */
    int leftEncoderPos;                 /* 当前左边里程计的积分位置 */
    int rightEncoderPos;                /* 当前右边里程计的积分位置 */
    int lineVelocity;                   /* 线速度 */
    int angularVelocity;                /* 角速度 */
    unsigned char chargeStatus;
    unsigned char batterySataus;
    unsigned short errorState;
};

#pragma pack(pop)

static struct termios old_opt;          /* 默认配置信息 */

/* 设置串口通信相关属性 */
static int set_opt(int fd)
{
    struct termios opt;

    /* 读取配置信息 */
    if (tcgetattr(fd, &opt) != 0) {
        perror("tcgetattr");
        fprintf(stderr, "##################### %s: %d: %s\n", __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }
    old_opt = opt;

    /* 波特率 */
    cfsetispeed(&opt, SPEED);
    cfsetospeed(&opt, SPEED);

    /* 8位数据位 */
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;

    /* 无校验位 */
    opt.c_cflag &= ~PARENB;
    opt.c_iflag &= ~INPCK;

    /* 1位停止位 */
    opt.c_cflag &= ~CSTOPB;

    /* 设置本地模式并启用接收器 */
    opt.c_cflag |= CLOCAL | CREAD;
    
    /* 设置接收最少字符数和超时时间 */
    opt.c_cc[VTIME] = 10;
    opt.c_cc[VMIN] = 0;

    /* 选择原始输入和输出 */
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_oflag &= ~OPOST;

    /* 关闭流控 */
    opt.c_cflag &= ~CRTSCTS;
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* 写入配置信息 */
    if (tcsetattr(fd, TCSAFLUSH, &opt) != 0) {
        perror("tcsetattr");
        fprintf(stderr, "##################### %s: %d: %s\n", __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }

    return 0;
}

bool setupConnection(RSHandle * handle)
{
    /* 打开串口设备 */
    *handle = open(SERIAL, O_RDWR | O_NOCTTY | O_NDELAY);
    if (*handle == -1) {
        perror("open");
        fprintf(stderr, "##################### %s: %d: %s\n", __FILE__, __LINE__, __FUNCTION__);
        return false;
    }

    /* 设置串口通信相关属性 */
    if (set_opt(*handle) == -1) {
        fprintf(stderr, "set_opt faild!\n");
        return false;
    }
    
    return true;
}

bool closeConnection(RSHandle * handle)
{
    /* 恢复默认配置 */
    if (tcsetattr(*handle, TCSAFLUSH, &old_opt) != 0) {
        perror("tcsetattr");
        fprintf(stderr, "##################### %s: %d: %s\n", __FILE__, __LINE__, __FUNCTION__);
        return false;
    }

    /* 关闭串口设备 */
    close(*handle);

    return true;
}

/* 按协议打包数据 */
static bool make_data(unsigned char *buff, unsigned short offset, unsigned int len, const void *data)
{
    if (buff == NULL || len == 0 || data == NULL) {
        fprintf(stderr, "argument error!\n");
        return false;
    }

    buff[0] = 0x12;                 /* 控制寄存器 */
    buff[1] = 0x02;                 /* 功能码：写入 */
    memcpy(buff + 2, &offset, 2);   /* 偏移地址 */
    memcpy(buff + 4, &len, 4);      /* 数据长度 */
    memcpy(buff + 8, data, len);    /* 数据 */
    
    return true;
}

/* 写串口函数封装 */
static ssize_t writen(int fd, const void *vptr, size_t n)
{
    size_t nleft;
    ssize_t nwritten;
    const char *ptr;

    ptr = (const char *)vptr;
    nleft = n;
    while (nleft > 0) {
        if ((nwritten = write(fd, ptr, nleft)) <= 0) {
            if (nwritten < 0 && errno == EINTR)
                nwritten = 0;
            else
                return -1;
        }

        nleft -= nwritten;
        ptr += nwritten;
    }

    return n;
}

/* 数据封包并发送 */
static bool send_data(RSHandle handle, const unsigned char *buff, int len)
{
    unsigned char txbuf[BUFF_SIZE];
    unsigned char *pbuf = txbuf;
    unsigned char checksum = 0;
    unsigned int count = 0;
    int i;

    if (buff == NULL || len <= 0) {
        fprintf(stderr, "argument error!\n");
        return false;
    }

    /* 加入帧头 */
    *pbuf++ = FRAME_HEAD;
    *pbuf++ = FRAME_HEAD;

    /* 加入转义字节、计算校验和 */
    for (i = 0; i < len; i++) {
        if (buff[i] == FRAME_CTRL || buff[i] == FRAME_HEAD || buff[i] == FRAME_TAIL)
            *pbuf++ = FRAME_CTRL;

        *pbuf++ = buff[i];
        checksum += buff[i];
    }

    /* 加入校验和 */
    if (checksum == FRAME_CTRL || checksum == FRAME_HEAD || checksum == FRAME_TAIL)
        *pbuf++ = FRAME_CTRL;

    *pbuf++ = checksum;
    
    /* 加入帧尾 */
    *pbuf++ = FRAME_TAIL;
    *pbuf++ = FRAME_TAIL;

    count = pbuf - txbuf;          /* 计算字节数 */
    
    /* 发送数据 */
    if (writen(handle, txbuf, count) != count) {
        perror("wirte");
        fprintf(stderr, "##################### %s: %d: %s\n", __FILE__, __LINE__, __FUNCTION__);
        return false;
    }

#if DEBUG
    fprintf(stdout, "\n\n@@@@@@@@@@@@@@@@@@@@@ Write Data:\n");
    for (i = 0; i < 30; i++)
        fprintf(stdout, "%02X ", txbuf[i]);
    fprintf(stdout, "\n");
#endif

    return true;
}

bool setLineVelocity(RSHandle handle, float lineVelo)
{
    unsigned char buff[BUFF_SIZE];
    int line = (int)lineVelo;

    if (!make_data(buff, 0, sizeof(line), &line))
        return false;

    if (!send_data(handle, buff, sizeof(line) + 8))
        return false;

    return true;
}

bool setAngleVelocity(RSHandle handle, float AngleVelo)
{
    unsigned char buff[BUFF_SIZE];
    int angle = (int)AngleVelo;

    if (!make_data(buff, 4, sizeof(angle), &angle))
        return false;

    if (!send_data(handle, buff, sizeof(angle) + 8))
        return false;

    return true;
}

bool setLineAngleVelocity(RSHandle handle, float lineVelo, float AngleVelo)
{
    unsigned char buff[BUFF_SIZE];
    struct control_register data = {
        (int)lineVelo,
        (int)AngleVelo
    };
    
    if (!make_data(buff, 0, sizeof(data), &data))
        return false;

    if (!send_data(handle, buff, sizeof(data) + 8))
        return false;

    return true;
}

static ssize_t tread(int fd, void *buf, size_t nbytes, unsigned int timout)
{
    int nfds;
    fd_set readfds;
    struct timeval tv;

    tv.tv_sec = 0;
    tv.tv_usec = timout;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    nfds = select(fd+1, &readfds, NULL, NULL, &tv);
    if (nfds < 0) {
        perror("select");
        return -1;
    } else if (nfds == 0) {
        errno = ETIME;
        return -1;
    }

    return read(fd, buf, nbytes);
}

/* 读串口函数封装 */
static ssize_t treadn(int fd, unsigned char *buf, size_t nbytes, unsigned int timout)
{
    size_t      nleft;
    ssize_t     nread;

    nleft = nbytes;
    while (nleft > 0) {
        if ((nread = tread(fd, buf, nleft, timout)) < 0) {
            if (nleft == nbytes)
                return -1;
            else
                break;      /* error, return amount read so far */
        } else if (nread == 0) {
            break;          /* EOF */
        }

        nleft -= nread;
        buf += nread;
    }

    return nbytes - nleft;      /* return >= 0 */
}

/* 获取数据 */
static bool get_data(RSHandle handle, struct status_register *dptr)
{
    unsigned char rxbuf[128] = {0};
    unsigned char buff[BUFF_SIZE] = {0};
    unsigned char *ptr = buff;
    unsigned char checksum = 0;
    unsigned short offset = 0;
    int len = 0, i;

    if (dptr == NULL) {
        fprintf(stderr, "argument error!\n");
        return false;
    }
    bzero(dptr, sizeof(*dptr));

    /* 清空缓冲区 */
    if (tcflush(handle, TCIFLUSH) == -1) {   
        fprintf(stderr, "##################### %s: %d: %s\n", __FILE__, __LINE__, __FUNCTION__);
        return false;
    }

    /* 读数据 */
    if ((len = treadn(handle, rxbuf, 128, TIMEOUT)) < 0) {
        perror("read");
        fprintf(stderr, "##################### %s: %d: %s\n", __FILE__, __LINE__, __FUNCTION__);
        return false;
    }

#if DEBUG
    fprintf(stdout, "\n\n@@@@@@@@@@@@@@@@@@@@@ Read Data:\n");
    fprintf(stdout, "len: %d\n", len);
    fprintf(stdout, "*****************************************\n");
    for (i = 0; i < len; i++)
        fprintf(stdout, "%02X ", rxbuf[i]);
    fprintf(stdout, "\n");
    fprintf(stdout, "*****************************************\n");
#endif

    /* 检验帧头 */
    for (i = 0; i < len; i++) {
        if (rxbuf[i] == FRAME_HEAD && rxbuf[i + 1] == FRAME_HEAD)
            break;
    }

    /* 检验帧尾，去掉转义字节并计算校验和 */
    for (i += 2; i < len; i++) {
        if ( rxbuf[i + 1] == FRAME_TAIL && rxbuf[i + 2] == FRAME_TAIL)
            break;

        if (rxbuf[i] == FRAME_CTRL)
            i++;
        *ptr++ = rxbuf[i];
        checksum += rxbuf[i];
    }

    if (i >= len || checksum != rxbuf[i])
        return false;

    offset = *(unsigned short *)(buff + 2);
    len = *(int *)(buff + 4);
    memcpy((unsigned char *)dptr + offset, buff + 8, len);

#if DEBUG
    fprintf(stdout, "offset: %-u\n", offset);
    fprintf(stdout, "len: %-d\n", len);
    fprintf(stdout, "*****************************************\n");
    fprintf(stdout, "timeStamp:         %-u\n", dptr->timeStamp);
    fprintf(stdout, "dropSensor:        %-u\n", dptr->dropSensor);
    fprintf(stdout, "irSensor:          %-u\n", dptr->irSensor);
    fprintf(stdout, "collisionSensor:   %-u\n", dptr->collisionSensor);
    fprintf(stdout, "angularPos:        %-d\n", dptr->angularPos);
    fprintf(stdout, "leftEncoderPos:    %-d\n", dptr->leftEncoderPos);
    fprintf(stdout, "rightEncoderPos:   %-d\n", dptr->rightEncoderPos);
    fprintf(stdout, "lineVelocity:      %-d\n", dptr->lineVelocity);
    fprintf(stdout, "angularVelocity:   %-d\n", dptr->angularVelocity);
    fprintf(stdout, "*****************************************\n");
#endif

    return true;
}

bool getRobotEncoderPos(RSHandle handle, short &leftPos, short &rightPos)
{
    struct status_register data;

    bzero(&data, sizeof(data));

    if (!get_data(handle, &data))
        return false;

    leftPos = data.leftEncoderPos;
    rightPos = data.rightEncoderPos;

    return true;
}

bool getRobotInfrared(RSHandle handle, bool *infraRed)
{
    struct status_register data;
    int i;

    bzero(&data, sizeof(data));

    if (!get_data(handle, &data))
        return false;

    for (i = 0; i < 5; i++) {
        if (data.irSensor >> i & 0x0001)
            infraRed[i] = true;
        else
            infraRed[i] = false;
    }

    return true;
}

/*
 * 由于银星目前版本调试程序没有上报电量信息
 * 所以该函数暂不可用
 */
bool getRobotBattery(RSHandle handle, unsigned char &percentage)
{
    struct status_register data;

    bzero(&data, sizeof(data));

    if (!get_data(handle, &data))
        return false;

    percentage = data.batterySataus;

    return true;
}
