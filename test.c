#include <iostream>
#include "robot_uart.h"
#include <unistd.h>

int main(void) 
{
    RSHandle fd;
    bool infraRed[5];
    short leftPos, rightPos; 

    setupConnection(&fd);

    setLineAngleVelocity(fd, 32, 32);
    sleep(1);

    getRobotEncoderPos(fd, leftPos, rightPos);
    std::cout << "leftPos: " << leftPos << std::endl;
    std::cout << "rightPos: " << rightPos << std::endl;

    getRobotInfrared(fd, infraRed);
    std::cout << "infraRed: "  << std::endl;
    for (int i = 0; i < 5; i++)
        std::cout << ' ' << infraRed[i] << std::endl;

    closeConnection(&fd);

    return 0;
}
