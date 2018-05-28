#ifndef ROB4_URSOCKET_H
#define ROB4_URSOCKET_H

#define UR_PORT 30002
#define UR_ADDRESS "169.254.178.76"

#define MOVE_SLEEP_MULTIPLIER 1100000 //1000000 = 100% of the motion time

#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

struct presentPositionStruct {
    float x;
    float y;
    float z;
};

class URSocket {
public:
    URSocket();

    ~URSocket();

    bool init();

    void shutdown();

    void command(std::string script);

    float getPresentX() { return presentPosition.x; }

    float getPresentY() { return presentPosition.y; }

    float getPresentZ() { return presentPosition.z; }

    void moveJ(const float &x, const float &y, const float &z, const float &t = 30, const float &r = 0);

    void moveL(const float &x, const float &y, const float &z, const float &t = 30, const float &r = 0);

    void moveJWait(const float &x, const float &y, const float &z, const float &t = 30, const float &r = 0);

    void moveLWait(const float &x, const float &y, const float &z, const float &t = 30, const float &r = 0);

    void closeGripper();

    void openGripper();

    void releaseGripper();

private:
    int socketUR;
    sockaddr_in destinationUR;

    presentPositionStruct presentPosition;
    bool isGripperClosed;

};

extern URSocket UR;

#endif //ROB4_URSOCKET_H