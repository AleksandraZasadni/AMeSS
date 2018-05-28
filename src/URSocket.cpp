#include "URSocket.h"

URSocket::URSocket() = default;

URSocket::~URSocket() = default;


bool URSocket::init() {
    socketUR = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socketUR < 0) {
        std::cout << "ERROR: UR socket creation FAILED!\n";
        return false;
    } else {
        destinationUR.sin_family = AF_INET;
        destinationUR.sin_port = htons(UR_PORT);
        destinationUR.sin_addr.s_addr = inet_addr(UR_ADDRESS);
        if (connect(socketUR, (sockaddr *) &destinationUR, sizeof(destinationUR)) == 0) {
            std::cout << "Info: UR socket connected!\n";
            return true;
        } else {
            std::cout << "ERROR: UR socket connection FAILED!\n";
            return false;
        }
    }
}

void URSocket::shutdown() {
    close(socketUR);
}

void URSocket::command(std::string script) {
    script.append("\n");
    send(socketUR, script.c_str(), script.size(), 0);
}


void URSocket::moveJ(const float &x, const float &y, const float &z, const float &t, const float &r) {
    command("movej(p[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) +
            ",1.2022,-2.9025,0],t=" + std::to_string(t) + ",r=" + std::to_string(r) + ")");
    presentPosition.x = x;
    presentPosition.y = y;
    presentPosition.z = z;
}

void URSocket::moveL(const float &x, const float &y, const float &z, const float &t, const float &r) {
    command("movel(p[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) +
            ",1.2022,-2.9025,0],t=" + std::to_string(t) + ",r=" + std::to_string(r) + ")");
    presentPosition.x = x;
    presentPosition.y = y;
    presentPosition.z = z;
}


void URSocket::moveJWait(const float &x, const float &y, const float &z, const float &t, const float &r) {
    moveJ(x, y, z, t, r);
    usleep(static_cast<__useconds_t>(MOVE_SLEEP_MULTIPLIER * t));
    presentPosition.x = x;
    presentPosition.y = y;
    presentPosition.z = z;
}

void URSocket::moveLWait(const float &x, const float &y, const float &z, const float &t, const float &r) {
    moveL(x, y, z, t, r);
    usleep(static_cast<__useconds_t>(MOVE_SLEEP_MULTIPLIER * t));
    presentPosition.x = x;
    presentPosition.y = y;
    presentPosition.z = z;
}

void URSocket::closeGripper() {
    UR.command("set_digital_out(1,True)");
    isGripperClosed = true;
}

void URSocket::openGripper() {
    UR.command("set_digital_out(0,True)");
    isGripperClosed = false;
}

void URSocket::releaseGripper() {
    if (isGripperClosed) {
        UR.command("set_digital_out(1,False)");
    } else {
        UR.command("set_digital_out(0,False)");
    }
}


URSocket UR; // NOLINT