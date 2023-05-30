#include "pico_comm.hpp"

int main()
{
    setupSerialComm();
    writeMotor(0);
}