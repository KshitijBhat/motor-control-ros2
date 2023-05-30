#include "pico_comm.hpp"

int main()
{
    setupSerialComm();
    writeMotor(1000);
    sleep(1);
    std::cout << "\n" << readEncoder()<<std::endl;
}