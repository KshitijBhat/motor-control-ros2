#include "pico_comm2.hpp"

const std::string SERIAL_PORT_2 = "/dev/ttyACM0" ;
float enc1;
float enc2;
int targetPos = 165;
int main()
{
    PicoComms comm_;
    comm_.connect(SERIAL_PORT_2);
    std::cout << "\nConnected: " << comm_.connected()<<std::endl;
    comm_.setPID(0.5, 0.0, 0.0);
    comm_.giveTargets(targetPos, 100);
    sleep(10);
    comm_.readEncoder(enc1, enc2);
    std::cout << "\n" << enc1 << " "<< enc2<<std::endl;
    comm_.disconnect();
    std::cout << "\nConnected: " << comm_.connected()<<std::endl;
}