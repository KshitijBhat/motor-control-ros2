#include "pico_comm2.hpp"

const std::string SERIAL_PORT_2 = "/dev/ttyACM0" ;
float enc1;
float enc2;
float targetPos = 1.57; 
int main()
{
    PicoComms comm_;
    comm_.connect(SERIAL_PORT_2);
    std::cout << "\nConnected: " << comm_.connected()<<std::endl;
    comm_.setPID(150, 0.0, 0.0);
    while (true){
        comm_.controlPosition(targetPos);
    }
}

