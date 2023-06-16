#include "pico_comm2.hpp"

const std::string SERIAL_PORT_2 = "/dev/ttyACM1" ;
float enc1;
float enc2;
float targetVel = 8; 
float targetPos = 3.14; 
float vel = 0; 
int main()
{
    PicoComms comm_;
    comm_.connect(SERIAL_PORT_2);
    std::cout << "\nConnected: " << comm_.connected()<<std::endl;
    // comm_.setPID(150, 0.0, 0.0);
    while(1){
        comm_.controlVelocity(targetVel);
        comm_.readDrivingEncoderVelocity(vel);
        std::cout << vel <<std::endl;
    }
}

