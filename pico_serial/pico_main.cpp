#include "pico_comm2.hpp"

const std::string SERIAL_PORT_2 = "/dev/ttyACM0" ;
float enc1;
float enc2;
float targetVel = 5; 
float targetPos = 3; 
float vel = 0; 

int main()
{
    PicoComms comm_;
    comm_.connect(SERIAL_PORT_2);
    std::cout << "\nConnected: " << comm_.connected()<<std::endl;
    comm_.writeMotor(0,100);
    // comm_.setVPID(4, 0, 0);
    // sleep(1);
    for (int i=0; i<5; i++){
        
        // comm_.controlLeg(targetPos, targetVel);
        // comm_.controlPosition(targetPos);
        // comm_.readEncoder(enc1, enc2);
        // std::cout << "Positions: "<< enc1 << " " << enc2 << std::endl;

        // comm_.controlVelocity(targetVel);
        // comm_.readDrivingEncoderVelocity(vel);
        // std::cout << "Velocity: "<< vel<< std::endl;
        comm_.writeMotor(-100,-100);
    }
    // comm_.writeMotor(0,0);
    
}

