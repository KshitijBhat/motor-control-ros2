#include "pico_comm2.hpp"

const std::string SERIAL_PORT_2 = "/dev/ttyACM0" ;
float enc1;
float enc2;
int main()
{
    PicoComms comm_;
    comm_.connect(SERIAL_PORT_2);
    comm_.writeMotor(0, 0);
    sleep(5);
    std::cout << "\nConnected: " << comm_.connected()<<std::endl;
    comm_.readEncoder(enc1, enc2);
    std::cout << "\n" << enc1 << " "<< enc2<<std::endl;
    comm_.disconnect();
    std::cout << "\nConnected: " << comm_.connected()<<std::endl;
}