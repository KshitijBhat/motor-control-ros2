#include "pico_comm.hpp"

const std::string SERIAL_PORT_2 = "/dev/ttyACM0" ;

int main()
{
    PicoComms comm_;
    comm_.connect(SERIAL_PORT_2);
    comm_.writeMotor(1000);
    sleep(5);
    std::cout << "\nConnected: " << comm_.connected()<<std::endl;
    std::cout << "\n" << comm_.readEncoder()<<std::endl;
    comm_.disconnect();
    std::cout << "\nConnected: " << comm_.connected()<<std::endl;
}