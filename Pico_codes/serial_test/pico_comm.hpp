#include <libserial/SerialPort.h>
#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <iostream>

#define PI 3.1416
#define ENCODER_TICKS_PER_REV 660


constexpr const char* const SERIAL_PORT_2 = "/dev/ttyACM0" ;
using namespace LibSerial ;
// Instantiate a SerialPort object.
SerialPort serial_port ;
size_t ms_timeout = 10 ;

void sendchr(std::string data_byte)
{
    // Write the data to the serial port.
    serial_port.Write(data_byte) ;

    // Wait until the data has actually been transmitted.
    serial_port.DrainWriteBuffer();

    // Print to the terminal what is being written to the serial port.
    std::cout << data_byte;
}

void writeMotor(int number)
{
    if(number>255){
        number = 255;
    }
    else if(number<-255){
        number = -255;
    }
    std::string string_data = std::to_string(number) + ".";
    sendchr(string_data);
}

float readEncoder()
{
    std::string read_buffer ;
    sendchr("\r");
    int encoder_val;
    try
    {
        // Read as many bytes as are available during the timeout period.
        serial_port.Read(read_buffer, 0, ms_timeout) ;
    }
    catch (const ReadTimeout&)
    {
        encoder_val = stoi(read_buffer);
    }
    return encoder_val*2*PI/ENCODER_TICKS_PER_REV;
}

int setupSerialComm()
{
    try
    {
        // Open the Serial Port at the desired hardware port.
        serial_port.Open(SERIAL_PORT_2) ;
    }
    catch (const OpenFailed&)
    {
        std::cerr << "The serial port did not open correctly." << std::endl ;
        return EXIT_FAILURE ;
    }

    // Set the baud rate of the serial port.
    serial_port.SetBaudRate(BaudRate::BAUD_115200) ;

    // Set the number of data bits.
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;

    // Turn off hardware flow control.
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;

    // Disable parity.
    serial_port.SetParity(Parity::PARITY_NONE) ;
    
    // Set the number of stop bits.
    serial_port.SetStopBits(StopBits::STOP_BITS_1) ;

    return 0;
}

