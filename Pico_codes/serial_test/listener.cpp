/**
 *  @example serial_port_write.cpp
 */

#include <libserial/SerialPort.h>
#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <iostream>

constexpr const char* const SERIAL_PORT_2 = "/dev/ttyACM0" ;
using namespace LibSerial ;
// Instantiate a SerialPort object.
SerialPort serial_port ;
size_t ms_timeout = 250 ;

/**
 * @brief This example reads the contents of a file and writes the entire 
 *        file to the serial port one character at a time. To use this
 *        example, simply utilize TestFile.txt or another file of your
 *        choosing as a command line argument.
 */

void readEncoder()
{
    DataBuffer read_buffer ;

    try
    {
        // Read as many bytes as are available during the timeout period.
        serial_port.Read(read_buffer, 0, ms_timeout) ;
    }
    catch (const ReadTimeout&)
    {
        for (size_t i = 0 ; i < read_buffer.size() ; i++)
        {
            std::cout << read_buffer.at(i) << std::flush ;
        }

        std::cerr << "The Read() call timed out waiting for additional data." << std::endl ;
    }
}

int main()
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

    // Read characters from the input file and write them to the serial port. 
    std::cout << "Writing input file contents to the serial port." << std::endl ;
    
    // Create a variable to store data from the input file and write to the
    // serial port.
    
    readEncoder();
    
    // Successful program completion.
    std::cout << "The example program successfully completed!" << std::endl ;
    return EXIT_SUCCESS ;
}