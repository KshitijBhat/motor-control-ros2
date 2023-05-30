#ifndef PICO_COMM_HPP
#define PICO_COMM_HPP

#include <libserial/SerialPort.h>
#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <iostream>

#define PI 3.1416
#define ENCODER_TICKS_PER_REV 660

class PicoComms{
    public:
        void sendchr(std::string data_byte)
        {
            // Write the data to the serial port.
            serial_port_.Write(data_byte) ;

            // Wait until the data has actually been transmitted.
            serial_port_.DrainWriteBuffer();

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
                serial_port_.Read(read_buffer, 0, ms_timeout) ;
            }
            catch (const LibSerial::ReadTimeout&)
            {
                encoder_val = stoi(read_buffer);
            }
            return encoder_val*2*PI/ENCODER_TICKS_PER_REV;
        }

        int connect(const std::string &serial_device)
        {
            try
            {
                // Open the Serial Port at the desired hardware port.
                serial_port_.Open(serial_device) ;
            }
            catch (const LibSerial::OpenFailed&)
            {
                std::cerr << "The serial port did not open correctly." << std::endl ;
                return EXIT_FAILURE ;
            }

            // Set the baud rate of the serial port.
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200) ;

            // Set the number of data bits.
            serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8) ;

            // Turn off hardware flow control.
            serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE) ;

            // Disable parity.
            serial_port_.SetParity(LibSerial::Parity::PARITY_NONE) ;
            
            // Set the number of stop bits.
            serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1) ;

            return 0;
        }
        void disconnect()
        {
            serial_port_.Close();
        }

        bool connected() const
        {
            return serial_port_.IsOpen();
        }

    private:
        LibSerial::SerialPort serial_port_ ;
        size_t ms_timeout = 10 ;
};

#endif

