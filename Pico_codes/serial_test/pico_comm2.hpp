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
        }

        void writeMotor(int motorCommand1, int motorCommand2)
        {
            if(motorCommand1>255){
                motorCommand1 = 255;
            }
            else if(motorCommand1<-255){
                motorCommand1 = -255;
            }
            if(motorCommand2>255){
                motorCommand2 = 255;
            }
            else if(motorCommand2<-255){
                motorCommand2 = -255;
            }
            std::string string_data = std::to_string(motorCommand1) + " " + std::to_string(motorCommand2) + ".";
            sendchr(string_data);
        }

        void readEncoder(float &encoder_val1, float &encoder_val2)
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
                std::string delimiter = "\t";
                size_t del_pos = read_buffer.find(delimiter);
                std::string token_1 = read_buffer.substr(0, del_pos);
                std::string token_2 = read_buffer.substr(del_pos + delimiter.length());

                encoder_val1 = std::atoi(token_1.c_str())*2*PI/ENCODER_TICKS_PER_REV;
                encoder_val2 = std::atoi(token_2.c_str())*2*PI/ENCODER_TICKS_PER_REV;
            }
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