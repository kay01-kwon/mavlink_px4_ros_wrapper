#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <cstdlib>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <signal.h>
#include <mavlink/v2.0/common/mavlink.h>
#include <iostream>

using std::cout;
using std::endl;

#ifndef B460800
#define B480800 460800
#endif



class Serial_Port
{
public:

    // Constructor
    Serial_Port();
    Serial_Port(const char *uart_name_, int baudrate_);
    
    // Destructor
    ~Serial_Port();

    int read_message(mavlink_message_t &message);
    int write_message(const mavlink_message_t &message);

    bool is_running(){
        return is_open;
    }

    void start();
    void stop();

private:

    int fd;
    mavlink_status_t lastStatus;
    pthread_mutex_t lock;

    void initialize_defaults();

    bool debug;
    const char *uart_name;
    int baudrate;
    bool is_open;

    int _open_port(const char* port);
    bool _setup_port(int baud, int data_bit, int stop_bits, bool parity, bool hardware_control);
    int _read_port(uint8_t &cp);
    int _write_port(char *buf, unsigned len);

};
#endif