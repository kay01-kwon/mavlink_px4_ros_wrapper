#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <mavlink/v2.0/common/mavlink.h>
#include <px4_gps_rtk/autopilot.h>
#include <px4_gps_rtk/serial_port.h>

char *uart_name = (char*) "/dev/ttyACM0";
int baudrate = 57600;

Serial_Port *port_quit;
Autopilot_Interface *autopilot_interface_quit;

int main()
{
    Serial_Port *port;
    port = new Serial_Port(uart_name,baudrate);

    Autopilot_Interface autopilot_interface(port);

    autopilot_interface.arm_disarm(false);
    usleep(100);

    port->start();
    autopilot_interface.start();

    Mavlink_Messages messages;
    mavlink_highres_imu_t imu;
    mavlink_attitude_t attitude;
    mavlink_global_position_int_t global_position_int;
    mavlink_gps_raw_int_t gps_raw_int;

    while (1)
    {
        messages = autopilot_interface.current_messages;
        imu = messages.highres_imu;
        attitude = messages.attitude;
        
        std::cout<<"Attitude roll"<<messages.attitude.roll<<endl;
        sleep(0.01);
    }

    port_quit->stop();
    autopilot_interface_quit->stop();
}