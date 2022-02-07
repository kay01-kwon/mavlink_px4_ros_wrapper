#ifndef AUTOPILOT_H
#define AUTOPILOT_H
#include <ros/ros.h>
#include "serial_port.h"
#include <signal.h>
#include <iostream>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>
#include <mutex>
#include <mavlink/v2.0/common/mavlink.h>

using std::string;

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);

struct Mavlink_Messages{
    
    int sysid;
    int compid;

    // Heartbeat
    mavlink_heartbeat_t heartbeat;
    
    // System status
    mavlink_sys_status_t sys_status;

    // Battery status
    mavlink_battery_status_t battery_status;

    // Radio Status
    mavlink_radio_status_t radio_status;

    // Local Position
    mavlink_local_position_ned_t local_position;

    // Global Position
    mavlink_global_position_int_t global_position_int;

    // GPS Raw Int
    mavlink_gps_raw_int_t gps_raw_int;

    // HiRes IMU
    mavlink_highres_imu_t highres_imu;

    // Attitude
    mavlink_attitude_t attitude;

    
};


class Autopilot_Interface
{

    public:

    // Constructor
    Autopilot_Interface();
    Autopilot_Interface(Serial_Port *port_);
    ~Autopilot_Interface();

    char reading_status;
    char writing_status;
    char control_status;
    uint64_t write_count;

    int system_id;
    int autopilot_id;
    int companion_id;

    Mavlink_Messages current_messages;
    
    void read_messages();
    int write_messages(mavlink_message_t message);

    int arm_disarm(bool flag);
    void enable_onboard_control();
    void disenable_onboard_control();

    void start();
    void stop();

    void start_read_thread();
    void start_write_thread();

    void handle_quit(int sig);

    private:

    Serial_Port *port;

    bool time_to_exit;

    pthread_t read_tid;
    pthread_t write_tid;

    mavlink_message_t message;

    void read_thread();
    void write_thread();

    int toggle_onboard_control(bool flag);

};

#endif