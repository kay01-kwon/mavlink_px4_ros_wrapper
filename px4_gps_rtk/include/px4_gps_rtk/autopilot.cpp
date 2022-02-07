#include "autopilot.h"

// Constructor
Autopilot_Interface::Autopilot_Interface(Serial_Port *port_)
{
    write_count = 0;

    reading_status = 0;
    writing_status = 0;
    control_status = 0;
    time_to_exit = false;

    read_tid = 0;
    write_tid = 0;

    system_id = 0;
    autopilot_id = 0;
    companion_id = 0;

    current_messages.sysid = system_id;
    current_messages.compid = autopilot_id;

    port = port_;
}

// Destructor
Autopilot_Interface::~Autopilot_Interface()
{

}

void Autopilot_Interface::read_messages()
{
    bool success;
    bool received_all = false;

    success = port->read_message(message);


    while(!time_to_exit && received_all)
    {
        if(success)
        {
            current_messages.sysid = message.sysid;
            current_messages.compid = message.compid;

            switch (message.msgid)
            {

                case MAVLINK_MSG_ID_BATTERY_STATUS:
                {
                    mavlink_msg_battery_status_decode(&message,&(current_messages.battery_status));
                    break;
                }

                case MAVLINK_MSG_ID_RADIO_STATUS:
                {
                    mavlink_msg_radio_status_decode(&message,&(current_messages.radio_status));
                    break;
                }
                
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    mavlink_msg_global_position_int_decode(&message,&(current_messages.global_position_int));
                    break;
                }

                case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    mavlink_msg_gps_raw_int_decode(&message,&(current_messages.gps_raw_int));
                    break;
                }

                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    mavlink_msg_highres_imu_decode(&message,&(current_messages.highres_imu));
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    mavlink_msg_attitude_decode(&message,&(current_messages.attitude));
                    break;
                }
            
                default:
                    break;
            } // End of switch msgid
        } // End of if read message

        received_all = true;
        if(writing_status > false)
            usleep(100);
    } // End of While

    return;
}

int Autopilot_Interface::write_messages(mavlink_message_t message)
{

    int len = port->write_message(message);

    write_count++;
    return len;
}

void Autopilot_Interface::enable_onboard_control()
{
    if(control_status == false)
    {
        cout<<"Enable Onboard Mode\n";

        int success = toggle_onboard_control(true);

        if(success)
            control_status = true;
        else
            fprintf(stderr,"Error: on-board mode not set, could not write message\n");
    
    printf("\n");
    }
}

void Autopilot_Interface::disenable_onboard_control()
{
    if(control_status == true)
    {
        cout<<"Disable On board Mode\n";

        int successs = toggle_onboard_control(false);

        if(successs)
            control_status = false;
        else
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
    
        printf("\n");
    }
}

int Autopilot_Interface::arm_disarm(bool flag)
{
    if(flag)
        printf("ARM ROTORS \n");
    else
        printf("DISARM ROTORS \n");

    mavlink_command_long_t com = {0};
    com.target_system = system_id;
    com.target_component = autopilot_id;
    com.command = MAV_CMD_COMPONENT_ARM_DISARM;
    com.confirmation = true;
    com.param1 = (float) flag;
    com.param2 = 21196;

    mavlink_msg_command_long_encode(system_id,companion_id,&message,&com);

    int len = port->write_message(message);

    return len;
}

int Autopilot_Interface::toggle_onboard_control(bool flag)
{

    mavlink_set_mode_t set_mode = {0};
    set_mode.target_system = system_id;
    set_mode.custom_mode = MAV_MODE_AUTO_DISARMED;
    
    mavlink_msg_set_mode_encode(system_id,companion_id,&message,&set_mode);
    
    int len = port->write_message(message);

    return len;
}

void Autopilot_Interface::start()
{
    int result;

    if(!port->is_running())
    {
        fprintf(stderr,"ERROR: port not open\n");
		throw 1;
    }

    printf("START READ THREAD\n");

    result = pthread_create(&read_tid,NULL,&start_autopilot_interface_read_thread,this);

    if(result == 0) 
        printf("Success to create thread\n");
    else
        printf("Fail to create thread\n");

    
    cout<<"CHECK FOR MESSAGES"<<endl;
    
    while ( not current_messages.sysid )
    {
        printf("%d",current_messages.sysid);
        if (time_to_exit)
            return;
        usleep(500000);
    }

    printf("Found \n");

    // System ID
    if(not system_id)
    {
        system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );   
    }
    
    // Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;
}

void Autopilot_Interface::stop()
{
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the port separately    
}


void Autopilot_Interface::start_read_thread()
{
    if(reading_status !=0 )
    {
		fprintf(stderr,"read thread already running\n");
		return;        
    }
    else
    {
        read_thread();
        return;
    }
}

void Autopilot_Interface::start_write_thread()
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}    
}

void Autopilot_Interface::handle_quit(int sig)
{
    disenable_onboard_control();

    try{
        stop();
    }
    catch(int error)
    {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");        
    }

}

void Autopilot_Interface::read_thread()
{
    reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}

void Autopilot_Interface::write_thread()
{
    writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
	}

	// signal end
	writing_status = false;

	return;
}

void *start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;    
}

void *start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;    
}