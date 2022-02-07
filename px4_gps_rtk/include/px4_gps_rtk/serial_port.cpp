#include "serial_port.h"

// Constructor
Serial_Port::Serial_Port()
{
    initialize_defaults();
}

// Override
Serial_Port::Serial_Port(const char *uart_name_, int baudrate_)
{
    initialize_defaults();
    uart_name = uart_name_;
    baudrate = baudrate_;
}

Serial_Port::~Serial_Port()
{
    pthread_mutex_destroy(&lock);
}


void Serial_Port::initialize_defaults()
{
    debug = false;
    fd = -1;
    is_open = false;

    uart_name = (char*) "/dev/ttyUSB0";
    baudrate = 57600;

    int result = pthread_mutex_init(&lock,NULL);
    if(result !=0)
    {
        cout<<"\n mutex init failed\n";
        throw 1;
    }
}

int Serial_Port::read_message(mavlink_message_t &message)
{
    uint8_t cp;
    mavlink_status_t status;
    uint8_t msgReceived = false;

    int result = _read_port(cp);

    if(result > 0)
    {
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1,cp,&message,&status);

        if( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug)
        {
            cout<<"ERROR: Dropped"<<(unsigned)status.packet_rx_drop_count<<" PACKETS\n";
            unsigned char v = cp;
        }
        lastStatus = status;
    }
    else
    {
        cout<<"ERROR: Could not read from fd "<<fd<<endl;
    }

    if(msgReceived && debug)
    {
        cout<<"Received message from serial with ID "<<message.msgid<<" (sys: "<<message.sysid<<" |comp: "<<message.compid<<" )\n";
        unsigned int i;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        // Check message write length
        unsigned int messageLength = mavlink_msg_to_send_buffer(buffer,&message);

        if(messageLength > MAVLINK_MAX_PACKET_LEN)
        {
            cout<<"FATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n";
        }
        else
        {
            for(i = 0; i <messageLength; i++)
            {
                unsigned char v = buffer[i];
            }
        }
    }

    return msgReceived;
}

int Serial_Port::write_message(const mavlink_message_t &message)
{
    char buf[300];

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*) buf, &message);

    // Write buffer to serial port, locks port while wait
    int bytesWritten = _write_port(buf,len);

    return bytesWritten;
}

void Serial_Port::start()
{
    cout<<"Open Port\n";

    fd = _open_port(uart_name);

    if(fd == -1)
    {
        cout<<"Failure, could not open port\n";
        throw EXIT_FAILURE;
    }

    bool success = _setup_port(baudrate, 8, 1, false, false);

    if(!success)
    {
        cout<<"Failure, could not configure port \n";
        throw EXIT_FAILURE;
    }

    if(fd <= 0)
    {
        printf("Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
        throw EXIT_FAILURE;
    }

	// --------------------------------------------------------------------------
	//   CONNECTED!
	// --------------------------------------------------------------------------
	printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	lastStatus.packet_rx_drop_count = 0;

	is_open = true;

	printf("\n");

	return;


}

void Serial_Port::stop()
{
    printf("CLOSED PORT\n");

    int result = close(fd);

    if(result)
    {
        fprintf(stderr,"WARNING: Error on port close (%i)\n", result );   
    }
    is_open = false;

    cout<<"\n";

}

int Serial_Port::_open_port(const char* port)
{
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    	// Check for Errors
	if (fd == -1)
	{
		/* Could not open the port. */
		return(-1);
	}

	// Finalize
	else
	{
		fcntl(fd, F_SETFL, 0);
	}

	// Done!
	return fd;    
}

bool Serial_Port::_setup_port(int baud,int data_bits,int start_bits,bool parity,bool hardware_control)
{
	// Check file descriptor
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}

	// Read file descritor configuration
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
						 ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	////struct termios options;
	////tcgetattr(fd, &options);

	// Apply baudrate
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}

	// Finally, apply the configuration
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}

	// Done!
	return true;

}

int Serial_Port::_read_port(uint8_t &cp)
{
    // Lock
    pthread_mutex_lock(&lock);
    
    int result = read(fd,&cp,1);

    // Unlock
    pthread_mutex_unlock(&lock);

    return result;
}

int Serial_Port::_write_port(char *buf,unsigned len)
{
    // Lock
    pthread_mutex_lock(&lock);

    // Write packet via serial link
    const int bytesWritten = static_cast<int>(write(fd,buf,len));

    // Wait until all data has been written
	tcdrain(fd);

    pthread_mutex_unlock(&lock);

    return bytesWritten;
}