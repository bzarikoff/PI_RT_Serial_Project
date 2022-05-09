#define _GNU_SOURCE


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/file.h>
#include <string.h>
#include "serial.h"

#define TRUE 1
#define FALSE -1

int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
 B38400, B19200, B9600, B4800, B2400, B1200, B300, };
 
 int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200, 300,
 38400, 19200, 9600, 4800, 2400, 1200, 300, };
 
 //
 // COonfigure the UART on the PI
 //
 int uart_config(int serial_port)
 {
	 // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }  
	 
	 return TRUE;
 }
 
 
 int Transmit(int fd, char *string_to_send)
 {
	 int nwrite;
	 
	 nwrite = write(fd,string_to_send,strlen(string_to_send));
	 printf("string length is %i\n", strlen(string_to_send));
	  if(nwrite < 0)
	  {
			printf("write error\n");
	  }
	  else if(nwrite != strlen(string_to_send))
	  {
		  printf("%i bytes send out of %i!\n", nwrite, strlen(string_to_send));
	  }
	  else if(nwrite == strlen(string_to_send))
	  {
		printf("All %i bytes sent\n", strlen(string_to_send));
	  }
	  else
	  {
		printf("Undefined\n");
	  } 
	  
	  return nwrite;	 	 
	 
 }
 
 
 void set_speed(int fd, int speed)
{
 int i;
 int status;
 struct termios Opt;
 tcgetattr(fd,&Opt);
 for (i= 0;i<sizeof(speed_arr)/sizeof(int);i++)
	 {
	 if(speed == name_arr[i])
		 {
		 tcflush(fd, TCIOFLUSH);
		 cfsetispeed(&Opt, speed_arr[i]);
		 cfsetospeed(&Opt, speed_arr[i]);
		 status = tcsetattr(fd, TCSANOW, &Opt);
		 
		 if(status != 0)
			perror("tcsetattr fd1");
		 return;
		 }
	 tcflush(fd,TCIOFLUSH);
	 }
}


int test_function(void)															
{
	
	printf("testfunction\n");
	return 3;
}

int set_Parity(int fd,int databits,int stopbits,int parity)
{
 struct termios options;
 if( tcgetattr( fd,&options)!= 0)
 {
	 perror("SetupSerial 1");
	 return(FALSE);
 }
 
 options.c_cflag &= ~CSIZE;
 
 switch(databits)
 {
	 case 7:
	 options.c_cflag |= CS7;
	 break;
	 case 8:
	 options.c_cflag |= CS8;
	 break;
	 default:
	 fprintf(stderr,"Unsupported data size\n");
	 return (FALSE);
 }
 
 switch(parity)
 {
	 case 'n':
	 case 'N':
		 options.c_cflag &= ~PARENB; /* Clear parity enable */
		 options.c_iflag &= ~INPCK; /* Enable parity checking */
		 options.c_iflag &= ~(ICRNL|IGNCR);
		 options.c_lflag &= ~(ICANON );
		break;
	 case 'o':
	 case 'O':
		 options.c_cflag |= (PARODD | PARENB);
		 options.c_iflag |= INPCK; /* Disnable parity checking */
		 break;
	 case 'e':
	 case 'E':
		 options.c_cflag |= PARENB; /* Enable parity */
		 options.c_cflag &= ~PARODD;
		 options.c_iflag |= INPCK; /* Disnable parity checking */
		 break;
	 case 'S':
	 case 's': /*as no parity*/
		 options.c_cflag &= ~PARENB;
		 options.c_cflag &= ~CSTOPB;
		 break;
	 default:
		 fprintf(stderr,"Unsupported parity\n");
		 return (FALSE);
 }
 
 switch(stopbits)
 {
	 case 1:
		 options.c_cflag &= ~CSTOPB;
		 break;
	 case 2:
		 options.c_cflag |= CSTOPB;
		 break;
	 default:
		 fprintf(stderr,"Unsupported stop bits\n");
		 return (FALSE);
 }
 /* Set input parity option */
 if(parity != 'n')
 options.c_iflag |= INPCK;
 options.c_cc[VTIME] = 150; // 15 seconds
 options.c_cc[VMIN] = 0;
 tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
 
 if(tcsetattr(fd,TCSANOW,&options) != 0)
 {
	perror("SetupSerial 3");
	return (FALSE);
 }
 return (TRUE);
}
