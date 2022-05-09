

int test_function(void);
int set_Parity(int fd,int databits,int stopbits,int parity);
void set_speed(int fd, int speed);
int Transmit(int fd, char *string_to_send);
int uart_config(int fd);


enum Serial_state
{
	Transmitting = 0,
	Waiting_for_ack = 1,
	Reading = 2,
	Standby = 3
};
