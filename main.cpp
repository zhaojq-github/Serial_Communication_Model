#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include "serialport.h" 

int main()
{
	SerialPort serialport;
	SerialPort::Port_INFO *pPort;
	/* Init SerialPort */
	int COM_id = 0;
	pPort = (SerialPort::Port_INFO *)malloc(sizeof(SerialPort::Port_INFO)); 
	if(pPort == NULL) {
		fprintf(stderr, "Error malloc...  [FAIL]\n");
	}
	memset(pPort, 0, sizeof(SerialPort::Port_INFO)); 
	int result = serialport.InitPort(pPort, COM_id); 
	if(result < 0) { 
		fprintf(stderr, "Error Running SerialPort...  [FAIL]\n"); 
	}
	fprintf(stdout, "SerialPort [%s] running...  [OK]\n", pPort->name); 
		
	/* COM Send */
	const char* serial_sendmsg = "COM Send Message";
	int msg_len = strlen(serial_sendmsg);	
	int send_nbyte = serialport.sendnPort(pPort,const_cast<char*>(serial_sendmsg),msg_len);
	if (send_nbyte > 0) {
		fprintf(stdout, "COM_Send: [%d] %s...  [OK]\n", send_nbyte, serial_sendmsg);
	}
	
	/* COM Receive */
	int receive_nbyte = 0;
	int size = COM_BUFFER_SIZE;
	char response_buf[COM_BUFFER_SIZE] = {'\0'};
	while (1) {
		receive_nbyte = serialport.signal_recvnPort(pPort, response_buf, size);
		if(receive_nbyte > 0) {
			for(int i = 0; i < receive_nbyte; i++) {
				response_buf[i] = response_buf[i];
			}
			response_buf[receive_nbyte] = '\0';
		}
		if (receive_nbyte == 0) break;
		fprintf(stdout, "COM_Receive: [%d] %s...  [OK]\n", receive_nbyte, response_buf);  
	}
	return 0;
}
