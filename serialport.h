#ifndef __SERIALPORT_H__ 
#define __SERIALPORT_H__ 

#include <termios.h> 

#define flag 1  
#define noflag 0

//ComPort config
#define COM_PORT 0
#define COM_BUFFER_SIZE 5
#define PORT_SPEED 115200
#define PORT_DATABITS	8
#define PORT_PARITY 'N'
#define PORT_STOPBITS 1

class SerialPort {
	public:
		typedef struct Port_info_t 
		{ 
			int fd;
			pthread_mutex_t mt;
			char name[24];
			struct termios ntm;
		} Port_INFO; 
	
	public:
		int InitPort(Port_INFO *pPort, int);
		int cleanPort(Port_INFO *pPort);
		
		int sendnPort(Port_INFO *ptty,char *pbuf,int size); 
	  int signal_recvnPort(Port_INFO *pPort,char *pbuf,int size);
		
	private:
		Port_INFO *readyPort(int id, Port_INFO *pPort);
		int setPortSpeed(Port_INFO *pPort, int port_speed);
		int setPortParity(Port_INFO *pPort,int databits,int parity,int stopbits);
		
		int recvnPort(Port_INFO *ptty,char *pbuf,int size); 
		
		int lockPort(Port_INFO *pPort);
		int unlockPort(Port_INFO *pPort);	
};
#endif // __SERIALPORT_H__


