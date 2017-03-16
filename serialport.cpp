// system header files
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h>
#include <string.h>
#include <sys/types.h> 
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <sys/file.h>
#include <fcntl.h> 
#include <errno.h> 
#include <pthread.h>
#include <iostream>
using namespace std; 

// user-defined header files
#include "likely.h" 
#include "serialport.h" 

const int SLEEP_TIME_INTERVAL = 2;
const int RETRY_INIT_INTERVAL = 5;
int wait_flag = noflag;

SerialPort::Port_INFO *SerialPort::readyPort(int id, Port_INFO *pPort) 
{
	pthread_mutex_init(&pPort->mt,NULL); 
	/*Ubuntu虚拟机
	 *sudo gpasswd --add zhaojq dialout
	 *logout
	 *开启open /dev/ttyS0权限
	*/
	sprintf(pPort->name,"/dev/ttyS%d",id);
	/*A8开发板*/
	//sprintf(pPort->name,"/dev/s3c2410_serial%d",id); 
	
	pPort->fd = open(pPort->name, O_RDWR | O_NOCTTY | O_NDELAY); 
	if (unlikely(pPort->fd < 0)) {
		fprintf(stderr, "Open Serial [%s] Error...  [FAIL]\n", pPort->name); 
		free(pPort); 
		return NULL; 
	}
	return pPort; 
} 

int SerialPort::cleanPort(Port_INFO *pPort) 
{ 
	if(likely(pPort->fd > 0)) { 
		close(pPort->fd); 
		pPort->fd = -1; 
		free(pPort); 
		pPort = NULL; 
	}
	return 0; 
} 

/*c_cflag:控制模式标志,指定终端硬件控制信息
 *	CLOCAL:设置,则忽略调制解调器线路状态,即该设备是本地连接
 *	CREAD:设置,则启用接收装置,可以接收字符
*/
int SerialPort::setPortSpeed(Port_INFO *pPort, int port_speed) 
{ 
	/*tcgetattr:用于获取终端相关的参数*/
	if(unlikely(tcgetattr(pPort->fd,&pPort->ntm) != 0)) { 
		fprintf(stderr, "Set Serial [%s] Speed Error...  [FAIL]\n",pPort->name);
		return -1; 
	}

	int speed = 0;
	switch(port_speed) 
	{ 
		case 300: 
			speed = B300; 
			break; 
		case 1200: 
			speed = B1200; 
			break; 
		case 2400: 
			speed = B2400; 
			break; 
		case 4800: 
			speed = B4800; 
			break; 
		case 9600: 
			speed = B9600; 
			break; 
		case 19200: 
			speed = B19200; 
			break; 
		case 38400: 
			speed = B38400; 
			break; 
		case 115200: 
			speed = B115200; 
			break;
		default: 
			fprintf(stderr, "Unsupported Port Speed...  [FAIL]\n");
			return -1; 
	} 
	/*设置输入波特率*/
	cfsetispeed(&pPort->ntm, speed);
	/*设置输出波特率*/
	cfsetospeed(&pPort->ntm, speed);
	pPort->ntm.c_cflag |= (CLOCAL | CREAD);	//Enable the receiver and set local mode
	/*tcsetattr:用于设置终端相关的参数
	 *	TCSANOW:不等数据传输完毕就立即改变属性
	 *	TCSADRAIN:等待所有数据传输完毕才改变属性
	 *	TCSAFLUSH:清空输入输出缓冲区才改变属性
	*/
	tcsetattr(pPort->fd,TCSANOW,&pPort->ntm);
	fprintf(stdout, "Set SerialPort Speed:[%d]...  [OK]\n", port_speed); 
	return 0; 
} 

/*
 *c_cflag:
 *	CSIZE:字节大小屏蔽,指明发送和接收的每个字节的位数(长度不包括奇偶校验位).取值范围为CS5,CS6,CS7,CS8
 *	PARENB:控制奇偶性的产生和检测.设置,则对输出字符产生奇偶位,对输入字符执行奇偶性校验
 *	PARODD:设置,则输出和输入字符的奇偶性都是奇;否则为偶
 *	CSTOPB:设置两个停止位;否则为一个停止位
 *	CRTSCTS:使用RTS/CTS流控制
 *c_iflag:输入模式标志,控制终端输入方式
 *	INPCK:设置,则输入奇偶校验起作用;否则不起作用
 *c_oflag:输出模式标志,控制终端输出方式
 *	OPOST:设置,则执行实现定义的输出处理
 *c_lflag:本地模式标志,控制终端编辑功能
 *	ICANON:使用标准输入模式
 *	ECHO:设置,则将输入字符回送到终端设备
 *	ECHOE:可见擦除符
 *	ISIG:设置,则当输入INTR,OUIT,SUSP时,产生相应信号
*/
int SerialPort::setPortParity(Port_INFO *pPort,int databits,int parity,int stopbits) 
{ 
	if(unlikely(tcgetattr(pPort->fd,&pPort->ntm) != 0)) { 
		fprintf(stderr, "Set Serial [%s] Parity Error...  [FAIL]\n",pPort->name);
		return -1; 
	}
	pPort->ntm.c_cflag &= ~CSIZE;
	/*设置数据位*/
	switch (databits) 
	{ 
		case 7: 
			pPort->ntm.c_cflag |= CS7; 
			break; 
		case 8: 
			pPort->ntm.c_cflag |= CS8; 
			break; 
		default: 
			fprintf(stderr, "Unsupported Port Data bits...  [FAIL]\n");
			return -1; 
	} 
	/*设置奇偶校验位*/
	switch (parity) 
	{ 
		/*No parity 无奇偶校验位*/
		case 'n': 
		case 'N': 
			pPort->ntm.c_cflag &= ~PARENB;	/* Clear parity enable */ 
			pPort->ntm.c_iflag &= ~INPCK; 	/* Enable parity checking */ 
			break;
		
		/*Odd parity 奇校验*/ 
		case 'o': 
		case 'O':
			pPort->ntm.c_cflag |= (PARODD | PARENB);
			pPort->ntm.c_iflag |= INPCK; 	/* Disnable parity checking */ 
			break;
		/*Even parity 偶校验*/	 
		case 'e': 
		case 'E': 
			pPort->ntm.c_cflag |= PARENB; 	/* Enable parity */ 
			pPort->ntm.c_cflag &= ~PARODD;
			pPort->ntm.c_iflag |= INPCK; 
			break; 
		case 'S': 
		case 's': /*as no parity*/ 
			pPort->ntm.c_cflag &= ~PARENB; 
			pPort->ntm.c_cflag &= ~CSTOPB; 
			break; 
		default: 
			fprintf(stderr, "Unsupported Port Parity...  [FAIL]\n");
			return -1; 
	} 
	/*设置停止位*/
	switch (stopbits) 
	{ 
		case 1: 
			pPort->ntm.c_cflag &= ~CSTOPB; 
			break; 
		case 2: 
			pPort->ntm.c_cflag |= CSTOPB; 
			break; 
		default: 
			fprintf(stderr, "Unsupported Port Stop bits...  [FAIL]\n");
			return -1; 
	}
	pPort->ntm.c_cflag &= ~CRTSCTS;	// disable hardware flow control
	pPort->ntm.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
	pPort->ntm.c_oflag  &= ~OPOST;
	/*tcflush:清空终端未完成的输入/输出请求及数据
	 *	TCIFLUSH:刷清输入队列
	 *	TCOFLUSH:刷清输出队列
	 *	TCIOFLUSH:刷清输入/输出队列
	*/
	tcflush(pPort->fd, TCIFLUSH); 
	/*等待时间(百毫秒)*/
	pPort->ntm.c_cc[VTIME] = 5;	
	/*等待的最小字节数*/
	pPort->ntm.c_cc[VMIN] = 1;
	tcsetattr(pPort->fd,TCSANOW,&pPort->ntm);
	fprintf(stdout, "Set SerialPort Data bits:[%d], Parity:[%c], Stop bits:[%d]...  [OK]\n", databits, parity, stopbits);
	return 0; 
} 

int SerialPort::InitPort(Port_INFO *pPort, int COM_id)
{	
	int i=0;
	while(RETRY_INIT_INTERVAL - i) {
		if(COM_id < 0) COM_id = COM_PORT;
		pPort = readyPort(COM_id, pPort); 
		if(unlikely(pPort == NULL)) {
			sleep(SLEEP_TIME_INTERVAL); i++; continue;
		}
		fprintf(stdout, "Ready Serial [%s]...  [OK]\n",pPort->name); 

		lockPort(pPort);
		bzero(&pPort->ntm, sizeof(pPort->ntm)); 
		if(unlikely(setPortSpeed(pPort, PORT_SPEED) < 0)) {
			sleep(SLEEP_TIME_INTERVAL); i++; continue;
		}

		if(unlikely(setPortParity(pPort, PORT_DATABITS, PORT_PARITY, PORT_STOPBITS) < 0)) {
			sleep(SLEEP_TIME_INTERVAL); i++; continue;
		}
		unlockPort(pPort);
		return 0;
	}
	return -1;
}

void signal_handler_IO (int status)  
{
	wait_flag = noflag;
}

int SerialPort::recvnPort(SerialPort::Port_INFO *pPort,char *pbuf,int size) 
{ 
	int ret,left;
	char *ptmp;

	ret = 0;
	left = size;
	ptmp = pbuf;

	while(left > 0) {
		pthread_mutex_lock(&pPort->mt); 
		ret = read(pPort->fd,ptmp,left); 
		pthread_mutex_unlock(&pPort->mt); 
		if(likely(ret > 0)) {
			left -= ret; 
			ptmp += ret; 
		} else if(ret <= 0) {
			tcflush(pPort->fd, TCIOFLUSH);
			continue;
		} 
	}
	return size - left; 
}

int SerialPort::signal_recvnPort(Port_INFO *pPort,char *pbuf,int size)
{
	int nread = 0;
	struct sigaction saio;
	saio.sa_handler = signal_handler_IO;  
	sigemptyset (&saio.sa_mask);  
	saio.sa_flags = 0;  
	saio.sa_restorer = NULL;
	/*
	 *应用层启用异步通知:
	 *当设备可写时,设备驱动函数发送一个信号给内核,告知内核有数据可读,在条件不满足之前,并不会造成阻塞
	*/
	//设置对信号的处理操作程序,SIGIO:文件描述符准备就绪,可以开始进行输入/输出操作
	sigaction(SIGIO, &saio, NULL);  
	//指定一个进程作为文件的"属主"
	fcntl(pPort->fd, F_SETOWN, getpid ());  
	//在设备文件中添加FASYNC标志
	int f_flags = fcntl(pPort->fd, F_GETFL);
	fcntl(pPort->fd, F_SETFL, f_flags|FASYNC); 

	while (wait_flag == noflag) { 
		memset (pbuf, 0, size);  
		nread = recvnPort(pPort, pbuf, size); 
		wait_flag = flag; /*wait for new input */  
	}
	return nread;
}

int SerialPort::sendnPort(SerialPort::Port_INFO *pPort,char *pbuf,int size) 
{
	int ret,left; 
	char *ptmp; 

	ret = 0; 
	left = size; 
	ptmp = pbuf;

	while(left > 0) { 
		pthread_mutex_lock(&pPort->mt);
		ret = write(pPort->fd,ptmp,left);
		pthread_mutex_unlock(&pPort->mt); 
		if(likely(ret > 0)) { 
			left -= ret; 
			ptmp += ret; 
		} else if(ret <= 0) {
			tcflush(pPort->fd, TCIOFLUSH);
			continue;
		} 
	}
	return size - left; 
} 

int SerialPort::lockPort(Port_INFO *pPort) 
{ 
	if(unlikely(pPort->fd < 0)) { 
		return 1; 
	}
	/*LOCK_EX:建立互斥锁定*/
	return flock(pPort->fd,LOCK_EX); 
}

int SerialPort::unlockPort(Port_INFO *pPort) 
{ 
	if(unlikely(pPort->fd < 0)) { 
		return 1; 
	}
	/*LOCK_UN：解除文件锁定*/
	return flock(pPort->fd,LOCK_UN); 
}
