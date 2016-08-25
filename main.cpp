#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <sys/time.h>

#include "Bluetooth.h"
 
#define BLUETOOTH_CMD_LEN 16
#define RCV_BUF_SIZE 1000
#define DELAY_TIME 2
#define SpeedStep 0x0010

typedef enum {
	SpeedUp,
	SpeedDown,
	Stop,
	TurnLeft,
	TurnRight
}Moto_CMD;
static Moto_CMD cmd;

static struct termios initial_settings, new_settings;  
static int peek_character = -1;  
pthread_t thread[2];
pthread_mutex_t mut;
int fd;
p_bluetooth_frame_t BT_CMD = (p_bluetooth_frame_t)malloc(sizeof(bluetooth_frame));//p_bluetooth_frame_t));
//如果不分配空间，会出现【段错误(核心已转储)】报错
char* remnant = (char*)malloc(16*sizeof(char)) ;
int offset = 0;
p_bluetooth_frame_t RecvBuf = (p_bluetooth_frame_t)malloc(1000*sizeof(bluetooth_frame));
char* buf = (char*)malloc(RCV_BUF_SIZE*sizeof(char)) ;

int set_port(int fd,int  nbits)
{
    struct termios newtio,oldtio;
    if(tcgetattr(fd,&oldtio)!=0)
    {
        perror("pei zhi cuo wu1\n");
        return -1;
    }
 
    bzero(&newtio,sizeof(newtio)); //清零
    newtio.c_cflag |=CLOCAL|CREAD;//用于本地连接和接收使能
 
    newtio.c_cflag &=~CSIZE;//设置数据位
    switch(nbits)
    {
    case 7:
        newtio.c_cflag |=CS7;break;
    case 8:
        newtio.c_cflag |=CS8;break;
    }
 
    //设置奇校验位
        newtio.c_cflag &= ~PARENB; //无奇偶校验
 
    //设置波特率
        cfsetispeed(&newtio,B115200);
        cfsetospeed(&newtio,B115200);
        //cfsetispeed(&newtio,B9600);
        //cfsetospeed(&newtio,B9600);
 
        //设置停止位
        newtio.c_cflag &= ~INPCK;
        newtio.c_cflag &= ~CSTOPB;
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 
   	newtio.c_oflag &= ~OPOST;
    	newtio.c_oflag &= ~(ONLCR | OCRNL);    //添加的
 
    	newtio.c_iflag &= ~(ICRNL | INLCR);
    	newtio.c_iflag &= ~(IXON | IXOFF | IXANY);    //添加的
    	newtio.c_cc[VTIME] = 0;
    	newtio.c_cc[VMIN] = 1;
	tcflush(fd, TCIOFLUSH);
 
        if((tcsetattr(fd,TCSANOW,&newtio))!=0)
        {
            perror("pei zhi cuo wu2\n");
            return -1;
        }
 
        printf("Set port over \n");
        return 0;
    }

 
static int Bluetooth_Write_Serial(void* cmd, int length = BLUETOOTH_CMD_LEN)
{
        int j= write(fd,cmd,length);	
	return 0;
}

static void Read_Handle(char* pack, int size)
{
	 int nBufLen, nIdx, nSize;
	
    	 static char* pRecvBuf = remnant;
   	 static int nStartLen = offset;

	memcpy(pRecvBuf + nStartLen, pack, size);
	
    	nBufLen = nStartLen + size;
	nIdx = 0;

    	while(nBufLen-nIdx >= sizeof(bluetooth_frame))
	{
		RecvBuf = (p_bluetooth_frame_t)(pRecvBuf+nIdx);
		if(RecvBuf->start != start_of_frame)
		{
			nIdx++;
			continue;
		}
		
		nSize = sizeof(bluetooth_frame);

		if(nBufLen-nIdx >= nSize)
		{
			//RobotNB_AckCmd(RecvBuf);
			printf("Received a Package:................................\n");
			printf("start: %d\n", RecvBuf->start );
			printf("pwml: %d\n", RecvBuf->pwm_l);
			printf("pwm_r: %d\n", RecvBuf->pwm_r );
			printf("encoder_l: %d\n", RecvBuf->encoder_l);
			printf("encoder_r: %d\n", RecvBuf->encoder_r );
			printf("speed_l: %d\n", RecvBuf->speed_l);
			printf("speed_r: %d\n", RecvBuf->speed_r );
			printf("end: %d\n", RecvBuf->end);
			nIdx += nSize;
			continue;
		}
		else
		{
			break;
		}
	}

	if(nBufLen-nIdx > 0)
	{
		memcpy(pRecvBuf, pRecvBuf+nIdx, nBufLen-nIdx);
		nStartLen = nBufLen-nIdx;
	}
	else
	{
		nStartLen = 0;
	}
}


static void *Read_Thread(void*)
{
        printf("Read_Thread Begin\n");
	pthread_mutex_lock(&mut);
	while(1)
        {
        //sleep(DELAY_TIME);      
        //pthread_mutex_lock(&mut);
        //printf("Read begin...............");
        int k=read(fd,buf,RCV_BUF_SIZE);
	if(k>0){		
		RecvBuf->start = (int16_t)((int16_t)buf[0] << 8 | (int16_t)buf[1]);
		RecvBuf->pwm_l = (int16_t)buf[2] << 8 | (int16_t)buf[3];
		RecvBuf->pwm_r = (int16_t)buf[4] << 8 | (int16_t)buf[5];
		RecvBuf->encoder_l = (int16_t)buf[6] << 8 | (int16_t)buf[7];
		RecvBuf->encoder_r = (int16_t)buf[8] << 8 | (int16_t)buf[9];
		RecvBuf->speed_l = (int16_t)buf[10] << 8 | (int16_t)buf[11];
		RecvBuf->speed_r = (int16_t)buf[12] << 8 | (int16_t)buf[13];
		RecvBuf->end = (int16_t)buf[14] << 8 | (int16_t)buf[15];		

		printf("Received a Package:................................\n");
		printf("start: %d\n", RecvBuf->start );
		printf("pwml: %d\n", RecvBuf->pwm_l);
		printf("pwm_r: %d\n", RecvBuf->pwm_r );
		printf("encoder_l: %d\n", RecvBuf->encoder_l);
		printf("encoder_r: %d\n", RecvBuf->encoder_r );
		printf("speed_l: %d\n", RecvBuf->speed_l);
		printf("speed_r: %d\n", RecvBuf->speed_r );
		printf("end: %d\n", RecvBuf->end);	
	}
	
	if( k < 0)
		printf("Error while reading !\n");
        //pthread_mutex_unlock(&mut);
        //sleep(1);
        }
	pthread_mutex_unlock(&mut);
	
        printf("thread2 Stop\n");
        pthread_exit(NULL);
 
}
 
 
void thread_create(void)
 {
    int temp;
     memset(thread, 0, 2*sizeof(thread));          //comment1
      /*创建线程*/ 
       if((temp = pthread_create(&thread[1], NULL, Read_Thread, NULL)) != 0)  //comment3
         printf("2 faile\n");
       else
            printf("X2create success\n");
        }
 
void init_keyboard() //配置终端模式为非加工模式

{
    tcgetattr( 0, &initial_settings );
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr( 0, TCSANOW, &new_settings );
}

  
void close_keyboard()  
{  
    tcsetattr(0, TCSANOW, &initial_settings);  
}  
  
int kbhit()  
{  
    unsigned char ch;  
    int nread;  
  
    if (peek_character != -1){
		peek_character = -1;
		return 1;  
    }
    new_settings.c_cc[VMIN]=0;  
    tcsetattr(0, TCSANOW, &new_settings);  
    nread = read(0,&ch,1);  
    new_settings.c_cc[VMIN]=1;  
    tcsetattr(0, TCSANOW, &new_settings);  
    if(nread == 1)   
    {  
        peek_character = ch;  
        return 1;  
    }  
    return 0;  
}  

int readch()

{
    char ch;

    if ( peek_character != -1 )
    {
        ch = peek_character;

        peek_character = -1;

        return(ch);
    }

    read( 0, &ch, 1 );

    return(ch);
}

static void MotoCtrl()//Moto_CMD cmd)//, char* param)
{
	BT_CMD->start = (int16_t)start_of_frame;
	BT_CMD->end = (int16_t)end_of_frame;
	
	BT_CMD->encoder_l = (int16_t)0x0021&0x7FFF;
	BT_CMD->encoder_r = (int16_t)0x0021&0x7FFF;
	BT_CMD->speed_l = (int16_t)0x3500&0x7FFF;
	BT_CMD->speed_r = (int16_t)0x3500&0x7FFF;
		
	switch(cmd)
	{
		case SpeedUp:{ // Choose the side with higher speed, increase the speed and adapt the other side
			printf("Speed Up\n");
			if(BT_CMD->pwm_l  != 0 || BT_CMD->pwm_r != 0)
				BT_CMD->pwm_l  = (BT_CMD->pwm_l  > BT_CMD->pwm_r ? BT_CMD->pwm_l : BT_CMD->pwm_r );
			BT_CMD->pwm_l  = BT_CMD->pwm_l  + SpeedStep;
			BT_CMD->pwm_r = BT_CMD->pwm_l ;
			break;}
		case SpeedDown:// Choose the side with lower speed, increase the speed and adapt the other side
			printf("Speed Down\n");
			if(BT_CMD->pwm_l  != 0 || BT_CMD->pwm_r != 0)
				BT_CMD->pwm_l  = (BT_CMD->pwm_l  < BT_CMD->pwm_r ? BT_CMD->pwm_l : BT_CMD->pwm_r );
			BT_CMD->pwm_l  -= SpeedStep;
			BT_CMD->pwm_r = BT_CMD->pwm_l ;			
			break;
		case TurnLeft:
			printf("Turn Left\n");
			if(BT_CMD->pwm_l  == 0 && BT_CMD->pwm_r == 0)
				BT_CMD->pwm_l  = 10;
			BT_CMD->pwm_r  = (BT_CMD->pwm_l  > BT_CMD->pwm_r ? BT_CMD->pwm_l : BT_CMD->pwm_r );
			BT_CMD->pwm_l = -(BT_CMD->pwm_r );
			break;
		case TurnRight:
			printf("Turn Right\n");
			if(BT_CMD->pwm_l  == 0 && BT_CMD->pwm_r == 0)
				BT_CMD->pwm_l  = 10;
			BT_CMD->pwm_l  = (BT_CMD->pwm_l  > BT_CMD->pwm_r ? BT_CMD->pwm_l : BT_CMD->pwm_r );
			BT_CMD->pwm_r = -(BT_CMD->pwm_l);			
			break;
		case Stop:
			BT_CMD->pwm_l  = 0;
			BT_CMD->pwm_r = 0;
			break;
		default:
			break;
	}
	
	//using speed instead of pwm

	BT_CMD->speed_l = BT_CMD->pwm_l;
	BT_CMD->speed_r = BT_CMD->pwm_r;
	
	char bluetooth_tx_buffer[16];
	bluetooth_tx_buffer[0] = BT_CMD->start >> 8;
	bluetooth_tx_buffer[1] = BT_CMD->start & 0x00FF;
	bluetooth_tx_buffer[2] = BT_CMD->pwm_l >> 8;
	bluetooth_tx_buffer[3] = BT_CMD->pwm_l & 0x00FF;
	bluetooth_tx_buffer[4] = BT_CMD->pwm_r >> 8;
	bluetooth_tx_buffer[5] = BT_CMD->pwm_r & 0x00FF;
	bluetooth_tx_buffer[6] = BT_CMD->encoder_l >> 8;
	bluetooth_tx_buffer[7] = BT_CMD->encoder_l & 0x00FF;
	bluetooth_tx_buffer[8] = BT_CMD->encoder_r >> 8;
	bluetooth_tx_buffer[9] = BT_CMD->encoder_r & 0x00FF;
	bluetooth_tx_buffer[10] = BT_CMD->speed_l >> 8;
	bluetooth_tx_buffer[11] = BT_CMD->speed_l & 0x00FF;
	bluetooth_tx_buffer[12] = BT_CMD->speed_r >> 8;
	bluetooth_tx_buffer[13] = BT_CMD->speed_r & 0x00FF;
	bluetooth_tx_buffer[14] = BT_CMD->end >> 8;
	bluetooth_tx_buffer[15] = BT_CMD->end & 0x00FF;
	
	/*BT_CMD->pwm_l = ((BT_CMD->pwm_l & 0x00FF)<<8 | (BT_CMD->pwm_l & 0xFF00));
	BT_CMD->pwm_r = ((BT_CMD->pwm_r & 0x00FF)<<8 | (BT_CMD->pwm_r & 0xFF00));
	BT_CMD->speed_l = BT_CMD->pwm_l;
	BT_CMD->speed_r = BT_CMD->pwm_r;*/
	
	Bluetooth_Write_Serial(bluetooth_tx_buffer,10); //BT_CMD
	usleep(20000);
	//sleep(1);
	Bluetooth_Write_Serial(bluetooth_tx_buffer+10,6); //&(BT_CMD->speed_l)
}



int main(void) {
        int i;
	int ch;
	//printf("%d\n",sizeof(int16_t));
        fd=open("/dev/ttyUSB0",O_RDWR|O_NOCTTY);
        if(-1==fd){
		printf("it can't open the communitation\n");
		return -1;
        }
        i=set_port(fd, 8);
        if(i<0)
        {
            perror("Error while configuring !\n");
            return 0;
        }
	init_keyboard();  
        pthread_mutex_init(&mut,NULL);
        printf("creat thread\n");
        thread_create();
        printf("Program On..... \n");
        while(ch != 27)
	{
		if(kbhit()){
			ch = readch();			
			/*printf("Key number is: %d\n",ch);
			printf("Key is: %c\n",ch);*/
			switch(ch)
			{
				case 'w':
					cmd = SpeedUp;
					break;
				case 's':
					cmd = SpeedDown;
					break;
				case 'a':
					cmd = TurnLeft;
					break;
				case 'd':
					cmd = TurnRight;
					break;
				case 't':
					cmd = Stop;
					break;
				default:
					break;
			}
			MotoCtrl();//,param);
		}
        }
        close(fd);
	close_keyboard();
        return 0; 
}
