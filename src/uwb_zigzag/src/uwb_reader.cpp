// ====== ROS ======
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PoseStamped.h"

#include "serial/serial.h"
#include <iostream>
#include <fstream>

#include <math.h>
#include <string>

#define MSG_BUF_CONTR  10
#define MSG_BUF_SIZE   105

/************************************¿ÉÐÞ¸Ä***************************************/
#define  IMU_BUF_CONTR   0
#define  IMU1_BUF_CONTR  1
#define  IMU2_BUF_CONTR  2
#define  UWB_BUF_CONTR   3

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned char u8;

ros::Publisher uwb_pub;

/********************************************************************************/
typedef  struct
{
	uint8_t       *Start; 
	uint8_t       *QEnd;  
	uint8_t       *OSQIn;
	uint8_t       *OSQOut; 
	uint8_t       OSQSize; 
	uint8_t     OSQEntries;
} buf_type; 


typedef  struct
{
	buf_type  buf_contr_op;
	uint8_t   msgbuf[MSG_BUF_SIZE];

}MsgContr_op;

typedef  struct
{
	void(*msg_init)(buf_type * _os, uint8_t  *_Event, uint8_t buf_size);  //³õÊ¼»¯»º´æ
	uint8_t(*msg_read)(buf_type * _os, uint8_t *p);  //¶Á»º´æÒ»¸ö×Ö½Ú
	void(*msg_write)(buf_type * _os, uint8_t *p); //Ð´Èë»º´æÒ»¸ö×Ö½Ú

}MsgQueue_t;

typedef enum { EMPTY = 3, OK = 1, FULL = 2 }buf;

#define UWB_BUF_SIZE 56

#define UWB_DATA_VALID   1
#define UWB_DATA_INVALID 0
typedef struct
{
	void(*Handle)(void);
	char mid[2];   //message class  
	uint8_t mask;  //ÑÚÂë
	float Range0;  //±êÇ©µ½»ùÕ¾0µÄ¾àÀë
	float Range1;  //±êÇ©µ½»ùÕ¾1µÄ¾àÀë
	float Range2;  //±êÇ©µ½»ùÕ¾2µÄ¾àÀë
	float Range3;  //±êÇ©µ½»ùÕ¾3µÄ¾àÀë
	uint8_t AT;    //TÊÇ±êÇ©,AÊÇID
	uint8_t flag;  //Êý¾ÝÓÐÐ§ÐÔ  1 ÓÐÐ§  0 ÎÞÐ§

}UWB_TypeDef;


static void  buf_init(buf_type * _os, uint8_t  *_Event, uint8_t buf_size);
static uint8_t   buf_read(buf_type * _os, uint8_t *p);
static void  buf_write(buf_type * _os, uint8_t *p);

MsgQueue_t MsgQueue =
{
   buf_init,
   buf_read,
   buf_write
};

MsgContr_op MsgContr[MSG_BUF_CONTR]; 

/**********************************************************/
static void  buf_init(buf_type * _os, uint8_t  *_Event, uint8_t buf_size)  //
{
	uint8_t i;
	for (i = 0; i < buf_size; i++)
	{
		_Event[i] = 0;
	}

	_os->Start = _Event;
	_os->QEnd = _Event + buf_size;
	_os->OSQIn = _Event;
	_os->OSQOut = _Event;
	_os->OSQSize = buf_size;
	_os->OSQEntries = 0;
}

static uint8_t  buf_rxnum(buf_type *p)
{
	if (p->OSQOut > p->OSQIn) {
		return   (p->QEnd - p->OSQOut) + (p->OSQIn - p->Start);
	}
	if (p->OSQOut < p->OSQIn) {
		return  p->OSQIn - p->OSQOut;
	}
	return 0;
}

static uint8_t   buf_read(buf_type * _os, uint8_t *p)
{
	if (buf_rxnum(_os) > 0) {  //ÓÐÊý¾ÝÔÊÐí¶Á
		*p = *((_os->OSQOut)++);
		if (_os->OSQOut == _os->QEnd) {
			_os->OSQOut = _os->Start;

		}
		return OK;
	}
	return EMPTY;
}


static void  buf_write(buf_type * _os, uint8_t *p)
{
	if (_os->OSQIn == _os->QEnd)
		_os->OSQIn = _os->Start;
	*((_os->OSQIn)++) = *p;
	if (_os->OSQIn == _os->QEnd)
		_os->OSQIn = _os->Start;
}

 void UART1_IRQHandler(u8 data)
{
	{
		MsgQueue.msg_write(&MsgContr[UWB_BUF_CONTR].buf_contr_op, &data);
	}
}

 static void Uwb_Handle(void);
 UWB_TypeDef  uwb =
 {
	Uwb_Handle
 };

 static uint8_t buff[42] = { EMPTY, };//µÚÒ»¸ö×Ö½Ú´æ·Å¸½¼ÓÐÅÏ¢£¬ËµÃ÷¶ÁÈ¡ÐÅÏ¢µÄÓÐÐ§ÐÔ£¬¶þ¼¶´æ´¢Æ÷

 static uint8_t * read_uwb_buf(void)
 {
	 uint8_t data, temp;
	 uint16_t i;
	 static uint8_t index = 0;
	 static uint8_t count = 0;

	 if ((*buff) == EMPTY)
	 {  //ËµÃ÷µ±Ç°»¹Ã»ÓÐ´Ó»º´æÖÐÈ¡Êý¾Ý»òÕßÈ¡µÃµÄÊý¾Ý²»ÍêÕû,ÐèÒª¼ÌÐø
		 
		 if (*(buff + 1) != 'm')
		 {     //»¹Ã»ÓÐÕÒµ½ ±¨Í·£¬ËÑË÷±¨Í·
			//  ROS_INFO("*(buff + 1) is %d",*(buff + 1));
			 for (i = 0; i < UWB_BUF_SIZE; i++)
			 {
				 temp = MsgQueue.msg_read(&MsgContr[UWB_BUF_CONTR].buf_contr_op, &data);
				 
				 if (temp == EMPTY)     //Èç¹ûËÑÍê»º´æÖÐµÄËùÓÐ´æ´¢µÄÐÅÏ¢¶¼Ã»ÓÐÕÒµ½±¨Í·£¬±¾´ÎËÑË÷Ê§°Ü¡£
					 return  buff;
				 if (data == 'm')
				 {
					 *(buff + 1) = data;        //¶Á³öµÄ±¨Í·Ð´Èëµ½¶þ¼¶»º´æ
					 index = 40;
					 break;                //Ìø³öÑ°ÕÒ±¨Í·£¬¼ÌÐøÑ­ÕÒÊ£ÏÂµÄÊý¾Ý
				 }
			 }
		 }

		 for (; index > 0;)
		 {             //¶ÁÈ¡ÓàÏÂµÄ×Ö½Ú
			 temp = MsgQueue.msg_read(&MsgContr[UWB_BUF_CONTR].buf_contr_op, &data);
			 if (temp == EMPTY)        //µ±Ç°»º´æÖÐÃ»ÓÐ¿É¶ÁÊý¾ÝÁË
				 return  buff;
			 *(buff + 2 + count++) = data;
			 index--;
			 if (index == 0)
			 {
				 count = 0;
				 //				if(CRC_Check(buff1))
				 {
					 *buff = FULL;           //¶þ¼¶´æ´¢Æ÷Í·²¿ÐÅÏ¢
					 return  buff;
				 }
				 for (i = 1; i < 42; i++) //Ð£ÑéÍ¨²»¹ý,ÖØÐÂÀ´¹ý
					 buff[i] = 0;
				 buff[0] = EMPTY;
				 return  buff;
			 }
		 }
	 }
 }

 static void Uwb_Handle(void)
 {
	 uint8_t *buf;
	 uint16_t i;

	 buf = read_uwb_buf();

	 if (*buf == FULL) 
	 {
		 uwb.mid[0] = buf[1];
		 uwb.mid[1] = buf[2];
		 uwb.mask = (buf[4] - 0x30) * 10 + buf[5] - 0x30;
		 uwb.Range0 = ((buf[8] - 0x30) * 10000 + (buf[9] - 0x30) * 1000 + (buf[10] - 0x30) * 100 + (buf[11] - 0x30) * 10 + (buf[12] - 0x30)) / 1000.0;
		 uwb.Range1 = ((buf[15] - 0x30) * 10000 + (buf[16] - 0x30) * 1000 + (buf[17] - 0x30) * 100 + (buf[18] - 0x30) * 10 + (buf[19] - 0x30)) / 1000.0;
		 uwb.Range2 = ((buf[22] - 0x30) * 10000 + (buf[23] - 0x30) * 1000 + (buf[24] - 0x30) * 100 + (buf[25] - 0x30) * 10 + (buf[26] - 0x30)) / 1000.0;
		 uwb.Range3 = ((buf[29] - 0x30) * 10000 + (buf[30] - 0x30) * 1000 + (buf[31] - 0x30) * 100 + (buf[32] - 0x30) * 10 + (buf[33] - 0x30)) / 1000.0;
		 for (i = 1; i < 42; i++)
			 *(buf + i) = 0;
		 buf[0] = EMPTY;
		 uwb.flag = UWB_DATA_VALID;
	 }
 }

void uwbDataRead()
{
	geometry_msgs::PoseStamped uwbData;
	
	static ros::Time current_time;
    current_time = ros::Time::now();

	static int usefulNum = 0;

    uwbData.header.stamp = current_time;
    uwbData.header.frame_id = "uwbData";

    uwbData.pose.position.x = uwb.mid[0];
	uwbData.pose.position.y = uwb.mid[1];
    uwbData.pose.position.z = uwb.mask;

	uwbData.pose.orientation.x = uwb.Range0;
	uwbData.pose.orientation.y = uwb.Range1;
	uwbData.pose.orientation.z = uwb.Range2;
	uwbData.pose.orientation.w = uwb.Range3;

	if( (uwb.mid[1] == 'c') && (uwb.mask %2 == 1))
	{
		// ROS_INFO(" distance is : %f.",uwb.Range0);
		uwb_pub.publish( uwbData );
	}
}

int main(int argc, char *argv[])
{
  	ros::init(argc, argv, "uwb");
	ros::NodeHandle nh;

	uwb_pub = nh.advertise<geometry_msgs::PoseStamped>("/myuwb", 10);

	std::string port("/dev/uwb");
	// std::string port("/dev/ttyUSB0");

	// Argument 2 is the baudrate
	unsigned long baud = 115200;

	// port, baudrate, timeout in milliseconds
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1));

	if(my_serial.isOpen())
	{
		ROS_INFO(" serial open success");
	}
	else
	{
		ROS_INFO(" serial open failed");
	}

	ROS_INFO("starting to publish uwb topic...\n");

	int counter = 0;
	MsgQueue.msg_init(&MsgContr[UWB_BUF_CONTR].buf_contr_op, MsgContr[UWB_BUF_CONTR].msgbuf, MSG_BUF_SIZE);
	// Test the timeout, there should be 1 second between prints
	while (ros::ok()) 
	{
		std::string result = my_serial.read(1);
		// ROS_INFO(" ros ok");
		if (result.length()) 
		{
			counter++;
			// ROS_INFO(" result.at(0) is : %c .\n",result.at(0));
			UART1_IRQHandler(result.at(0));	
		}

		uwb.Handle();

		if(uwb.flag)
		{
			uwbDataRead();
			// ROS_INFO("counter is %d .", counter);
			counter = 0;

			uwb.flag = 0;
		}

		// loop_rate.sleep();
	}

	return 0;
}

