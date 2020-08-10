#ifndef __MODBUS_H__
#define __MODBUS_H__

#include "func_usart.h"



/*------------------------------------------------------------------------------
ModBus 库文件使用方法
1.在main.c文件开头部分定义"#define MODBUS_HR_NB xxxx",确定HR数量
2.在Modbus.c文件开头部分定义"#define SLV_N xxxx",确定从机地址。这步若作为主机使用时可忽视
------------------------------------------------------------------------------*/

		/////*我自己加的*/////
#define MODBUS_HR_NB 20          //定义HR空间大小
#define MODBUS_SLAVE_ARRAY 50			 //定义作为从机使用时数组大小
#define MODBUS_MASTER_ARRAY 20			 //定义作为主机使用时数组大小
#define SLV_N 0x01               //定义从机编号

#define MODBUS_MASTER //选主机模式还是从机模式
//#define MODBUS_SLAVE  
//选模式没有意义，最后可以删掉

extern unsigned char rn_size,ms_rx_size; //接收长度，发送长度
extern unsigned short HR[MODBUS_HR_NB];
extern unsigned char slave_array[MODBUS_SLAVE_ARRAY];
extern unsigned char master_array[MODBUS_MASTER_ARRAY];

extern unsigned char tx_suc,rx_f;
extern int sta;

		/////*End*/////

#define MODBUS_SLV_BASE_NB	5//最小数据长度

typedef enum{
MD_FUNC_NULL,
MD_RD_COIL=1,//Read Coil Status√
MD_RD_HR=3,//Read Holding Registers√
MD_FR_SCOIL=5,//Force Single Coil√
MD_FR_SHR=6,//Preset Single Register√
MD_FR_MCOIL=15,//Force Multiple Coils√
MD_FR_MHR=16,//Preset Multiple Regs√
MD_FR_MHR_RDHR=23//Read/Write 4X Registers√
} MD_FUNC;


#define MODBUS_FAIL				0
#define MODBUS_SUCCESS			1
#define MODBUS_SUCCESS_EXT		2
#define MODBUS_FORMAT_OK		0
#define MODBUS_ERR_TIMEOUT      -1      //超时
#define MODBUS_ERR_BYTELESS     -2      //字节数少
#define MODBUS_ERR_CRC          -3      //CRC错
#define MODBUS_ERR_SLV          -4      //从机地址错
#define MODBUS_ERR_FUNC         -5      //从机命令错
#define MODBUS_ERR_BYTE         -6      //接收字节数错
#define MODBUS_ERR_BYTECOUNT    -7      //ByteCount错
#define MODBUS_ERR_ADR          -8      //地址错
#define MODBUS_ERR_DAT          -9      //数据错
#define MODBUS_ERR_N            -10     //字节数错
#define MODBUS_SLV_RETURN_NML	1
#define MODBUS_SLV_RETURN_EXT	2

//MODBUS 常量结构体
typedef struct {
	unsigned char slv;		//从机地址
	unsigned char func;		//命令
	unsigned char b_xch;	//选项
	unsigned char b_ext;	//状态
	unsigned short hr_n;	//HR的数量
	unsigned short *phr;	//HR地址
	unsigned short da_adr;	//数据地址
	unsigned short da_n;	//数据个数/数据内容
	unsigned short rww_adr;	//RW命令的写入地址
	unsigned short rww_n;	//RW命令的写入数量
	}MODBUS_T;
	
	#define	MODBUS_ADR_BOADCAST 0x00
	#define MODBUS_SLV_IAP 0xff
	

typedef enum {
	MD_MST_STA_NULL,
	MD_MST_STA_OK,			//发送并接收完成
	MD_MST_STA_READY,		//上一帧完成，可以启动发送
	MD_MST_STA_INPROCESS,	//正在进程中
	MD_MST_STA_SEND_ERR,	//发送错误
	MD_MST_STA_REC_ERR		//接收错误
} MD_MST_STA;


typedef enum{
	MODBUS_BIG_ENDIAN,
	MODBUS_LITTLE_ENDIAN
 } MODBUS_HR_MODE_E;

extern void modbus_init(MODBUS_T *p,unsigned short *phr,unsigned short hr_n,MODBUS_HR_MODE_E  b_little_endian);
extern int modbus_slv_rec(MODBUS_T *p,unsigned char *rb,int n);//从机接收
extern int modbus_slv_send(MODBUS_T *p,unsigned char *rb);//从机发送
extern int modbus_master_send(MODBUS_T *p,unsigned char *rb);//主机发送
extern int modbus_master_rec(MODBUS_T *p,unsigned char *rb,int n);//主机接收

#define MODBUS_OT_RO	1
#define MODBUS_OT_RW	0

#define MODBUS_SLV_NULL	0		//无操作
#define MODBUS_SLV_OK	1		//操作正确
#define MODBUS_SLV_FORMAT_OK	2	//格式正确,但不是自己的数据包
#define MODBUS_SLV_FORMAT_ERR	3	//格式错误
#define MODBUS_SLV_FUNC_ERR		4	//命令错误




/////*我自己加的*/////
extern char getcoilbit(unsigned short addr);//读取指定位置线圈
extern char readcoil(unsigned short start_addr,unsigned short coil_size,unsigned char *ar);//读取指定长度的线圈
extern void alter_a_coil(unsigned short addr,unsigned short instruction);//修改单个coil
extern void alter_size_coil(unsigned short start_addr,unsigned short quantity,unsigned char *ar);//修改指定长度的线圈
//下面是做主机的时候用的
extern void set_modbus_struct(MODBUS_T *p,unsigned char slv,unsigned char func,unsigned short da_adr,unsigned short da_n,unsigned short rww_adr,unsigned short rww_n);
//用于做主机时一开始给结构体里的各个数据赋值
extern void alter_HR_coil_master(unsigned char *al);//这个函数就专门用来在修改多个的时候用，用来写入修改用的数据
extern void coil_hr(MODBUS_T *p,unsigned short *phr,unsigned char *al);//主机读取后在主机HR中显示修改coil
/////*End*/////

//---------------------------------------------------------------------------
#define MODBUS_COIL(ADR)		((HR[(ADR)/16] & (0x01 << (ADR&0x0f)))!=0) 	//返回COIL地址所在的内容
#define SET_MODBUS_COIL(ADR)	(HR[(ADR)/16] |= (0x01 << (ADR&0x0f)))		//置位COIL
#define CLR_MODBUS_COIL(ADR)	(HR[(ADR)/16] &= ~(0x01 << (ADR&0x0f)))	//清零COIL
#define CPL_MODBUS_COIL(ADR)	(HR[(ADR)/16] ^= (0x01 << (ADR&0x0f)))		//翻转COIL
//---------------------------------------------------------------------------
#define MODBUS_ADR(HRN) (unsigned short)(&HRN-HR)



#endif
