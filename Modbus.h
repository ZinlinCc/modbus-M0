#ifndef __MODBUS_H__
#define __MODBUS_H__

#include "func_usart.h"



/*------------------------------------------------------------------------------
ModBus ���ļ�ʹ�÷���
1.��main.c�ļ���ͷ���ֶ���"#define MODBUS_HR_NB xxxx",ȷ��HR����
2.��Modbus.c�ļ���ͷ���ֶ���"#define SLV_N xxxx",ȷ���ӻ���ַ���ⲽ����Ϊ����ʹ��ʱ�ɺ���
------------------------------------------------------------------------------*/

		/////*���Լ��ӵ�*/////
#define MODBUS_HR_NB 20          //����HR�ռ��С
#define MODBUS_SLAVE_ARRAY 50			 //������Ϊ�ӻ�ʹ��ʱ�����С
#define MODBUS_MASTER_ARRAY 20			 //������Ϊ����ʹ��ʱ�����С
#define SLV_N 0x01               //����ӻ����

#define MODBUS_MASTER //ѡ����ģʽ���Ǵӻ�ģʽ
//#define MODBUS_SLAVE  
//ѡģʽû�����壬������ɾ��

extern unsigned char rn_size,ms_rx_size; //���ճ��ȣ����ͳ���
extern unsigned short HR[MODBUS_HR_NB];
extern unsigned char slave_array[MODBUS_SLAVE_ARRAY];
extern unsigned char master_array[MODBUS_MASTER_ARRAY];

extern unsigned char tx_suc,rx_f;
extern int sta;

		/////*End*/////

#define MODBUS_SLV_BASE_NB	5//��С���ݳ���

typedef enum{
MD_FUNC_NULL,
MD_RD_COIL=1,//Read Coil Status��
MD_RD_HR=3,//Read Holding Registers��
MD_FR_SCOIL=5,//Force Single Coil��
MD_FR_SHR=6,//Preset Single Register��
MD_FR_MCOIL=15,//Force Multiple Coils��
MD_FR_MHR=16,//Preset Multiple Regs��
MD_FR_MHR_RDHR=23//Read/Write 4X Registers��
} MD_FUNC;


#define MODBUS_FAIL				0
#define MODBUS_SUCCESS			1
#define MODBUS_SUCCESS_EXT		2
#define MODBUS_FORMAT_OK		0
#define MODBUS_ERR_TIMEOUT      -1      //��ʱ
#define MODBUS_ERR_BYTELESS     -2      //�ֽ�����
#define MODBUS_ERR_CRC          -3      //CRC��
#define MODBUS_ERR_SLV          -4      //�ӻ���ַ��
#define MODBUS_ERR_FUNC         -5      //�ӻ������
#define MODBUS_ERR_BYTE         -6      //�����ֽ�����
#define MODBUS_ERR_BYTECOUNT    -7      //ByteCount��
#define MODBUS_ERR_ADR          -8      //��ַ��
#define MODBUS_ERR_DAT          -9      //���ݴ�
#define MODBUS_ERR_N            -10     //�ֽ�����
#define MODBUS_SLV_RETURN_NML	1
#define MODBUS_SLV_RETURN_EXT	2

//MODBUS �����ṹ��
typedef struct {
	unsigned char slv;		//�ӻ���ַ
	unsigned char func;		//����
	unsigned char b_xch;	//ѡ��
	unsigned char b_ext;	//״̬
	unsigned short hr_n;	//HR������
	unsigned short *phr;	//HR��ַ
	unsigned short da_adr;	//���ݵ�ַ
	unsigned short da_n;	//���ݸ���/��������
	unsigned short rww_adr;	//RW�����д���ַ
	unsigned short rww_n;	//RW�����д������
	}MODBUS_T;
	
	#define	MODBUS_ADR_BOADCAST 0x00
	#define MODBUS_SLV_IAP 0xff
	

typedef enum {
	MD_MST_STA_NULL,
	MD_MST_STA_OK,			//���Ͳ��������
	MD_MST_STA_READY,		//��һ֡��ɣ�������������
	MD_MST_STA_INPROCESS,	//���ڽ�����
	MD_MST_STA_SEND_ERR,	//���ʹ���
	MD_MST_STA_REC_ERR		//���մ���
} MD_MST_STA;


typedef enum{
	MODBUS_BIG_ENDIAN,
	MODBUS_LITTLE_ENDIAN
 } MODBUS_HR_MODE_E;

extern void modbus_init(MODBUS_T *p,unsigned short *phr,unsigned short hr_n,MODBUS_HR_MODE_E  b_little_endian);
extern int modbus_slv_rec(MODBUS_T *p,unsigned char *rb,int n);//�ӻ�����
extern int modbus_slv_send(MODBUS_T *p,unsigned char *rb);//�ӻ�����
extern int modbus_master_send(MODBUS_T *p,unsigned char *rb);//��������
extern int modbus_master_rec(MODBUS_T *p,unsigned char *rb,int n);//��������

#define MODBUS_OT_RO	1
#define MODBUS_OT_RW	0

#define MODBUS_SLV_NULL	0		//�޲���
#define MODBUS_SLV_OK	1		//������ȷ
#define MODBUS_SLV_FORMAT_OK	2	//��ʽ��ȷ,�������Լ������ݰ�
#define MODBUS_SLV_FORMAT_ERR	3	//��ʽ����
#define MODBUS_SLV_FUNC_ERR		4	//�������




/////*���Լ��ӵ�*/////
extern char getcoilbit(unsigned short addr);//��ȡָ��λ����Ȧ
extern char readcoil(unsigned short start_addr,unsigned short coil_size,unsigned char *ar);//��ȡָ�����ȵ���Ȧ
extern void alter_a_coil(unsigned short addr,unsigned short instruction);//�޸ĵ���coil
extern void alter_size_coil(unsigned short start_addr,unsigned short quantity,unsigned char *ar);//�޸�ָ�����ȵ���Ȧ
//��������������ʱ���õ�
extern void set_modbus_struct(MODBUS_T *p,unsigned char slv,unsigned char func,unsigned short da_adr,unsigned short da_n,unsigned short rww_adr,unsigned short rww_n);
//����������ʱһ��ʼ���ṹ����ĸ������ݸ�ֵ
extern void alter_HR_coil_master(unsigned char *al);//���������ר���������޸Ķ����ʱ���ã�����д���޸��õ�����
extern void coil_hr(MODBUS_T *p,unsigned short *phr,unsigned char *al);//������ȡ��������HR����ʾ�޸�coil
/////*End*/////

//---------------------------------------------------------------------------
#define MODBUS_COIL(ADR)		((HR[(ADR)/16] & (0x01 << (ADR&0x0f)))!=0) 	//����COIL��ַ���ڵ�����
#define SET_MODBUS_COIL(ADR)	(HR[(ADR)/16] |= (0x01 << (ADR&0x0f)))		//��λCOIL
#define CLR_MODBUS_COIL(ADR)	(HR[(ADR)/16] &= ~(0x01 << (ADR&0x0f)))	//����COIL
#define CPL_MODBUS_COIL(ADR)	(HR[(ADR)/16] ^= (0x01 << (ADR&0x0f)))		//��תCOIL
//---------------------------------------------------------------------------
#define MODBUS_ADR(HRN) (unsigned short)(&HRN-HR)



#endif
