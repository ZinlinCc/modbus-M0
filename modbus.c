#include "Modbus.h"
#include "utility.h"
#include "chksums.h"
#include "string.h"



#define MODBUS_COIL_EN//��Ȧ�й�ָ��
#define MODBUS_HR_EN//��ȡ���޸ı��ּĴ���
#define MODBUS_RW_EN//ָ��23


/////*���Լ������*/////
unsigned short HR[MODBUS_HR_NB]={0x1234,0x5678,0};//HR�ռ�
unsigned char slave_array[MODBUS_SLAVE_ARRAY]={0};//������
unsigned char master_array[MODBUS_MASTER_ARRAY]={0};//�����������������С��㶨�����þ���
unsigned char rn_size,ms_rx_size; //���ճ��ȣ����ͳ���
int sta;//�����жϴ��������ӻ��Ƿ���Ҫ�������ݵĸ���

unsigned char tx_suc,rx_f;//����������Ϊ����ʹ��ʱ�õ�

/////*End*/////


//==============================================================================
void modbus_init(MODBUS_T *p,unsigned short *phr,unsigned short hr_n,MODBUS_HR_MODE_E  b_little_endian)
{
	memset(p,0,sizeof(MODBUS_T));//��սṹ�塣
	p->phr=phr;//hr����ĵ�ַ,ָ��������Ļ�����ֱ�Ӹ����飬��ֱ�Ӷ�Ӧ����һ������ĵ�ַ
	p->hr_n=hr_n;//hr����Ĵ�С
	p->b_xch=!b_little_endian;//xch�Ĺ���֪���ˣ��������Ϊë����д������һ��
	p->slv=SLV_N;//�ӻ���ַ����ɿ�������һ���ڴӻ�ģʽ�в��У�����ģʽ��ᱻ���ǵ�
}
//==============================================================================

//================================�Լ��ӵ�=========================================

//��ȡcoil.������
char getcoilbit(unsigned short addr)//��ȡָ��λ����Ȧ
{
	unsigned char position,read_bit_pos;
	unsigned short hr_val;	
	
	position=addr/16;//ȷ���ǵڼ���HR����Ӧ������򵥵ķ�����
	
	read_bit_pos=addr%16;//ȷ�϶�ȡ����Ȧ�ľ���λ��
	hr_val=HR[position];//�Ͳ���HRֱ�Ӳ����ˣ�ʡ����Ϊ��֪��ԭ���޸ĵ��r(�s���t)�q
	if( hr_val & (1<<read_bit_pos))
		return 1;
	else 
		return 0;
}

//�޸ĵ���ָ��λ��coil
void alter_a_coil(unsigned short addr,unsigned short instruction)
{
	unsigned char position,tar_coil;
	
	position=addr/16;
	
	tar_coil=addr%16;
	if(instruction == 0)//�ض�λ��0
		HR[position] &= ~(1<<tar_coil);
	else if(instruction == 0xFF00)//�ض�λ��1
		HR[position] |= (1<<tar_coil);
}

//��ȡָ�����ȵ���Ȧ
char readcoil(unsigned short start_addr,unsigned short coil_size,unsigned char *ar)//��ʼ��ַ������,��������
{
	unsigned char send_size,k,l,coil_mid,addr_mid;
	unsigned char send_buf_pos,i=0;//i����Ҫ��ʼ��һ�£���ֵֹ��Ϊ0
	unsigned char exit;
	
	send_size=coil_size/8;
	if((coil_size%8) != 0)
		send_size++;//ȷ�����͵����ݲ��ֳ����м����ֽ�,bit to byte
	
	addr_mid=start_addr;//��ʼ��ַ�����м���,��ַ�ں���Ҫ�����жϣ����ܱ��޸�
	
	for(k=0;k<send_size;k++)//byte
	{
		send_buf_pos=3+i;
		ar[send_buf_pos]=0;//��0����Ҫ����������,ʡ�İ���
		for(l=0;l<8;l++)//bit
		{
			coil_mid=getcoilbit(addr_mid++);
			ar[send_buf_pos]|=coil_mid<<l;
			if(addr_mid == (start_addr+coil_size))//����޸Ĳ���8��bit�ģ��ɴ��ж�ѭ��
			{
				exit=1;
				break;
			}
		}
		i++;
		if(exit==1)//�￩  (>��<;))))..... ��~~
			break;
	}
	return send_size;
}
//�޸�ָ�����ȵ���Ȧ
void alter_size_coil(unsigned short start_addr,unsigned short quantity,unsigned char *ar)
{
	unsigned char l,k,bytecount;
	unsigned short position,ins;
	unsigned char exit;
	
	bytecount=ar[6];//ȷ���м���byte���Ա���ѭ��	
	position = start_addr;//��ʼλ��
	
	for(k=0;k<bytecount;k++)//�޸�byte��ѭ��
	{
		for(l=0;l<8;l++)
		{	
				if(ar[7+k] & (1<<l))//�ж�ָ��
					ins=0xFF00;//ON
				else ins=0;//OFF
				
				alter_a_coil(position++,ins);
 				if(position==(start_addr+quantity))
				{
					exit=1;
					break;
				}			
		}
			if(exit==1)
			break;
	}
}
//����������ʱһ��ʼ���ṹ����ĸ������ݸ�ֵ
void set_modbus_struct(MODBUS_T *p,unsigned char slv,unsigned char func,unsigned short da_adr,unsigned short da_n,unsigned short rww_adr,unsigned short rww_n)
{//�ṹ�塢���ܡ���ʼ��ַ���������ݻ������(ָ��23��)������ʼ��ַ��(ָ��23��)���ĸ���
	p->func=func;
	p->da_adr=da_adr;
	p->da_n=da_n;
	p->rww_adr=rww_adr;
	p->rww_n=rww_n;
	p->slv=slv;
}
void alter_HR_coil_master(unsigned char *al)//�����ר���������޸Ķ����ʱ����
{
	//ָ��15��16Ҫ�޸ĵ�ֵ������д����al[7]��ʼ��
  al[7]=0xFF;
	al[8]=0x03;
	//ָ��23Ҫ��al[11]��ʼ

}

void coil_hr(MODBUS_T *p,unsigned short *phr,unsigned char *al)//������ȡ��������HR����ʾ�޸�coil
{
	unsigned char position,hr_pos,len;
	unsigned char z_o,i,k;
	unsigned char exit;
	
	position = p->da_adr;
	
	for(k=0;k<al[2];k++)
	{
		for(i=0;i<8;i++)
		{
			if((al[3+k]&(1<<i))!=0)
				z_o=1;
			else 
				z_o=0;
			
			hr_pos = position/16;//ȷ��HR���
			len = position++%16;//ȷ���޸ĵ���HR�е���һλ,��һ����һ����ȷ
			if(z_o==0)
				*(phr+hr_pos) &= ~(1<<len);
			else
				*(phr+hr_pos) |= (1<<len);
			
			if(position==(p->da_adr+p->da_n))
				{
					exit=1;
					break;
				}		
		}
		if(exit==1)
			break;
	}
}

//==============================================================================
//Modbus �ӻ�����
//p:ָ���ж˿ڽṹ���ָ�룬rb:���ڽ��յ���������ɵ�����
//����1����ʾ����һ�����ݳɹ�
//n����������ݳ���
int modbus_slv_rec(MODBUS_T *p,unsigned char *rb,int n)//�ӻ�����
{
	unsigned char func;
	unsigned short da_adr,da_n,rww_adr,rww_n,crc;
	unsigned short hr_nb,adr_end1;
	unsigned char r;

#ifdef MODBUS_COIL_EN
	unsigned char b_coil_ok,b_coil_bc_ok,byte_count;
	unsigned short *pt16,i;
#endif
	unsigned char b_hr_ok,b_hr_bc_ok;	
	unsigned short *phr;
	
	phr=p->phr;//hr��ַ

	if(n < MODBUS_SLV_BASE_NB)//��������������ݳ���̫��
	{
		return(MODBUS_ERR_BYTELESS);		//�ֽ���̫��
	}

	
	if((rb[0] != MODBUS_ADR_BOADCAST) && (rb[0] != p->slv) && (rb[0] != MODBUS_SLV_IAP))//�򵥵�˵���Ǵӻ���ַ����ʱ�Ĵ�����
	{
		return(MODBUS_ERR_SLV);
	}
	
	crc = ModBus_FastCRC(rb,n-2);//�ӻ���������CRC
	
	if(crc != short_rd_buf(&rb[n-2]))//����ӻ��������CRC�������������Ĳ�һ��
	{
		return(MODBUS_ERR_CRC);
	}
	
	func=rb[1];		//����
	da_adr=char_hl_short(rb[2],rb[3]);//�ѷ������ļĴ�����ַƴ��һ�������ĵ�ַ
	da_n=char_hl_short(rb[4],rb[5]);//�ѷ������Ĳ�������ƴ��һ������������(�ߵͰ�λƴһ��ʮ��λ����)
	
#ifdef MODBUS_COIL_EN
	byte_count=rb[6];	//���Ķ����Ȧʱ���ֽڼ�����������Ȧʱ����ע�͵�����ȦҲ���ڸ��Ķ����Ȧ��һ���������õ�
#endif

	
	adr_end1=da_adr+da_n;//��ַ��������һ��.�����ж��Ƿ񳬳��洢����
	hr_nb=p->hr_n;//HR������
	
	
#ifdef MODBUS_COIL_EN
	b_coil_ok=((da_n < (256*8)) && (adr_end1<=hr_nb*16));//ȷ��û�г�����Χ.256*8����̫���ˣ�ֻ�ǲ鿴�����Ļ�Ӧ����hr_nb*16
	b_coil_bc_ok=(byte_count == (da_n+7)/8);
#endif
		
	b_hr_ok=(adr_end1<=hr_nb);//��һ��֪�����ж�Ҫ�����������Ƿ񳬹���HR�ռ�ļ��ޡ���ʼ��ַ���ϲ�������Ҫ�ǳ��˾ͻᱨ��
	p->b_ext=!b_hr_ok;//�������ķ�Χ��ȷ�Ļ�������p->b_ext��ֵӦ����0
	
	b_hr_bc_ok=1;
	
	p->func=func;
	p->da_adr=da_adr;
	p->da_n=da_n;
	p->rww_adr=char_hl_short(rb[6],rb[7]);//��7���ֽں͵�8���ֽ�ƴһ��
	p->rww_n=char_hl_short(rb[8],rb[9]);//��9���͵�10��ƴһ��.rb[0]�ǵ�һ��
	r=0;	
	switch(func)
	{
	//==========================================================================
	#ifdef MODBUS_COIL_EN
		case MD_RD_COIL:	//��ȡ��Ȧ
			if(b_coil_ok)
			{
				r=1;
			}
			break;
		case MD_FR_MCOIL:	//ǿ�ƶ����Ȧ
			if(b_coil_ok && b_coil_bc_ok)
			{
				r=1;
			}
			break;
		case MD_FR_SCOIL:	//ǿ�Ƶ�����Ȧ
			if(da_adr<(hr_nb*16))//da_adr<hr_nb�����ﲻ�ԣ�da_adr����Ȧ�ĵ�ַ��һ��hr����16����Ȧ������hr������Ӧ�û��ǳ���16
			{
				pt16=phr+(da_adr/16);
				i=1<<(da_adr %16);
				
				if(da_n==0xff00)
				{
					*pt16 |= i;
				}
				else if(da_n==0x0000)
				{
					*pt16 &= ~i;
				}
				else
				{
					break;
				}
				r=1;
			}
			break;
	#endif
	//==========================================================================		
	#ifdef MODBUS_RW_EN	
		case MD_FR_MHR_RDHR:	//23 �޸Ĳ���ȡ
			p->rww_adr=rww_adr=char_hl_short(rb[6],rb[7]);		//RWָ��޸�p->wr_adr��p->wr_n
			p->rww_n=rww_n=char_hl_short(rb[8],rb[9]);
			if(b_hr_ok)
			{
				if((rww_adr+rww_n)<=hr_nb)
				{
					short_copy_xch(phr+rww_adr,&rb[11],rww_n,p->b_xch);
				}
			}
			r=1;
			break;
	#endif
	//==========================================================================		
	#ifdef MODBUS_HR_EN	
		case MD_RD_HR:		//03����ȡHR
			r=1;			 //������ȡ
			break;
		case MD_FR_SHR:		//06���޸ĵ���HR
			if(n==8 && da_adr<hr_nb)
			{
				*(phr+da_adr)=da_n;
				r=1;
			}
			break;
		case MD_FR_MHR:		//�޸Ķ���Ĵ���
			if(b_hr_ok && b_hr_bc_ok)
			{	
				short_copy_xch(phr+da_adr,&rb[7],da_n,p->b_xch);
			}
			r=1;
			break;
	#endif
	//==========================================================================		
		default:
					//�����ʽ��
			break;
	}
	//==========================================================================
	if(!r)
	{
		p->func=0;
	}
	//==========================================================================
	r+=p->b_ext;
	return(r);
}
//==================================================================================================



//p:ָ���ж˿ڽṹ���ָ�룬rb��ָ������������ڵ�����
int modbus_slv_send(MODBUS_T *p,unsigned char *rb)//�ӻ�����
{
	unsigned short rsp_n=0;
	unsigned short *phr,da_n,da_adr,crc;
	unsigned char b_ext;
	
	
	/*���Լ��ӵĶ���*/
	uint8_t i,len;//�����ⷽ��ÿ������
	/*End��*/
	
	b_ext=p->b_ext;
	
	phr=p->phr;
	da_adr=p->da_adr;
	da_n=p->da_n;
	
	switch(p->func)
	{
	//==========================================================================
	#ifdef MODBUS_COIL_EN
		case MD_RD_COIL:	//��ȡ��Ȧ
		  rb[2]=readcoil(da_adr,da_n,rb);
			rsp_n=rb[2]+3;//�������������ǳ���CRC����ĳ��ȣ�����û��
			break;
		case MD_FR_MCOIL:	//ǿ�ƶ����Ȧ
			alter_size_coil(da_adr,da_n,rb);
			rsp_n=6;
			break;
		case MD_FR_SCOIL:	//ǿ�Ƶ�����Ȧ
			rsp_n=6;
			break;
	#endif
	//==========================================================================		
	#ifdef MODBUS_RW_EN	
		case MD_FR_MHR_RDHR:	//�޸Ĳ���ȡ
			
		/////*���Լ��ӵĲ���*/////
				
		rb[2]=da_n*2;//��ʼ��Ʒ������ݴ�
		rsp_n=da_n;
		i=da_adr;
		len=3;
		while(rsp_n--)
		{
		rb[len++]=__REV16(*(phr+i));//��λ
		rb[len++]=*(phr+i);//��λ
		i++;
		}
		rsp_n=3+da_n*2;//���¸�rsp_n��ֵ��������ܳ���-2
		
		/////*End*/////
		
		break;
	#endif
	#ifdef MODBUS_HR_EN	
		case MD_RD_HR:		//��ȡHR
	#endif
			rb[2]=da_n*2;//��ȡ���ֽ������ڼĴ���������2
			rsp_n=da_n*2+3;//���Ӧ�����������㷢�͵����ݳ��ȵģ��������ǰ��λ���ϼĴ���������λ����
			
						/////*���Լ��ӵĲ���*/////
						i=p->da_adr;//��ȡ�Ĵ�����ַ
						len=3;
						while(da_n--)
						{//16λ�ŵ�8λ��ֻ�ܷŽ�ȥ�Ͱ�λ
							rb[len++]=__REV16(*(phr+i));//��λ.__REV16()����������ߵͰ�λ�Ե�
							rb[len++]=*(phr+i);//��λ
							i++;
						}
						/////*End*/////
					
			break;
	//==========================================================================		
		case MD_FR_SHR:		//�޸�һ���Ĵ���
			
			 /////*���Լ��ӵ�*/////
				
				rb[2]=__REV16(da_adr);//Ŀ��Ĵ����ĵ�ַ����λ
				rb[3]=da_adr;//Ŀ��Ĵ����ĵ�ַ����λ
				rb[4]=__REV16(*(phr+i));//�޸ĺ��ֵ����λ
				rb[5]=*(phr+i);//�޸ĺ��ֵ����λ
		    rsp_n=6;
			/////*End*/////		
		
		  break;
		case MD_FR_MHR:		//�޸Ķ���Ĵ��������մӵ��߸���ʼ������
			
			rb[2]=__REV16(da_adr);//��ʼ��ַ����λ
			rb[3]=da_adr;//������ַ����λ
			rb[4]=__REV16(da_n);//������������λ
			rb[5]=da_n;//������������λ
			rsp_n=6;//������������Ȼ����¸�ֵ�����������������������м�����
			break;
	//==========================================================================		
		default:
					//�����ʽ��
			break;
	//==========================================================================
	}

	if(rsp_n!=0)
	{
		if(rb[0]==MODBUS_ADR_BOADCAST)
		{			
			rsp_n=0;		//�㲥��������
		}			   
		else
		{
			if(rb[0] !=MODBUS_SLV_IAP)
			{
				rb[0]=p->slv;
			}
			rb[1]=p->func;
			crc=ModBus_FastCRC(rb,rsp_n);			//����CRC
			short_wr_buf(rb+rsp_n,crc);		//����CRC����rb[rsp_n]���λ�ÿ�ʼ������BYTE
			rsp_n+=2;		  
		}
	}
	return(rsp_n);
}

//**************************************************************************************************************//
//p:ָ���ж˿ڽṹ���ָ�룬rb��ָ����������λ��ַ
int modbus_master_send(MODBUS_T *p,unsigned char *rb)
{
	unsigned short send_n=0;
	unsigned short *phr,da_n,da_adr,rww_adr,rww_n,crc;
	unsigned char b_ext;

	
	b_ext=p->b_ext;//״̬��Ӧ���������ж��Ƿ񳬹�HR�����Χ��
		
	phr=p->phr;//HR��ַ
	da_adr=p->da_adr;//���ݵ�ַ
	da_n=p->da_n;//���ݸ���/����
	rww_adr=p->rww_adr;//RW�����д���ַ
	rww_n=p->rww_n;//RW�����д������

	
	rb[0]=p->slv;//�ӻ�����
	rb[1]=p->func;//������
	rb[2]=da_adr >> 8;//������ʼ��ַ�߰�λ
	rb[3]=da_adr & 0xff;//������ʼ��ַ�Ͱ�λ
	rb[4]=da_n >> 8;//����/���ݸ߰�λ
	rb[5]=da_n & 0xff;//����/���ݵͰ�λ

	
	switch(p->func)
	{	
	//==========================================================================
	#ifdef MODBUS_COIL_EN
		case MD_RD_COIL:	//��ȡ��Ȧ
			send_n=6;
			break;
		case MD_FR_SCOIL:	//ǿ�Ƶ�����Ȧ
			send_n=6;
			break;
		case MD_FR_MCOIL:	//ǿ�ƶ����Ȧ
			send_n=da_n/8;
		  if((da_n%8) != 0)
			{
				send_n++;
			}
			rb[6]=send_n;//�õ�byte_count
			send_n=7+rb[6];//�õ����ȣ��ܳ���-2
		
			break;
	#endif
	//==========================================================================		
	#ifdef MODBUS_RW_EN	
		case MD_FR_MHR_RDHR:	//23��ǿ�Ʋ���ȡ Read/Write 4X Registers
			rb[6]=rww_adr>>8;
			rb[7]=rww_adr & 0xff;
			rb[8]=rww_n >> 8;
			rb[9]=rww_n;//��ʵrb��char���ͣ�ֱ�Ӹ�ֵ�Ļ�Ҳ��ֱ����8λ
			rb[10]=rww_n * 2;	//�ֽڼ���;
			send_n=11 + rww_n * 2;
			break;
	#endif
	#ifdef MODBUS_HR_EN	
		case MD_RD_HR:		//��ȡHR
			send_n=6;
			break;
	#endif
		//==========================================================================		
		case MD_FR_SHR:		//�޸ĵ���HR
			if(da_adr<p->hr_n)//�ж��޸ĵ�ַ��û�г���HR�����Χ
			{
				//da_n=*(phr+da_adr);//��֪��ΪʲôҪ��һ����Ҫ��
				rb[4]=da_n >> 8;
				rb[5]=da_n & 0xff;
				p->da_n=da_n;
				send_n=6;
			}
			break;
		case MD_FR_MHR:		//�޸Ķ��HR
			rb[6]=da_n*2;
			send_n=da_n*2+7;
			break;
	//==========================================================================		
		default:
					//�����ʽ��
			break;
	//==========================================================================
	}

	if(send_n!=0)
	{
		rb[0]=p->slv;//�ӻ����
		rb[1]=p->func;//������
		crc=ModBus_FastCRC(rb,send_n);			//����CRC
		short_wr_buf(rb+send_n,crc);		//����CRC
		send_n+=2;		  
	}
	return(send_n);
}
//==============================================================================
//Modbus �ӻ�����
//p:ָ���ж˿ڽṹ���ָ�룬rb��ָ�����������������,n:�������ݳ���
//����1����ʾ����һ�����ݳɹ�
int modbus_master_rec(MODBUS_T *p,unsigned char *rb,int n)
{
	unsigned char func;
	unsigned short crc;

	unsigned char r;
	unsigned short check;

#ifdef MODBUS_COIL_EN
	unsigned char b_coil_ok,b_coil_bc_ok;
#endif
	unsigned char b_hr_match;	
	unsigned short *phr;
	
	phr=p->phr;

 	if(n < MODBUS_SLV_BASE_NB)//���ݳ���̫��
	{
		return(MODBUS_ERR_BYTELESS);		//�ֽ���̫��
	}
	
	
	if(rb[0] != p->slv)//�ӻ���Ų���
	{
		return(MODBUS_ERR_SLV);
	}
	
	crc = ModBus_FastCRC(rb,n-2);//����У��
	
	if(crc != short_rd_buf(&rb[n-2]))//��У��λ����
	{
		return(MODBUS_ERR_CRC);
	}
	
	func=rb[1];		//����
	
	if(p->func != func)//�����ͬ
	{
		return(MODBUS_ERR_FUNC);
	}
	
	
	b_hr_match=(rb[2] == ((p->da_n*2) & 0xff));//�Ƚ϶����������������Ƿ��Ԥ��һ�¡���ͬ�򷵻�1
	
	r=MODBUS_FAIL;	//0
	
	switch(func)
	{
	//==========================================================================
	#ifdef MODBUS_COIL_EN
		case MD_RD_COIL:	//��ȡ��Ȧ
	
		r=p->da_n/8;	
		if((p->da_n%8)!=0)
		r++;
		if((r+5)!=n)
			r=MODBUS_FAIL;//�жϷ������������Ƿ���ȷ
		
		coil_hr(p,phr,rb);
		
		r=MODBUS_SUCCESS;
			break;
		case MD_FR_MCOIL:	//ǿ�ƶ����Ȧ
			
			check=rb[2];
			check<<=8;
			check |= rb[3];
			if(check!=p->da_adr)
				r=MODBUS_FAIL;//��鷵�صĵ�ַ�Ƿ���ȷ
			
			check=rb[4];
			check<<=8;
			check |= rb[5];
			if(check!=p->da_n)
				r=MODBUS_FAIL;//��鷵�ص��޸������Ƿ���ȷ
			
		r=MODBUS_SUCCESS;
			break;
		case MD_FR_SCOIL:	//ǿ�Ƶ�����Ȧ
			if(n!=8)
				r=MODBUS_FAIL;//�ж����ݳ����Ƿ���ȷ
			
			check=rb[2];
			check<<=8;
			check |= rb[3];
			if(check!=p->da_adr)
				r=MODBUS_FAIL;//��鷵�صĵ�ַ�Ƿ���ȷ
			
			check=rb[4];
			check<<=8;
			check |= rb[5];
			if(check!=p->da_n)
				r=MODBUS_FAIL;//��鷵�ص��޸������Ƿ���ȷ
			
			r=MODBUS_SUCCESS;
			break;
	#endif
	//==========================================================================		
	#ifdef MODBUS_RW_EN	
		case MD_FR_MHR_RDHR:	//ǿ�Ʋ���ȡ
			
			if(b_hr_match)//==1
			{
				if(!p->b_ext)//==0
				{
					short_copy_xch(phr+p->da_adr,&rb[3],p->da_n,p->b_xch);//rb[3]�Ķ�������HR[da_adr]�У�p->da_adr������Ϊƫ�Ƶ�ַ)
				}
				r=MODBUS_SUCCESS;
			}
			
			break;
	#endif	
	#ifdef MODBUS_HR_EN	
		case MD_RD_HR:		//��ȡHR��03
	#endif
			if(b_hr_match)//==1
			{
				if(!p->b_ext)//==0
				{
					short_copy_xch(phr+p->da_adr,&rb[3],p->da_n,1);//rb[3]�Ķ�������HR[da_adr]�У�p->da_adr������Ϊƫ�Ƶ�ַ)
				}
				r=MODBUS_SUCCESS;
			}
			break;
	//==========================================================================		
	#ifdef MODBUS_HR_EN	
		case MD_FR_SHR:		//ǿ�Ƶ���HR
			
			check=rb[2];
			check<<=8;
			check |= rb[3];
			if(check!=p->da_adr)
				r=MODBUS_FAIL;//��鷵�صĵ�ַ�Ƿ���ȷ
			
			check=rb[4];
			check<<=8;
			check |= rb[5];
			if(check!=p->da_n)
				r=MODBUS_FAIL;//��鷵�ص��޸������Ƿ���ȷ		
			
			r=MODBUS_SUCCESS;
			break;
		case MD_FR_MHR:		//ǿ�ƶ��HR
			
			check=rb[2];
			check<<=8;
			check |= rb[3];
			if(check!=p->da_adr)
				r=MODBUS_FAIL;//��鷵�صĵ�ַ�Ƿ���ȷ
			
			check=rb[4];
			check<<=8;
			check |= rb[5];
			if(check!=p->da_n)
				r=MODBUS_FAIL;//��鷵�ص��޸������Ƿ���ȷ				
			
			r=MODBUS_SUCCESS;
			break;
	#endif
	//==========================================================================		
		default:
					//�����ʽ��
			break;
	}
	//==========================================================================
	return(r);
}

	

//****************************************************************************************************//
