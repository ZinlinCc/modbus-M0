#include "Modbus.h"
#include "utility.h"
#include "chksums.h"
#include "string.h"



#define MODBUS_COIL_EN//线圈有关指令
#define MODBUS_HR_EN//读取或修改保持寄存器
#define MODBUS_RW_EN//指令23


/////*我自己定义的*/////
unsigned short HR[MODBUS_HR_NB]={0x1234,0x5678,0};//HR空间
unsigned char slave_array[MODBUS_SLAVE_ARRAY]={0};//接收区
unsigned char master_array[MODBUS_MASTER_ARRAY]={0};//发送区。这俩区域大小随便定，够用就行
unsigned char rn_size,ms_rx_size; //接收长度，发送长度
int sta;//用来判断处理结果，从机是否需要返回数据的根据

unsigned char tx_suc,rx_f;//这两个在作为主机使用时用到

/////*End*/////


//==============================================================================
void modbus_init(MODBUS_T *p,unsigned short *phr,unsigned short hr_n,MODBUS_HR_MODE_E  b_little_endian)
{
	memset(p,0,sizeof(MODBUS_T));//清空结构体。
	p->phr=phr;//hr区域的地址,指针型数组的话这里直接给数组，能直接对应到第一个数组的地址
	p->hr_n=hr_n;//hr区域的大小
	p->b_xch=!b_little_endian;//xch的功能知道了，但是这个为毛这样写可以问一下
	p->slv=SLV_N;//从机地址。这可可以设置一下在从机模式中才有，主机模式里会被覆盖掉
}
//==============================================================================

//================================自己加的=========================================

//读取coil.土方法
char getcoilbit(unsigned short addr)//读取指定位置线圈
{
	unsigned char position,read_bit_pos;
	unsigned short hr_val;	
	
	position=addr/16;//确定是第几个HR。这应该是最简单的方法。
	
	read_bit_pos=addr%16;//确认读取的线圈的具体位置
	hr_val=HR[position];//就不对HR直接操作了，省的因为不知名原因被修改掉╮(╯▽╰)╭
	if( hr_val & (1<<read_bit_pos))
		return 1;
	else 
		return 0;
}

//修改单个指定位置coil
void alter_a_coil(unsigned short addr,unsigned short instruction)
{
	unsigned char position,tar_coil;
	
	position=addr/16;
	
	tar_coil=addr%16;
	if(instruction == 0)//特定位置0
		HR[position] &= ~(1<<tar_coil);
	else if(instruction == 0xFF00)//特定位置1
		HR[position] |= (1<<tar_coil);
}

//读取指定长度的线圈
char readcoil(unsigned short start_addr,unsigned short coil_size,unsigned char *ar)//起始地址，长度,返回数组
{
	unsigned char send_size,k,l,coil_mid,addr_mid;
	unsigned char send_buf_pos,i=0;//i还是要初始化一下，防止值不为0
	unsigned char exit;
	
	send_size=coil_size/8;
	if((coil_size%8) != 0)
		send_size++;//确定发送的数据部分长度有几个字节,bit to byte
	
	addr_mid=start_addr;//开始地址赋给中间量,地址在后面要参与判断，不能被修改
	
	for(k=0;k<send_size;k++)//byte
	{
		send_buf_pos=3+i;
		ar[send_buf_pos]=0;//清0后面要操作的数组,省的碍事
		for(l=0;l<8;l++)//bit
		{
			coil_mid=getcoilbit(addr_mid++);
			ar[send_buf_pos]|=coil_mid<<l;
			if(addr_mid == (start_addr+coil_size))//最后修改不满8个bit的，由此中断循环
			{
				exit=1;
				break;
			}
		}
		i++;
		if(exit==1)//溜咯  (>Д<;))))..... 逃~~
			break;
	}
	return send_size;
}
//修改指定长度的线圈
void alter_size_coil(unsigned short start_addr,unsigned short quantity,unsigned char *ar)
{
	unsigned char l,k,bytecount;
	unsigned short position,ins;
	unsigned char exit;
	
	bytecount=ar[6];//确定有几个byte，以便跑循环	
	position = start_addr;//起始位置
	
	for(k=0;k<bytecount;k++)//修改byte的循环
	{
		for(l=0;l<8;l++)
		{	
				if(ar[7+k] & (1<<l))//判断指令
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
//用于做主机时一开始给结构体里的各个数据赋值
void set_modbus_struct(MODBUS_T *p,unsigned char slv,unsigned char func,unsigned short da_adr,unsigned short da_n,unsigned short rww_adr,unsigned short rww_n)
{//结构体、功能、起始地址、数据内容或个数、(指令23用)读的起始地址、(指令23用)读的个数
	p->func=func;
	p->da_adr=da_adr;
	p->da_n=da_n;
	p->rww_adr=rww_adr;
	p->rww_n=rww_n;
	p->slv=slv;
}
void alter_HR_coil_master(unsigned char *al)//这个就专门用来在修改多个的时候用
{
	//指令15和16要修改的值在这里写，从al[7]开始加
  al[7]=0xFF;
	al[8]=0x03;
	//指令23要从al[11]开始

}

void coil_hr(MODBUS_T *p,unsigned short *phr,unsigned char *al)//主机读取后在主机HR中显示修改coil
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
			
			hr_pos = position/16;//确定HR编号
			len = position++%16;//确定修改的是HR中的哪一位,这一步不一定正确
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
//Modbus 从机任务
//p:指向串行端口结构体的指针，rb:串口接收到的数据组成的数组
//返回1：表示接收一包数据成功
//n：处理的数据长度
int modbus_slv_rec(MODBUS_T *p,unsigned char *rb,int n)//从机接收
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
	
	phr=p->phr;//hr地址

	if(n < MODBUS_SLV_BASE_NB)//如果发过来的数据长度太短
	{
		return(MODBUS_ERR_BYTELESS);		//字节数太少
	}

	
	if((rb[0] != MODBUS_ADR_BOADCAST) && (rb[0] != p->slv) && (rb[0] != MODBUS_SLV_IAP))//简单的说就是从机地址出错时的处理方法
	{
		return(MODBUS_ERR_SLV);
	}
	
	crc = ModBus_FastCRC(rb,n-2);//从机自主计算CRC
	
	if(crc != short_rd_buf(&rb[n-2]))//如果从机算出来的CRC和主机发过来的不一样
	{
		return(MODBUS_ERR_CRC);
	}
	
	func=rb[1];		//命令
	da_adr=char_hl_short(rb[2],rb[3]);//把发过来的寄存器地址拼成一个完整的地址
	da_n=char_hl_short(rb[4],rb[5]);//把发过来的操作数量拼成一个完整的数量(高低八位拼一起，十六位数据)
	
#ifdef MODBUS_COIL_EN
	byte_count=rb[6];	//更改多个线圈时的字节计数，不用线圈时可以注释掉，线圈也就在更改多个线圈这一条命令中用到
#endif

	
	adr_end1=da_adr+da_n;//地址和数量加一起.用于判断是否超出存储区域
	hr_nb=p->hr_n;//HR的数量
	
	
#ifdef MODBUS_COIL_EN
	b_coil_ok=((da_n < (256*8)) && (adr_end1<=hr_nb*16));//确认没有超过范围.256*8可能太大了，只是查看数量的话应该是hr_nb*16
	b_coil_bc_ok=(byte_count == (da_n+7)/8);
#endif
		
	b_hr_ok=(adr_end1<=hr_nb);//这一步知道是判断要操作的数量是否超过了HR空间的极限。开始地址加上操作数量要是超了就会报错。
	p->b_ext=!b_hr_ok;//如果上面的范围正确的话，这里p->b_ext的值应该是0
	
	b_hr_bc_ok=1;
	
	p->func=func;
	p->da_adr=da_adr;
	p->da_n=da_n;
	p->rww_adr=char_hl_short(rb[6],rb[7]);//第7个字节和第8个字节拼一块
	p->rww_n=char_hl_short(rb[8],rb[9]);//第9个和第10个拼一块.rb[0]是第一个
	r=0;	
	switch(func)
	{
	//==========================================================================
	#ifdef MODBUS_COIL_EN
		case MD_RD_COIL:	//读取线圈
			if(b_coil_ok)
			{
				r=1;
			}
			break;
		case MD_FR_MCOIL:	//强制多个线圈
			if(b_coil_ok && b_coil_bc_ok)
			{
				r=1;
			}
			break;
		case MD_FR_SCOIL:	//强制单个线圈
			if(da_adr<(hr_nb*16))//da_adr<hr_nb，这里不对，da_adr是线圈的地址，一个hr里有16个线圈，所以hr的数量应该还是乘以16
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
		case MD_FR_MHR_RDHR:	//23 修改并读取
			p->rww_adr=rww_adr=char_hl_short(rb[6],rb[7]);		//RW指令，修改p->wr_adr，p->wr_n
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
		case MD_RD_HR:		//03，读取HR
			r=1;			 //正常读取
			break;
		case MD_FR_SHR:		//06，修改单个HR
			if(n==8 && da_adr<hr_nb)
			{
				*(phr+da_adr)=da_n;
				r=1;
			}
			break;
		case MD_FR_MHR:		//修改多个寄存器
			if(b_hr_ok && b_hr_bc_ok)
			{	
				short_copy_xch(phr+da_adr,&rb[7],da_n,p->b_xch);
			}
			r=1;
			break;
	#endif
	//==========================================================================		
		default:
					//命令格式错
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



//p:指向串行端口结构体的指针，rb：指向接收数据所在的数组
int modbus_slv_send(MODBUS_T *p,unsigned char *rb)//从机发送
{
	unsigned short rsp_n=0;
	unsigned short *phr,da_n,da_adr,crc;
	unsigned char b_ext;
	
	
	/*我自己加的定义*/
	uint8_t i,len;//放在这方便每次清零
	/*End↑*/
	
	b_ext=p->b_ext;
	
	phr=p->phr;
	da_adr=p->da_adr;
	da_n=p->da_n;
	
	switch(p->func)
	{
	//==========================================================================
	#ifdef MODBUS_COIL_EN
		case MD_RD_COIL:	//读取线圈
		  rb[2]=readcoil(da_adr,da_n,rb);
			rsp_n=rb[2]+3;//这里整出来又是除了CRC以外的长度，倒是没错。
			break;
		case MD_FR_MCOIL:	//强制多个线圈
			alter_size_coil(da_adr,da_n,rb);
			rsp_n=6;
			break;
		case MD_FR_SCOIL:	//强制单个线圈
			rsp_n=6;
			break;
	#endif
	//==========================================================================		
	#ifdef MODBUS_RW_EN	
		case MD_FR_MHR_RDHR:	//修改并读取
			
		/////*我自己加的部分*/////
				
		rb[2]=da_n*2;//开始设计返回数据串
		rsp_n=da_n;
		i=da_adr;
		len=3;
		while(rsp_n--)
		{
		rb[len++]=__REV16(*(phr+i));//高位
		rb[len++]=*(phr+i);//低位
		i++;
		}
		rsp_n=3+da_n*2;//重新给rsp_n赋值。这次是总长度-2
		
		/////*End*/////
		
		break;
	#endif
	#ifdef MODBUS_HR_EN	
		case MD_RD_HR:		//读取HR
	#endif
			rb[2]=da_n*2;//读取的字节数等于寄存器个数乘2
			rsp_n=da_n*2+3;//这个应该是用来计算发送的数据长度的，这里就是前三位加上寄存器的数据位数。
			
						/////*我自己加的部分*/////
						i=p->da_adr;//提取寄存器地址
						len=3;
						while(da_n--)
						{//16位放到8位，只能放进去低八位
							rb[len++]=__REV16(*(phr+i));//高位.__REV16()这个函数：高低八位对调
							rb[len++]=*(phr+i);//低位
							i++;
						}
						/////*End*/////
					
			break;
	//==========================================================================		
		case MD_FR_SHR:		//修改一个寄存器
			
			 /////*我自己加的*/////
				
				rb[2]=__REV16(da_adr);//目标寄存器的地址，高位
				rb[3]=da_adr;//目标寄存器的地址，低位
				rb[4]=__REV16(*(phr+i));//修改后的值，高位
				rb[5]=*(phr+i);//修改后的值，低位
		    rsp_n=6;
			/////*End*/////		
		
		  break;
		case MD_FR_MHR:		//修改多个寄存器，接收从第七个开始是数据
			
			rb[2]=__REV16(da_adr);//开始地址，高位
			rb[3]=da_adr;//启动地址，低位
			rb[4]=__REV16(da_n);//操作数量，高位
			rb[5]=da_n;//操作数量，低位
			rsp_n=6;//到这里这个长度会重新赋值，所以上面可以用这个当下中间量。
			break;
	//==========================================================================		
		default:
					//命令格式错
			break;
	//==========================================================================
	}

	if(rsp_n!=0)
	{
		if(rb[0]==MODBUS_ADR_BOADCAST)
		{			
			rsp_n=0;		//广播，不返回
		}			   
		else
		{
			if(rb[0] !=MODBUS_SLV_IAP)
			{
				rb[0]=p->slv;
			}
			rb[1]=p->func;
			crc=ModBus_FastCRC(rb,rsp_n);			//计算CRC
			short_wr_buf(rb+rsp_n,crc);		//放置CRC，从rb[rsp_n]这个位置开始放两个BYTE
			rsp_n+=2;		  
		}
	}
	return(rsp_n);
}

//**************************************************************************************************************//
//p:指向串行端口结构体的指针，rb：指向发送数组首位地址
int modbus_master_send(MODBUS_T *p,unsigned char *rb)
{
	unsigned short send_n=0;
	unsigned short *phr,da_n,da_adr,rww_adr,rww_n,crc;
	unsigned char b_ext;

	
	b_ext=p->b_ext;//状态，应该是用来判断是否超过HR的最大范围。
		
	phr=p->phr;//HR地址
	da_adr=p->da_adr;//数据地址
	da_n=p->da_n;//数据个数/内容
	rww_adr=p->rww_adr;//RW命令的写入地址
	rww_n=p->rww_n;//RW命令的写入数量

	
	rb[0]=p->slv;//从机编码
	rb[1]=p->func;//功能码
	rb[2]=da_adr >> 8;//操作起始地址高八位
	rb[3]=da_adr & 0xff;//操作起始地址低八位
	rb[4]=da_n >> 8;//个数/内容高八位
	rb[5]=da_n & 0xff;//个数/内容低八位

	
	switch(p->func)
	{	
	//==========================================================================
	#ifdef MODBUS_COIL_EN
		case MD_RD_COIL:	//读取线圈
			send_n=6;
			break;
		case MD_FR_SCOIL:	//强制单个线圈
			send_n=6;
			break;
		case MD_FR_MCOIL:	//强制多个线圈
			send_n=da_n/8;
		  if((da_n%8) != 0)
			{
				send_n++;
			}
			rb[6]=send_n;//得到byte_count
			send_n=7+rb[6];//得到长度，总长度-2
		
			break;
	#endif
	//==========================================================================		
	#ifdef MODBUS_RW_EN	
		case MD_FR_MHR_RDHR:	//23，强制并读取 Read/Write 4X Registers
			rb[6]=rww_adr>>8;
			rb[7]=rww_adr & 0xff;
			rb[8]=rww_n >> 8;
			rb[9]=rww_n;//其实rb是char类型，直接赋值的话也就直给低8位
			rb[10]=rww_n * 2;	//字节计数;
			send_n=11 + rww_n * 2;
			break;
	#endif
	#ifdef MODBUS_HR_EN	
		case MD_RD_HR:		//读取HR
			send_n=6;
			break;
	#endif
		//==========================================================================		
		case MD_FR_SHR:		//修改单个HR
			if(da_adr<p->hr_n)//判断修改地址有没有超出HR的最大范围
			{
				//da_n=*(phr+da_adr);//不知道为什么要这一步。要问
				rb[4]=da_n >> 8;
				rb[5]=da_n & 0xff;
				p->da_n=da_n;
				send_n=6;
			}
			break;
		case MD_FR_MHR:		//修改多个HR
			rb[6]=da_n*2;
			send_n=da_n*2+7;
			break;
	//==========================================================================		
		default:
					//命令格式错
			break;
	//==========================================================================
	}

	if(send_n!=0)
	{
		rb[0]=p->slv;//从机编号
		rb[1]=p->func;//功能码
		crc=ModBus_FastCRC(rb,send_n);			//计算CRC
		short_wr_buf(rb+send_n,crc);		//放置CRC
		send_n+=2;		  
	}
	return(send_n);
}
//==============================================================================
//Modbus 从机任务
//p:指向串行端口结构体的指针，rb：指向接收数据所在数组,n:接收数据长度
//返回1：表示接收一包数据成功
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

 	if(n < MODBUS_SLV_BASE_NB)//数据长度太短
	{
		return(MODBUS_ERR_BYTELESS);		//字节数太少
	}
	
	
	if(rb[0] != p->slv)//从机编号不符
	{
		return(MODBUS_ERR_SLV);
	}
	
	crc = ModBus_FastCRC(rb,n-2);//计算校验
	
	if(crc != short_rd_buf(&rb[n-2]))//若校验位不符
	{
		return(MODBUS_ERR_CRC);
	}
	
	func=rb[1];		//命令
	
	if(p->func != func)//命令不相同
	{
		return(MODBUS_ERR_FUNC);
	}
	
	
	b_hr_match=(rb[2] == ((p->da_n*2) & 0xff));//比较读回来的数据数量是否和预料一致。相同则返回1
	
	r=MODBUS_FAIL;	//0
	
	switch(func)
	{
	//==========================================================================
	#ifdef MODBUS_COIL_EN
		case MD_RD_COIL:	//读取线圈
	
		r=p->da_n/8;	
		if((p->da_n%8)!=0)
		r++;
		if((r+5)!=n)
			r=MODBUS_FAIL;//判断发回来的数量是否正确
		
		coil_hr(p,phr,rb);
		
		r=MODBUS_SUCCESS;
			break;
		case MD_FR_MCOIL:	//强制多个线圈
			
			check=rb[2];
			check<<=8;
			check |= rb[3];
			if(check!=p->da_adr)
				r=MODBUS_FAIL;//检查返回的地址是否正确
			
			check=rb[4];
			check<<=8;
			check |= rb[5];
			if(check!=p->da_n)
				r=MODBUS_FAIL;//检查返回的修改数据是否正确
			
		r=MODBUS_SUCCESS;
			break;
		case MD_FR_SCOIL:	//强制单个线圈
			if(n!=8)
				r=MODBUS_FAIL;//判断数据长度是否正确
			
			check=rb[2];
			check<<=8;
			check |= rb[3];
			if(check!=p->da_adr)
				r=MODBUS_FAIL;//检查返回的地址是否正确
			
			check=rb[4];
			check<<=8;
			check |= rb[5];
			if(check!=p->da_n)
				r=MODBUS_FAIL;//检查返回的修改数据是否正确
			
			r=MODBUS_SUCCESS;
			break;
	#endif
	//==========================================================================		
	#ifdef MODBUS_RW_EN	
		case MD_FR_MHR_RDHR:	//强制并读取
			
			if(b_hr_match)//==1
			{
				if(!p->b_ext)//==0
				{
					short_copy_xch(phr+p->da_adr,&rb[3],p->da_n,p->b_xch);//rb[3]的东西拷到HR[da_adr]中，p->da_adr可以作为偏移地址)
				}
				r=MODBUS_SUCCESS;
			}
			
			break;
	#endif	
	#ifdef MODBUS_HR_EN	
		case MD_RD_HR:		//读取HR，03
	#endif
			if(b_hr_match)//==1
			{
				if(!p->b_ext)//==0
				{
					short_copy_xch(phr+p->da_adr,&rb[3],p->da_n,1);//rb[3]的东西拷到HR[da_adr]中，p->da_adr可以作为偏移地址)
				}
				r=MODBUS_SUCCESS;
			}
			break;
	//==========================================================================		
	#ifdef MODBUS_HR_EN	
		case MD_FR_SHR:		//强制单个HR
			
			check=rb[2];
			check<<=8;
			check |= rb[3];
			if(check!=p->da_adr)
				r=MODBUS_FAIL;//检查返回的地址是否正确
			
			check=rb[4];
			check<<=8;
			check |= rb[5];
			if(check!=p->da_n)
				r=MODBUS_FAIL;//检查返回的修改数据是否正确		
			
			r=MODBUS_SUCCESS;
			break;
		case MD_FR_MHR:		//强制多个HR
			
			check=rb[2];
			check<<=8;
			check |= rb[3];
			if(check!=p->da_adr)
				r=MODBUS_FAIL;//检查返回的地址是否正确
			
			check=rb[4];
			check<<=8;
			check |= rb[5];
			if(check!=p->da_n)
				r=MODBUS_FAIL;//检查返回的修改数据是否正确				
			
			r=MODBUS_SUCCESS;
			break;
	#endif
	//==========================================================================		
		default:
					//命令格式错
			break;
	}
	//==========================================================================
	return(r);
}

	

//****************************************************************************************************//
