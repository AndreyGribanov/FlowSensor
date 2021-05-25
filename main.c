#include "stm32f10x.h"
#include "delay.h" //��������� ��������
#include "crc16.h"//������� � �������� ����������� �����
#include "modbus.h"//������ ������� � ������������ ������ �� ���������
#include "flash.h"//������ � ������� �����������������
#include "math.h"//����������� ���������� ��, ������������� ��� ���������� ������� �������������� ��������
#include "calculation_coef.h"//��������� �������� ������������� n � k


#define F_APB2 72000000//������� ������������ UART1
#define F_APB1x2 72000000//������� ������������ ��������


//#define pi  3.1415926535
uint8_t cnt_data=0;//������� ���������� ������� � ����� ������
uint16_t buffer_data[125];//����� ��� ���������� ������ � �������� ������
uint8_t buffer_in[256];//����� ��� ������ ������=������������ ����� ������ � modbus RTU
uint8_t  buffer_out[256];//����� ��� �������� ������=������������ ����� ������ � modbus RTU
uint16_t  index_in= 0;//������� �������� �������� ������ ������
uint16_t index_out=0;//������� �������� �������� ������ ��������

uint16_t adc_in7=0;                      //�������� �������� � ���
uint16_t mean_adc_in7=0;                 //������� �������� ��� �� ���������� ���������
int16_t cnt4;// ������ ��� �������� ���������������

uint16_t k_PWM=9500; // ���������������� ���������� ���������������   PWM � ����������
uint8_t  arithmetical_mean=10;//����� ������� �� �������� ������ ������� �������������

float U_0=3.6	;//�������� ���������� ��� ���������� ������
uint16_t ADC_U_0=1340;// �������� ��� ��� ���������� ������    
uint16_t V_flow_x100=0; //�������� ������       
uint16_t U_fact;//����������� �������� ���������� �� ������ � ������� 
float k_U;
//��������� ��� ������������:
float n=0.42;
float k=0.89;




float zn;//�����������

uint8_t flag_indata=0;//���� ��������� �������� ������ ��� ������� ����������

//��������� �� ���������(����� ��������):
 uint16_t addres=0x10;//����� ���������� � ����,
uint16_t addres_new;//�������� ������ ������ ����� ������������
//� ��������� ���������� � ����������������� �������!!!10, ��� 0x10
uint16_t	timeout=0;//�������� ������� �������� ������(
uint32_t bautrate=115200;//�������� UART1
uint16_t stopbit=0;//0-1,2-2,3-1.5; 
uint16_t parity=0;//0-��� �������� ��������, 1 � ����� �������� ��������
uint8_t parity_status=0;//��� �������� �������� 0-���,1 �����
uint16_t time11bit;//����� �� �������� 1 �����, � ������ �����,����, ��������
int16_t cnt=0;// �����  ��� �������� ����� ������������	
uint8_t flag_reset=0;//����  ������������
uint8_t flag_calculation=0;//����  �������� n � k







//---���������------------------------------------------------------------------------------------
//void clean_buffer();
void clean_buffer( uint8_t *adr_buffer, uint32_t byte_cnt);
void init_uart(void);
void init_RCC(void);
void USART1_IRQHandler(void);
//void send_UART(  uint8_t* value);
void init_ADC(void);
void read_ADC(void);
void Send_Modbas(void);
void init_Tim3(void);
void TIM3_IRQHandler(void);
void iwdg_init(uint16_t tw);
void measurement(void);//����� �������� �� ����� ���
void init_Tim2(void);
void set_PWM(void);
void write_data_to_massiv(void);
void add_CRC(void);//��������� CRC � ����� ���������� ������ 
void send_RS485(  uint8_t* value);// ��������� �������� ������ � ���� rs-485
void Start_Led(void);//��������� ��� ��������

//------------------------------------------------------------------------------------------------
void init_uart()
{
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // ��������� ������������ �����PORTA
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // enable clock for Altirnate function
  RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;//���������� usart
	//USART1->BRR 	= 0x1D4C;//0x9C4;//����������� �� �������� 9600 ��� USART_BRR = (fck + baudrate /2 ) / baudrate,fck=72000000��
	
	USART1->BRR =  (F_APB2+(bautrate/2))/bautrate;
	USART1->CR2 = stopbit<<12;
	USART1->CR1 = parity<<10;
	USART1->CR1 = parity_status<<9;
	USART1->CR1 	|= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE |	 USART_CR1_RXNEIE;
	 //ue-�������� ���� usart,te-��� ����������,re-��� ��������,rxneie-��� ���������� �� ������ �����.
	
		NVIC_EnableIRQ (USART1_IRQn);// ������� CMSIS ����������� ���������� � NVIC
	
}
//-------------------------------------------------------------------------------------------------------------

void init_RCC()//reset & clock control
{

RCC->CR |= RCC_CR_HSEON;//��������� ��������� HSE,������� �����
while (!(RCC->CR & RCC_CR_HSERDY)) {}; // ���� ����������

RCC->CR |= RCC_CR_HSION;//��������� ��������� HSI,���������� rc ��������� ��� ������ � �������� ����
while (!(RCC->CR & RCC_CR_HSIRDY)) {}; // ���� ����������

RCC->CFGR &= ~RCC_CFGR_SW; //������� ����� ������ ��������� ��������� �������(����������� �� ����������� rc)
while((RCC->CFGR & RCC_CFGR_SWS)) {} //�������� ������������ �� HSI
		
FLASH->ACR |= FLASH_ACR_PRFTBE; // Enable Prefetch Buffer,��� ��������� ������ ��������������� �������
while(!(FLASH->ACR & FLASH_ACR_PRFTBS)) {};	
FLASH->ACR &= ~FLASH_ACR_LATENCY; // �����������.
//FLASH->ACR |= FLASH_ACR_LATENCY_2; // Two wait states ������ 7796,�������)))
	FLASH->ACR |=2;//��� ����� ������� � ��������
//FLASH->ACR |=FLASH_ACR_HLFCYA;//Flash half cycle access enable, ��� ��������� ������� � ����������� �����

	
//PLLMul=x9,USB prescaler=1.5(�� ���������),AHB prc=1,APB1 psc=2,APB2 psc=1,����� SYSCLK=APB=APB2=72,APB1=36 M��,USB=48	




	
 RCC->CFGR |= RCC_CFGR_PLLSRC; //���������� ������� ��� PLL ������ HSE (������� - ����� �� 8 ���)	
 RCC->CFGR &=~RCC_CFGR_SW; // �������� ���� SW0, SW
	RCC->CR &= ~RCC_CR_PLLON; //��������� ��������� PLL
 RCC->CFGR &= ~RCC_CFGR_PLLMULL; //�������� PLLMULL
	RCC->CFGR |= RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_1 | RCC_CFGR_PLLMULL_2; //���������� ��������� 9  (����� 72 ���)	
 RCC->CFGR |= RCC_CFGR_PPRE1_2;	//APB1 psc=2
	
	 
	
 RCC->CR |= RCC_CR_PLLON; //�������� ��������� PLL
 while(!(RCC->CR & RCC_CR_PLLRDY)) {} //�������� ���������� PLL

	
 RCC->CFGR |= RCC_CFGR_SW_PLL; //������� ���������� ��������� ������� PLL	 
while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {} //�������� ������������ �� PLL 
	
}

//-----------------------------------------------------------------------------------------------------------

void init_GPIO()
{
	  RCC->APB2ENR|= RCC_APB2ENR_AFIOEN; // enable clock for Altirnate function
		AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE;//!!���������� pb3 � pb4 � pa15 �� jtag �� ��������� ����� ��������
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
     // A15
  	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; 
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

// B13 pin ��������� �������
	 GPIOB->CRH &= ~(GPIO_CRH_CNF13_0 | GPIO_CRH_CNF13_1);//������ 00 � cnf13,����������� �����,push-pull
	//01-�������� ����������,10� 11 �������������� �������,� ������ �����:00-����������,01-Z ����,10 �������� PxODR=1 �+,=0 � GND
  GPIOB->CRH |= GPIO_CRH_MODE13_1;//������ 1 � mode13 ,10,���� �������� 2 ���
	//00-����� �����,01-10���,11-50���
	GPIOB->BSRR |= GPIO_BSRR_BR13;
	
	
	
	
	// A8 pin EN
	 GPIOA->CRH &= ~(GPIO_CRH_CNF8_0 | GPIO_CRH_CNF8_1);//������ 00 � cnf13,����������� �����,push-pull
	//01-�������� ����������,10� 11 �������������� �������,� ������ �����:00-����������,01-Z ����,10 �������� PxODR=1 �+,=0 � GND
  GPIOA->CRH |= GPIO_CRH_MODE8_1;//������ 1 � mode13 ,10,���� �������� 2 ���
	//00-����� �����,01-10���,11-50���
	

GPIOA->BSRR |= GPIO_BSRR_BR8;

	// A9 pin TX
	 GPIOA->CRH &= ~(GPIO_CRH_CNF9_0 | GPIO_CRH_CNF9_1);//������ 00 � cnf13,����������� �����,push-pull
	GPIOA->CRH		|= GPIO_CRH_CNF9_1;
	
	//01-�������� ����������,10� 11 �������������� �������,� ������ �����:00-����������,01-Z ����,10 �������� PxODR=1 �+,=0 � GND
  GPIOA->CRH |= GPIO_CRH_MODE9_1;//������ 1 � mode13 ,10,���� �������� 2 ���
	//00-����� �����,01-10���,11-50���
	

GPIOA->BSRR |= GPIO_BSRR_BS9;

	// A10 pin RX
		
GPIOA->CRH		&= ~GPIO_CRH_CNF10;																// Clear CNF bit 10
GPIOA->CRH		|= GPIO_CRH_CNF10_0;																// 01-Z ����
GPIOA->CRH		&= ~GPIO_CRH_MODE10;	                             //00-����� �����
	
	
	
	
	
	// B14 ��������� �������
 GPIOB->CRH &= ~(GPIO_CRH_CNF14_0 | GPIO_CRH_CNF14_1);//������ 00 � cnf13,����������� �����,push-pull
	//01-�������� ����������,10� 11 �������������� �������,� ������ �����:00-����������,01-Z ����,10 �������� PxODR=1 �+,=0 � GND
  GPIOB->CRH |= GPIO_CRH_MODE14_1;//������ 1 � mode13 ,10,���� �������� 2 ���
	//00-����� �����,01-10���,11-50���
	GPIOB->BSRR |= GPIO_BSRR_BR14;




}
//--------------------------------------------------------------------------------------------------------------
void init_Tim2(void)//�������� � �������� ���
{ 
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;//��� ������������ ����� A
RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
RCC->APB1ENR |=RCC_APB1ENR_TIM2EN;// �������� ������������ ������� TIM2.APB1 ������� 36 ���


  // ����� ��������  �� TIM2_ch4 PA3
  GPIOA->CRL &= ~GPIO_CRL_MODE2; 
  GPIOA->CRL &= ~GPIO_CRL_CNF3;// MODE5 = 0x03 (11b)-50���; CNF5 = 0x02 (10b)
  GPIOA->CRL |= (GPIO_CRL_CNF3_1 | GPIO_CRL_MODE3);//����������� ����� �������������� �������

 //(���� PPRE1!=0 �� ������ ����������� � ��������� �������� ���� APB1,36*2 M��
TIM2->PSC = 1;//�������� ����� 1
TIM2->ARR = 10000;//�������� ������������,������������� �������� ��� ����� ����� ��� ������ ����
TIM2->CCR4 = 0;//����. ����������	
TIM2->CCER |= TIM_CCER_CC4E;//��� 4 �����
TIM2->BDTR |= TIM_BDTR_MOE;//MOE: Main output enable	
TIM2->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; //����� ��� 1 4 ������//PWM mode 1, ������ ��� 4 �����	
	
TIM2->CR1 &= ~TIM_CR1_DIR;//������� �����,��� ����������� ����� �������(�� ��������������0)
TIM2->CR1 &= ~TIM_CR1_CMS;//������������ �� ������, Fast PWM
TIM2->CR1 |= TIM_CR1_CEN;//�������� �������,��� ���������� ������ �������



}

//------------------------------------------------------------------------------------------------------------------
void USART1_IRQHandler(void)//���������� ���������� �� ������� ������ � ����
{
 if (USART1->SR & USART_SR_RXNE)//���� ������ ������(������ ���� RXNE)..,����� ������� ��� ����� ������ DR
	{
		
		buffer_in[index_in] = USART1->DR;				
			index_in++;			
		TIM3->CNT = 0;//��� ������ �������� ����� ������������� �������, �� ����� ��� ����� � ���������� �� ������������
		TIM3->CR1 |= (TIM_CR1_CEN|TIM_CR1_OPM);	//��������� ������ ������� ���� CEN � ������������� ����� ������ ������� ���������� ���� OPM
	
		}
 	
}


//-------------------------------------------------------------------------------------------------------------

void clean_buffer( uint8_t *adr_buffer, uint32_t byte_cnt)//������� ������ ������
{
	 		while( byte_cnt--)
	*adr_buffer++=0;
	}

//-----------------------------------------------------------------------------------------------------------
void init_ADC()
{
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // ��������� ������������ �����PORTA
//A7 = adc_12_in 7
// ���������������� PORT A7 ��� ���������� ����
GPIOA->CRL &= ~GPIO_CRL_MODE7; // �������� ���� MODE 00 � ����� �����
GPIOA->CRL &= ~GPIO_CRL_CNF7; // �������� ���� CNF 00-���������� �����(���� ��� z-����)
RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // �������� ������������ ���
	
RCC->CFGR |=RCC_CFGR_ADCPRE_DIV8; // ������ �������� �������� �������� ������� 2����,00-�������� �� 2
//adc ����������� apb2 ����� �������� 2,4,6,8.(������������ ������� 14���)
ADC1->CR1 = 0; // �������� ������� ����������
ADC1->SQR1 = 0; // �������� ������� SQR1
ADC1->CR2 |= ADC_CR2_CAL; // ���� ���������� ��� CAL �������� CR2,����� ���������� ��������� � 0
while (!(ADC1->CR2 & ADC_CR2_CAL)){} // ����� ��������� ����������,���� �� ��������� ���
//���������� ���������� ���� ��� �� ��������� ���
ADC1->SQR3 |=7; // ����� ������� ������ ��� ������(�� �� ���������)
ADC1->CR2 |= ADC_CR2_EXTSEL; // ������� ���������� ������� ������ SWSTART.111 16..19 ���
 ADC1->CR2 |= ADC_CR2_EXTTRIG;// ��������� ������� ������ ����������� ������
ADC1->CR2 |= ADC_CR2_ADON; // �������� ���
}
//-------------------------------------------------------------------------------------------------------------
void read_ADC()
{
 ADC1->CR2 |= ADC_CR2_SWSTART; //start converting
while (!(ADC1->SR & ADC_SR_EOC) ){} //Wait
adc_in7= ADC1->DR;  //read variable

}

//-------------------------------------------------------------------------------------------------------
void init_Tim3()//�������� �� ����������� �������,����� ���� �������� ��������� ������.
{ RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
     TIM3->PSC = (uint16_t)((F_APB1x2/1000000)-1);	//������������� ������������, ���� ������ ������� �� ������� 1��
     
	   TIM3->CCR1=(uint16_t)(time11bit);
	
	   TIM3->ARR = (uint16_t)(timeout);	//������������� �������� ������������ �������, � ������ � �������� ��� ������� ������������ ������� ���������� 
     TIM3->EGR |= TIM_EGR_UG;	//���������� ������� ���������� ��� ������ ������ � �������� PSC � ARR

	       TIM3->CNT = 0;
         TIM3->SR = 0; // ���������� ����� ����������
         TIM3->DIER |= TIM_DIER_CC1IE; // ��������� ���������� ������������� �������� 

NVIC_SetPriority(TIM3_IRQn, 15);//������ ���������
NVIC_EnableIRQ(TIM3_IRQn);


}

//----------------------------------------------------------------------------------------------------------------

void TIM3_IRQHandler()//��������� ������� � ���� ������ ������������ �������
	{
  
     if (TIM3->SR &TIM_SR_CC1IF )//���� ������� ������ �������� ����� 11 ���
       //���� ������ �������� ������, ���� ������� ����� ������� ���������� ������ ������  
				//TIM3->CNT = 0;
     {  
if ((buffer_in[0]==addres)||(buffer_in[0]==0))//���� ��� ������ ��� ������ ������ ��� ����������������� �������
	 {if	( verification_CRCin())
	 {flag_indata=1;//��� ���� ������, �� ���� ����������
   
		 
//		delay_us(timeout*0.33);//0.67-���� ���� Tim3

	Modbus_handler_data();
  add_CRC();//�������� ����������� �����
		 //		Modbus_handler_data(); //����� ���������� ��� �� 50 ���
//		 
		 
		 //��� ���� ������ ������ �� RS-485 � �������� ���� �� RS-485
		//delay_us(timeout*0.33+1.33*timeout);//0.67-���� ���� Tim3
    //delay_us(timeout-time11bit-50);
		while (!(TIM3->SR &TIM_SR_UIF ) ){}//����, ���� ������� �� ��������� �� �����, ��� ����� ��������� �������� �������
		 
		 
		 Send_Modbas(); 
		 
	 } 		 
		 
		 else //���� �� ������ �����������
	 clean_buffer(buffer_in,index_in);
			index_in=0;
	 }
		else//���� ��� �� ���� ������
   clean_buffer(buffer_in,index_in);
	index_in=0;

       
			}
	

	  TIM3->SR = 0; // ���������� ����� ����������
}
//-----------------------------------------------------------------------------------------------------------	
void trigg_red(void)
{
	static int8_t a = 0;
	static	int16_t cnt3=100;
if (cnt3)
cnt3--;
 else 
 {	 cnt3=47;//������������� �������� �������� ����������
if (a)
{GPIOB->BSRR |= GPIO_BSRR_BR13;
a = 1 - a;}
else
{
GPIOB->BSRR |= GPIO_BSRR_BS13;
a = 1 - a;
}
}

}

//-------------------------------------------------------------------------------------------------------------
void measurement(void)//����� �������� �� ����� ���,���������� �������� �� 5 ����� ������ ���������� �������
{
	
		float V_flow_x10=0;
  static	int32_t sum7=0;
	//int16_t cnt4=((arithmetical_mean*4)-1);
if (cnt4)


{	//if ((cnt4==4)||(cnt4==8)||(cnt4==12)||(cnt4==16))
 if ((!(cnt4%4))&&(!(cnt4==0)))//���� ��� ������� �� ������� � �� ����

	{read_ADC();//������ ��� 1
		//adc_in7=2188;
	 sum7=sum7+adc_in7;	
	}
	cnt4--;
}
	else//��������� ��� cnt=0
{
read_ADC();//������ ��� 2
	//adc_in7=2188	;//������,��� �������!
 sum7=sum7+adc_in7;	
	mean_adc_in7=(uint16_t)(sum7/arithmetical_mean);
	sum7=0;
	U_fact=(uint16_t)((mean_adc_in7*k_U)*100);//
	
	
	
//	V_flow_x100=(uint16_t)(100*pow(((mean_adc_in7*k_U-ADC_U_0*k_U)*(mean_adc_in7*k_U+ADC_U_0*k_U)),(1/n))/zn);



	V_flow_x10=(10*pow(((mean_adc_in7*k_U-ADC_U_0*k_U)*(mean_adc_in7*k_U+ADC_U_0*k_U)),(1/n))/zn);
	
	if (V_flow_x10>=1000.0) V_flow_x100=10000;//����������� ������������� ��������
    else V_flow_x100=(uint16_t)(V_flow_x10*10);

	write_data_to_massiv();
	cnt_data++;
	if (cnt_data>125)cnt_data=125;
	
	cnt4=(arithmetical_mean*4)-1;	
}




}
//-------------------------------------------------------------------------------------------------------------
void set_PWM(void)//������������� ���������� ��������� ��������������� ������
{
//
//TIM2->CCR4 =(uint16_t	)((V_flow_x100*(TIM2->ARR)/1000)*((float)k_PWM/(TIM2->ARR))) ;
TIM2->CCR4 =(uint16_t	)(V_flow_x100*10*((float)k_PWM/(TIM2->ARR))) ;
}
//-------------------------------------------------------------------------------------------------------------
void write_data_to_massiv(void)
{
for( uint8_t i=124;i>0;i--)
{
buffer_data[i]=buffer_data[i-1];
}
buffer_data[0]=V_flow_x100;

}
//-------------------------------------------------------------------------------------------------------------
void trigg_blue()
{

if (cnt>0)
{GPIOB->BSRR |= GPIO_BSRR_BS14;//��������� �����
GPIOB->BSRR |= GPIO_BSRR_BR13;

cnt--;}
else	
{GPIOB->BSRR |= GPIO_BSRR_BR14;

}//��������� �����

}
//---------------------------------------------------------------------------------------------------------------
void iwdg_init(uint16_t tw) //���������� ���������� ������
{
 // ��� IWDG_PR=7 Tmin=6,4�� RLR=T��*40/256
 IWDG->KR=0x5555; // ���� ��� ������� � ������� ��
 IWDG->PR=7; // ������ ����������� ������������ �� 4(0)�� 256(7���0b111)
 IWDG->RLR=tw*40/256; //  ������� ������������ ��������� ����� ������,tw ����� � ���
 IWDG->KR=0xAAAA; // ������������ �� ��
 IWDG->KR=0xCCCC; // ���� ������� �� ��
}
//-------------------------------------------------------------------------------------------------------------	
void Calc_Timing(void)
{
time11bit=(uint16_t) ((double)((1.3 * 11)/bautrate)*1000000);//��������� ����� �������� ������ ����� 	
	//1.3 �����, ��� ����� ���������� �������� ������
//3.5 * 11 / 115000 = 0.335 ��.
if(!timeout)
{	
timeout=(uint16_t) ((double)((3.5 * 11)/bautrate)*1000000);//��������� ����� �������� ��� ������ �������� �������� 
//����� � ���, ��� 2400 ����� 16041 ���
if (bautrate>19200)timeout=1750; //�� �������� ���������
}

}
//----------------------------------------------------------------------------------------------------------------
void add_CRC(void)//��������� CRC � ����� ���������� ������ 
{
uint16_t CRC1;
CRC1	= crc16(buffer_out,index_out );
 buffer_out[index_out]=CRC1>>8;
index_out++;	
buffer_out[index_out]=CRC1& 0xFF;
index_out++;		
}
//-----------------------------------------------------------------------------------------------------------
void send_RS485(  uint8_t* value)// ��������� �������� ������ � ���� rs-485
{
	GPIOA->BSRR |= GPIO_BSRR_BS8;//En
	USART1->SR &= ~USART_SR_TC;//����� ����� TC ������� � ���� 0
	int i=0;
	while (i<=index_out)
	{while(!(USART1->SR & USART_SR_TXE)) {}  // ������� ����� ��������� ����� ��������
	 USART1->DR = value[i]; // �������� ������ � �����, �������� ��������
	i++;
	}
 	while(!(USART1->SR & USART_SR_TC)) {} //���� ��������� ��������,��������� ����� TC

	GPIOA->BSRR |= GPIO_BSRR_BR8;//En

}
//----------------------------------------------------------------------------------------------
void Send_Modbas()//��������� CRC � ����� ���������� ������ � ���������� �� � UART
{

send_RS485(buffer_out);//�������� �� ������ �������� ������ ������������ ������	
clean_buffer(buffer_out,index_out+1);//������� �� ������� ��������� � �������
index_out=0;

}//_
//---------------------------------------------------------------------------------------------
void Reset_modul(void)//������������ �����������
{

__disable_irq ();// 
 // __set_FAULTMASK(1);// ��������� ��� ����������� ����������
NVIC_SystemReset();// ����������� �����,���������� � ����� core_cm3.h

}
//----------------------------------------------------------------------------------------------------------
void Start_Led(void)//��������� ��� ��������
{
GPIOB->BSRR |= GPIO_BSRR_BR13;
GPIOB->BSRR |= GPIO_BSRR_BR14;//��������� �������

GPIOB->BSRR |= GPIO_BSRR_BS13;
GPIOB->BSRR |= GPIO_BSRR_BS14;
delay_ms(400);
GPIOB->BSRR |= GPIO_BSRR_BR13;
GPIOB->BSRR |= GPIO_BSRR_BR14;	
delay_ms(200);		
}



//-------------------------------------------------------------------------------------------------------------
int main (void)
{



init_RCC();
	
read_costum();
Calc_Timing();//��������� �������� ��� ������ ��������� UART	
addres_new=addres;//���� ��������� �������� ��� �������	
init_GPIO();
Start_Led();//��������� ��� ��������	
init_uart();
init_Tim3();//�������� �� ����������� ��������	
init_ADC();// ��������� ������������� � ���� ���	
init_Tim2();
iwdg_init(3000);//������ ����������� ������� � ���������� 5 ���
	
__enable_irq ();// ��������� ���������� ����������, �� ��������� ���������	
	index_in=0;
		
k_U=U_0/ADC_U_0;//��������� ���������� �������� �������� 	��� � ����������

zn=((pow(k,(1.0/n)))*(pow(U_0,(2.0/n))));	//�������� ����������� � ������� ������������,���� ���, �� ��� ���������
	
cnt4=(arithmetical_mean*4)-1;
//calculation_of_coefficients();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	


	while(1)//������ 100 �� 0.01 ���
	{
		

	

	 
		
 if (flag_indata)//�������� ������
  
{ 
cnt=50;//����� ��������� ������ ����������
	flag_indata=0;
}
	 
	 
	






measurement();
//V_flow_x100=553;//!!!!!!!!!!!!!!!!!!!!!!!!!

set_PWM();
//TIM2->CCR4=10000;///!!!!!!!!!!!!!!!!!!!!!!!!!!!



trigg_red();
trigg_blue();


if(flag_reset) Reset_modul(); 

if (flag_calculation)
{calculation_of_coefficients();
write_costum();
flag_calculation=0;
Reset_modul();	

}	 
	 
IWDG->KR=0xAAAA; // ������������ �������� ����������� 
 delay_ms(10);//����� ������ ������� while
	}//end while(1)
}//


