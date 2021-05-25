#include "stm32f10x.h"
#include "delay.h" //процедуры задержек
#include "crc16.h"//рассчет и проверка контрольной суммы
#include "modbus.h"//чтение запроса и формирование ответа по протоколу
#include "flash.h"//работа с памятью микрроконтроллера
#include "math.h"//стандартная библиотека си, разработанный для выполнения простых математических операций
#include "calculation_coef.h"//процедура рассчета коэффициентов n и k


#define F_APB2 72000000//Частота тактирования UART1
#define F_APB1x2 72000000//частота тактирования таймеров


//#define pi  3.1415926535
uint8_t cnt_data=0;//счетчик количества записей в буфер данных
uint16_t buffer_data[125];//буфер для накопления данных о скорости потока
uint8_t buffer_in[256];//буфер для приема данных=максимальной длине пакета в modbus RTU
uint8_t  buffer_out[256];//буфер для отправки данных=максимальной длине пакета в modbus RTU
uint16_t  index_in= 0;//текущее значение счетчика буфера приема
uint16_t index_out=0;//текущее значение счетчика буфера передачи

uint16_t adc_in7=0;                      //значение считаное с АЦП
uint16_t mean_adc_in7=0;                 //среднее значение АЦП из нескольких измерений
int16_t cnt4;// счетик для среднего арифметического

uint16_t k_PWM=9500; // корректировочный коэффицент преобразователя   PWM в напряжение
uint8_t  arithmetical_mean=10;//число замеров из которого ситаем среднее арифметиеское

float U_0=3.6	;//значение напряжения при отсутствии потока
uint16_t ADC_U_0=1340;// значение АЦП при отсутствии потока    
uint16_t V_flow_x100=0; //скорость потока       
uint16_t U_fact;//фактическое значение напряжения на выходе с датчика 
float k_U;
//Параметры для линеаризации:
float n=0.42;
float k=0.89;




float zn;//знаменатель

uint8_t flag_indata=0;//флаг корректно принятых данных для данного устройства

//параметры по умолчанию(после прошивки):
 uint16_t addres=0x10;//адрес устройства в сети,
uint16_t addres_new;//значение нового адреса после перезагрузки
//в терминале отправляем в шестнадцатиричной системе!!!10, это 0x10
uint16_t	timeout=0;//значение времени таймаута модбас(
uint32_t bautrate=115200;//скорость UART1
uint16_t stopbit=0;//0-1,2-2,3-1.5; 
uint16_t parity=0;//0-без контроля четности, 1 с битом контроля четности
uint8_t parity_status=0;//тип контроля четности 0-чет,1 нечет
uint16_t time11bit;//время на отправку 1 байта, с учетом старт,стоп, четность
int16_t cnt=0;// будем  для моргания синим использовать	
uint8_t flag_reset=0;//флаг  перезагрузки
uint8_t flag_calculation=0;//флаг  рассчета n и k







//---ПРОТОТИПЫ------------------------------------------------------------------------------------
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
void measurement(void);//замер значения на входе ацп
void init_Tim2(void);
void set_PWM(void);
void write_data_to_massiv(void);
void add_CRC(void);//добавляет CRC в конце посылаемых данных 
void send_RS485(  uint8_t* value);// процедура отправки данных в порт rs-485
void Start_Led(void);//индикация при загрузке

//------------------------------------------------------------------------------------------------
void init_uart()
{
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Разрешить тактирование портаPORTA
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // enable clock for Altirnate function
  RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;//подключаем usart
	//USART1->BRR 	= 0x1D4C;//0x9C4;//настраиваем на скорость 9600 бод USART_BRR = (fck + baudrate /2 ) / baudrate,fck=72000000Гц
	
	USART1->BRR =  (F_APB2+(bautrate/2))/bautrate;
	USART1->CR2 = stopbit<<12;
	USART1->CR1 = parity<<10;
	USART1->CR1 = parity_status<<9;
	USART1->CR1 	|= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE |	 USART_CR1_RXNEIE;
	 //ue-включить блок usart,te-вкл передатчик,re-вкл приемник,rxneie-вкл прерывание по приему байта.
	
		NVIC_EnableIRQ (USART1_IRQn);// Функции CMSIS разрешающие прерывания в NVIC
	
}
//-------------------------------------------------------------------------------------------------------------

void init_RCC()//reset & clock control
{

RCC->CR |= RCC_CR_HSEON;//Запускаем генератор HSE,внешний кварц
while (!(RCC->CR & RCC_CR_HSERDY)) {}; // Ждем готовности

RCC->CR |= RCC_CR_HSION;//Запускаем генератор HSI,внутренний rc генератор для записи в регистры флеш
while (!(RCC->CR & RCC_CR_HSIRDY)) {}; // Ждем готовности

RCC->CFGR &= ~RCC_CFGR_SW; //Очистка битов выбора источника тактового сигнала(тактируемся от внутреннего rc)
while((RCC->CFGR & RCC_CFGR_SWS)) {} //Ожидание переключения на HSI
		
FLASH->ACR |= FLASH_ACR_PRFTBE; // Enable Prefetch Buffer,бит включения буфера предварительной выборки
while(!(FLASH->ACR & FLASH_ACR_PRFTBS)) {};	
FLASH->ACR &= ~FLASH_ACR_LATENCY; // Предочистка.
//FLASH->ACR |= FLASH_ACR_LATENCY_2; // Two wait states СТРОКА 7796,КОСЯЧЕК)))
	FLASH->ACR |=2;//так более вяжется с мануалом
//FLASH->ACR |=FLASH_ACR_HLFCYA;//Flash half cycle access enable, бит включения доступа к половинному циклу

	
//PLLMul=x9,USB prescaler=1.5(по умолчанию),AHB prc=1,APB1 psc=2,APB2 psc=1,тогда SYSCLK=APB=APB2=72,APB1=36 MГц,USB=48	




	
 RCC->CFGR |= RCC_CFGR_PLLSRC; //Источником сигнала для PLL выбран HSE (внешний - кварц на 8 МГц)	
 RCC->CFGR &=~RCC_CFGR_SW; // Очистить биты SW0, SW
	RCC->CR &= ~RCC_CR_PLLON; //Отключить генератор PLL
 RCC->CFGR &= ~RCC_CFGR_PLLMULL; //Очистить PLLMULL
	RCC->CFGR |= RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_1 | RCC_CFGR_PLLMULL_2; //Коефициент умножения 9  (будет 72 МГЦ)	
 RCC->CFGR |= RCC_CFGR_PPRE1_2;	//APB1 psc=2
	
	 
	
 RCC->CR |= RCC_CR_PLLON; //Включить генератор PLL
 while(!(RCC->CR & RCC_CR_PLLRDY)) {} //Ожидание готовности PLL

	
 RCC->CFGR |= RCC_CFGR_SW_PLL; //Выбрать источником тактового сигнала PLL	 
while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {} //Ожидание переключения на PLL 
	
}

//-----------------------------------------------------------------------------------------------------------

void init_GPIO()
{
	  RCC->APB2ENR|= RCC_APB2ENR_AFIOEN; // enable clock for Altirnate function
		AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE;//!!освобождаю pb3 и pb4 и pa15 от jtag по умолчанию после загрузки
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
     // A15
  	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; 
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

// B13 pin индикатор красный
	 GPIOB->CRH &= ~(GPIO_CRH_CNF13_0 | GPIO_CRH_CNF13_1);//запись 00 в cnf13,двухтактный выход,push-pull
	//01-открытый колллектор,10и 11 альтернативная функция,в режиме входа:00-аналоговый,01-Z сост,10 подтяжка PxODR=1 К+,=0 К GND
  GPIOB->CRH |= GPIO_CRH_MODE13_1;//запись 1 в mode13 ,10,макс скорость 2 МГц
	//00-режим входа,01-10МГц,11-50МГц
	GPIOB->BSRR |= GPIO_BSRR_BR13;
	
	
	
	
	// A8 pin EN
	 GPIOA->CRH &= ~(GPIO_CRH_CNF8_0 | GPIO_CRH_CNF8_1);//запись 00 в cnf13,двухтактный выход,push-pull
	//01-открытый колллектор,10и 11 альтернативная функция,в режиме входа:00-аналоговый,01-Z сост,10 подтяжка PxODR=1 К+,=0 К GND
  GPIOA->CRH |= GPIO_CRH_MODE8_1;//запись 1 в mode13 ,10,макс скорость 2 МГц
	//00-режим входа,01-10МГц,11-50МГц
	

GPIOA->BSRR |= GPIO_BSRR_BR8;

	// A9 pin TX
	 GPIOA->CRH &= ~(GPIO_CRH_CNF9_0 | GPIO_CRH_CNF9_1);//запись 00 в cnf13,двухтактный выход,push-pull
	GPIOA->CRH		|= GPIO_CRH_CNF9_1;
	
	//01-открытый колллектор,10и 11 альтернативная функция,в режиме входа:00-аналоговый,01-Z сост,10 подтяжка PxODR=1 К+,=0 К GND
  GPIOA->CRH |= GPIO_CRH_MODE9_1;//запись 1 в mode13 ,10,макс скорость 2 МГц
	//00-режим входа,01-10МГц,11-50МГц
	

GPIOA->BSRR |= GPIO_BSRR_BS9;

	// A10 pin RX
		
GPIOA->CRH		&= ~GPIO_CRH_CNF10;																// Clear CNF bit 10
GPIOA->CRH		|= GPIO_CRH_CNF10_0;																// 01-Z сост
GPIOA->CRH		&= ~GPIO_CRH_MODE10;	                             //00-режим входа
	
	
	
	
	
	// B14 индикатор зеленый
 GPIOB->CRH &= ~(GPIO_CRH_CNF14_0 | GPIO_CRH_CNF14_1);//запись 00 в cnf13,двухтактный выход,push-pull
	//01-открытый колллектор,10и 11 альтернативная функция,в режиме входа:00-аналоговый,01-Z сост,10 подтяжка PxODR=1 К+,=0 К GND
  GPIOB->CRH |= GPIO_CRH_MODE14_1;//запись 1 в mode13 ,10,макс скорость 2 МГц
	//00-режим входа,01-10МГц,11-50МГц
	GPIOB->BSRR |= GPIO_BSRR_BR14;




}
//--------------------------------------------------------------------------------------------------------------
void init_Tim2(void)//Работает в качестве ЦАП
{ 
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;//Вкл тактирование порта A
RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
RCC->APB1ENR |=RCC_APB1ENR_TIM2EN;// Включаем тактирование таймера TIM2.APB1 частота 36 МГц


  // выход импульса  на TIM2_ch4 PA3
  GPIOA->CRL &= ~GPIO_CRL_MODE2; 
  GPIOA->CRL &= ~GPIO_CRL_CNF3;// MODE5 = 0x03 (11b)-50МГц; CNF5 = 0x02 (10b)
  GPIOA->CRL |= (GPIO_CRL_CNF3_1 | GPIO_CRL_MODE3);//двухтактный выход альтернативной функции

 //(если PPRE1!=0 то таймет тактируется с удвоенной частотой шины APB1,36*2 MГц
TIM2->PSC = 1;//делитель равен 1
TIM2->ARR = 10000;//значение перезагрузки,максимального значения при счете вверх или начала вниз
TIM2->CCR4 = 0;//коэф. заполнения	
TIM2->CCER |= TIM_CCER_CC4E;//вкл 4 канал
TIM2->BDTR |= TIM_BDTR_MOE;//MOE: Main output enable	
TIM2->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; //режим шим 1 4 канала//PWM mode 1, прямой ШИМ 4 канал	
	
TIM2->CR1 &= ~TIM_CR1_DIR;//считаем вверх,бит направления счета таймера(по умолчаниюравен0)
TIM2->CR1 &= ~TIM_CR1_CMS;//выравнивание по фронту, Fast PWM
TIM2->CR1 |= TIM_CR1_CEN;//включаем счётчик,бит разрешения работы таймера



}

//------------------------------------------------------------------------------------------------------------------
void USART1_IRQHandler(void)//обработчик прерываний по приходу данных в порт
{
 if (USART1->SR & USART_SR_RXNE)//если пришли данные(поднят флаг RXNE)..,сброс вручную или после чтения DR
	{
		
		buffer_in[index_in] = USART1->DR;				
			index_in++;			
		TIM3->CNT = 0;//при каждом принятом байте перезапускаем счетчик, не давая ему войти в прерывание по переполнению
		TIM3->CR1 |= (TIM_CR1_CEN|TIM_CR1_OPM);	//Запускаем таймер записью бита CEN и устанавливаем режим Одного прохода установкой бита OPM
	
		}
 	
}


//-------------------------------------------------------------------------------------------------------------

void clean_buffer( uint8_t *adr_buffer, uint32_t byte_cnt)//очистка буфера данных
{
	 		while( byte_cnt--)
	*adr_buffer++=0;
	}

//-----------------------------------------------------------------------------------------------------------
void init_ADC()
{
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Разрешить тактирование портаPORTA
//A7 = adc_12_in 7
// Сконфигурировать PORT A7 как аналоговый вход
GPIOA->CRL &= ~GPIO_CRL_MODE7; // Очистить биты MODE 00 в режим входа
GPIOA->CRL &= ~GPIO_CRL_CNF7; // Очистить биты CNF 00-аналоговый режим(есть еще z-сост)
RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Включить тактирование АЦП
	
RCC->CFGR |=RCC_CFGR_ADCPRE_DIV8; // Задать значение делителя тактовой частоты 2бита,00-делитель на 2
//adc тактируется apb2 через делитель 2,4,6,8.(максимальная частота 14мгц)
ADC1->CR1 = 0; // Обнулить регистр управления
ADC1->SQR1 = 0; // Обнулить регистр SQR1
ADC1->CR2 |= ADC_CR2_CAL; // Пуск калибровки бит CAL регистра CR2,после калибровки сбросится в 0
while (!(ADC1->CR2 & ADC_CR2_CAL)){} // Ждать окончания калибровки,пока не сбросится бит
//калибровка проводится один раз до включения ацп
ADC1->SQR3 |=7; // номер первого канала при опросе(он же последний)
ADC1->CR2 |= ADC_CR2_EXTSEL; // Выбрать источником запуска разряд SWSTART.111 16..19 бит
 ADC1->CR2 |= ADC_CR2_EXTTRIG;// Разрешить внешний запуск регулярного канала
ADC1->CR2 |= ADC_CR2_ADON; // Включить АЦП
}
//-------------------------------------------------------------------------------------------------------------
void read_ADC()
{
 ADC1->CR2 |= ADC_CR2_SWSTART; //start converting
while (!(ADC1->SR & ADC_SR_EOC) ){} //Wait
adc_in7= ADC1->DR;  //read variable

}

//-------------------------------------------------------------------------------------------------------
void init_Tim3()//отвечает за обнаружение момента,когда надо начинать обработку данных.
{ RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
     TIM3->PSC = (uint16_t)((F_APB1x2/1000000)-1);	//устанавливаем предделитель, чтоб таймер работал на частоте 1МЦ
     
	   TIM3->CCR1=(uint16_t)(time11bit);
	
	   TIM3->ARR = (uint16_t)(timeout);	//устанавливаем значение переполнения таймера, а значит и значение при котором генерируется Событие обновления 
     TIM3->EGR |= TIM_EGR_UG;	//Генерируем Событие обновления для записи данных в регистры PSC и ARR

	       TIM3->CNT = 0;
         TIM3->SR = 0; // Сбрасываем флаги прерываний
         TIM3->DIER |= TIM_DIER_CC1IE; // разрешаем прерывание придостижении значения 

NVIC_SetPriority(TIM3_IRQn, 15);//низкий приоритет
NVIC_EnableIRQ(TIM3_IRQn);


}

//----------------------------------------------------------------------------------------------------------------

void TIM3_IRQHandler()//обнаружен таймаут и надо срочно обрабатывать даннные
	{
  
     if (TIM3->SR &TIM_SR_CC1IF )//если счетчик достиг значения длины 11 бит
       //либо читаем принятые данные, либо очищаем буфер приемаи сбрасываем индекс приема  
				//TIM3->CNT = 0;
     {  
if ((buffer_in[0]==addres)||(buffer_in[0]==0))//если это данные для нашего адреса или широковещательная команда
	 {if	( verification_CRCin())
	 {flag_indata=1;//это наши данные, их надо обработать
   
		 
//		delay_us(timeout*0.33);//0.67-было дано Tim3

	Modbus_handler_data();
  add_CRC();//добавили контрольную сумму
		 //		Modbus_handler_data(); //время выполнения где то 50 мкс
//		 
		 
		 //это если данные пришли по RS-485 и отвечать надо по RS-485
		//delay_us(timeout*0.33+1.33*timeout);//0.67-было дано Tim3
    //delay_us(timeout-time11bit-50);
		while (!(TIM3->SR &TIM_SR_UIF ) ){}//Ждем, пока счетчик не досчитает до конца, тем самым достигнет значения тайминг
		 
		 
		 Send_Modbas(); 
		 
	 } 		 
		 
		 else //если не прошли верификацию
	 clean_buffer(buffer_in,index_in);
			index_in=0;
	 }
		else//если это не наши данные
   clean_buffer(buffer_in,index_in);
	index_in=0;

       
			}
	

	  TIM3->SR = 0; // Сбрасываем флаги прерываний
}
//-----------------------------------------------------------------------------------------------------------	
void trigg_red(void)
{
	static int8_t a = 0;
	static	int16_t cnt3=100;
if (cnt3)
cnt3--;
 else 
 {	 cnt3=47;//периодичность моргания красного светодиода
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
void measurement(void)//замер значения на входе ацп,вычисление среднего из 5 через равные промежутки времени
{
	
		float V_flow_x10=0;
  static	int32_t sum7=0;
	//int16_t cnt4=((arithmetical_mean*4)-1);
if (cnt4)


{	//if ((cnt4==4)||(cnt4==8)||(cnt4==12)||(cnt4==16))
 if ((!(cnt4%4))&&(!(cnt4==0)))//если нет остатка от деления и не ноль

	{read_ADC();//запуск ацп 1
		//adc_in7=2188;
	 sum7=sum7+adc_in7;	
	}
	cnt4--;
}
	else//последнее при cnt=0
{
read_ADC();//запуск ацп 2
	//adc_in7=2188	;//убрать,это отладка!
 sum7=sum7+adc_in7;	
	mean_adc_in7=(uint16_t)(sum7/arithmetical_mean);
	sum7=0;
	U_fact=(uint16_t)((mean_adc_in7*k_U)*100);//
	
	
	
//	V_flow_x100=(uint16_t)(100*pow(((mean_adc_in7*k_U-ADC_U_0*k_U)*(mean_adc_in7*k_U+ADC_U_0*k_U)),(1/n))/zn);



	V_flow_x10=(10*pow(((mean_adc_in7*k_U-ADC_U_0*k_U)*(mean_adc_in7*k_U+ADC_U_0*k_U)),(1/n))/zn);
	
	if (V_flow_x10>=1000.0) V_flow_x100=10000;//ограничение передаваемого значения
    else V_flow_x100=(uint16_t)(V_flow_x10*10);

	write_data_to_massiv();
	cnt_data++;
	if (cnt_data>125)cnt_data=125;
	
	cnt4=(arithmetical_mean*4)-1;	
}




}
//-------------------------------------------------------------------------------------------------------------
void set_PWM(void)//устанавливает скважность импульсов пропорционально потоку
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
{GPIOB->BSRR |= GPIO_BSRR_BS14;//индикатор синий
GPIOB->BSRR |= GPIO_BSRR_BR13;

cnt--;}
else	
{GPIOB->BSRR |= GPIO_BSRR_BR14;

}//индикатор синий

}
//---------------------------------------------------------------------------------------------------------------
void iwdg_init(uint16_t tw) //подключаем сторожевой таймер
{
 // Для IWDG_PR=7 Tmin=6,4мс RLR=Tмс*40/256
 IWDG->KR=0x5555; // Ключ для доступа к таймеру СМ
 IWDG->PR=7; // запись коэффицента преддедителя от 4(0)до 256(7или0b111)
 IWDG->RLR=tw*40/256; //  регистр перезагрузки вычисляем число тактов,tw время в сек
 IWDG->KR=0xAAAA; // Перезагрузка см СМ
 IWDG->KR=0xCCCC; // Пуск таймера см СМ
}
//-------------------------------------------------------------------------------------------------------------	
void Calc_Timing(void)
{
time11bit=(uint16_t) ((double)((1.3 * 11)/bautrate)*1000000);//вычисляем время передачи одного байта 	
	//1.3 иначе, при смене количества стоповых виснем
//3.5 * 11 / 115000 = 0.335 мс.
if(!timeout)
{	
timeout=(uint16_t) ((double)((3.5 * 11)/bautrate)*1000000);//вычисляем время таймаута для данной скорости передачи 
//время в мкс, при 2400 равен 16041 мкс
if (bautrate>19200)timeout=1750; //см описание протокола
}

}
//----------------------------------------------------------------------------------------------------------------
void add_CRC(void)//добавляет CRC в конце посылаемых данных 
{
uint16_t CRC1;
CRC1	= crc16(buffer_out,index_out );
 buffer_out[index_out]=CRC1>>8;
index_out++;	
buffer_out[index_out]=CRC1& 0xFF;
index_out++;		
}
//-----------------------------------------------------------------------------------------------------------
void send_RS485(  uint8_t* value)// процедура отправки данных в порт rs-485
{
	GPIOA->BSRR |= GPIO_BSRR_BS8;//En
	USART1->SR &= ~USART_SR_TC;//сброс флага TC записью в него 0
	int i=0;
	while (i<=index_out)
	{while(!(USART1->SR & USART_SR_TXE)) {}  // Ожидаем когда очистится буфер передачи
	 USART1->DR = value[i]; // Помещаем данные в буфер, начинаем передачу
	i++;
	}
 	while(!(USART1->SR & USART_SR_TC)) {} //ждем окончания передачи,установки флага TC

	GPIOA->BSRR |= GPIO_BSRR_BR8;//En

}
//----------------------------------------------------------------------------------------------
void Send_Modbas()//добавляет CRC в конце посылаемых данных и отправляет их в UART
{

send_RS485(buffer_out);//указываю на первое значение буфера отправляемых данных	
clean_buffer(buffer_out,index_out+1);//индексы по разному считаются у буферов
index_out=0;

}//_
//---------------------------------------------------------------------------------------------
void Reset_modul(void)//перезагрузка контроллера
{

__disable_irq ();// 
 // __set_FAULTMASK(1);// Запрещаем все маскируемые прерывания
NVIC_SystemReset();// Программный сброс,определена в файле core_cm3.h

}
//----------------------------------------------------------------------------------------------------------
void Start_Led(void)//индикация при загрузке
{
GPIOB->BSRR |= GPIO_BSRR_BR13;
GPIOB->BSRR |= GPIO_BSRR_BR14;//индикатор зеленый

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
Calc_Timing();//вычисляем тайминги для разных скоростей UART	
addres_new=addres;//чтоб корректно выводить при запросе	
init_GPIO();
Start_Led();//индикация при загрузке	
init_uart();
init_Tim3();//отвечает за обнаружение таймаута	
init_ADC();// Выполнить инициализацию и пуск АЦП	
init_Tim2();
iwdg_init(3000);//запуск сторожевого таймера с интервалом 5 сек
	
__enable_irq ();// Разрешаем глобальные прерывания, по умолчанию разрешены	
	index_in=0;
		
k_U=U_0/ADC_U_0;//вычисляем коэффицент перевода значения 	АЦП в напряжение

zn=((pow(k,(1.0/n)))*(pow(U_0,(2.0/n))));	//значение знаменателя в формуле линеаризации,один раз, тк оно постоянно
	
cnt4=(arithmetical_mean*4)-1;
//calculation_of_coefficients();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	


	while(1)//период 100 Гц 0.01 сек
	{
		

	

	 
		
 if (flag_indata)//получены данные
  
{ 
cnt=50;//время включения синего светодиода
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
	 
IWDG->KR=0xAAAA; // Перезагрузка счетчика сторожевого 
 delay_ms(10);//время одного прохода while
	}//end while(1)
}//


