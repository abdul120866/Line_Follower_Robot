#include "stm32f1xx.h"
#include "Clock_Config.h"
#include "Delay_ms.h"
#include "stdio.h"
#include "myPID.h"



void rightpwm (void);
void leftpwm (void);
void PidController(void);
void leftsensor (int T_2_3,int T_2_4);
void rightsensor (int T_3_3,int T_3_4);
void sharp_turn(void);

  static int SensorRead = 0x00000000;
  int MotorLeftSpeed;
  int MotorRightSpeed;
  const uint8_t MotorRightSpeedMax = 100;
	const uint8_t MotorLeftSpeedMax = 100;
	//const uint8_t MotorRightSpeedInit = 50;
	//const uint8_t basespeedl = 50;
	//const int ARR_const = 10;

void generalInit(void){


  PID->position =0;
  PID-> Kp = 0.0190; //0.0190 Proportional Gain
	PID-> Ki = 0.00250 ; //0.00250 Integral Gain
	PID-> Kd = 10;//10  Derivative Gain

	PID-> proportional = 0;
	PID->integrational= 0;
	PID->Derivative= 0;
  PID-> tot_error = 0;
 // PID-> maxspeedr = 100;
	//PID-> maxspeedl = 100;
  	PID-> MotorRightSpeedBase = 40; // Base speed
	PID-> MotorLeftSpeedBase = 40; // Base speed
	PID-> Var_ARR = 10;
  PID-> lastError = 0;
//  PID-> i= 0;
//  PID-> k= 0;
   PID-> motorspeed= 0;
	PID-> Variable = 0;
  PID-> active_count =0;
	PID-> last_end=0;
  PID->  white_count=0;
 for (int i = 0; i < 10; ++i) {
    PID->errordatas[i] = 0;
}
  for (int abc = 0; abc < 10; ++abc) {
    PID->info[abc] = 0;
}
}
void GPIOConfig (void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable GPIOA Clock
	RCC->APB2ENR |=RCC_APB2ENR_IOPBEN; // Enable GPIOB Clock
	////////////////////////////////////////////
	GPIOA->CRL &= ~(0xffffffff); //Clear bits
	GPIOB->CRL &= ~(0xff); //Clear bits
	GPIOB->CRH &= ~(0xffffffff); //Clear bits
	////////////////////////////////////////////
	GPIOB->CRL |= 0x000000BB;// // PB0,1 ALTERNATE FUNCTION 50MHZ (TIMER2)
  GPIOB->CRH |= 0x0000BB00;// PB10,11 ALTERNATE FUNCTION 50MHZ (TIMER3)

}

void motor_config (int MotorLeftSpeed, int MotorRightSpeed) // Adjusting the Motors duty cyle i.e. speeds
{
		if(MotorLeftSpeed < 0)
	{
	  rightsensor ((PID->Var_ARR)*0,-1*(PID->Var_ARR)*MotorLeftSpeed);
	}
	else
	{
	  rightsensor ((PID->Var_ARR)*MotorLeftSpeed,(PID->Var_ARR)*0);
	}
	if(MotorRightSpeed < 0)
	{
		leftsensor ((PID->Var_ARR)*0,-1*(PID->Var_ARR)*MotorRightSpeed);
	}
	else
	{
		leftsensor ((PID->Var_ARR)*MotorRightSpeed,(PID->Var_ARR)*0);
	}

}

void Sensor_config (void) // GPIO Config for QTR8RC SENSOR
{
	// FIRST CHARGE THE CAPACITORS BY WRITING ODR 1
	GPIOA->CRL |= 0x33333333;  // set pin as output
	GPIOA->CRL &= ~(GPIO_CRL_CNF0) & ~(GPIO_CRL_CNF1) & ~(GPIO_CRL_CNF2) & ~(GPIO_CRL_CNF3) & ~(GPIO_CRL_CNF4) & ~(GPIO_CRL_CNF5) & ~(GPIO_CRL_CNF6) & ~(GPIO_CRL_CNF7);
	GPIOA->ODR |= 0xFF;		// set pin as high

	delay_us(12);	// Wait a few microsec. (from data sheet)

	// Then set the sensors as input to read the data (position)
	GPIOA->CRL &= ~(3<<28) & ~(3<<24) & ~(3<<20) & ~(3<<16) & ~(3<<12) & ~(3<<8) & ~(3<<4) & ~(3<<0); // set pin as input
	GPIOA->CRL |= (2<<30) | (2<<26) | (2<<22) | (2<<18) | (2<<14) | (2<<10) | (2<<6) | (2<<2);	// set pin with pull-up/pull-down
	delay_ms(6);

}

int Sensor_Read()
{
	Sensor_config();
    int real_pos = 0;
		int real_var = 0;
    int real_active = 0;

		// Multiply each sensor digital input by 1000 to create a position value
		if(((GPIOA->IDR & (1<<0))==(1<<0)))  //sensor1
			{
				SensorRead = 0x00000001;
				real_pos+=1000;
				real_active++;
				PID-> last_end=1; // Means robot tend to right
			}
		if(((GPIOA->IDR & (1<<1))==(1<<1)))  //sensor2
			{
				SensorRead = 0x00000010;
        real_pos+=2000;
				real_active++;
			}
		if(((GPIOA->IDR & (1<<2))==(1<<2)))  //sensor3
			{
				SensorRead = 0x00000100;
				real_pos+=3000;
				real_active++;
			}
		if(((GPIOA->IDR & (1<<3))==(1<<3)))  //sensor4
			{
				SensorRead = 0x00001000;
				real_pos+=4000;
				real_active++;
			}
		if(((GPIOA->IDR & (1<<4))==(1<<4)))  //sensor5
			{
				SensorRead = 0x00010000;
				real_pos+=5000;
				real_active++;
			}
		if(((GPIOA->IDR & (1<<5))==(1<<5)))  //sensor6
			{
				SensorRead = 0x00100000;
        real_pos+=6000;
				real_active++;
			}
		if(((GPIOA->IDR & (1<<6))==(1<<6)))  //sensor7
			{
				SensorRead = 0x01000000;
				real_pos+=7000;
				real_active++;
			}
		if(((GPIOA->IDR & (1<<7))==(1<<7)))  //sensor8
			{
				SensorRead = 0x10000000;
				real_pos+=8000;
				real_active++;
				PID->last_end=0;		// Means robot tend to left
			}
		PID->info[0] = (GPIOA->IDR & (1<<0));
		PID->info[1] = (GPIOA->IDR & (1<<1));
		PID->info[2] = (GPIOA->IDR & (1<<2));
		PID->info[3] = (GPIOA->IDR & (1<<3));
		PID->info[4] = (GPIOA->IDR & (1<<4));
		PID->info[5] = (GPIOA->IDR & (1<<5));
		PID->info[6] = (GPIOA->IDR & (1<<6));
		PID->info[7] = (GPIOA->IDR & (1<<7));
			PID->position = real_pos/real_active;
        PID->active_count = real_active;
				PID->Variable = real_var;
		if (PID->active_count == 0) // No black line detecting
		{
			PID->white_count++;
		}
		else
		{
			PID->white_count = 0;
		}
			return real_pos/real_active;
}

void rightpwm (void) // PWM Configuration for TIM2
{
 RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable timer2
 AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_FULLREMAP;
 TIM2->CCER |= TIM_CCER_CC4E;	// capture/compare 4 enabled
 TIM2->CCER |= TIM_CCER_CC3E;	// capture/compare 3 enabled
 TIM2->CR1 |= TIM_CR1_ARPE; //Auto-reload preload enable
 TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE; //lost 4
 TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE; //lost 3

 //PWM freq = Fclk/PSC/ARR  72MHz/1000
 //PWM Duty = CCR4/ARR

 TIM2->PSC = 72-1; //72 MHz divided by:
 TIM2->ARR = 1000; //1000, to get 72e6/1e3

}

void leftpwm (void) // PWM Configuration for TIM3
{
 RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
 RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable timer3

 TIM3->CCER |= TIM_CCER_CC4E;	// capture/compare 4 enabled
 TIM3->CCER |= TIM_CCER_CC3E;	// capture/compare 3 enabled
 TIM3->CR1 |= TIM_CR1_ARPE; //Auto-reload preload enable
 TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE; //lost 4
 TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE; //lost 3

 //PWM freq = Fclk/PSC/ARR  72MHz/1000
 //PWM Duty = CCR4/ARR

 TIM3->PSC = 72-1; //72 MHz divided by:
 TIM3->ARR = 1000; //1000, to get 72e6/1e3

}

void leftsensor (int T_2_3,int T_2_4)
{

TIM2->CCR3= T_2_3; //duty cycle ch3
TIM2->CCR4= T_2_4; //duty cycle ch4
TIM2->EGR |= TIM_EGR_UG; //
TIM2->CR1 |= TIM_CR1_CEN; //
}

void rightsensor (int T_3_3,int T_3_4)
{

TIM3->CCR4= T_3_4; //duty cycle ch4
TIM3->CCR3= T_3_3; //duty cycle ch3
TIM3->EGR |= TIM_EGR_UG; //
TIM3->CR1 |= TIM_CR1_CEN; //
}



void sharp_turn() {	// Detecting the sharp turn
	if (PID->white_count < 25) // Count a bit to make sure that there is no black line
	{
		if (PID->last_end == 1){
			MotorLeftSpeed=15;
			MotorRightSpeed=-25;
			motor_config(MotorLeftSpeed, MotorRightSpeed);
		}
		else if(PID->last_end==0)
		{
			MotorLeftSpeed=-25;
			MotorRightSpeed=15;
			motor_config(MotorLeftSpeed, MotorRightSpeed);
		}
		 else;
	}
	else
	{
		if (PID->last_end == 1){
			MotorLeftSpeed=70;
			MotorRightSpeed=-50;
		}
		else
		{
			MotorLeftSpeed=-70;
			MotorRightSpeed=50;
		}
	}
}



void past_errors (int error)  // Store the last error for using Integral Gain
{
  for (int aa = 9; aa > 0; aa--) {
      PID->errordatas[aa] = PID->errordatas[aa-1];
      PID->errordatas[0] = error;
	}
}

int errors_sum (int index)  // Sum the stored errors to determine Integral Gain
{
  int sum = 0;
  for (PID->k = 0; PID->k < index; PID->k ++)
  {
      sum +=PID->errordatas[PID->k];
  }
  return sum;
}

void PidController()  // PID Controller Design
	{
	PID->position = Sensor_Read();
  int error = 4500 - (PID->position); // Reference is 4500 which is the average of two middle sensor
	past_errors(error);

  PID->proportional = error;
  PID->integrational = errors_sum(5);
  PID->Derivative = error - PID->lastError;

  PID->lastError = error;

  PID->motorspeed = (PID->proportional)*(PID->Kp) + (PID->integrational)*(PID->Ki) + (PID->Derivative)*(PID->Kd);


  MotorLeftSpeed = PID->MotorLeftSpeedBase + PID->motorspeed;// Control the left motor speed by using PID value with respect to error i.e. position
  int MotorRightSpeed = PID->MotorRightSpeedBase - PID->motorspeed; // Control the right motor speed by using PID value with respect to error i.e. position

  if (MotorLeftSpeed > MotorLeftSpeedMax)
	{
    MotorLeftSpeed = MotorLeftSpeedMax;
	}
  if (MotorRightSpeed > MotorRightSpeedMax)
	{
    MotorRightSpeed = MotorRightSpeedMax;
	}


	if (PID->active_count==0) // At the end of line turn around and start the line again
	{
		sharp_turn();
	}

	motor_config(MotorLeftSpeed, MotorRightSpeed);
}



int main()
{
  generalInit();
	initClockPLL();
	GPIOConfig();
	TIM2Config();
	rightpwm ();
  leftpwm();

while(1)
{
	 	PidController();
}
return 0;
}
