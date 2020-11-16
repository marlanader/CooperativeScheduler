#include "stm32f4xx.h"  
#include "system_stm32f4xx.h"
typedef struct _TaskDefinition {
    uint32_t id;                /* Task id */
    uint32_t priority;          /* Task priority */
} TaskDefinition;
typedef struct _DelayedTaskDefinition {
    uint32_t id;                /* Task id */
    int delay_time;          /* Task Delay Time */
	uint32_t priority;  
} DelayedTaskDefinition;

static uint8_t error_msg[] = "Error  !!\n";
static uint8_t error_msg2[] = "No Task to Run !!\n";
static uint8_t error_msg3[] = "Error in DelayedQueue!!\n";
static uint8_t error_msg4[] = "Error in Handler\n";
static uint8_t pressedMsg[] = "Button is pressed !!\n";
static uint8_t releasedMsg[] = "Button is released !!\n";
static char buttonPressed = 1;
static char timerFlag = 0;
static volatile uint8_t stopFlag = 0;
static TaskDefinition ready_queue[8]; //Ready Queue - 8 is the max number of tasks
static DelayedTaskDefinition delayed_queue[8]; //Delayed Queue - 8 is the max number of tasks
static int delayed_queue_length=0; //delayed queue length
static int queue_length=0;//current length of the ready queue
static uint32_t running_task=0; //the id of the running task
static int tick_counter=0;

void SysTick_Handler(void);
void USART2_IRQHandler(void);
void EXTI0_IRQHandler(void);
static void sendUART(uint8_t * data, uint32_t length);
static uint8_t receiveUART(void);
void QueTask(void *Task,uint32_t,uint32_t);
static void  Dispatch(void);
static void  Sort(int);
static void QueDelayTask(int,uint32_t,uint32_t);
static void  Init(void);
static void ReRunMe(int,uint32_t,uint32_t);
 void TaskA(void);
 void TaskB(void);
 void TaskC(void);
  void TaskD(void);
	void TaskE(void);
	void TaskF(void);
	void TaskG(void);
	void TaskH(void);
void RunTask(void);


static void Init()
{
  
	for (int i=0;i<queue_length;i++)
	{
	  ready_queue[i].id=0;
		ready_queue[i].priority=0;
	}
	for (int j=0;j<delayed_queue_length;j++)
	{
	  delayed_queue[j].id=0;
		delayed_queue[j].delay_time=0;
	}
	running_task=0;
}
//Sort Function
static void Sort(int ready)
{
	int i,j;

	if (ready)
							{
	for (i = 0; i < queue_length; i++) 
     {
            for (j = i + 1; j < queue_length; j++) 
            {
							
									uint32_t temp,temp2;
                if (ready_queue[i].priority > ready_queue[j].priority) 
                {
                    temp = ready_queue[i].id;
									  temp2 = ready_queue[i].priority;
                    ready_queue[i].id = ready_queue[j].id;
									  ready_queue[i].priority = ready_queue[j].priority;
                    ready_queue[j].id = temp;
									  ready_queue[j].priority = temp2;
									
                }
							}
						}
					}
							else if(ready==0)
							{
								for (i = 0; i < delayed_queue_length; i++) 
          {
            for (j = i + 1; j < delayed_queue_length; j++) 
            {
								int temp4;
								uint32_t temp3,temp5;
							  if (delayed_queue[i].delay_time > delayed_queue[j].delay_time) 
                {
                    temp3 = delayed_queue[i].id;
									  temp4 = delayed_queue[i].delay_time;
                    temp5 =delayed_queue[i].priority;
									  delayed_queue[i].id = delayed_queue[j].id;
									  delayed_queue[i].delay_time = delayed_queue[j].delay_time;
									  delayed_queue[i].priority=delayed_queue[j].priority;
                    delayed_queue[j].id = temp3;
									  delayed_queue[j].delay_time = temp4;
									delayed_queue[j].priority=temp5;
                }
							
							
							}
            }
        }

}
static void QueDelayTask(int time, uint32_t delayed_task_id,uint32_t delayed_task_priority )
{
	
if(delayed_queue_length>0)																					
{	int i;
	int x=0;															
  Sort(x);

    /* Find the first task having priority greater than the current task */
    for (i = 0; i < delayed_queue_length; i++) {
        if (time< delayed_queue[i].delay_time) //next priority here is the priority of the new task
					{
					
            break;
        }
    }

    /* Rearrange the queue */
    for (int j = delayed_queue_length; j > i; j--) {

        /* Shift the elements to the right */
        delayed_queue[j] = delayed_queue[j - 1];
    }
		 delayed_queue[i] = (DelayedTaskDefinition){.id = delayed_task_id, //new task id
			                                    .delay_time=time,
                                          .priority = delayed_task_priority}; //new task priority
}		
else
{
  delayed_queue[0] = (DelayedTaskDefinition){.id = delayed_task_id, //new task id
			                                    .delay_time=time,
                                          .priority = delayed_task_priority}; //new task priority
}
delayed_queue_length++;

}
static void ReRunMe(int x,uint32_t priority,uint32_t id)
{

	if (x!=0)
	{
		
	QueDelayTask(x,id,priority);	
	}
	else if(x==0)
	{
		switch((int) id)
		{
		  case 1:QueTask(TaskA,priority,id);break;
			case 2:QueTask(TaskB,priority,id);break;
      case 3:QueTask(TaskC,priority,id);break;
			case 4:QueTask(TaskD,priority,id);break;
			case 5:QueTask(TaskE,priority,id);break;
			case 6:QueTask(TaskF,priority,id);break;
			case 7:QueTask(TaskG,priority,id);break;
			case 8:QueTask(TaskH,priority,id);break;
		  default:sendUART(error_msg3, sizeof(error_msg3));
		}
	  
	}
	
}
void RunTask()
{

  switch(running_task)
		{
		  case 1:TaskA();break;
			case 2:TaskB();break;	
   		case 3:TaskC();break;
      case 4:TaskD();break;
			case 5:TaskE();break;		
			case 6:TaskF();break;	
      case 7:TaskG();break;
			case 8:TaskH();		
      default:sendUART(error_msg, sizeof(error_msg));
		}
}
	
//QueueTask function
 void QueTask(void *Task, uint32_t task_priority, uint32_t task_ID)
{
	int i,j;
    
	if(queue_length>0)
	{
		 Sort(1);
    /* Find the first task having priority greater than the current task */
    for (i = 0; i < queue_length; i++) {
        if (task_priority< ready_queue[i].priority) //next priority here is the priority of the new task
					{
					
            break;
        }
    }

    /* Rearrange the queue */
    for (j = queue_length; j > i; j--) {

        /* Shift the elements to the right */
        ready_queue[j] = ready_queue[j - 1];
    }
		 ready_queue[i] = (TaskDefinition){.id = task_ID, //new task id
                                          .priority = task_priority}; //new task priority
		
	}

  else if (queue_length==0)
	{
	  ready_queue[0] = (TaskDefinition){.id = task_ID, //new task id
                                          .priority = task_priority}; //new task priority
	}		
	queue_length++;
}

//Dispatch Function
static void Dispatch(){
	int i;

	if (queue_length>1 )
	{
		
		running_task=ready_queue[0].id; //this is the id of the current running task
		RunTask();
	for (i=0; i < queue_length-1 ; i++) {

        /* Shift the tasks one block behind */
        ready_queue[i] = ready_queue[i + 1];
    }
	  
		
		running_task=0;
		queue_length--;
		 /* Update the length of the queue */
	}
	else if(queue_length==1)
	{ 
		// only one task is in the ready queue
		running_task=ready_queue[0].id; //this is the id of the current running task
		RunTask();
		running_task=0;
		queue_length--;
	}
  else if(queue_length<1)
	{
	  // sendUART(error_msg2, sizeof(error_msg2));
	}
	}
void SysTick_Handler(void)  {

for (int n=0;n<delayed_queue_length;n++)
	{
		if(delayed_queue[n].delay_time!=0)
		{delayed_queue[n].delay_time=delayed_queue[n].delay_time-1;  //decrement delay every tick
	}
}

	for (int n=0;n<delayed_queue_length;n++)
	{
		
		
		if(delayed_queue[n].delay_time==0)
		{
			
			switch(delayed_queue[n].id)
			{
			  case 1: QueTask(TaskA,delayed_queue[n].priority,delayed_queue[n].id);break;
				case 2: QueTask(TaskB,delayed_queue[n].priority,delayed_queue[n].id);break;
				case 3: QueTask(TaskC,delayed_queue[n].priority,delayed_queue[n].id);break;
				case 4: QueTask(TaskD,delayed_queue[n].priority,delayed_queue[n].id);break;
				case 5: QueTask(TaskE,delayed_queue[n].priority,delayed_queue[n].id);break;
				case 6: QueTask(TaskF,delayed_queue[n].priority,delayed_queue[n].id);break;
				case 7: QueTask(TaskG,delayed_queue[n].priority,delayed_queue[n].id);break;
				case 8: QueTask(TaskH,delayed_queue[n].priority,delayed_queue[n].id);break;
				default: sendUART(error_msg4, sizeof(error_msg4));
			}
		
			for (int i=0; i < delayed_queue_length-1 ; i++) {

        /* Shift the tasks one block behind */
        delayed_queue[i] = delayed_queue[i + 1];
        }

	   delayed_queue_length--;
		   
		
     }

	
	}
		
		
	tick_counter++;
	
}

void USART2_IRQHandler(void) {
	/* pause/resume UART messages */
	stopFlag = !stopFlag;
	
	/* dummy read */
	(void)receiveUART();
}

void EXTI0_IRQHandler(void) {
		/* Clear interrupt request */
		EXTI->PR |= 0x01;
		/* send msg indicating button state */
		if(buttonPressed)
		{
				sendUART(pressedMsg, sizeof(pressedMsg));
				buttonPressed = 0;
		}
		else
		{
				sendUART(releasedMsg, sizeof(releasedMsg));
				buttonPressed = 1;
		}
}

static void sendUART(uint8_t * data, uint32_t length)
{
	 for (uint32_t i=0; i<length; ++i){
      // add new data without messing up DR register
      uint32_t value = (USART2->DR & 0x00) | data[i];
		  // send data
			USART2->DR = value;
      // busy wait for transmit complete
      while(!(USART2->SR & (1 << 6)));
		  // delay
      for(uint32_t j=0; j<1000; ++j);
      }
}

static uint8_t receiveUART()
{
	  // extract data
	  uint8_t data = USART2->DR & 0xFF;
	
	  return data;
}

static void gpioInit()
{	
    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC->AHB1ENR |= (1 << 0);

    // set pin modes as alternate mode 7 (pins 2 and 3)
    // USART2 TX and RX pins are PA2 and PA3 respectively
    GPIOA->MODER &= ~(0xFU << 4); // Reset bits 4:5 for PA2 and 6:7 for PA3
    GPIOA->MODER |=  (0xAU << 4); // Set   bits 4:5 for PA2 and 6:7 for PA3 to alternate mode (10)

    // set pin modes as high speed
    GPIOA->OSPEEDR |= 0x000000A0; // Set pin 2/3 to high speed mode (0b10)

    // choose AF7 for USART2 in Alternate Function registers
    GPIOA->AFR[0] |= (0x7 << 8); // for pin A2
    GPIOA->AFR[0] |= (0x7 << 12); // for pin A3
}

static void uartInit()
{
	
    // enable USART2 clock, bit 17 on APB1ENR
    RCC->APB1ENR |= (1 << 17);
	
	  // USART2 TX enable, TE bit 3
    USART2->CR1 |= (1 << 3);

    // USART2 rx enable, RE bit 2
    USART2->CR1 |= (1 << 2);
	
	  // USART2 rx interrupt, RXNEIE bit 5
    USART2->CR1 |= (1 << 5);

    // baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)
    //   for fCK = 16 Mhz, baud = 115200, OVER8 = 0
    //   USARTDIV = 16Mhz / 115200 / 16 = 8.6805
    // Fraction : 16*0.6805 = 11 (multiply fraction with 16)
    // Mantissa : 8
    // 12-bit mantissa and 4-bit fraction
    USART2->BRR |= (8 << 4);
    USART2->BRR |= 11;

    // enable usart2 - UE, bit 13
    USART2->CR1 |= (1 << 13);
}
 void TaskA(void)
{ 
	static uint8_t number_str[2];
	sprintf(number_str, "%d", tick_counter);
	sendUART(number_str, sizeof(number_str));
	static uint8_t run_msg[] = {":Task A running\n"};
  sendUART(run_msg, sizeof(run_msg));

	ReRunMe(3,3,1);

}
 void TaskB(void)
{
	static uint8_t number_str[2];
	sprintf(number_str, "%d", tick_counter);
	sendUART(number_str, sizeof(number_str));
  static uint8_t run_msg2[] = ":Task B running\n";
  sendUART(run_msg2, sizeof(run_msg2));

	ReRunMe(1,4,2);

}
void TaskC(void)
{
	static uint8_t number_str[2];
	sprintf(number_str, "%d", tick_counter);
	sendUART(number_str, sizeof(number_str));
  static uint8_t run_msg3[] = ":Task C running\n";
  sendUART(run_msg3, sizeof(run_msg3));
	ReRunMe(4,2,3);

}
void TaskD(void)
{
	static uint8_t number_str[2];
	sprintf(number_str, "%d", tick_counter);
	sendUART(number_str, sizeof(number_str));
  static uint8_t run_msg3[] = ":Task D running\n";
  sendUART(run_msg3, sizeof(run_msg3));
	ReRunMe(11,3,4);

}
void TaskE(void)
{
	static uint8_t number_str[2];
	sprintf(number_str, "%d", tick_counter);
	sendUART(number_str, sizeof(number_str));
  static uint8_t run_msg3[] = ":Task E running\n";
  sendUART(run_msg3, sizeof(run_msg3));


}
void TaskF(void)
{
	static uint8_t number_str[2];
	sprintf(number_str, "%d", tick_counter);
	sendUART(number_str, sizeof(number_str));
  static uint8_t run_msg3[] = ":Task F running\n";
  sendUART(run_msg3, sizeof(run_msg3));


}
void TaskG(void)
{
	static uint8_t number_str[2];
	sprintf(number_str, "%d", tick_counter);
	sendUART(number_str, sizeof(number_str));
  static uint8_t run_msg3[] = ":Task G running\n";
  sendUART(run_msg3, sizeof(run_msg3));


}
void TaskH(void)
{
	static uint8_t number_str[2];
	sprintf(number_str, "%d", tick_counter);
	sendUART(number_str, sizeof(number_str));
  static uint8_t run_msg3[] = ":Task H running\n";
  sendUART(run_msg3, sizeof(run_msg3));


}
int main()
{	
	  /* startup code initialization */
	  SystemInit();
	  SystemCoreClockUpdate();
	  /* intialize UART */
	  gpioInit();
		/* intialize UART */
	  uartInit();
	  /* enable SysTick timer to interrupt system every second */
	  SysTick_Config(SystemCoreClock/10);
	  /* enable interrupt controller for USART2 external interrupt */
		NVIC_EnableIRQ(USART2_IRQn);
		/* Unmask External interrupt 0 */
		EXTI->IMR |= 0x0001;
	  /* Enable rising and falling edge triggering for External interrupt 0 */
		EXTI->RTSR |= 0x0001;
		EXTI->FTSR |= 0x0001;
	  /* enable interrupt controller for External interrupt 0 */
		NVIC_EnableIRQ(EXTI0_IRQn);
	  Init(); // initialize the scheduler data structures
   
	  QueTask(TaskB,4,2); 
    //QueTask(TaskC,2,3);
		//QueTask(TaskD,3,4);
		QueTask(TaskA,3,1); 
    while (1)
		{
	      Dispatch();
			
      
		}
		
	  
		
		/*SysTick_100ms;*/
}
