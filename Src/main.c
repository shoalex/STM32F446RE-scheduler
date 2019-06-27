/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
/*
 * ABI: r0..r3 are caller-saved (scratch registers), R4..R12 are callee-saved.
 * It is appropriate to use R12 for a system call opcode (saved by NVIC). The
 * stack pointer points to the current extent of the stack -- it is decremented
 * before being used as index in a store. The stack grows downwards, to lower
 * addresses. When an interrupt is processed, 8 registers are stored. LR is set
 * to a special value that makes an ordinary function return into a return from
 * interrupt. The LR value indicates which stack is going to be used (process
 * or main) and can be modified before return.
 *
 *                  ____________________
 *           Stack |                    |
 *                 |                    |
 *    higher       |        R4          | <-- SP saved in TCB (64B context)
 *  addresses      |        R5          |   ^
 *      |  ^       |        R6          |   |
 *      |  |       |        R7          |   | 8 registers pushed by handler:
 *      |  |       |        R8          |   | R4..R11
 *      |  |       |        R9          |   | Full task context is now stored
 *      V  |       |        R10         |   |
 *         |       |        R11         |   |
 *     direction   |        R0          | <-- SP when SVC handler gets control
 *     of growth   |        R1          |   ^
 *                 |        R2          |   |
 *                 |        R3          |   | 8 registers are pushed by
 *                 |        R12         |   | the NVIC hardware:
 *                 |        LR (R14)    |   | xPSR, PC, LR, R12, R3..R0
 *                 |        PC (R15)    |   |
 *                 |       xPSR         |   |
 *                 |                    | <-- SP before SVC
 *                 |      (stuff)       |
 *       Stack +   |                    |
 *       StackSize |____________________|
 *
 */





/*
void scheduler_init(void);
void scheduler_yield(void);

#define SCHEDULER_NUM_TASKS 2

/*
 * Constant information about the system's tasks.
 */
/*
struct task_data {
	void (*entry_point)(void);
	void *stack;
	unsigned stack_size;
};

extern const struct task_data tasks[SCHEDULER_NUM_TASKS];


/* Current state of a task */
/*
struct task_control_block {
	/* Current stack pointer when switched out */
/*	void *sp;
	/* Task is runnable, i.e. not blocked waiting for something */
	bool runnable;
/*};

struct stack_frame {
	/* Registers that the software has to push on the stack, after the
	 * NVIC has pushed some specific registers.
	 * Corresponds to registers pushed by "STMDB %0!, {r4-r11}". */
/*	struct {
		uint32_t registers[8];
	} software_frame;

	/* Registers pushed on the stack by NVIC (hardware), before the other
	 * registers defined above. */
/*	struct {
		uint32_t r0;
		uint32_t r1;
		uint32_t r2;
		uint32_t r3;
		uint32_t r12;
		void *lr;
		void *pc;
		uint32_t psr;
	} nvic_frame;
};


static struct task_control_block tcbs[SCHEDULER_NUM_TASKS];
static uint8_t current_task;
static const uint8_t STACK_FILL = 0xa5;


static void return_from_task(void)
{
	while (true);
}

void scheduler_init(void)
{
	int i;
	for (i = 0; i < SCHEDULER_NUM_TASKS; i++) {
		//const struct task_data *task = &tasks[i];

		/* Set stack pointer to beginning of stack */
		//tcbs[i].sp = (uint8_t *)task->stack + task->stack_size;

		/* Fill stack with a canary value to ease debugging. */
		//memset(task->stack, STACK_FILL, task->stack_size);

		/* Push the same thing that a PendSV would push on the task's
		 * stack, with dummy values for the general purpose registers.
		 */
/*		tcbs[i].sp -= sizeof(struct stack_frame);
		struct stack_frame *frame = (struct stack_frame *)tcbs[i].sp;
		frame->nvic_frame.r0 = 0xff00ff00;
		frame->nvic_frame.r1 = 0xff00ff01;
		frame->nvic_frame.r2 = 0xff00ff02;
		frame->nvic_frame.r3 = 0xff00ff03;
		frame->nvic_frame.r12 = 0xff00ff0c;
		frame->nvic_frame.lr = return_from_task;
		//frame->nvic_frame.pc = task->entry_point;
		frame->nvic_frame.psr = 0x21000000; /* Default, allegedly */
/*		frame->software_frame.registers[0] = 0xff00ff04;
		frame->software_frame.registers[1] = 0xff00ff05;
		frame->software_frame.registers[2] = 0xff00ff06;
		frame->software_frame.registers[3] = 0xff00ff07;
		frame->software_frame.registers[4] = 0xff00ff08;
		frame->software_frame.registers[5] = 0xff00ff09;
		frame->software_frame.registers[6] = 0xff00ff0a;
		frame->software_frame.registers[7] = 0xff00ff0b;

		tcbs[i].runnable = true;
	}
}

/*
 * Logic for doing a task switch. This implementation simply switches to the
 * next available task in the list.
 */
/*
static void scheduler_switch(void)
{
	//assert(tcbs[current_task].sp >= tasks[current_task].stack);
	//assert(((uint8_t *)tasks[current_task].stack)[0] == STACK_FILL);

	/* Find the next runnable task in a round-robin fashion. There must
	 * always be a runnable task, such as an idle task. */
/*	do {
		current_task = (current_task + 1) % SCHEDULER_NUM_TASKS;
	} while (!tcbs[current_task].runnable);
}

/**
 * This PendSV interrupt is triggered by a task doing a system call.
 *
 * The interrupt can come from a task that uses the main or program stack
 * pointer (MSP or PSP). In the latter case we are free to do whatever we want
 * here, because the task's stack won't be affected (ISRs run on the main
 * stack). In the former case, we need to take some care with which registers
 * (R4..R11) the compiler pushes for its own use.
 */
/*void __attribute__((naked)) pend_sv_handler(void)
{
	const uint32_t RETURN_ON_PSP = 0xfffffffd;

	/* 0. NVIC has already pushed some registers on the program/main stack.
	 * We are free to modify R0..R3 and R12 without saving them again, and
	 * additionally the compiler may choose to use R4..R11 in this function.
	 * If it does so, the naked attribute will prevent it from saving those
	 * registers on the stack, so we'll just have to hope that it doesn't do
	 * anything with them before our stm or after our ldm instructions.
	 * Luckily, we don't ever intend to return to the original caller on the
	 * main stack, so this question is moot. */

	/* Read the link register */
	uint32_t lr;
/*	__asm__("MOV %0, lr" : "=r" (lr));

	if (lr & 0x4) {
		/* This PendSV call was made from a task using the PSP */

		/* 1. Push all other registers (R4..R11) on the program stack */
/*		void *psp;
		__asm__(
			/* Load PSP to a temporary register */
/*			"MRS %0, psp\n"
			/* Push context relative to the address in the temporary
			 * register, update register with resulting address */
/*			"STMDB %0!, {r4-r11}\n"
			/* Put back the new stack pointer in PSP (pointless) */
/*			"MSR psp, %0\n"
			: "=r" (psp));

		/* 2. Store that PSP in the current TCB */
/*		tcbs[current_task].sp = psp;
	} else {
		/* This PendSV call was made from a task using the MSP. This
		 * code is not equipped to return to the main task, but we store
		 * the proper registers here anyway for good form. */

		/* 1. Push all other registers (R4..R11) on the main stack */
/*		__asm__(
			/* Push context on main stack */
/*			"STMDB SP!, {r4-r11}");
	}

	/* 3. Call context switch function, changes current TCB */
/*	scheduler_switch();

	/* 4. Load PSP from TCB */
	/* 5. Pop R4..R11 from the program stack */
/*	void *psp = tcbs[current_task].sp;
	__asm__(
		/* Pop context relative temporary register, update register */
/*		"LDMFD %0!, {r4-r11}\n"
		/* Put back the stack pointer in PSP */
/*		"MSR psp, %0\n"
		: : "r" (psp));

	/* 6. Return. NVIC will pop registers and find the PC to use there. */
/*	__asm__("bx %0" : : "r"(RETURN_ON_PSP));
}*/

/*void scheduler_yield(void)
{
	/* Trigger PendSV, causing pend_sv_handler to be called immediately */
	//SCB_ICSR |= SCB_ICSR_PENDSVSET;
/*	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
}
*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef unsigned short priority_t;
typedef unsigned long (*TaskFunction_t)(char * pcName, uint16_t usStackDepth,
		void *pvParameters);
typedef struct _node {
	TaskFunction_t function;
	char * pcName;
	uint16_t usStackDepth;
	void *pvParameters;
	priority_t uxPriority;
	struct _node *next;
} node;

unsigned long TaskCreate(TaskFunction_t function, char * pcName,
		uint16_t usStackDepth, void *pvParameters, priority_t uxPriority);
bool TaskStop = false;
unsigned long TaskFunction(char * pcName, uint16_t usStackDepth,
		void *pvParameters) {


		printf("\r%s\n", pcName);

	return (unsigned long) pvParameters;
}

node *blocked = NULL;
node *running = NULL;
node *ready = NULL;

void printList() {
	node *ptr = ready;
	printf("\r[ ");

	//start from the beginning
	while (ptr != NULL) {
		printf("(%d) ", ptr->uxPriority);
		ptr = ptr->next;
	}

	printf(" ]");
}

//insert link at the first location
void insertFirst(TaskFunction_t function, char * pcName, uint16_t usStackDepth,
		void *pvParameters, int uxPriority) {
	//create a link
	node *link = (node*) malloc(sizeof(node));
	link->function = function;
	link->pcName = pcName;
	link->usStackDepth = usStackDepth;
	link->uxPriority = uxPriority;
	link->pvParameters = pvParameters;
	//point it to old first node
	link->next = ready;

	//point first to new first node
	ready = link;
}
void insertFirstNodeRunning(node* head_ref) {
	//create a link

	head_ref->next = running;

	//point first to new first node
	running = head_ref;
}
void insertFirstNodeReady(node* head_ref) {
	//create a link

	head_ref->next = ready;

	//point first to new first node
	ready = head_ref;
}
//delete first item
node* deleteFirst() {

	//save reference to first link
	node *tempLink = ready;

	//mark next to first link as first
	ready = ready->next;

	//return the deleted link
	return tempLink;
}
node* deleteFirstRunning() {

	//save reference to first link
	node *tempLink = running;

	//mark next to first link as first
	running = running->next;

	//return the deleted link
	return tempLink;
}
//is list empty
bool ReadyIsEmpty() {
	return ready == NULL;
}
bool RunningIsEmpty() {
	return running == NULL;
}
int length() {
	int length = 0;
	node *current;

	for (current = ready; current != NULL; current = current->next) {
		length++;
	}
	return length;
}

void sort() {

	int i, j, k, tempData;
	char* tempPCname;
	uint16_t tepStack;
	void* temppvParameters;
	node *current;
	node *next;

	int size = length();
	k = size;

	for (i = 0; i < size - 1; i++, k--) {
		current = ready;
		next = ready->next;

		for (j = 1; j < k; j++) {

			if (current->uxPriority < next->uxPriority) {

				tempPCname = current->pcName;
				current->pcName = next->pcName;
				next->pcName = tempPCname;

				tepStack = current->usStackDepth;
				current->usStackDepth = next->usStackDepth;
				next->usStackDepth = tepStack;

				temppvParameters = current->pvParameters;
				current->pvParameters = next->pvParameters;
				next->pvParameters = temppvParameters;

				tempData = current->uxPriority;
				current->uxPriority = next->uxPriority;
				next->uxPriority = tempData;

			}

			current = current->next;
			next = next->next;
		}
	}
}

void reverse(node** head_ref) {
	node* prev = NULL;
	node* current = *head_ref;
	node* next;

	while (current != NULL) {
		next = current->next;
		current->next = prev;
		prev = current;
		current = next;
	}

	*head_ref = prev;
}

unsigned long TaskCreate(TaskFunction_t function, char * pcName,
		uint16_t usStackDepth, void *pvParameters, priority_t uxPriority) {
	insertFirst(function, pcName, usStackDepth, pvParameters, uxPriority);
	return 0;
}

/* USER CODE END 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM6)
	{
		if(ready->uxPriority>running->uxPriority || ready->uxPriority==running->uxPriority)
		{
			//scheduler_switch();
			node *tempready=deleteFirst();
			node *temprunning=deleteFirstRunning();
			insertFirstNodeRunning(tempready);
			insertFirstNodeReady(temprunning);
			sort();
		}
	}
}
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	//scheduler_init();
	MX_TIM6_Init();
	//scheduler_yield();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	/* USER CODE END 2 */
	TaskCreate((void*) TaskFunction, "task 2", 1, (void*) 10000, 1);
	TaskCreate((void*) TaskFunction, "task 1", 1, (void*) 10000, 1);
	sort();
	printList();

	TaskFunction_t t = NULL;
	unsigned long a = 0;
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (!ReadyIsEmpty()) {
			node *temp = deleteFirst();
			insertFirstNodeRunning(temp);
		} else {
			TaskCreate((void*) TaskFunction, "idle", 1, (void*) 10000, 1);
		}
		//printList();
		while (!RunningIsEmpty()) {

			t = running->function;
			a = t(running->pcName, running->usStackDepth,
					running->pvParameters);
			printf("\r%lu", a);
			if (a == 0) {
				deleteFirstRunning();
			}

		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 24;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 64000-1;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

int _write(int file, char *data, int len) {
	// arbitrary timeout 1000
	HAL_UART_Transmit(&huart2, (uint8_t*) data, len, 1000);
	return 0;
}

int _read(int file, char *ptr, int len) {
	HAL_UART_Receive(&huart2, (uint8_t*) ptr++, 1, 0xffff);
	HAL_UART_Transmit(&huart2, (uint8_t *) (ptr - 1), 1, 10);

	if (*(ptr - 1) == 0x0D) {
		HAL_UART_Transmit(&huart2, (uint8_t *) "\n", 1, 10);
		*(ptr - 1) = '\n';
	}

	return 1;
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
