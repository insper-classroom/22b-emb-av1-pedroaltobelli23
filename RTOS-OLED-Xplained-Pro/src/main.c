#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/* Definindo os inputs do motor*/
#define IN1_PIO     PIOD
#define IN1_PIO_ID  ID_PIOD
#define IN1_PIO_PIN 30
#define IN1_PIO_PIN_MASK (1 << IN1_PIO_PIN)

#define IN2_PIO     PIOA
#define IN2_PIO_ID  ID_PIOA
#define IN2_PIO_PIN 6
#define IN2_PIO_PIN_MASK (1 << IN2_PIO_PIN)

#define IN3_PIO     PIOC
#define IN3_PIO_ID  ID_PIOC
#define IN3_PIO_PIN 19
#define IN3_PIO_PIN_MASK (1 << IN3_PIO_PIN)

#define IN4_PIO     PIOA
#define IN4_PIO_ID  ID_PIOA
#define IN4_PIO_PIN 2
#define IN4_PIO_PIN_MASK (1 << IN4_PIO_PIN)

/* DEfinindo os botoes*/
#define PRI_PIO			PIOD
#define PRI_PIO_ID		ID_PIOD
#define PRI_PIO_IDX		28
#define PRI_PIO_IDX_MASK	(1u << PRI_PIO_IDX)

#define SEC_PIO			PIOC
#define SEC_PIO_ID		ID_PIOC
#define SEC_PIO_IDX		31
#define SEC_PIO_IDX_MASK	(1u << SEC_PIO_IDX)

#define TER_PIO			PIOA
#define TER_PIO_ID		ID_PIOA
#define TER_PIO_IDX		19
#define TER_PIO_IDX_MASK	(1u << TER_PIO_IDX)


/** RTOS  */
#define TASK_MODO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MODO_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MOTOR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MOTOR_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
static void BUT_init(void);

QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;


/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void apaga_tela() {
	gfx_mono_draw_filled_rect(0, 0, 120, 30, GFX_PIXEL_CLR);
}

void but1_callback(void) {
	int id1=45;
	xQueueSendFromISR(xQueueModo,&id1,0);
}

void but2_callback(void) {
	int id2=90;
	xQueueSendFromISR(xQueueModo,&id2,0);
}

void but3_callback(void) {
	int id3=100;
	xQueueSendFromISR(xQueueModo,&id3,0);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
	gfx_mono_draw_string("olaaa", 0, 20, &sysfont);
	BUT_init();

	for (;;)  {
		int id=0;
		int passos = 0;
		if(xQueueReceive(xQueueModo,&id,1000)){
			if(id==45){
				apaga_tela();
				gfx_mono_draw_string("45", 0, 20, &sysfont);
			}else if(id==100){
				apaga_tela();
				gfx_mono_draw_string("100", 0, 20, &sysfont);
			}else if(id==90){
				apaga_tela();
				gfx_mono_draw_string("90", 0, 20, &sysfont);
			}
		}

	}
}

static void task_motor(void *pvParameters) {
	for (;;)  {

	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	pmc_enable_periph_clk(PRI_PIO_ID);
	pmc_enable_periph_clk(SEC_PIO_ID);
	pmc_enable_periph_clk(TER_PIO_ID);
	
	pmc_enable_periph_clk(IN1_PIO_ID);
	pmc_enable_periph_clk(IN2_PIO_ID);
	pmc_enable_periph_clk(IN3_PIO_ID);
	pmc_enable_periph_clk(IN4_PIO_ID);
	
	pio_configure(PRI_PIO, PIO_INPUT, PRI_PIO_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(SEC_PIO, PIO_INPUT, SEC_PIO_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(TER_PIO, PIO_INPUT, TER_PIO_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	
	pio_configure(IN1_PIO, PIO_OUTPUT_0, IN1_PIO_PIN_MASK, PIO_DEFAULT);
	pio_configure(IN2_PIO, PIO_OUTPUT_0, IN2_PIO_PIN_MASK, PIO_DEFAULT);
	pio_configure(IN3_PIO, PIO_OUTPUT_0, IN3_PIO_PIN_MASK, PIO_DEFAULT);
	pio_configure(IN4_PIO, PIO_OUTPUT_0, IN3_PIO_PIN_MASK, PIO_DEFAULT);
	
	pio_handler_set(PRI_PIO, PRI_PIO_ID, PRI_PIO_IDX_MASK, PIO_IT_FALL_EDGE,
	but1_callback);
	pio_handler_set(SEC_PIO, SEC_PIO_ID, SEC_PIO_IDX_MASK, PIO_IT_FALL_EDGE,
	but2_callback);
	pio_handler_set(TER_PIO, TER_PIO_ID, TER_PIO_IDX_MASK, PIO_IT_FALL_EDGE,
	but3_callback);
	
	pio_enable_interrupt(PRI_PIO,PRI_PIO_IDX_MASK);
	pio_enable_interrupt(SEC_PIO,SEC_PIO_IDX_MASK);
	pio_enable_interrupt(TER_PIO,TER_PIO_IDX_MASK);
	
	pio_get_interrupt_status(PRI_PIO);
    pio_get_interrupt_status(SEC_PIO);
    pio_get_interrupt_status(TER_PIO);
	
	
	//pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	NVIC_EnableIRQ(PRI_PIO_ID);
	NVIC_SetPriority(PRI_PIO_ID, 4);

	NVIC_EnableIRQ(SEC_PIO_ID);
	NVIC_SetPriority(SEC_PIO_ID, 4);

	NVIC_EnableIRQ(TER_PIO_ID);
	NVIC_SetPriority(TER_PIO_ID, 4);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	
	xQueueModo = xQueueCreate(32,sizeof(int));
	/* Create task to control oled */
	if (xTaskCreate(task_modo, "modo", TASK_MODO_STACK_SIZE, NULL, TASK_MODO_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create modo task\r\n");
	}
	
	if (xTaskCreate(task_motor, "motor", TASK_MOTOR_STACK_SIZE, NULL, TASK_MOTOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create motor task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
