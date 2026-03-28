/*
 * u_sensor.c
 *
 * Created on: 2026
 * Author: nikiemao
 *
 * Description:
 * Reads three ultrasonic distance sensors via UART and displays
 * the measured distances on the LCD.
 *
 * Hardware Pin Assignments:
 *   PA2  - USART2_TX  (Sensor 1), AF7
 *   PA3  - USART2_RX  (Sensor 1), AF7
 *   PA7  - USART3_TX  (Sensor 2), AF7
 *   PA5  - USART3_RX  (Sensor 2), AF7
 *   PC12 - UART5_TX   (Sensor 3), AF8
 *   PD2  - UART5_RX   (Sensor 3), AF8
 *   PB1  - Pi trigger output (HIGH = trigger Pi)
 *   PA4  - Pi done input (HIGH = Pi finished)
 *
 * All three UARTs: 115200 baud, 8N1, no flow control
 */

#include "stm32u5xx.h"
#include "main.h"
#include "msoe_stm_clock.h"
#include "msoe_stm_delay.h"
#include "msoe_stm_lcd.h"

typedef enum {
    STATE_IDLE,       // Listening for objects
    STATE_PROCESSING  // Waiting for Pi to finish (Debouncing)
} SystemState_t;

/* ---- Global variables ---- */
uint8_t trigger_cmd = 0x55;  // command byte to trigger ultrasonic sensor reading
int distances[3] = {0, 0, 0}; // measured distances for 3 sensors (in mm)

/* ---- Object detection parameters ---- */
#define MIN_DETECTION_DIST_MM   100    // minimum distance to consider valid (ignore noise)
#define MAX_DETECTION_DIST_MM   1000   // maximum distance to consider as "object present"
#define DETECTION_TIME_MS       3000   // time object must be present before triggering Pi (3 seconds)
#define LOOP_TIME_MS            140    // approximate time per main loop iteration (ms)

volatile uint32_t object_detect_start_time = 0;  // timestamp when object first detected
volatile uint8_t  object_detected_flag = 0;      // 1 if object currently in range
volatile uint8_t  pi_triggered = 0;              // 1 if we already triggered the Pi for this detection

/* ---- Function prototypes ---- */
void initGPIO(void);
void initUSART2(void);
void initUSART3(void);
void initUART5(void);
void uart_tx_byte(USART_TypeDef *uart, uint8_t data);
int  uart_rx_byte(USART_TypeDef *uart, uint8_t *data, uint32_t timeout_ms);
void uart_flush_rx(USART_TypeDef *uart);
void uart_clear_errors(USART_TypeDef *uart);
int  read_sensor(USART_TypeDef *uart, int sensor_idx);
uint8_t check_object_in_range(int dist);
void process_object_detection(void);
void trigger_raspberry_pi(void);
void clear_raspberry_pi_trigger(void);
uint8_t is_pi_done(void);

volatile SystemState_t current_state = STATE_IDLE;

int main(void)
{
	/* Clock setup to 160 MHz using msoe library */
	msoe_clk_setup(160);
	msoe_delay_init();

	/* Enable peripheral clocks */
	RCC->AHB3ENR  |= RCC_AHB3ENR_PWREN;    // enable PWR clock
	PWR->SVMCR    |= (1 << 29);             // set IO2SV bit to allow GPIOG use
	RCC->AHB2ENR1 |= 0x000000FF;            // enable GPIOA - GPIOH clocks

	/* Enable UART peripheral clocks on APB1 */
	RCC->APB1ENR1 |= (1 << 17);  // USART2 clock enable
	RCC->APB1ENR1 |= (1 << 18);  // USART3 clock enable
	RCC->APB1ENR1 |= (1 << 20);  // UART5  clock enable

	/* Initialize peripherals */
	initGPIO();
	LCD_IO_Init();
	LCD_clear();
	LCD_print_str(1, 0, "System Booting");

	initUSART2();
	initUSART3();
	initUART5();

	/* Main loop - poll sensors and display distances */
	while (1)
	{
		LCD_print_str(2, 0, "Waiting...");

		distances[0] = read_sensor(USART2, 0);
		msoe_delay_ms(20);

		/* Read Sensor 2 (USART3 - PA7/PA5) */
		distances[1] = read_sensor(USART3, 1);
		msoe_delay_ms(20);

		/* Read Sensor 3 (UART5 - PC12/PD2) */
		distances[2] = read_sensor(UART5, 2);

		/* Display all three distances on LCD */
		LCD_print_str(3, 0, "S1:");
		LCD_print_udec5(3, 3, distances[0]);
		LCD_print_str(3, 9, "mm");

		LCD_print_str(4, 0, "S2:");
		LCD_print_udec5(4, 3, distances[1]);
		LCD_print_str(4, 9, "mm");

		LCD_print_str(5, 0, "S3:");
		LCD_print_udec5(5, 3, distances[2]);
		LCD_print_str(5, 9, "mm");

		/* Debug: Show PA4 pin state on LCD row 7 so we can verify button works */
		if (is_pi_done()) {
			LCD_print_str(7, 0, "PA4=HIGH (btn) ");
		} else {
			LCD_print_str(7, 0, "PA4=LOW        ");
		}

		/* Process object detection and trigger Raspberry Pi if needed */
		process_object_detection();

		msoe_delay_ms(100); // poll rate control
	}

	return 0;
}

/*
 * Function to initialize GPIO pins for UART alternate functions and control pins
 * Inputs: none
 * Outputs: none
 *
 * USART2: PA2 (TX, AF7), PA3 (RX, AF7)
 * USART3: PA7 (TX, AF7), PA5 (RX, AF7)
 * UART5:  PC12 (TX, AF8), PD2 (RX, AF8)
 * PB1:    Output - Pi trigger
 * PA4:    Input  - Pi done signal (pull-down, active HIGH)
 */
void initGPIO(void)
{
	/* ---- USART2 pins: PA2 (TX), PA3 (RX) ---- */
	GPIOA->MODER &= ~((3 << 4) | (3 << 6));
	GPIOA->MODER |=  ((2 << 4) | (2 << 6));
	GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12));
	GPIOA->AFR[0] |=  ((7 << 8) | (7 << 12));
	GPIOA->OTYPER &= ~((1 << 2) | (1 << 3));
	GPIOA->OSPEEDR |= ((3 << 4) | (3 << 6));
	GPIOA->PUPDR &= ~((3 << 4) | (3 << 6));
	GPIOA->PUPDR |=  ((1 << 4) | (1 << 6));

	/* ---- USART3 pins: PA7 (TX), PA5 (RX) ---- */
	GPIOA->MODER &= ~((3 << 10) | (3 << 14));
	GPIOA->MODER |=  ((2 << 10) | (2 << 14));
	GPIOA->AFR[0] &= ~((0xF << 20) | (0xF << 28));
	GPIOA->AFR[0] |=  ((7 << 20) | (7 << 28));
	GPIOA->OTYPER  &= ~((1 << 5) | (1 << 7));
	GPIOA->OSPEEDR |=  ((3 << 10) | (3 << 14));
	GPIOA->PUPDR   &= ~((3 << 10) | (3 << 14));
	GPIOA->PUPDR   |=  ((1 << 10) | (1 << 14));

	/* ---- UART5 pins: PC12 (TX), PD2 (RX) ---- */
	GPIOC->MODER &= ~(3 << 24);
	GPIOC->MODER |=  (2 << 24);
	GPIOC->AFR[1] &= ~(0xF << 16);
	GPIOC->AFR[1] |=  (8 << 16);
	GPIOC->OTYPER  &= ~(1 << 12);
	GPIOC->OSPEEDR |=  (3 << 24);
	GPIOC->PUPDR   &= ~(3 << 24);
	GPIOC->PUPDR   |=  (1 << 24);

	GPIOD->MODER &= ~(3 << 4);
	GPIOD->MODER |=  (2 << 4);
	GPIOD->AFR[0] &= ~(0xF << 8);
	GPIOD->AFR[0] |=  (8 << 8);
	GPIOD->OTYPER  &= ~(1 << 2);
	GPIOD->OSPEEDR |=  (3 << 4);
	GPIOD->PUPDR   &= ~(3 << 4);
	GPIOD->PUPDR   |=  (1 << 4);

	/* ---- PB1: Output to trigger Raspberry Pi ---- */
	GPIOB->MODER &= ~(3 << 2);
	GPIOB->MODER |=  (1 << 2);    // output mode

	/* ---- PA4: Input for Pi "done" signal ---- */
	GPIOA->MODER &= ~(3 << 8);   // input mode (0b00)
	GPIOA->PUPDR &= ~(3 << 8);
	GPIOA->PUPDR |=  (2 << 8);   // pull-down (reads LOW when not driven)

	return;
}

/*
 * Function to initialize USART2 for sensor 1
 * 115200 baud, 8N1, TX and RX enabled
 */
void initUSART2(void)
{
	USART2->CR1 &= ~(1 << 0);
	USART2->CR1 &= ~((1 << 12) | (1 << 28));
	USART2->CR1 &= ~(1 << 10);
	USART2->CR1 &= ~(1 << 15);
	USART2->CR2 &= ~(3 << 12);
	USART2->BRR = 1388;
	USART2->CR3 &= ~((1 << 8) | (1 << 9));
	USART2->CR1 &= ~(1 << 29);
	USART2->CR1 |= (1 << 3);
	USART2->CR1 |= (1 << 2);
	USART2->CR1 |= (1 << 0);
}

/*
 * Function to initialize USART3 for sensor 2
 * 115200 baud, 8N1, TX and RX enabled
 */
void initUSART3(void)
{
	USART3->CR1 &= ~(1 << 0);
	USART3->CR1 &= ~((1 << 12) | (1 << 28));
	USART3->CR1 &= ~(1 << 10);
	USART3->CR1 &= ~(1 << 15);
	USART3->CR2 &= ~(3 << 12);
	USART3->BRR  = 1388;
	USART3->CR3 &= ~((1 << 8) | (1 << 9));
	USART3->CR1 &= ~(1 << 29);
	USART3->CR1 |= (1 << 3);
	USART3->CR1 |= (1 << 2);
	USART3->CR1 |= (1 << 0);
}

/*
 * Function to initialize UART5 for sensor 3
 * 115200 baud, 8N1, TX and RX enabled
 */
void initUART5(void)
{
	UART5->CR1 &= ~(1 << 0);
	UART5->CR1 &= ~((1 << 12) | (1 << 28));
	UART5->CR1 &= ~(1 << 10);
	UART5->CR1 &= ~(1 << 15);
	UART5->CR2 &= ~(3 << 12);
	UART5->BRR  = 1388;
	UART5->CR3 &= ~((1 << 8) | (1 << 9));
	UART5->CR1 &= ~(1 << 29);
	UART5->CR1 |= (1 << 3);
	UART5->CR1 |= (1 << 2);
	UART5->CR1 |= (1 << 0);
}

/*
 * Transmit one byte over UART (blocking, polled)
 */
void uart_tx_byte(USART_TypeDef *uart, uint8_t data)
{
	while (!(uart->ISR & (1 << 7)));
	uart->TDR = data;
	while (!(uart->ISR & (1 << 6)));
}

/*
 * Receive one byte over UART with timeout (blocking, polled)
 * Returns 0 on success, -1 on timeout
 */
int uart_rx_byte(USART_TypeDef *uart, uint8_t *data, uint32_t timeout_ms)
{
	volatile uint32_t count = 0;
	uint32_t max_count = timeout_ms * 16000;

	while (!(uart->ISR & (1 << 5))) {
		count++;
		if (count >= max_count) {
			return -1;
		}
	}

	*data = (uint8_t)(uart->RDR & 0xFF);
	return 0;
}

/*
 * Flush any pending data in the UART RX buffer
 */
void uart_flush_rx(USART_TypeDef *uart)
{
	volatile uint8_t dummy;
	while (uart->ISR & (1 << 5)) {
		dummy = (uint8_t)(uart->RDR & 0xFF);
		(void)dummy;
	}
}

/*
 * Clear all UART error flags (ORE, FE, NE, PE)
 */
void uart_clear_errors(USART_TypeDef *uart)
{
	uart->ICR |= (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
}

/*
 * Read one distance measurement from an ultrasonic sensor.
 * Sends trigger (0x55), receives 4-byte frame [0xFF][DATA_H][DATA_L][CHECKSUM].
 * Returns distance in mm, or last good value on error.
 */
int read_sensor(USART_TypeDef *uart, int sensor_idx)
{
	static int last_distance[3] = {0, 0, 0};

	if (sensor_idx < 0 || sensor_idx > 2) sensor_idx = 0;

	uart_clear_errors(uart);
	uart_flush_rx(uart);

	uart_tx_byte(uart, trigger_cmd);

	uint8_t buf[4];
	for (int i = 0; i < 4; i++) {
		if (uart_rx_byte(uart, &buf[i], 100) != 0) {
			return last_distance[sensor_idx];
		}
	}

	if (buf[0] != 0xFF) {
		return last_distance[sensor_idx];
	}

	uint8_t checksum = (buf[0] + buf[1] + buf[2]) & 0xFF;
	if (buf[3] != checksum) {
		return last_distance[sensor_idx];
	}

	last_distance[sensor_idx] = (buf[1] << 8) | buf[2];
	return last_distance[sensor_idx];
}

/*
 * Check if a distance reading indicates an object in range
 */
uint8_t check_object_in_range(int dist)
{
	return (dist >= MIN_DETECTION_DIST_MM && dist <= MAX_DETECTION_DIST_MM);
}

/*
 * Trigger the Raspberry Pi by setting PB1 high
 */
void trigger_raspberry_pi(void)
{
	GPIOB->BSRR = (1 << 1);
	LCD_print_str(6, 0, "Pi TRIGGERED   ");
}

/*
 * Clear the Raspberry Pi trigger by setting PB1 low
 */
void clear_raspberry_pi_trigger(void)
{
	GPIOB->BSRR = (1 << 17);
	LCD_print_str(6, 0, "Pi IDLE        ");
}

/*
 * Check if Raspberry Pi has signaled it's done (PA4 HIGH)
 */
uint8_t is_pi_done(void)
{
	return (GPIOA->IDR & (1 << 4)) ? 1 : 0;
}

/*
 * Process object detection state machine with Pi handshake.
 *
 * STATE_IDLE:
 *   - Monitor sensors for object in range
 *   - If detected for DETECTION_TIME_MS, trigger Pi -> STATE_PROCESSING
 *
 * STATE_PROCESSING:
 *   - Wait for PA4 HIGH (Pi done)
 *   - Clear PB1, return to STATE_IDLE
 */
void process_object_detection(void)
{
	static uint32_t tick_counter = 0;

	uint32_t ticks_needed = DETECTION_TIME_MS / LOOP_TIME_MS;
	if (ticks_needed < 1) ticks_needed = 1;

	uint8_t object_now = check_object_in_range(distances[0]) ||
	                     check_object_in_range(distances[1]) ||
	                     check_object_in_range(distances[2]);

	switch (current_state) {

	case STATE_IDLE:
		if (object_now) {
			if (!object_detected_flag) {
				object_detected_flag = 1;
				object_detect_start_time = tick_counter;
				LCD_print_str(6, 0, "DETECTING...   ");
			} else {
				uint32_t elapsed_ticks = tick_counter - object_detect_start_time;
				uint32_t elapsed_ms = elapsed_ticks * LOOP_TIME_MS;
				uint32_t remaining_ms = (DETECTION_TIME_MS > elapsed_ms) ? (DETECTION_TIME_MS - elapsed_ms) : 0;
				LCD_print_str(6, 0, "WAIT ");
				LCD_print_udec5(6, 5, remaining_ms / 1000);
				LCD_print_str(6, 10, "s    ");

				if (elapsed_ticks >= ticks_needed) {
					trigger_raspberry_pi();
					pi_triggered = 1;
					current_state = STATE_PROCESSING;
				}
			}
		} else {
			if (object_detected_flag) {
				object_detected_flag = 0;
				LCD_print_str(6, 0, "READY          ");
			}
		}
		break;

	case STATE_PROCESSING:
		if (is_pi_done()) {
			clear_raspberry_pi_trigger();
			pi_triggered = 0;
			object_detected_flag = 0;
			current_state = STATE_IDLE;
			LCD_print_str(6, 0, "Pi DONE        ");
		} else {
			LCD_print_str(6, 0, "Pi WORKING...  ");
		}
		break;
	}

	tick_counter++;
}
