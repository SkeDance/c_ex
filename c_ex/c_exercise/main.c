#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <string.h>

#define F_CPU 8000000UL
#define PWM_FREQ 2000
#define BAUD 9600L
#define UBRRL_value (F_CPU/(BAUD*16))-1

#define LED1_on()	PORTD |= (1 << 5)
#define LED1_off()	PORTD &=~ (1 << 5)
#define LED2_on()	PORTB |= (1 << 2)
#define LED2_off()	PORTB &=~ (1 << 2)
#define DEVICE_ADDRESS 0x01// адрес устройства


volatile uint8_t received_data[3];
volatile uint8_t received_index = 0;
volatile uint8_t led_on = 0;
volatile uint16_t ms_count = 0;//счетчик для светодиода
volatile uint16_t ms_count2 = 0;//счетчик для светодиода
volatile uint16_t voltage;//принимаемая уставка
volatile int z = 0;
volatile int msg_rec;//флаг приема посылки
volatile uint16_t control_signal;
volatile uint16_t setpoint;

int32_t integral = 0; // Интегральная составляющая
int32_t integral_max = 1000000; // Максимальное значение интегральной составляющей
int32_t integral_min = -1000000; // Минимальное значение интегральной составляющей

int16_t previous_error = 0; // Предыдущая ошибка для вычисления производной

uint16_t dt = 1; // Интервал времени в миллисекундах

float Kp = 0.07; // значение пропорционального коэффициента
float Ki = 0.027; // Интегральный коэффициент
float Kd = 10; // Дифференциальный коэффициент

//message $01$03$E8$D5 - для 1000 мВ

uint8_t crc8(const uint8_t *data, uint8_t len){
	uint8_t crc = 0x00;
	while(len--){
		uint8_t extract = *data++;
		for(uint8_t tempI = 8; tempI; tempI--){
			uint8_t sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if(sum){
				crc ^= 0x8C;
			}
			extract >>= 1;
		}
	}
	return crc;
}

ISR(USART_RXC_vect){
	received_data[received_index] = UDR;
	received_index++;
	
	if(received_index == 4){
		
		if(received_data[0] == DEVICE_ADDRESS){
			
			if(crc8(received_data, 3) == received_data[3]){
				led_on = 1;
				msg_rec = 1;
				LED1_on();// индикатор успешного приема посылки
				TCNT0 = 0;// сброс таймера
				voltage = (received_data[1] << 8) | received_data[2];// декодирование принимаемой уставки из посылки
			}
		}
		memset(received_data, 0, sizeof(received_data));//зануление массива
		received_index = 0;//сброс индекса принимаемого байта посылки
	}
}

void init_USART(){
	UBRRL = UBRRL_value;
	UBRRH = UBRRL_value >> 8;
	UCSRB |= (1 << TXEN)|(1 << RXEN);
	UCSRC |= (1 << URSEL)|(1 << UCSZ0)|(1 << UCSZ1);
	UCSRB |= (1 << RXCIE);
	sei();
}

void timer0_init() {
	TCCR0 = 0b00000101;
	TCNT0 = 0;
	TIMSK |= (1 << TOIE0);
}


ISR(TIMER0_OVF_vect) {
	if (led_on) {
		z++;
		if(z == 10){
			led_on = 0;
			LED1_off();
			z = 0;
		}
	}
}

void init_timer2() {
	// Настройка таймера 2 для генерации прерываний каждые 1 мс
	TCCR2 |= (1 << WGM21);
	OCR2 = 124; // Сравниваемое значение (для 1 мс при тактовой частоте 8 МГц и предделителе 64)
	TIMSK |= (1 << OCIE2); // Включение прерывания при совпадении с OCR2
	TCCR2 |= (1 << CS22); // Предделитель 64 (для 8 МГц это дает 1 мс)
}

ISR(TIMER2_COMP_vect) {
	ms_count++;
	ms_count2++;
}

void delay_ms(uint16_t ms) {
	ms_count = 0;
	while (ms_count < ms);
}

void init_adc() {
	
	ADCSRA |= (1 << ADEN);//ADC enable
	ADCSRA |= (1 << ADFR);//ADC free running 
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);//division factor
	
	//AVcc Voltage reference - 5V
	ADMUX |= (1 << REFS0);
	ADMUX  &=~ (1 << REFS1);
	
	ADMUX &=~ (1 << ADLAR);//right adjust of ADC register
	
	//ADC channel 1
	ADMUX |= (1 << MUX0);
	ADMUX &=~ (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
	
	ADCSRA |= (1 << ADSC);//start conversion	
}

uint16_t read_adc() {
	while(1){
		if(ADCSRA & (1 << 4)){
			ADCSRA |= (1 << 4);	
		}
		return ADC;	
	}
}

void init_led() {
	DDRB |= (1 << PB0);//PB0 output
	DDRB |= (1 << 1);//PWM output
	DDRB |= (1 << 2);//LED2
	DDRD |= (1 << PD5);//LED1
	DDRC &=~ (1 << 1);//ADC1 input	
	
}

void init_pwm() {
	
	//Mode 8 - Phase and Frequency Correct PWM
	TCCR1A &=~ (1<<WGM10);
	TCCR1A &=~ (1<<WGM11);
	TCCR1B &=~ (1<<WGM12);
	TCCR1B |= (1<<WGM13);
	
	//PWM Frequency
	ICR1 = 65535;
	
	//prescaler 8
	TCCR1B &=~ (1<<CS11);
	TCCR1B |= (1<<CS10);
	TCCR1B &=~ (1<<CS12);
	
	//PWM OC1A non-invering mode
	TCCR1A |= (1<<COM1A1);
	TCCR1A &=~ (1<<COM1A0);
	
	//Low level signal PB1
	PORTB &=~ (1<<1);
	
}

void set_pwm() {
	OCR1A = control_signal; // Установка значения ШИМ
}

int main(void) {
	
	uint16_t current_value;
	int16_t error;
	int16_t derivative;

	init_adc();
	init_pwm();
	init_timer2();
	timer0_init();
	init_led();
	init_USART();
	
	sei();

	while (1) {
		if (ms_count2 >= dt) { // Проверка, прошел ли интервал времени
			ms_count2 = 0; // Сброс счётчика миллисекунд
			setpoint = (float)1023 * voltage / (float)5000; // принимаемая уставка
			current_value = read_adc();
			if(setpoint > current_value){
				error = setpoint - current_value;
				integral += error * dt;
				//предотвращение переполнения
				if (integral > integral_max) {
					integral = integral_max;
				} 
				else if (integral < integral_min) {
					integral = integral_min;
				}	
				derivative = (error - previous_error) / dt;
				previous_error = error;
				control_signal = (Kp * error + Ki * integral + Kd * derivative); 
			}
			else{
				error = current_value - setpoint;
				integral -= error * dt;
				//предотвращение переполнения
				if (integral > integral_max) {
					integral = integral_max;
				}
				else if (integral < integral_min) {
					integral = integral_min;
				}
				derivative = (error - previous_error) / dt;
				previous_error = error;
				control_signal = (Kp * error + Ki * integral + Kd * derivative);
			}
						
			// Ограничение управляющего сигнала в пределах допустимых значений
			if (control_signal < 0) 
			{
				control_signal = 0;
			}
			else if (control_signal > 62000) {
				control_signal = 62000;
			}
		
			set_pwm(control_signal); // Применение управляющего воздействия	

		// Проверка значения АЦП и мигание светодиода
			uint16_t lower_bound = setpoint - (setpoint * 0.05);
			uint16_t upper_bound = setpoint + (setpoint * 0.05);
			if(msg_rec == 1){
				if (current_value >= lower_bound && current_value <= upper_bound) {
					if(ms_count >= 500){
						PORTB ^= (1 << 2);
						ms_count = 0;
					}
				}
				else {
					if(ms_count >= 100){
						PORTB ^= (1 << 2);
						ms_count = 0;
					}
				}
			}
		}		
	}
	return 0;
}

//timer1 used for PWM
//timer0 and timer2 used to count ms for indication