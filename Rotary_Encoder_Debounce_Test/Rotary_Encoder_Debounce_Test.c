/*
 * Rotary_Encoder_Debounce_Test.c
 *
 * Created: 2015/09/03 12:13:29
 *  Author: gizmo
 * 
 * PORTB[PB1]     : Rotary Encoder SW
 * PORTB[PB2]     : Rotary Encoder A
 * PORTB[PB3]     : Rotary Encoder B
 *
 * PORTB[PB4]     : SW
 *
 * PORTB[PB5]     : LED
 *
 * PORTC[PC3]     : LED
 * PORTC[PC2]     : LED
 *
 * PORTD          : LEDx8
 *
 */

#define F_CPU 8000000UL	// 8MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdint.h>

// Rotary Encoder SW
volatile uint8_t re_sw;
volatile uint8_t re_sw_rd;

void init_switches()
{
	// Pin Change Interruptの有効化
	PCICR = (1 << PCIE0) | (1 << PCIE2);
	//PCMSK0 = 0b11000001;	// PORTB
	//PCMSK2 = 0b00111111;	// PORTD
	PCMSK0 = 0b00000010;
	
	// TIMER0 オーバーフロー割り込みの有効化
	TCCR0B = 0x00;	// Timer0停止
	TIMSK0 = (1 << TOIE0);
}

ISR (TIMER0_OVF_vect)
{
	// 割り込みごとにLEDを点滅（デバッグ用）
	PORTC ^= (1 << PC3);
	
	// Timer0停止
	TCCR0B = 0x00;
	
	if (re_sw_rd == (~PINB & 0b00000010)) {
		re_sw ^= re_sw_rd;
	}
	
	// Pin Change Interrupt有効化
	PCICR = (1 << PCIE0) | (1 << PCIE2);
}

void pin_change_interrupt_handler()
{
	// 割り込みごとにLEDを点滅（デバッグ用）
	PORTC ^= (1 << PC2);
	
	// Pin Change Interruptを無効化
	PCICR = 0x00;
	
	// Switches
	re_sw_rd = (~PINB & 0b00010010);
	
	// Timer0を起動
	TCCR0B = 0x05;	// プリスケーラ－:1024, 1/(8MHz/1024)=128us
	//TCNT0 = 248;	// 128us*(256-248)=1024us
	TCNT0 = 216;	// 128us*(256-216)=5120us
}

ISR (PCINT0_vect)
{
	pin_change_interrupt_handler();
}

ISR (PCINT2_vect)
{
	pin_change_interrupt_handler();
}

/*------------------------------------------------------------------------/
 * Rotary Encoder
 *
 ------------------------------------------------------------------------*/
// 戻り値: ロータリーエンコーダーの回転方向
//         0:変化なし 1:時計回り -1:反時計回り
//
int8_t read_re(void)
{
	static uint8_t index;
	int8_t ret_val = 0;
	uint8_t rd;
	
	rd = ((PINB & 0b00001100) >> 2);
	
	_delay_ms(1);
	
	if (rd == ((PINB & 0b00001100) >> 2)) {
		//PORTC ^= (1 << PC2);	// (デバッグ用)
		
		index = (index << 2) | rd;
		index &= 0b1111;
		
		switch (index) {
		// 時計回り
		case 0b0001:	// 00 -> 01
		case 0b1110:	// 11 -> 10
			ret_val = 1;
			break;
		// 反時計回り
		case 0b0010:	// 00 -> 10
		case 0b1101:	// 11 -> 01
			ret_val = -1;
			break;
		}	
	}
	
	return ret_val;
}

int main(void)
{
	uint8_t re_count;
	
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;

	// Rotary Encoder input / pullup
	PORTB |= 0b00001110;
	
	// SW input / pullup
	PORTB |= 0b00010000;
		
	// LED
	DDRD |= 0xFF;
	DDRC |= (1 << PC2) | (1 << PC3);
	DDRB |= (1 << PB5);
		
	// LED Check
	PORTC |= (1 << PC2) | (1 << PC3);
	PORTB |= (1 << PB5);
	for (int i = 0; i <= 8; i++) {
		PORTD = (0xFF >> i);
		_delay_ms(100);
	}
	PORTC &= ~(1 << PC3);
	_delay_ms(100);
	PORTC &= ~(1 << PC2);
	_delay_ms(100);
	PORTB &= ~(1 << PB5);
		
	init_switches();
		
	sei();
	
	re_count = 0;
    while(1)
    {
		re_count += read_re();
		 
		// Rotary Encoderのカウントを表示
		PORTD = re_count;
		
		// SWの状態を表示
		if (re_sw & (1 << PB1)) {
			PORTB |= (1 << PB5);
		} else {
			PORTB &= ~(1 << PB5);
		}		
	}
}
