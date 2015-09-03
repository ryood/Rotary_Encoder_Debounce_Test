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

// Rotary Encoder
volatile uint8_t re_data;
volatile uint8_t re_index;
volatile uint8_t re_index_rd;

void init_switches()
{
	// Pin Change Interruptの有効化
	PCICR = (1 << PCIE0) | (1 << PCIE2);
	//PCMSK0 = 0b11000001;	// PORTB
	//PCMSK2 = 0b00111111;	// PORTD
	PCMSK0 = 0b00001110;
	
	// TIMER0 オーバーフロー割り込みの有効化
	TCCR0B = 0x00;	// Timer0停止
	TIMSK0 = (1 << TOIE0);
	
	// TIMER2 オーバーフロー割り込みの有効化
	TCCR2B = 0x00;	// Timer2停止
	TIMSK2 = (1 << TOIE2);
}

ISR (TIMER0_OVF_vect)
{
	//PORTC ^= (1 << PC2);
	
	// Timer2が起動中でなければPin Change Interruptを有効化
	/*
	if (TCNT2 == 0)
		PCICR = (1 << PCIE0) | (1 << PCIE2);
	*/
}

ISR (TIMER2_OVF_vect)
{
	// Timer2停止
	TCCR2B = 0x00;	// Timer2停止
	
	// 割り込みごとにLEDを点滅（デバッグ用）
	PORTC ^= (1 << PC3);
	
	/*
	uint8_t rd = (PINB & (1 << PB2) >> 2)| (PINB & (1 << PB3) >> 2);
	
	if (re_index_rd == rd) {
		re_index = (re_index << 2) | rd;
		re_index &= 0b1111;
	
		switch (re_index) {
		// 時計回り
		case 0b0001:	// 00 -> 01
		case 0b1110:	// 11 -> 10
			re_data = 0x55;
			break;
		// 反時計回り
		case 0b0010:	// 00 -> 10
		case 0b1101:	// 11 -> 01
			re_data = 0xaa;
			break;
		}
	}
	*/	
	PCICR = (1 << PCIE0) | (1 << PCIE2);
}

void pin_change_interrupt_handler()
{
	// Pin Change Interruptを無効化
	PCICR = 0x00;
	
	// 割り込みごとにLEDを点滅（デバッグ用）
	PORTC ^= (1 << PC3);
	
	//re_index_rd = (PINB & (1 << PB2) >> 2)| (PINB & (1 << PB3) >> 2);
	
	//PORTD = PINB;
	
	// Timer0を起動
	//TCCR0B = 0x07;	// プリスケーラ：1024
	//TCNT0 = 80;		// about: 10ms	
	
	// Timer2を起動
	TCCR2B = 0x07;	// プリスケーラ－:1024, 1/(8MHz/1024)=128us
	TCNT2 = 248;	// 128us*(256-248)=1024us
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
/*
int8_t readRE(void)
{
	static uint8_t index;
	int8_t retVal = 0;
	
	index = (index << 2) | (RE_PIN & _BV(RE_A)) | (RE_PIN & _BV(RE_B));
	index &= 0b1111;
	
	switch (index) {
		// 時計回り
		case 0b0001:	// 00 -> 01
		case 0b1110:	// 11 -> 10
		retVal = 1;
		break;
		// 反時計回り
		case 0b0010:	// 00 -> 10
		case 0b1101:	// 11 -> 01
		retVal = -1;
		break;
	}
	
	_delay_ms(2);	// (とりあえず)チャタリング防止
	
	return retVal;
}
*/

int main(void)
{
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
	
    while(1)
    {
		//PORTD = re_data;
		//PORTD = re_index;
		
		/*
		if (PCICR) {
			PORTB |= (1 << PB5);
		} else {
			PORTB &= ~(1 << PB5);			
		}
		*/
		
	}
}
	