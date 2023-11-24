//Projeto Gigantes de MDF - Engenharia da Computação

//Grupo: 
//Adriano Fernandes Scarabelli			RA: 201092
//Roberto Gomes Lima Neto				RA: 202786
//Pedro Henrique Vieira Tavares Luiz 		RA: 181912
//Hugo Galluzzi Alvarenga 				RA: 193573

//Obs: Foi utilizado um LED no lugar do laser do projeto 
//para ser possível a sua simulação no programa SimulIDE

#ifndef F_CPU
#define F_CPU  16000000UL
#endif

#include <avr/io.h>	
#include <stdio.h>
#include <avr/interrupt.h>

#define TIMER_FREQUENCY 1

uint16_t adcValue;		//Variável para o valor analógico(LDR)

//Função que define os pinos de cada componente:

void setup() {

//LEDs:

	DDRD |= (1<<PD4);		//Configura o LED no pino PD4
	DDRD |= (1<<PD7);		//Configura o LED no pino PD7
	DDRB |= (1<<PB0);		//Configura o LED no pino PB0	

//Motores:

	DDRD |= (1<<PD5);		//Configura o motor no pino PD5
	DDRD |= (1<<PD6);		//Configura o motor 2 no pino PD6
	
//Laser:

	DDRD |= (1<<PD2);		//Configura o laser no pino PD2
	
}

//Variáveis para controle de tempo do laser:

volatile uint16_t ledTimer = 0;		//Tempo de funcionamento laser
volatile uint8_t ledState = 0;		//Estado do laser

//Função responsável pelo Timer1:

void initTimer1() {

	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<CS12) | (1<<CS10);
	
	OCR1A = F_CPU/(2*1024*TIMER_FREQUENCY) - 1;	  //Equação de frequencia
		
	TIMSK1 |= (1<<OCIE1A);

	sei();		//Habilita as interrupções globais

}

//Função responsável pelo Timer2:

void timer2_init() {

    TCCR2A |= (1 << WGM21);
    TCCR2B |= (1 << CS22) | (1 << CS20);

    OCR2A = 109;

}

//Função que configura o PWM:

void PWM_Init() {

    // Configuração do Timer0 para controle PWM

    TCCR0A |= (1<<COM0A1) | (1<<COM0B1);
    TCCR0A |= (1<<CS00);
    TCCR0B |= (1<<CS00);

    OCR0A = 0;     // Define o valor inicial do PWM OCR0A como 0
    OCR0B = 0;	    // Define o valor inicial do PWM OCR0B como 0

}

//Função que configura o botão1:

void Button_Init() {

    DDRB &= ~(1 << PB4);		//Configuração do pino do botão como entrada

    PORTB |= (1 << PB4);	  //Habilita pull-up interno para o pino do botão

}

//Função que configura o botão2:

void Button_Init2() {

    DDRB &= ~(1 << PB5);		//Configuração do pino do botão como entrada

    PORTB |= (1 << PB5);	  //Habilita pull-up interno para o pino do botão

}

//Configuração da leitura de valores do LDR: 

ISR(TIMER1_COMPA_vect) {

    PORTC ^= (1 << PC0);  		//Atualiza o estado do pino PC0

    ADCSRA |= (1 << ADSC);  		//Inicia uma nova conversão ADC

}

//Configuração para piscar o laser a cada 1 segundo:

ISR(TIMER2_COMPA_vect) {

    ledTimer++;

    if (ledTimer >= 1000) {

        ledTimer = 0;			//Configura o valor da variável como 0
        ledState = !ledState;  	//Inverte o estado do LED

        PORTD ^= (1 << PD2);   	//Atualiza o estado do LED no hardware

    }

}

//Definição do adcValue como analógico

ISR(ADC_vect){

	adcValue = ADC;		//Configura o adcValue como valor analógico

};

//Função permite o trabalho com valores analógicos

void initADC() {

	ADMUX |= (1<<REFS0);
	
	ADMUX |= (0<<MUX0)|(0<<MUX1)|(0<<MUX2)|(0<<MUX3);
	
	ADCSRA |= (1<<ADEN) | (1 << ADPS2) | (1 << ADPS0);
	ADCSRA |= (1<<ADIE);

};

//Função principal onde o projeto funciona:

int main(void) {

    setup();			//Aciona a função Setup

    initTimer1();		//Aciona a função initTimer1

    timer2_init();		//Aciona a função timer2_init

    PWM_Init();		//Aciona a função PWM_Init

    Button_Init();		//Aciona a função Button_Init

    Button_Init2();		//Aciona a função Button_Init2

    initADC();			//Aciona a função initADC

    sei();			//Habilita as interrupções globais

    TIMSK1 |= (1 << OCIE1A);
    TIMSK2 |= (1 << OCIE2A);

    Serial.begin(9600);			//Habilita o monitor serial
	
    PORTD ^= (1 << PD4);			//Liga o primeiro LED de vida
    PORTD ^= (1 << PD7);			//Liga o segundo LED de vida
    PORTB ^= (1 << PB0);			//Liga o terceiro LED de vida

    while (1) {
	
		Serial.println(adcValue);	//Mostra o valor detectado pelo
								//LDR no monitor serial
								
		if (!(PINB & (1 << PB4))) {

          	OCR0A = 200;  //Define o valor do PWM para ligar o motor 2

          }

		else if (!(PINB & (1 << PB5))) {
		
			OCR0B = 200;	//Define o valor do PWM para ligar o motor
		
		}
		
		//Condição para apagar um LED quando o LDR detectar valor alto
		//e também para desligar os motores neste mesmo momento
		
		else if (adcValue>=500) {
		
    			PORTD ^= (1 << PD4);		//Altera o valor do LED para 0

			OCR0A = 0;				//Desliga o motor 2
		  	OCR0B = 0;				//Desliga o motor
		
		}

		//Condição que mantém os motores desligados:   

		else {
	
            
			OCR0A = 0; 			//Desliga o motor 2 quando o botão 
								//não está pressionado


		  	OCR0B = 0; 			//Desliga o motor quando o botão 
								//não está pressionado

       	}
		
    }

    return 0;
}