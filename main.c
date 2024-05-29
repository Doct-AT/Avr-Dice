#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED1 PB2 
#define LED2 PB1
#define LED3 PD7
#define LED4 PB0 
#define BUTN PD6

int time =2000;


void USART_Init(unsigned int baud)
{
    unsigned int ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data
}

void USART_Transmit(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = data;
}

void ADC_Init()
{
    ADMUX |= (1 << REFS0); // AVCC with external capacitor at AREF pin
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC prescaler 128
    ADCSRA |= (1 << ADEN); // Enable ADC
}

uint16_t ADC_Read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // Select ADC channel
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC))
        ; // Wait for conversion to complete
    return ADC;
}


void randomSeed(uint16_t seed)
{
    srand(seed);
}

long random(long min, long max)
{
    return min + rand() / (RAND_MAX / (max - min + 1) + 1);
}

int main(void)
{
  long ran;
    //button initialization
    DDRD &=  ~(1 << BUTN );

    //led initalization
    DDRB |= (1 << LED1);
    DDRB |= (1 << LED2);
    DDRD |= (1 << LED3);
    DDRB |= (1 << LED4);

    USART_Init(9600);
    ADC_Init();
    randomSeed(ADC_Read(2)); // Using ADC channel 2 for randomness

    


     PORTB &= ~(1 << PIN1);
     PORTB &= ~(1 << PIN2);
     PORTD &= ~(1 << PIN3);
     PORTB &= ~(1 << PIN4);

    while (1)
    {
      if ((PIND & (1 << BUTN))) {
      ran = random(1,7);
      if(ran == 1){
        PORTB |= (1 << PIN4);
        _delay_ms(time);
      }
      if (ran == 2){
        PORTB |= (1 << PIN1);
        _delay_ms(time);
      }
      if (ran == 3){
        PORTD |= (1 << PIN3);
        PORTB |= (1 << PIN4);
        _delay_ms(time);
      }
      if (ran == 4){
        PORTB |= (1 << PIN1);
        PORTD |= (1 << PIN3);
        _delay_ms(time);
      }
      if (ran == 5){
        PORTB |= (1 << PIN1);
        PORTD |= (1 << PIN3);
        PORTB |= (1 << PIN4);
        _delay_ms(time);
      }
      if (ran == 6){
        PORTB |= (1 << PIN1);
        PORTB |= (1 << PIN2);
        PORTB |= (1 << PIN4);
        _delay_ms(time);
      }
      }
     PORTB &= ~(1 << PIN1);
     PORTB &= ~(1 << PIN2);
     PORTD &= ~(1 << PIN3);
     PORTB &= ~(1 << PIN4);
    }
    return 0;
}
