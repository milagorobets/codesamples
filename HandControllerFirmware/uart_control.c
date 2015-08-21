#include "common.h"

void initializeUART(void) {
		// Set Baud rate
		// Formula: (fosc)/(16xBaudRate) - 1

   		UBRR0 = 64; // Rounded down from 64.1

   		UCSR0C = _BV(UCSZ01) | _BV(UCSZ00 ); 
									 // Asynchronous mode
									 // No Parity
									 // 1 Stop bit
									 // 8 Data bits
		UCSR0A = 0x00; // No double speed

		// Enable UART interrupts
		UCSR0B = (0 << TXCIE0)|(0 << UDRIE0) | (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);

   		// Enable The receiver and transmitter

	}


	void USARTRead(){
	   // poll for data available
	  //while ( (UCSR0A & _BV(RXC0)) == 0 );
        
		ring_data_t temp = UDR0;
    	ring_insert((Ring_t*)&uart_ring, temp);
		
	}

	void USARTWrite(char data){
	   // wait for data register to be ready
       while ( (UCSR0A & _BV(UDRE0)) == 0 );

	   // load b for transmission
	   UDR0 = data;
	}

	void USARTWrite_String(char data[]){

		int i = 0;
		int word_length = 0;
		char letter;
		

		word_length = strlen(data);

		for (i=0; i < word_length; i++){
			letter = data[i];
			USARTWrite(letter);
		}
	   
	}
	ISR(USART0_RX_vect){
		ring_data_t temp = UDR0;
    	ring_insert((Ring_t*)&uart_ring, (unsigned char)temp);	
	}
//#define USART0_UDRE_vect  _VECTOR(21)  /* USART0 Data register Empty */
//#define USART0_TX_vect    _VECTOR(22)  /* USART0, Tx Complete */
