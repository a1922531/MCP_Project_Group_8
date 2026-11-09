/*---------------------------

List of Contributors:

ANDREW BUXTON	A1887577
LUKE HUMPHRIS	A1922531
CHRISTOPHER DUTCH	A1926149

---------------------------*/

#ifndef UART_H
#define UART_H

#include <stdint.h>

void UART_Init(void);
uint8_t UART_Rx(void);
void UART_Tx(uint8_t byte);
#endif
