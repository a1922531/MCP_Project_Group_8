#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "uart.h"
#include "kobuki.h"

void Kobuki_Tx ( uint8_t * payload , uint8_t size_payload ) {
uint8_t checksum = 0 , i ;
UART_Tx(0xAA); // Send header 0
UART_Tx(0x55); // Send header 1
UART_Tx(size_payload); // Send size of payload in bytes
checksum ^= size_payload ; // Calculate running checksum

// Send each byte of payload one -by -one
for( i = 0; i < size_payload; i++) {
UART_Tx(payload[i]) ; // Send payload byte
checksum ^= payload[i];
}
UART_Tx (checksum) ; // Send checksum
}

uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback) {
uint8_t feedback_header = 0, size_payload = 0, checksum = 0, rx_checksum = 0;
uint32_t i;

// 1. Find the valid 0xAA 0x55 sequence
while (1) {
    feedback_header = UART_Rx();
    if (feedback_header == 0xAA) {
        feedback_header = UART_Rx();
        if (feedback_header == 0x55) {
            break; // Valid start found, exit finding loop
        }
    }
}

// 2. Read Payload Length, first byte is size of payload in bytes
size_payload = UART_Rx();
checksum ^= size_payload; // checksum is XOR'ed value of entire bytestream

// 3. Determine safe boundary to prevent array overflow
uint32_t limit = (size_payload < size_feedback) ? size_payload : size_feedback;

// 4. Read into array of bytestream, this is the basic sensor data
for (i = 0; i < limit; i++) {
    feedback[i] = UART_Rx();
    checksum ^= feedback[i];
}

// 5. Read and discard remaining payload (just for checksum), this is the size of rest of byestream
for (i = limit; i < size_payload; i++) {
    checksum ^= UART_Rx();
}

// 6. Verify Checksum
rx_checksum = UART_Rx(); // checksum from kobuki - final byte

return (checksum != rx_checksum); // 0 = Success, 1 = Error
}

// kobuki read datastream
void Kobuki_Read ( Kobuki_Typedef * kobuki ) {
// Init buffer & checksum
uint8_t kobuki_rx_buffer[SIZE_FEEDBACK];
uint8_t checksum_result;
// Read kobuki Tx data stream
checksum_result = Kobuki_Rx (kobuki_rx_buffer , SIZE_FEEDBACK) ;
// If checksum 'checks out', update sensor variables from buffer
if (checksum_result == 0) {
kobuki-> bumper = kobuki_rx_buffer [BUMPER];
kobuki-> wheeldrop = kobuki_rx_buffer [WHEEL_DROP];
kobuki-> cliff = kobuki_rx_buffer [CLIFF];
kobuki-> button = kobuki_rx_buffer [BUTTON];
}
// otherwise do not update
else {}
}


void Kobuki_Drive(int16_t speed){

uint8_t speed_lsb = 0;
uint8_t speed_msb = 0;
// Construct payload
uint8_t payload[] = {0x01,0x04,speed_lsb,speed_msb,0x00,0x00};

// saturation to avoid going over speed limit
if ( speed > SPEED_LIMIT ){
speed = SPEED_LIMIT;
}
else if ( speed < -SPEED_LIMIT ){
speed = (~SPEED_LIMIT)+1;
}
// Split LSB & MSB for payload
// There is no truncation warning due to stdint
payload[2] = (uint8_t)speed;
payload[3] = (uint8_t)(speed >> 8);

Kobuki_Tx(payload,sizeof(payload));
}

void Kobuki_Rotate(int16_t speed) {

uint8_t speed_lsb = 0;
uint8_t speed_msb = 0;
// Construct payload
uint8_t payload[] = {0x01,0x04,speed_lsb,speed_msb,0x01,0x00};
// saturation to avoid going over speed limit
if ( speed > SPEED_LIMIT ){
speed = SPEED_LIMIT;
}
else if ( speed < -SPEED_LIMIT ){
speed = (~SPEED_LIMIT)+1;
}

//split lsb & msb for payload
payload[2] = (uint8_t)speed;
payload[3] = (uint8_t)(speed >> 8);

Kobuki_Tx(payload,sizeof(payload));
}