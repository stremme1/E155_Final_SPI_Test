// STM32L432KC_USART.c
// Source code for USART functions

#include "STM32L432KC_USART.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"
#include "STM32L4xx/Device/Include/stm32l432xx.h"

// HSI frequency (16 MHz)
#define HSI_FREQ 16000000

USART_TypeDef * id2Port(int USART_ID) {
    USART_TypeDef * USART;
    switch(USART_ID){
        case(USART1_ID) :
            USART = USART1;
            break;
        case(USART2_ID) :
            USART = USART2;
            break;
        default :
            USART = 0;
    }
    return USART;
}

USART_TypeDef * initUSART(int USART_ID, int baud_rate) {
    gpioEnable(GPIO_PORT_A);  // Enable clock for GPIOA
    RCC->CR |= RCC_CR_HSION;  // Turn on HSI 16 MHz clock

    USART_TypeDef * USART = id2Port(USART_ID); // Get pointer to USART

    switch(USART_ID){
        case USART1_ID :
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Set USART1EN
            RCC->CCIPR |= (0b10 << RCC_CCIPR_USART1SEL_Pos); // Set HSI16 (16 MHz) as USART clock source

            // Configure pin modes as ALT function
            pinMode(PA9, GPIO_ALT); // TX
            pinMode(PA10, GPIO_ALT); // RX

            // Configure correct alternate functions for USART1
            // CRITICAL: Clear AFR bits BEFORE setting to avoid conflicts
            // PA9: USART1_TX (AF7 = 0b111) - bits 7:4 in AFR[1]
            // PA10: USART1_RX (AF7 = 0b111) - bits 11:8 in AFR[1]
            GPIOA->AFR[1] &= ~(0b1111 << GPIO_AFRH_AFSEL9_Pos);   // Clear bits 7:4 for PA9
            GPIOA->AFR[1] |= (0b111 << GPIO_AFRH_AFSEL9_Pos);      // Set PA9 to AF7 (USART1_TX)
            GPIOA->AFR[1] &= ~(0b1111 << GPIO_AFRH_AFSEL10_Pos);  // Clear bits 11:8 for PA10
            GPIOA->AFR[1] |= (0b111 << GPIO_AFRH_AFSEL10_Pos);     // Set PA10 to AF7 (USART1_RX)

            break;
        case USART2_ID :
            RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // Set USART2EN
            RCC->CCIPR |= (0b10 << RCC_CCIPR_USART2SEL_Pos); // Set HSI16 (16 MHz) as USART clock source

            // Configure pin modes as ALT function
            pinMode(PA2, GPIO_ALT); // TX
            pinMode(PA3, GPIO_ALT); // RX

            // Configure correct alternate functions (both pins use AF7 for USART2)
            // CRITICAL: Clear AFR bits BEFORE setting to avoid conflicts with previous configuration
            // PA2: USART2_TX (AF7 = 0b111) - bits 11:8 in AFR[0]
            // PA3: USART2_RX (AF7 = 0b111) - bits 15:12 in AFR[0]
            GPIOA->AFR[0] &= ~(0b1111 << GPIO_AFRL_AFSEL2_Pos);  // Clear bits 11:8 for PA2
            GPIOA->AFR[0] |= (0b111 << GPIO_AFRL_AFSEL2_Pos);     // Set PA2 to AF7 (USART2_TX)
            GPIOA->AFR[0] &= ~(0b1111 << GPIO_AFRL_AFSEL3_Pos);  // Clear bits 15:12 for PA3
            GPIOA->AFR[0] |= (0b111 << GPIO_AFRL_AFSEL3_Pos);     // Set PA3 to AF7 (USART2_RX)
            break;
    }

    // Set M = 00
    USART->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1);    // M=00 corresponds to 1 start bit, 8 data bits, n stop bits
    USART->CR1 &= ~USART_CR1_OVER8; // Set to 16 times sampling freq
    USART->CR2 &= ~USART_CR2_STOP;  // 0b00 corresponds to 1 stop bit

    // Set baud rate (see RM 38.5.4 for details)
    // Tx/Rx baud = f_CK / (16 * USARTDIV) for 16x oversampling
    // f_CK = 16 MHz (HSI)
    // BRR format: Bits 15:4 = DIV_Mantissa, Bits 3:0 = DIV_Fraction
    // USARTDIV = DIV_Mantissa + (DIV_Fraction / 16)
    // CRITICAL FIX: Formula is f_CK / (16 * baud_rate), NOT (f_CK * 16) / baud_rate
    // For 115200 baud: USARTDIV = 16000000 / (16 * 115200) = 8.680555...
    // Mantissa = 8, Fraction = 0.680555 * 16 = 10.888... ≈ 11 (0xB)
    // BRR = (8 << 4) | 11 = 0x008B
    // Calculate with integer math: USARTDIV = f_CK / (16 * baud_rate)
    // The correct formula: USARTDIV = f_CK / (16 * baud_rate)
    // For integer calculation with rounding: compute (f_CK * 16 + 8*baud_rate) / (16 * baud_rate)
    // This gives us USARTDIV * 256 (scaled by 16*16) with rounding
    uint32_t usartdiv_scaled_256 = ((HSI_FREQ << 4) + (baud_rate << 3)) / baud_rate;  // USARTDIV * 256, rounded
    uint16_t div_mantissa = usartdiv_scaled_256 >> 8;            // Upper bits = mantissa
    uint16_t div_fraction = (usartdiv_scaled_256 >> 4) & 0x0F;   // Next 4 bits = fraction
    USART->BRR = (div_mantissa << 4) | div_fraction;

    USART->CR1 |= USART_CR1_UE;     // Enable USART
    USART->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable transmission and reception

    return USART;
}

void sendChar(USART_TypeDef * USART, char data){
    while(!(USART->ISR & USART_ISR_TXE));
    USART->TDR = data;
    while(!(USART->ISR & USART_ISR_TC));
}

void sendString(USART_TypeDef * USART, char * charArray){

    uint32_t i = 0;
    do{
        sendChar(USART, charArray[i]);
        i++;
    }
    while(charArray[i] != 0);
}

char readChar(USART_TypeDef * USART) {
        // CRITICAL: Caller must check RXNE flag before calling this function
        // Reading RDR when RXNE=0 returns undefined data
        // This function assumes RXNE is already set (data ready)
        if (!(USART->ISR & USART_ISR_RXNE)) {
            // RXNE not set - return 0 as safe default (caller should have checked)
            return 0;
        }
        char data = USART->RDR;  // Reading RDR automatically clears RXNE
        return data;
}

void readString(USART_TypeDef * USART, char* charArray){
    int i = 0;
    do{
        charArray[i] = readChar(USART);
        i++;
    }
    while(USART->ISR & USART_ISR_RXNE);
}

