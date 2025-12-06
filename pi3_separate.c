// ====================== ADC AND UART PARTS ONLY !! =================================
// ========== DO NOT FORGET TO GIVE CREDIT TO PS CODE ==========

#include <stdint.h>

typedef struct{
	volatile uint32_t ISR;     //0
	volatile uint32_t IER;     //4
	volatile uint32_t CR;      //8
	volatile uint32_t CFGR;    //C
	volatile uint32_t CFG2;    //10
	volatile uint32_t SMPR1;   //14
	volatile uint32_t SMPR2;   //18
	uint32_t reserved2;        //1C
	volatile uint32_t TR1;     //20
	volatile uint32_t TR2;     //24
	volatile uint32_t TR3;     //28
	uint32_t reserved3;        //2C
	volatile uint32_t SQR1;    //30
	volatile uint32_t SQR2;    //34
	volatile uint32_t SQR3;    //38
	volatile uint32_t SQR4;    //3C
	volatile uint32_t DR;      //40
	uint32_t reserved4[2];     //44 48
	volatile uint32_t JSQR;    //4C
	uint32_t reserved5[4];     //50 54 58 5C
	volatile uint32_t OFR1;    //60
	volatile uint32_t OFR2;    //64
	volatile uint32_t OFR3;    //68
	volatile uint32_t OFR4;    //6C
	uint32_t reserved6[4];     //70 74 78 7C
	volatile uint32_t JDR1;    //80
	volatile uint32_t JDR2;    //84
	volatile uint32_t JDR3;    //88
	volatile uint32_t JDR4;    //8C
	uint32_t reserved7[4];     //90 94 98 9C
	volatile uint32_t AWD2CR;  //A0
	volatile uint32_t AWD3CR;  //A4
	uint32_t reserved8[2];     //A8 AC
	volatile uint32_t DIFSEL;  //B0
	volatile uint32_t CALFACT; //B4
} ADCType;

typedef struct{
	volatile uint32_t CSR; //0
	uint32_t reserved1;    //4
	volatile uint32_t CCR; //8
	volatile uint32_t CDR; //C
} ADCCommon;

typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
	volatile uint32_t BRR;
	uint32_t reserved;
	volatile uint32_t SECCFGR;
} GPIOType;

typedef struct{
	volatile uint32_t CR1; //0
	volatile uint32_t CR2; //4
	volatile uint32_t CR3; //8
	volatile uint32_t BRR; //C
	uint32_t reserved1[2]; //10 14
	volatile uint32_t RQR; //18
	volatile uint32_t ISR; //1C
	volatile uint32_t ICR; //20
	volatile uint32_t RDR; //24
	volatile uint32_t TDR; //28
	volatile uint32_t PRESC; //2C
} LPUARTType;

// --- DEFINE ADRESLER ---
#define LPUART1 ((LPUARTType *) 0x40008000)
#define GPIOA   ((GPIOType *) 0x42020000)
#define GPIOG   ((GPIOType *) 0x42021800) // LPUART pinleri için (önceki koda göre)

#define ADC1 ((ADCType *) 0x42028000)
#define ADC ((ADCCommon *) 0x42028300)

#define RCC_CCIPR1 *((volatile uint32_t *) 0x40021088)
#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)
#define RCC_APB1ENR2 *((volatile uint32_t *) 0x4002105C)
#define PWR_CR1 *((volatile uint32_t *) 0x40007000)
#define PWR_CR2 *((volatile uint32_t *) 0x40007004)
#define RCC_APB1ENR1 *((volatile uint32_t *) 0x40021058)

#define ISER1 *((volatile uint32_t *) 0xE000E104) // ADC Interrupt için
#define ISER2 *((volatile uint32_t *) 0xE000E108) // LPUART Interrupt için

// --- GLOBAL DEĞİŞKENLER ---
volatile uint16_t water_level = 0; // Su sensörü değeri

// --- UART YARDIMCI FONKSİYONLARI ---
void LPUART1_SendChar(char c) {
    while (!(LPUART1->ISR & (1 << 7))); // TXE bekle
    LPUART1->TDR = c;
}

void LPUART1_SendString(char *str) {
    while (*str) {
        LPUART1_SendChar(*str++);
    }
}

// This function now prints a status string based on the raw ADC value.
// It replaces the previous integer printing logic.
void LPUART1_SendInteger(int num) {
    if (num <= 1600) {
        LPUART1_SendString("LOW"); // 0 - 1600
    } else if (num <= 1970) {
        LPUART1_SendString("MEDIUM"); // 1601 - 1800
    } else {
        LPUART1_SendString("HIGH"); // 1801 and above
    }
}

// Function to print sensor value as "x.yz" format using integer math

//void UART_SendFormattedFloat(uint16_t raw_value) {
//    // Formula: (Value * 4) / 2200
//    // To get 2 decimal places, we multiply by 100 first: (Value * 400) / 2200
//    uint32_t calc_x100 = ((uint32_t)raw_value * 400) / 2200;
//
//    int integer_part = calc_x100 / 100; // Example: 345 / 100 = 3
//    int decimal_part = calc_x100 % 100; // Example: 345 % 100 = 45
//
//    // 1. Print Integer Part
//    char buffer[10];
//    int i = 0;
//    if (integer_part == 0) {
//    	LPUART1_SendChar('0');
//    } else {
//        int temp = integer_part;
//        while (temp > 0) {
//            buffer[i++] = (temp % 10) + '0';
//            temp /= 10;
//        }
//        while (--i >= 0) {
//        	LPUART1_SendChar(buffer[i]);
//        }
//    }
//
//    // 2. Print Dot
//    LPUART1_SendChar('.');
//
//    // 3. Print Decimal Part (2 digits)
//    LPUART1_SendChar((decimal_part / 10) + '0'); // Tens digit
//    LPUART1_SendChar((decimal_part % 10) + '0'); // Ones digit
//}

// --- INIT FONKSİYONLARI ---
void init_LED_RED(void)
{
	RCC_AHB2ENR |= 1 << 0; // GPIOA Clock
	GPIOA->MODER &= ~(1 << 11); // PA5 Output (Bit 11'i temizle)
}

void init_ADC(void)
{
	RCC_AHB2ENR |= 1 << 0; // GPIOA Clock
	GPIOA->MODER |= 0b11 << 2; // PA1 Analog Mode

	RCC_AHB2ENR |= 1 << 13; // ADC Clock Enable
	ADC1->CR &= ~(1 << 29); // Deep-power down kapat
	ADC1->CR |= (1 << 28); // Voltage Regulator aç

	RCC_CCIPR1 |= 3 << 28; // ADC Clock = SYSCLK
	ADC->CCR |= 3 << 16; // HCLK/4


	ADC1->SMPR1 &= ~(0b111 << 18); // Sampling time

	ADC1->SQR1 &=  ~(0b1111 << 0);
	ADC1->SQR1 |= 6 << 6; // Channel 6

	ADC1->CR |= (1 << 31); // Calibrate
	while((ADC1->CR & (1 << 31)) != 0) {}

	ADC1->CR |= 1; // Enable ADC
	while((ADC1->ISR & 1) == 0) {}

	ADC1->CR |= 1 << 2; // Start Conversion
	ADC1->IER |= 1 << 2; // End of Conversion Interrupt Enable
	ISER1 |= 1 << 5; // NVIC Enable for ADC
}

void LPUART1_initialization(void)
{
	// LPUART Clock ve GPIO Ayarları (Önceki kodunuzdaki PG7/PG8 ayarları)
	RCC_APB1ENR1 |= 1 << 28; // Power Clock
	PWR_CR1 |= 1 << 14;
	PWR_CR2 |= 1 << 9; // VDDIO2

	RCC_AHB2ENR |= 1 << 6; // GPIOG Clock
	GPIOG->MODER &= ~(0b0101 << (7 * 2));
	GPIOG->MODER |= 0b1010 << (7 * 2); // Alt Function
	GPIOG->AFRL &= ~(0b0111 << (7 * 4));
	GPIOG->AFRL |= 0b1000 << (7 * 4); // AF8
	GPIOG->AFRH &= ~0b0111;
	GPIOG->AFRH |= 0b1000; // AF8

	RCC_APB1ENR2 |= 1; // LPUART Clock Enable
	LPUART1->BRR = 8888; // 115200 Baud (4MHz sysclk varsayımıyla)
	LPUART1->CR1 |= 1 << 29; // FIFO Enable (varsa)
	LPUART1->CR1 |= 0b11 << 2; // TE, RE Enable
	LPUART1->CR1 |= 1 << 5; // RXNE Interrupt Enable
	ISER2 |= 1 << 2; // NVIC Enable for LPUART
	LPUART1->CR1 |= 1; // UART Enable
}

// --- INTERRUPT HANDLERS ---

// ADC Interrupt: Sürekli su seviyesini okur ve günceller
void ADC1_2_IRQHandler(void)
{
	if((ADC1->ISR & 1<<2) != 0)
	{
		water_level = ADC1->DR;
		ADC1->CR |= 1<<2; // Yeni çevrimi başlat
	}
}

// UART Interrupt: Klavye girdisini dinler
void LPUART1_IRQHandler(void) {
	if ((LPUART1->ISR & (1 << 5)) != 0) // Veri geldi mi?
	{
		char received = (char)LPUART1->RDR;

		// Eğer 'W' veya 'w' tuşuna basıldıysa bayrağı kaldır
		if(received == 'W' || received == 'w') {
			LPUART1_SendString("\r\nWater Level: ");
			LPUART1_SendInteger(water_level);
			LPUART1_SendString("\r\n");
		}
	}
}

void __enable_irq(void) {
	__asm volatile("mov r0, #0 \n\t" "msr primask, r0 \n\t");
}

// --- MAIN FUNCTION ---
int main(void)
{
	init_LED_RED();
	init_ADC();
	LPUART1_initialization();
	__enable_irq();

	LPUART1_SendString("Sistem Hazir. Su seviyesi icin 'W' basin.\r\n");

	while(1)
	{

		__asm volatile("wfi"); // İşlemciyi uyut (Interrupt gelene kadar bekle)
	}
}
