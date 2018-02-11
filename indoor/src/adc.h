#define SELPIN 15   // Selection Pin
#define DATAOUT 13  // MOSI
#define DATAIN 12   // MISO
#define SPICLOCK 14 // Clock
#define ADC_MAX 4095
#define ADC_VCC 3.3

void initSpiAdc();

int readSpiAdc(int channel);