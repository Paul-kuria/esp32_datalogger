const unsigned long UpdateInterval = 0.5 * (60L * 1000000L); // Update delay in microseconds, currently 30-secs (1/2 of minute)

const int blueLED = 14; // blue LED next to GPS -- blink when packet sent
int pktCount = 0; // count how many packets sent

// LORA-TTGO
#define LORA_RESETT 14
#define LORA_CS 18
#define  LORA_IRQ 26


// SOIL SENSOR
#define I2C_SDA_2 21
#define I2C_SCL_2 22
#define SHT31_I2C_ADDR_44 0x44

// AIR SENSOR
#define DHTPIN 25