#define RX_PIN  10
#define DEBUG_PIN 13

unsigned char sbus_data[25];
int16_t channels[16];

void setup()  
{
  Serial.begin(9600);
  pinMode(RX_PIN, INPUT);
  pinMode(DEBUG_PIN, OUTPUT);
}

void loop() // run over and over
{
  readSBUSdata();
/*  Serial.print("Header:");
  Serial.println(sbus_data[0], HEX);*/
  update_channels();
  for(int i=0; i < 25; i++){
    Serial.print(sbus_data[i], HEX);
    Serial.print(",");
  }
/*  for(int i=0; i < 16; i++){
    Serial.print(channels[i], HEX);
    Serial.print("\t");
  }*/
  Serial.println();
/*  Serial.print("Flag:");
  Serial.println(sbus_data[23], HEX);
  Serial.print("Fooder:");
  Serial.println(sbus_data[24], HEX);*/
}


void update_channels(void) {
  uint8_t i;
  uint8_t sbus_pointer = 0;
  uint8_t byte_in_sbus = 1, bit_in_sbus = 0, ch = 0,bit_in_channel = 0;
  for (i=0; i<16; i++) {
    channels[i] = 0;
  }

  // process actual sbus data
  for (i=0; i<176; i++) {
    if (sbus_data[byte_in_sbus] & (1<<bit_in_sbus)) {
      channels[ch] |= (1<<bit_in_channel);
    }
    bit_in_sbus++;
    bit_in_channel++;

    if (bit_in_sbus == 8) {
      bit_in_sbus =0;
      byte_in_sbus++;
    }
    if (bit_in_channel == 11) {
      bit_in_channel =0;
      ch++;
    }
  }
}

inline int readSBUSdata(){
    volatile uint8_t *rxport = portInputRegister(digitalPinToPort(RX_PIN));
    uint8_t  mask = digitalPinToBitMask(RX_PIN);
    noInterrupts();
    for(uint8_t index = 0; index < 25; ){
      uint8_t  d = 0, i = 0x1;
      uint16_t waitCount = 0;
      while(!((*rxport) & mask)){
        waitCount++;
      }
      tunedDelay(8);
      do
      {
        uint8_t noti = ~i;
        DebugPulse(DEBUG_PIN);
        if ((*rxport) & mask)
          d |= i;
        else
          d &= noti;
        i <<= 1;
        tunedDelay(7);
      }while(i);
      
      sbus_data[index] = ~d;
      index++;
      
      if(waitCount > 1000){
        sbus_data[0] = ~d;
        index = 1;
        waitCount = 0;
      }
      tunedDelay(10);
    }
    interrupts();
}

inline void tunedDelay(uint16_t delay) { 
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+r" (delay), "+a" (tmp)
    : "0" (delay)
    );
}
inline void DebugPulse(uint8_t pin)
{
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));
  uint8_t val = *pport;
  *pport = val | digitalPinToBitMask(pin);
  *pport = val;
}

