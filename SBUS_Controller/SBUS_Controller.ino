//SBUS Controller for Arduino Fio

//Configuration
#define SBUS_PIN  11
#define SERIAL_RATE  9600


#include <FlexiTimer2.h>
#define SBUS_FORMAT(a) (/*Start bit*/0x00 | /*Data*/(((a)&0xFF)<<1) | /*Parity*/((((a) ^ ((a)>>1) ^ ((a)>>2) ^ ((a)>>3) ^ ((a)>>4) ^ ((a)>>5) ^ ((a)>>6) ^ ((a)>>7))&0x1)<<9))

uint8_t sbus_select = 0;
uint16_t sbus_data[2][25] = {0};
uint16_t channels[16] = {0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400, 0x400};

void setup()
{
  //Initialize I/O
  Serial.begin(SERIAL_RATE);
  pinMode(SBUS_PIN, OUTPUT);
  digitalWrite(SBUS_PIN, LOW);
  
  //Set initial sbus_data
  updateSBUSdata();
  
  //Set 14ms timer interrupt
  FlexiTimer2::set(114, 1.0/10000.0, writeSBUSdata);
  FlexiTimer2::start();
}

void loop()
{
  if(Serial.available()){
    char buf[16];
    Serial.readBytesUntil('\n', buf, sizeof(buf));
    
    //Normal Update
    if(buf[0] >= 'A' && buf[0] <= 'P' && buf[1] == ','){
      channels[buf[0] - 'A'] = atoi(&buf[2]);
    }
    //Quick Update
    switch(buf[0]){
      case 'n'://Neutral
        channels[0] = 0x400;
        channels[1] = 0x400;
        channels[2] = 0x400;
        channels[3] = 0x400;
        break;
      case 's'://Start Condition
        channels[0] = 0x16C;
        channels[1] = 0x16C;
        channels[2] = 0x16C;
        channels[3] = 0x16C;
        break;
      case 'e'://Stop Condition
        channels[0] = 0x400;
        channels[1] = 0x400;
        channels[2] = 0x16C;
        channels[3] = 0x400;
        break;        
    }
    
    //Update sbus_data
    updateSBUSdata();
  }
}

void updateSBUSdata()
{
  uint8_t sel = sbus_select ^ 1;
  
  //Initialize
  for (uint8_t i=0; i<25; i++){
    sbus_data[sel][i] = 0;
  }
  sbus_data[sel][0] = 0x0F;//Header
  sbus_data[sel][23] = 0x00;//Flag
  sbus_data[sel][24] = 0x00;//Footer
  
  //Set channel values
  uint8_t byte_in_sbus = 1, bit_in_sbus = 0, ch = 0, bit_in_channel = 0;
  for (uint8_t i=0; i<176; i++) {
    if (channels[ch] & (1<<bit_in_channel)) {
      sbus_data[sel][byte_in_sbus] |= (1<<bit_in_sbus);
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
  
  //Calculate Parities
  for (uint8_t i=0; i<25; i++) {
    sbus_data[sel][i] = SBUS_FORMAT(sbus_data[sel][i]);
  }
  
  //Switch buffer
  sbus_select = sel;
}

void writeSBUSdata()
{
    volatile uint8_t *sbus_port = portOutputRegister(digitalPinToPort(SBUS_PIN));
    uint8_t mask = digitalPinToBitMask(SBUS_PIN);
    uint8_t notmask = ~mask;

    noInterrupts();
    
    //Send data
    for(uint8_t index = 0; index < 25; index++){
      //Inverted data
      uint16_t val = ~sbus_data[sbus_select][index];
      
      //Send a Frame
      for(uint16_t i = 1; i&0x3FF; i<<=1){
        //Output a bit
        if(val&i) *sbus_port |= mask;
        else *sbus_port &= notmask;
        
        //Wait for 10us
        delayMicroseconds(7);
        __asm__ __volatile__ (
            "nop\n"
            "nop\n"
            "nop\n"
            "nop\n"
            "nop");
      }

      //Little wait
      __asm__ __volatile__ (
            "nop\n"
            "nop\n"
            "nop");

      //Two stop bits
      *sbus_port &= notmask;
      //Wait for 20us
      delayMicroseconds(13);
      __asm__ __volatile__ (
            "nop\n"
            "nop");
    }
    
    interrupts();
}

