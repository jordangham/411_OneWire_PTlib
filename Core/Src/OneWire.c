#include "OneWire.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
volatile uint8_t recvFlag;
volatile uint16_t rc_buffer[5];

/**
    @author Stanislav Lakhtin
    @date   11.07.2016
    @brief  Implementation of the 1wire protocol based on the libopencm3 library for the STM32F103 microcontroller
            Perhaps the library will work correctly on other uKs (verification required).
            Verification is necessary to ensure that the UART / USART is configured correctly for operation.
            in half duplex mode
            The general idea is to use a hardware USART uK to simulate 1wire operation.
            Devices are connected to the selected USART to the TX pin, which must be pulled up to the power line with a resistance of 4.7K.
            The library implementation loops RX to TX inside uK, leaving the RX pin available for use in other tasks.
            The implementation of the library assumes the possible simultaneous operation of both independent buses with all
            possible UART / USART in the microcontroller. In this case, all buses (up to 5 pieces) will be addressed and interrogated individually
 */


extern UART_HandleTypeDef huart2;
#define ow_uart huart2
#define OW_USART USART2
#define MAXDEVICES_ON_THE_BUS 1

/*********************************************************************************************/
typedef struct {
	int device;
	char info[30];
}DEVInfo;
DEVInfo devInfo;

float Temp[MAXDEVICES_ON_THE_BUS];

uint8_t devices;
OneWire ow;
uint32_t pDelay = 300, i;
uint8_t sensor;

Temperature t;


char *crcOK;

uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));

  /* Receive Data */
  return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
}

void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data));

  /* Transmit Data */
  USARTx->DR = (Data & (uint16_t)0x01FF);
}


uint8_t getUsartIndex(void);

void usart_setup(uint32_t baud) {

	ow_uart.Instance = OW_USART;
	ow_uart.Init.BaudRate = baud;
	ow_uart.Init.WordLength = UART_WORDLENGTH_8B;
	ow_uart.Init.StopBits = UART_STOPBITS_1;
	ow_uart.Init.Parity = UART_PARITY_NONE;
	ow_uart.Init.Mode = UART_MODE_TX_RX;
	ow_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	ow_uart.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_HalfDuplex_Init(&ow_uart) != HAL_OK)
	{
		//	    Error_Handler();
		__asm__("NOP");
	}

	__HAL_UART_ENABLE_IT(&ow_uart, UART_IT_RXNE);
}

void owInit(OneWire *ow) {
  int i=0, k = 0;
  for (; i < MAXDEVICES_ON_THE_BUS; i++) {
   uint8_t *r = (uint8_t *)&ow->ids[i];      
    k=0;
    for (; k < 8; k++)
    r[k] = 0;
  }
  k=0;
  for (; k < 8; k++)
    ow->lastROM[k] = 0x00;
  ow->lastDiscrepancy = 64;

}

void owReadHandler() { //USART interrupt handler
  uint8_t index = getUsartIndex();
  /* We check that we caused an interrupt due to RXNE. */
  if (((OW_USART->CR1 & USART_CR1_RXNEIE) != 0) &&
      ((OW_USART->SR & UART_FLAG_RXNE) != (uint16_t)RESET)) {

    /* We receive data from the periphery and reset the flag*/
		while ((OW_USART->SR & UART_FLAG_RXNE) == (uint16_t)RESET){;}
    rc_buffer[index] = USART_ReceiveData(OW_USART);              
    recvFlag &= ~(1 << index);//reset flag response received after
  }
}

/** Implementation of RESET on the 1wire bus
 *
 * @param N usart -- chosen to implement 1wire usart
 * @return Returns 1 if there is someone on the bus and 0 otherwise
 */

  uint16_t owResetCmd() {
	uint16_t owPresence;
	
	usart_setup(9600);

  owSend(0xF0); // Send RESET send a reset pulse
  owPresence = owEchoRead(); // We are waiting for PRESENCE on the bus and return what is

	usart_setup(115200);// reconfigure UART speed
  return owPresence;
}

uint8_t getUsartIndex() {// looks at the UART number with which work will go
//	uint8_t result;
//	if(OW_USART==USART1)result = 0;
//	else if (OW_USART==USART2)result = 1;
//	else if (OW_USART==USART3)result = 2;
	return 0;
}

void owSend(uint16_t data) {
  recvFlag |= (1 << getUsartIndex());//set the flag if we get into the interrupt handler there it will be reset
  USART_SendData(OW_USART, data);//send data
	while(__HAL_UART_GET_FLAG(&ow_uart, UART_FLAG_TC) == RESET);//waiting for the transfer to end
}

uint8_t owReadSlot(uint16_t data) {//we read we got a one or zero in response
  return (data == OW_READ) ? 1 : 0; //if 0xFF came, then bit = 1, something else bit = 0
}

uint16_t owEchoRead() {//
  uint8_t i = getUsartIndex();//get USART number
  uint16_t pause = 1000;
  while (recvFlag & (1 << i) && pause--);// wait until someone answers but no more pause
  return rc_buffer[i];//depending on the UART number used
}

uint8_t *byteToBits(uint8_t ow_byte, uint8_t *bits) {//decompose 1 byte into 8 bytes, encode so to speak in a package for 1wire
  uint8_t i;
  for (i = 0; i < 8; i++) {
    if (ow_byte & 0x01) {//if current bit in byte ==1 then
      *bits = WIRE_1; //replace with a number which, when transmitted via UART for 1 wire, will be unity t.e 0xFF
    } else {
      *bits = WIRE_0;// same for 0
    }
    bits++;
    ow_byte = ow_byte >> 1; //shift the processed bit
  }
  return bits; //return an array to pass
}

/**
 * The method sends sequentially 8 bytes, one for each bit in data
 * @param usart -- selected for 1wire UART emulation
 * @param d -- data
 */
void owSendByte(uint8_t d) {
  uint8_t data[8];
	int i;
  byteToBits(d, data);//convert bytes to bits "byte array for UART transmission and 1WIRE emulation"
  for (i = 0; i < 8; ++i) {
    owSend(data[i]);
  }
}


uint8_t bitsToByte(uint8_t *bits) {//takes the "encoded" array of bytes received via UART and makes a byte out of it))
  uint8_t target_byte, i;
  target_byte = 0;
  for (i = 0; i < 8; i++) {
    target_byte = target_byte >> 1;
    if (*bits == WIRE_1) {//if it came via USART 0xFF, then this is the 1st one we got
      target_byte |= 0x80;//set to 1 most significant bit
    }
    bits++;//move to next byte which is either 0=0x00 or 1=0xFF
  }
  return target_byte; //return the received byte
}

/* CRC8 count of array mas of length Len */
uint8_t owCRC(uint8_t *mas, uint8_t Len) {
  uint8_t i, dat, crc, fb, st_byt;
  st_byt = 0;
  crc = 0;
  do {
    dat = mas[st_byt];
    for (i = 0; i < 8; i++) {  // bit count in byte
      fb = crc ^ dat;
      fb &= 1;
      crc >>= 1;
      dat >>= 1;
      if (fb == 1) crc ^= 0x8c; // polynomial
    }
    st_byt++;
  } while (st_byt < Len); // byte count in array
  return crc;
}

uint8_t owCRC8(RomCode *rom){
  return owCRC((uint8_t*)rom, 7);                        
}

/*
 * return 1 if has got one more address
 * return 0 if hasn't
 * return -1 if error reading happened
 *
 * convert to callback functions to respond to errors
 */
int hasNextRom(OneWire *ow, uint8_t *ROM) {//
	uint8_t ui32BitNumber = 0;
  int zeroFork = -1;
	uint8_t i = 0;
  if (owResetCmd() == ONEWIRE_NOBODY) { //is there anyone on the bus
    return 0;
  }
  owSendByte(ONEWIRE_SEARCH);//
  do {
		uint8_t answerBit =0;
    int byteNum = ui32BitNumber / 8;
    uint8_t *current = (ROM) + byteNum;
    uint8_t cB, cmp_cB, searchDirection = 0;
    owSend(OW_READ); // read direct bit
    cB = owReadSlot(owEchoRead());//response from the sensor
    owSend(OW_READ); // read inverted bit
    cmp_cB = owReadSlot(owEchoRead());//response from the sensor
    if (cB == cmp_cB && cB == 1)//compare two answers
      return -1;//error nobody answered
    if (cB != cmp_cB) { //normal situation came either 10 or 01
      searchDirection = cB;//choose in which direction we will move further
			} else {//collision came 00 i.e. the current bit for ROMs is different
				if (ui32BitNumber == ow->lastDiscrepancy)//if the current position of the collision is equal to the previous one
        searchDirection = 1;//choose in which direction we will move further
      else {
        if (ui32BitNumber > ow->lastDiscrepancy) {//if we sewed on
          searchDirection = 0;//choose in which direction we will move further
        } else {
          searchDirection = (uint8_t) ((ow->lastROM[byteNum] >> ui32BitNumber % 8) & 0x01);
        }
        if (searchDirection == 0)
          zeroFork = ui32BitNumber;//remember the fork
      }
    }
    // save the beat
    if (searchDirection)
      *(current) |= 1 << ui32BitNumber % 8;//set a bit in the current byte byte
    answerBit = (uint8_t) ((searchDirection == 0) ? WIRE_0 : WIRE_1);// decide who to turn off
    owSend(answerBit);//we cut down the "interfering" devices
    ui32BitNumber++;//looking for the next beat
		} while (ui32BitNumber < 64);//until the whole ROM is found all bits
  ow->lastDiscrepancy = zeroFork;//remember the fork
  for (; i < 7; i++)
    ow->lastROM[i] = ROM[i];//remember last rom
  return ow->lastDiscrepancy > 0;
}

// Returns the number of devices on the bus, or an error code if the value is less than 0
int owSearchCmd(OneWire *ow) {
  int device = 0, nextROM;
  owInit(ow);
  do {
    nextROM = hasNextRom(ow, (uint8_t*)(&ow->ids[device])); //we pass a pointer to the structure where to put the next.ROM
    if (nextROM<0)
      return -1;
    device++;
		} while (nextROM && device < MAXDEVICES_ON_THE_BUS);//we are looking for while someone is there and these someone is no more define
		return device;//return the serial number of the sensor (device) on the bus
}

void owSkipRomCmd(OneWire *ow) {//sends a skip ROM command after that the next command will be
  owResetCmd();                 //for all devices on the bus
  owSendByte(ONEWIRE_SKIP_ROM);
}

void owMatchRomCmd(RomCode *rom) {//allows the master to access a specific slave device
	int i = 0;
  owResetCmd();
  owSendByte(ONEWIRE_MATCH_ROM);//referring to a specific device
  for (; i < 8; i++)
	owSendByte(*(((uint8_t *) rom) + i));//"we move through the structure as if through an array" with the first asterisk we get the i th byte from the structure
}

void owConvertTemperatureCmd(OneWire *ow, RomCode *rom) {
  owMatchRomCmd(rom);//allows the master to access a specific slave device
  owSendByte(ONEWIRE_CONVERT_TEMPERATURE);//tell the sensor it's time to convert the temperature
}

/**
 * Method for reading scratchad DS18B20 OR DS18S20
 * If sensor DS18B20 then data MUST be at least 9 byte
 * If sensor DS18S20 then data MUST be at least 2 byte
 * @param ow -- OneWire pointer
 * @param rom -- selected device on the bus
 * @param data -- buffer for data
 * @return data
 */
uint8_t *owReadScratchpadCmd(OneWire *ow, RomCode *rom, uint8_t *data) {//read sensor memory
  uint16_t b = 0, p;
  switch (rom->family) {
    case DS18B20:
    case DS18S20:
      p = 72;  //9*8 =72 == equals 9 bytes of data
      break;
    default:
      return data;

  }
  owMatchRomCmd(rom);
  owSendByte(ONEWIRE_READ_SCRATCHPAD);//send command to read memory
  while (b < p) {// until we have processed 9 bytes
    uint8_t pos = (uint8_t) ((p - 8) / 8 - (b / 8)); //position of the byte to be processed
    uint8_t bt; 
		owSend(OW_READ);
    bt = owReadSlot(owEchoRead());//read data
    if (bt == 1)
      data[pos] |= 1 << b % 8;//put the bit in the right position
    else
      data[pos] &= ~(1 << b % 8);//reset the bit at the correct position
    b++;//next bit
  }
  return data;
}

void owWriteDS18B20Scratchpad(OneWire *ow, RomCode *rom, uint8_t th, uint8_t tl, uint8_t conf) {
  if (rom->family != DS18B20)
    return;
  owMatchRomCmd(rom);//referring to a specific device
  owSendByte(ONEWIRE_WRITE_SCRATCHPAD);//we will memorize
  owSendByte(th);//temperature thresholds
  owSendByte(tl);
  owSendByte(conf);
}

/**
 * Get last mesaured temperature from DS18B20 or DS18S20. These temperature MUST be measured in previous
 * opearions. If you want to measure new value you can set reSense in true. In this case next invocation
 * that method will return value calculated in that step.
 * @param ow -- OneWire bus pointer
 * @param rom -- selected device
 * @param reSense -- do you want resense temp for next time?
 * @return struct with data
 */
Temperature readTemperature(OneWire *ow, RomCode *rom, uint8_t reSense) {
	Scratchpad_DS18B20 *sp;
	Scratchpad_DS18S20 *spP;
  Temperature t;
	uint8_t pad[9];
  t.inCelsus = 0x00;
  t.frac = 0x00;
  sp = (Scratchpad_DS18B20 *) &pad; 
  spP = (Scratchpad_DS18S20 *) &pad;
  switch (rom->family) {
    case DS18B20:
      owReadScratchpadCmd(ow, rom, pad);//read memory for DS18B20
      t.inCelsus = (int8_t) (sp->temp_msb << 4) | (sp->temp_lsb >> 4);//whole part
      t.frac = (uint8_t) ((((sp->temp_lsb & 0x0F)) * 10) >> 4);//fractional
      break;
    case DS18S20:
      owReadScratchpadCmd(ow, rom, pad);//read memory for DS18S20
      t.inCelsus = spP->temp_lsb >> 1;
      t.frac = (uint8_t) 5 * (spP->temp_lsb & 0x01);
      break;
    default:
      return t;
  }
  if (reSense) {
    owConvertTemperatureCmd(ow, rom);//you can immediately after the data was taken, we give the sensor a command to convert the temperature
  }
  return t;
}

void owCopyScratchpadCmd(OneWire *ow, RomCode *rom) {
  owMatchRomCmd(rom);
  owSendByte(ONEWIRE_COPY_SCRATCHPAD);
}

void owRecallE2Cmd(OneWire *ow, RomCode *rom) {
  owMatchRomCmd(rom);
  owSendByte(ONEWIRE_RECALL_E2);
}


int get_ROMid (void){
	if (owResetCmd() != ONEWIRE_NOBODY) {    // is anybody on the bus?
		devices = owSearchCmd(&ow);        // get the ROMid in the bus or return an error code
		if (devices <= 0) {
			while (1){
				pDelay = 1000000;
				for (i = 0; i < pDelay * 1; i++)    /* Wait a bit. */
					__asm__("nop");
			}

		}
		i = 0;
		for (; i < devices; i++) {//console output all found ROMs
			RomCode *r = &ow.ids[i];
			uint8_t crc = owCRC8(r);
			crcOK = (crc == r->crc)?"CRC OK":"CRC ERROR!";
			devInfo.device = i;

			sprintf(devInfo.info, "SN: %02X/%02X%02X%02X%02X%02X%02X/%02X", r->family, r->code[5], r->code[4], r->code[3],
					r->code[2], r->code[1], r->code[0], r->crc);

			if (crc != r->crc) {
				devInfo.device = i;
				sprintf (devInfo.info,"\n can't read cause CNC error");
			}
		}

	}
	pDelay = 1000000;
	for (i = 0; i < pDelay * 1; i++)
		__asm__("nop");

	if (strcmp(crcOK,"CRC OK") == 0) return 0;
	else return -1;
}

void get_Temperature (void)
{
	i=0;
	for (; i < devices; i++) {
		switch ((ow.ids[i]).family) {//what is our sensor
		case DS18B20:
			// the value of the previous measurement will be returned!
			t = readTemperature(&ow, &ow.ids[i], 1);
			Temp[i] = (float)(t.inCelsus*10+t.frac)/10.0;
			break;
		case DS18S20:
			t = readTemperature(&ow, &ow.ids[i], 1);
			Temp[i] = (float)(t.inCelsus*10+t.frac)/10.0;
			break;
		case 0x00:
			break;
		default:
			// error handler
			break;
		}
	}
//	pDelay = 4000000;
//	for (i = 0; i < pDelay * 1; i++){}   /* Wait a bit. */
}
