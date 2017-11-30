#include "lora.h"

static int _lora_frequency = 0; 
static int _lora_packet_index = 0;
static int _lora_implicit_header_mode = 0;
static void (*_lora_on_receive)(int) = NULL;
static int _lora_spi_frequency = SPI_FREQ_DIV_8M;
static int _lora_ss = DEFAULT_CS_PIN;
static int _lora_reset = DEFAULT_RESET_PIN;
static int _lora_dio0 = DEFAULT_IRQ_PIN;

static uint8_t single_transfer(uint8_t address, uint8_t value){
	
	if(_lora_ss != DEFAULT_CS_PIN) gpio_write(_lora_ss,0);
	spi_set_address(1,8,address);
	uint8_t data = spi_transfer_8(1, value);
	spi_clear_address(1);
	if(_lora_ss != DEFAULT_CS_PIN) gpio_write(_lora_ss,1);
	return data;
}

static uint8_t read_register(uint8_t address){
	return single_transfer(address & SPI_READ_MASK,0x00);
}

static void write_register(uint8_t address, uint8_t value){
	single_transfer(address  | SPI_WRITE_MASK, value);
}

static void lora_explicit_header_mode(){
  _lora_implicit_header_mode = 0;
  
  uint8_t mdm_cfg = read_register(REG_MODEM_CONFIG_1);
  write_register(REG_MODEM_CONFIG_1, mdm_cfg & 0xfe);
}

static void lora_implicit_header_mode(){
  _lora_implicit_header_mode = 1;
  uint8_t mdm_cfg = read_register(REG_MODEM_CONFIG_1);
  write_register(REG_MODEM_CONFIG_1, mdm_cfg | 0x01);
}

static void lora_handle_dio0_rise(){
  int irqFlags = read_register(REG_IRQ_FLAGS);

  // clear IRQ's
  write_register(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _lora_packet_index = 0;

    // read packet length
    int packetLength = _lora_implicit_header_mode ? read_register(REG_PAYLOAD_LENGTH) : read_register(REG_RX_NB_BYTES);

    // set FIFO address to current RX address
    write_register(REG_FIFO_ADDR_PTR, read_register(REG_FIFO_RX_CURRENT_ADDR));

    if (_lora_on_receive) {
      _lora_on_receive(packetLength);
    }

    // reset FIFO address
    write_register(REG_FIFO_ADDR_PTR, 0);
  }
}

static void lora_on_dio0_rise(uint8_t gpio_num){
  lora_handle_dio0_rise();
}

void lora_set_pins(int ss, int reset, int dio0){
	
	_lora_ss = ss;
	_lora_reset = reset;
	_lora_dio0 = dio0;
}

void lora_set_spi_frequency(uint32_t spi_freq){
	_lora_spi_frequency = spi_freq;
}	

int lora_begin(long frequency){
	
	if(_lora_ss != DEFAULT_CS_PIN) gpio_enable(_lora_ss, GPIO_OUTPUT);
	gpio_enable(_lora_reset, GPIO_OUTPUT);
    
    gpio_write(_lora_reset, 0);
    vTaskDelay(10/portTICK_PERIOD_MS);
    gpio_write(_lora_reset, 1);
    vTaskDelay(10/portTICK_PERIOD_MS);
	
	if(_lora_ss != DEFAULT_CS_PIN) gpio_write(_lora_ss,1);
	
	spi_init(1, SPI_MODE0, _lora_spi_frequency, 1, SPI_LITTLE_ENDIAN, ((_lora_ss != DEFAULT_CS_PIN)?true:false) ); // init SPI module
	
	uint8_t version = read_register(REG_VERSION);
	if (version != 0x12) {
		return 0;
	}
		
	lora_sleep();
	lora_set_frequency(frequency);
	
	// set base addresses
	write_register(REG_FIFO_TX_BASE_ADDR, 0);
	write_register(REG_FIFO_RX_BASE_ADDR, 0);

	  // set LNA boost
	uint8_t lna_status = read_register(REG_LNA);
	write_register(REG_LNA, lna_status | 0x03);

	  // set auto AGC
	write_register(REG_MODEM_CONFIG_3, 0x04);
	
	lora_set_tx_power_pa_boost(17);
	
	lora_idle();
	
	return 1;
	
}

void lora_end(){
  // put in sleep mode
  lora_sleep();
}

int lora_begin_packet(int implicitHeader){
  // put in standby mode
  lora_idle();

  if (implicitHeader) {
    lora_implicit_header_mode();
  } else {
    lora_explicit_header_mode();
  }

  // reset FIFO address and paload length
  write_register(REG_FIFO_ADDR_PTR, 0);
  write_register(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int lora_begin_packet_default(){
  return lora_begin_packet(0);
}

int lora_end_packet(){
  // put in TX mode
  write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  // wait for TX done
  while((read_register(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);

  // clear IRQ's
  write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  return 1;
}



int lora_parse_packet(int size){
  int packetLength = 0;
  int irqFlags = read_register(REG_IRQ_FLAGS);

  if (size > 0) {
    lora_implicit_header_mode();

    write_register(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    lora_explicit_header_mode();
  }

  // clear IRQ's
  write_register(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _lora_packet_index = 0;

    // read packet length
    if (_lora_implicit_header_mode) {
      packetLength = read_register(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = read_register(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    uint8_t fifo_current_addr = read_register(REG_FIFO_RX_CURRENT_ADDR);
    write_register(REG_FIFO_ADDR_PTR, fifo_current_addr);

    // put in standby mode
    lora_idle();
  } else if (read_register(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    write_register(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

int lora_packet_rssi(){
  return (read_register(REG_PKT_RSSI_VALUE) - (_lora_frequency < 868E6 ? 164 : 157));
}

float lora_packet_snr(){
  return ((int8_t)read_register(REG_PKT_SNR_VALUE)) * 0.25;
}


size_t lora_write(const uint8_t *buffer, size_t size){
  int currentLength = read_register(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    write_register(REG_FIFO, buffer[i]);
  }

  // update length
  write_register(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

size_t lora_write_default(uint8_t byte){
  return lora_write(&byte, sizeof(byte));
}

size_t lora_write_string(char * input){
	
	size_t string_size = strlen(input);
	for(int i = 0 ; i < string_size; i++)
	lora_write_default((unsigned char)input[i]);
	
	return string_size;
}

int lora_available(){
  return (read_register(REG_RX_NB_BYTES) - _lora_packet_index);
}

int lora_read(){
  if (!lora_available()) {
    return -1;
  }

  _lora_packet_index++;

  return read_register(REG_FIFO);
}

int lora_peek(){
  if (!lora_available()) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = read_register(REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = read_register(REG_FIFO);

  // restore FIFO address
  write_register(REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

void lora_on_receive(void(*callback)(int)){
	
  _lora_on_receive = callback;

  if (callback) {
    write_register(REG_DIO_MAPPING_1, 0x00);
    
	gpio_enable(_lora_dio0, GPIO_INPUT);
	gpio_set_interrupt(_lora_dio0, GPIO_INTTYPE_EDGE_POS, lora_on_dio0_rise);
	//printf("Interrupt Init Success!");
    //attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  } else {
	// printf("callback empty");
    //detachInterrupt(digitalPinToInterrupt(_dio0));
  }
}

void lora_receive(int size){
  if (size > 0) {
    lora_implicit_header_mode();

    write_register(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    lora_explicit_header_mode();
  }

  write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void lora_receive_default(){
  lora_receive(0);
}


void lora_idle(){
  write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void lora_sleep(){
	write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void lora_set_tx_power(int level, int outputPin){
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    write_register(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level < 2) {
      level = 2;
    } else if (level > 17) {
      level = 17;
    }

    write_register(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

void lora_set_tx_power_pa_boost(int level){
	lora_set_tx_power(level, PA_OUTPUT_PA_BOOST_PIN);
}

void lora_set_frequency(long frequency){
	
  _lora_frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  write_register(REG_FRF_MSB, (uint8_t)(frf >> 16));
  write_register(REG_FRF_MID, (uint8_t)(frf >> 8));
  write_register(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void lora_set_spreading_factor(int sf){
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    write_register(REG_DETECTION_OPTIMIZE, 0xc5);
    write_register(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    write_register(REG_DETECTION_OPTIMIZE, 0xc3);
    write_register(REG_DETECTION_THRESHOLD, 0x0a);
  }

  write_register(REG_MODEM_CONFIG_2, (read_register(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void lora_set_signal_bandwidth(long sbw){
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  write_register(REG_MODEM_CONFIG_1, (read_register(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void lora_set_coding_rate_4(int denominator){
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  write_register(REG_MODEM_CONFIG_1, (read_register(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void lora_set_preamble_length(long length){
  write_register(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  write_register(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void lora_set_sync_word(int sw){
  write_register(REG_SYNC_WORD, sw);
}

void lora_enable_crc(){
	uint8_t mdm_cfg = read_register(REG_MODEM_CONFIG_2);
  write_register(REG_MODEM_CONFIG_2, mdm_cfg | 0x04);
}

void lora_disable_crc(){
	uint8_t mdm_cfg = read_register(REG_MODEM_CONFIG_2);
  write_register(REG_MODEM_CONFIG_2, mdm_cfg & 0xfb);
}

uint8_t lora_random(){
  return read_register(REG_RSSI_WIDEBAND);
}
