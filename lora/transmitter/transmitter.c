#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "lora/lora.h"
#include "task.h"

inline void delay(int ms){
	vTaskDelay(ms/portTICK_PERIOD_MS);
}

uint32_t counter = 0;
void loop(void *pvParameters){
	while(1) {
		delay(1000);

		char counter_string[32];	  
		sprintf(counter_string, "%d", counter++);
	  
		lora_begin_packet_default();
		lora_write_string("| Counter : ");
		lora_write_string(counter_string);
		lora_end_packet();
	  
    }
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    
    lora_set_pins(15, DEFAULT_RESET_PIN, DEFAULT_IRQ_PIN);
    if (!lora_begin(915E6)) {
		printf("Starting LoRa failed!");
		while (1);
	}
   
    printf("SDK version:%s\n", sdk_system_get_sdk_version());
    xTaskCreate(loop, "loop", 1024, NULL, 2, NULL);
}
