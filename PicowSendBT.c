/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "spp_counter.c"

// *****************************************************************************
/* EXAMPLE_START(spp_counter): SPP Server - Heartbeat Counter over RFCOMM
 *
 * @text The Serial port profile (SPP) is widely used as it provides a serial
 * port over Bluetooth. The SPP counter example demonstrates how to setup an SPP
 * service, and provide a periodic timer over RFCOMM.   
 *
 * @text Note: To test, please run the spp_counter example, and then pair from 
 * a remote device, and open the Virtual Serial Port.
 */
// *****************************************************************************

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 
#include "btstack.h"
#include "btstack_run_loop.h"
#include "pico/stdlib.h"
#include "picow_bt_example_common.h"



#define RFCOMM_SERVER_CHANNEL 1
#define HEARTBEAT_PERIOD_MS 5

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static uint16_t rfcomm_channel_id;
static uint8_t  spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;


#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "btstack_run_loop.h"
#define LOW 0
#define HIGH 1



// Raspberry Pi PICO PIN ASSIGN
#define FSR_0       26
#define FSR_1       27
#define BATT_V      28  // battery_lv
#define RED_LED     0
#define YELLOW_LED  1
#define GREEN_LED   2
#define TEST_PIN    4   //
#define AD_CTL_1    21  // 
#define AD_CTL_2    20
#define AD_CTL_3    19
#define AD_CTL_4    18
#define AD_CTL_5    17
#define AD_CTL_6    16

#define TEST_LED_ON    (cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,1))
#define TEST_LED_OFF   (cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,0))
#define AD_CTL_1_ON    (gpio_put(AD_CTL_1,HIGH))
#define AD_CTL_1_OFF   (gpio_put(AD_CTL_1,LOW))
#define AD_CTL_2_ON    (gpio_put(AD_CTL_2,HIGH))
#define AD_CTL_2_OFF   (gpio_put(AD_CTL_2,LOW))
#define AD_CTL_3_ON    (gpio_put(AD_CTL_3,HIGH))
#define AD_CTL_3_OFF   (gpio_put(AD_CTL_3,LOW))
#define AD_CTL_4_ON    (gpio_put(AD_CTL_4,HIGH))
#define AD_CTL_4_OFF   (gpio_put(AD_CTL_4,LOW))
#define AD_CTL_5_ON    (gpio_put(AD_CTL_5,HIGH))
#define AD_CTL_5_OFF   (gpio_put(AD_CTL_5,LOW))
#define AD_CTL_6_ON    (gpio_put(AD_CTL_6,HIGH))
#define AD_CTL_6_OFF   (gpio_put(AD_CTL_6,LOW))
#define TEST_PIN_H     (gpio_put(TEST_PIN,HIGH))
#define TEST_PIN_L     (gpio_put(TEST_PIN,LOW))
#define RED_LED_OFF    (gpio_put(RED_LED,HIGH))
#define RED_LED_ON     (gpio_put(RED_LED,LOW))
#define YELLOW_LED_OFF    (gpio_put(YELLOW_LED,HIGH))
#define YELLOW_LED_ON     (gpio_put(YELLOW_LED,LOW))
#define GREEN_LED_OFF    (gpio_put(GREEN_LED,HIGH))
#define GREEN_LED_ON     (gpio_put(GREEN_LED,LOW))

uint16_t analogRead(uint8_t channel)
{
   adc_select_input(channel);
   return adc_read();
}


unsigned long previousMillis = 0;
const long interval = 10; // 

char stringData[56];  //(count%100) + 13 * 16 bits data + cr and lf+ null;

// FSR
uint16_t fsr[12];

int test_cnt;


uint16_t battery_voltage;

int status_data;

int print_cnt;

int count = 0;

void adc_control(){
  AD_CTL_1_ON;
  sleep_us(20);
  fsr[0]=analogRead(FSR_0);
  fsr[1]=analogRead(FSR_1);
  AD_CTL_1_OFF;
  AD_CTL_2_ON;
  sleep_us(20);
  fsr[2]=analogRead(FSR_0);
  fsr[3]=analogRead(FSR_1);
  AD_CTL_2_OFF;
  AD_CTL_3_ON;
  sleep_us(20);
  fsr[4]=analogRead(FSR_0);
  fsr[5]=analogRead(FSR_1);
  AD_CTL_3_OFF;
  AD_CTL_4_ON;
  sleep_us(20);
  fsr[6]=analogRead(FSR_0);
  fsr[7]=analogRead(FSR_1);
  AD_CTL_4_OFF;
  AD_CTL_5_ON;
  sleep_us(20);
  fsr[8]=analogRead(FSR_0);
  fsr[9]=analogRead(FSR_1);
  AD_CTL_5_OFF;
  AD_CTL_6_ON;
  sleep_us(20);
  fsr[10]=analogRead(FSR_0);
  fsr[11]=analogRead(FSR_1);
  AD_CTL_6_OFF;
}


void measure_data_and_setString() {

  char  send_data[29];

  count++;
  if (count == 1000) {
    count = 0;
  }
  //
  if (count % 100 == 0) {
    stringData[0] = 0x23;
   } else {
    stringData[0] = 0x24;
   }

  battery_voltage = analogRead(BATT_V);
    sprintf(&stringData[1],"%04X",battery_voltage);
  for(int i=0;i<12;i++)
      sprintf(&stringData[5+(i*4)],"%04X",fsr[i]);
  stringData[53]='\r';
  stringData[54]='\n';
  stringData[55]='\0';
  printf(stringData);
//  if(SerialBT.overflow());
//    Serial.println("Overflow");
//  if(SerialBT.availableForWrite())
//     SerialBT.write(stringData,sizeof(stringData));

}


void setup()
{
  gpio_init(FSR_0);gpio_set_dir(FSR_0, GPIO_IN);
  gpio_init(FSR_1);gpio_set_dir(FSR_1, GPIO_IN);
  gpio_init(BATT_V);gpio_set_dir(BATT_V, GPIO_IN);
  adc_gpio_init(FSR_0);
  adc_gpio_init(FSR_1);
  adc_gpio_init(BATT_V);
  adc_init();

//  pinMode(LR_CHECK,GPIO_IN);
//  pinMode(LED_BUILTIN, GPIO_OUT);

  gpio_init(AD_CTL_1);gpio_set_dir(AD_CTL_1,GPIO_OUT);
  gpio_init(AD_CTL_2);gpio_set_dir(AD_CTL_2,GPIO_OUT);
  gpio_init(AD_CTL_3);gpio_set_dir(AD_CTL_3,GPIO_OUT);
  gpio_init(AD_CTL_4);gpio_set_dir(AD_CTL_4,GPIO_OUT);
  gpio_init(AD_CTL_5);gpio_set_dir(AD_CTL_5,GPIO_OUT);
  gpio_init(AD_CTL_6);gpio_set_dir(AD_CTL_6,GPIO_OUT);
  gpio_init(RED_LED);gpio_set_dir(RED_LED,GPIO_OUT);
  gpio_init(YELLOW_LED);gpio_set_dir(YELLOW_LED,GPIO_OUT);
  gpio_init(GREEN_LED);gpio_set_dir(GREEN_LED,GPIO_OUT);
  gpio_init(TEST_PIN);gpio_set_dir(TEST_PIN,GPIO_OUT);

  AD_CTL_1_OFF;
  AD_CTL_2_OFF;
  AD_CTL_3_OFF;
  AD_CTL_4_OFF;
  AD_CTL_5_OFF;
  AD_CTL_6_OFF;

  RED_LED_ON;
  YELLOW_LED_OFF;
  GREEN_LED_OFF;

  for(int i=0;i<5;i++){ 
     TEST_LED_ON;
    sleep_ms(100);
    TEST_LED_OFF;
    sleep_ms(100);
 }
   test_cnt =0;
    print_cnt =0;
  //  time_over_cnt =0;

    sleep_ms(1000);
    RED_LED_OFF;

}


/* @section SPP Service Setup 
 *s
 * @text To provide an SPP service, the L2CAP, RFCOMM, and SDP protocol layers 
 * are required. After setting up an RFCOMM service with channel nubmer
 * RFCOMM_SERVER_CHANNEL, an SDP record is created and registered with the SDP server.
 * Example code for SPP service setup is
 * provided in Listing SPPSetup. The SDP record created by function
 * spp_create_sdp_record consists of a basic SPP definition that uses the provided
 * RFCOMM channel ID and service name. For more details, please have a look at it
 * in \path{src/sdp_util.c}. 
 * The SDP record is created on the fly in RAM and is deterministic.
 * To preserve valuable RAM, the result could be stored as constant data inside the ROM.   
 */

/* LISTING_START(SPPSetup): SPP service setup */ 
static void spp_service_setup(void){

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();

#ifdef ENABLE_BLE
    // Initialize LE Security Manager. Needed for cross-transport key derivation
    sm_init();
#endif

    rfcomm_init();
    rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);  // reserved channel, mtu limited by l2cap

    // init SDP, create record for SPP and register with SDP
    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "SPP Counter");
    sdp_register_service(spp_service_buffer);
    printf("SDP service record size: %u\n", de_get_len(spp_service_buffer));
}
/* LISTING_END */

/* @section Periodic Timer Setup
 * 
 * @text The heartbeat handler increases the real counter every second, 
 * and sends a text string with the counter value, as shown in Listing PeriodicCounter. 
 */

/* LISTING_START(PeriodicCounter): Periodic Counter */ 
static btstack_timer_source_t heartbeat;
static char lineBuffer[30];
static void  heartbeat_handler(struct btstack_timer_source *ts){
    static int counter = 0;

    if (rfcomm_channel_id){

     adc_control();
     measure_data_and_setString();
     printf("%s", stringData);
     rfcomm_request_can_send_now_event(rfcomm_channel_id);
    }

    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
} 

static void one_shot_timer_setup(void){
    // set one-shot timer
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);
}
/* LISTING_END */


/* @section Bluetooth Logic 
 * @text The Bluetooth logic is implemented within the 
 * packet handler, see Listing SppServerPacketHandler. In this example, 
 * the following events are passed sequentially: 
 * - BTSTACK_EVENT_STATE,
 * - HCI_EVENT_PIN_CODE_REQUEST (Standard pairing) or 
 * - HCI_EVENT_USER_CONFIRMATION_REQUEST (Secure Simple Pairing),
 * - RFCOMM_EVENT_INCOMING_CONNECTION,
 * - RFCOMM_EVENT_CHANNEL_OPENED, 
* - RFCOMM_EVETN_CAN_SEND_NOW, and
 * - RFCOMM_EVENT_CHANNEL_CLOSED
 */

/* @text Upon receiving HCI_EVENT_PIN_CODE_REQUEST event, we need to handle
 * authentication. Here, we use a fixed PIN code "0000".
 *
 * When HCI_EVENT_USER_CONFIRMATION_REQUEST is received, the user will be 
 * asked to accept the pairing request. If the IO capability is set to 
 * SSP_IO_CAPABILITY_DISPLAY_YES_NO, the request will be automatically accepted.
 *
 * The RFCOMM_EVENT_INCOMING_CONNECTION event indicates an incoming connection.
 * Here, the connection is accepted. More logic is need, if you want to handle connections
 * from multiple clients. The incoming RFCOMM connection event contains the RFCOMM
 * channel number used during the SPP setup phase and the newly assigned RFCOMM
 * channel ID that is used by all BTstack commands and events.
 *
 * If RFCOMM_EVENT_CHANNEL_OPENED event returns status greater then 0,
 * then the channel establishment has failed (rare case, e.g., client crashes).
 * On successful connection, the RFCOMM channel ID and MTU for this
 * channel are made available to the heartbeat counter. After opening the RFCOMM channel, 
 * the communication between client and the application
 * takes place. In this example, the timer handler increases the real counter every
 * second. 
 *
 * RFCOMM_EVENT_CAN_SEND_NOW indicates that it's possible to send an RFCOMM packet
 * on the rfcomm_cid that is include

 */ 

/* LISTING_START(SppServerPacketHandler): SPP Server - Heartbeat Counter over RFCOMM */
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);

/* LISTING_PAUSE */ 
    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;
    uint16_t  mtu;
    int i;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
/* LISTING_RESUME */ 
                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // ssp: inform about user confirmation request
                    printf("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", little_endian_read_32(packet, 8));
                    printf("SSP User Confirmation Auto accept\n");
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rfcomm_channel_id);
                    break;
               
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        printf("RFCOMM channel open failed, status 0x%02x\n", rfcomm_event_channel_opened_get_status(packet));
                    } else {
                        rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rfcomm_channel_id, mtu);
                    }
                    break;
                case RFCOMM_EVENT_CAN_SEND_NOW:
                    rfcomm_send(rfcomm_channel_id, (uint8_t*) stringData, (uint16_t) strlen(stringData));  
                    break;

/* LISTING_PAUSE */                 
                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("RFCOMM channel closed\n");
                    rfcomm_channel_id = 0;
                    break;
                
                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            printf("RCV: '");
            for (i=0;i<size;i++){
                putchar(packet[i]);
            }
            printf("'\n");
            break;

        default:
            break;
    }
/* LISTING_RESUME */ 
}
/* LISTING_END */



int btstack_main(int argc, const char * argv[]);



int btstack_main(int argc, const char * argv[]){
    (void)argc;
    (void)argv;

    one_shot_timer_setup();
    spp_service_setup();

    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_local_name("SPP Counter 00:00:00:00:00:00");

    // turn on!
    hci_power_control(HCI_POWER_ON);
    
    return 0;
}





int main() {
    stdio_init_all();

    int res = picow_bt_example_init();
    if (res){
        return -1;
    }

    setup();

    picow_bt_example_main();

    btstack_run_loop_execute();
}



