/* Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <byteswap.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"

#include "esp_system.h"
#include "esp_task_wdt.h"

#include "cjson.h"


#define WIFI_PASSWORD "{85208520}"//"qiangying"////CONFIG_WIFI_PASSWORD
#define WIFI_SSID     "ChinaMobile"//"dong_zhang"////CONFIG_WIFI_SSID

//tcp
int g_iSock_fd;
#define SERVER_IP  		"192.168.1.102"//"39.106.151.85"////
#define REMOTE_PORT		8088

//

#define ESPIDFV21RC 1

#if ESPIDFV21RC
  #include "esp_heap_alloc_caps.h"
#else
  #include "esp_heap_alloc_caps.h"
  #include "esp_heap_caps.h"
#endif

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/spi_reg.h"
#include "driver/hspi.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"

#include "soc/gpio_struct.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "camera.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "bitmap.h"
#include "driver/adc.h"


#define ADC2_TEST_CHANNEL 	2//ADC2_GPIO2_CHANNEL //CHANNEL2 IS io2
//(ADC2_CHANNEL_0)	// GPIO_NUM_4

//#include "esp_adc_cal.h"

void tcp_server_task(void* arg);
static void tcp_cli_task(void *pvParameters);

#define ESPI2SOUT	0
#if ESPI2SOUT

#include "driver/i2s.h"
#include <math.h>
#include "sounddata.h"
#define  I2S_DAC_BUILT_IN 0
#include "driver/dac.h"

#define SAMPLE_RATE     (36000)
#define I2S_NUM         (1)
#define WAVE_FREQ_HZ    (100)
#define PI 3.14159265

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)

volatile uint32_t isrCounter = 0;


//wav parameter
uint32_t chunkSize;
uint16_t numChannels;
uint32_t sampleRate;
uint16_t bitsPerSample;
uint8_t  data8;
uint16_t data16;

uint8_t  left;
// uint8_t  right;
uint16_t delayus;
//--end


static void setup_triangle_sine_waves(int bits)
{
    int *samples_data = malloc(((bits+8)/16)*SAMPLE_PER_CYCLE*4);
    unsigned int i, sample_val;
    double sin_float, triangle_float, triangle_step = (double) pow(2, bits) / SAMPLE_PER_CYCLE;

    printf("\r\nTest bits=%d free mem=%d, written data=%d\n", bits, esp_get_free_heap_size(), ((bits+8)/16)*SAMPLE_PER_CYCLE*4);

    triangle_float = -(pow(2, bits)/2 - 1);

    for(i = 0; i < SAMPLE_PER_CYCLE; i++) {
        sin_float = sin(i * PI / 180.0);
        if(sin_float >= 0)
            triangle_float += triangle_step;
        else
            triangle_float -= triangle_step;

        sin_float *= (pow(2, bits)/2 - 1);

        if (bits == 16) {
            sample_val = 0;
            sample_val += (short)triangle_float;
            sample_val = sample_val << 16;
            sample_val += (short) sin_float;
            samples_data[i] = sample_val;
        } else if (bits == 24) { //1-bytes unused
            samples_data[i*2] = ((int) triangle_float) << 8;
            samples_data[i*2 + 1] = ((int) sin_float) << 8;
        } else {
            samples_data[i*2] = ((int) triangle_float);
            samples_data[i*2 + 1] = ((int) sin_float);
        }

    }

    i2s_set_clk(I2S_NUM, SAMPLE_RATE, bits, 2);
    //Using push
    // for(i = 0; i < SAMPLE_PER_CYCLE; i++) {
    //     if (bits == 16)
    //         i2s_push_sample(0, &samples_data[i], 100);
    //     else
    //         i2s_push_sample(0, &samples_data[i*2], 100);
    // }
    // or write
    i2s_write_bytes(I2S_NUM, (const char *)samples_data, ((bits+8)/16)*SAMPLE_PER_CYCLE*4, 100);

    free(samples_data);
}

int play(const unsigned char *audio, uint32_t length ) {
  uint32_t count = 0;

  //check 'RIFF'
  if(strncmp((char*)audio, "RIFF", 4)){
    printf( "Error: It is not RIFF.");
    return -1;
  }

  //check 'WAVE'
  if(strncmp((char*)audio + 8, "WAVE", 4)){
	  printf( "Error: It is not WAVE.");
    return -1;
  }

  //check 'fmt '
  if(strncmp((char*)audio + 12, "fmt ", 4)){
	  printf( "Error: fmt not found.");
    return -1;
  }

  //get 'fmt ' data
  memcpy(&chunkSize, (char*)audio + 16, sizeof(chunkSize)); // size of fmt chunk
  memcpy(&numChannels, (char*)audio + 22, sizeof(numChannels)); // 1: mono or 2: stereo
  memcpy(&sampleRate, (char*)audio + 24, sizeof(sampleRate)); // 8kHz:8000 , 16kHz:16000, ...
  memcpy(&bitsPerSample, (char*)audio + 34, sizeof(bitsPerSample)); // 8bit or 16bit

  //skip to tne next chunk
  count = 16 + sizeof(chunkSize) + chunkSize;

  //skip to the data chunk
  while (strncmp((char*)audio + count , "data", 4)) {
    count += 4;
    memcpy(&chunkSize, (char*)audio + count, sizeof(chunkSize));
    count += sizeof(chunkSize) + chunkSize;
    if ( count > length ) return -1;
  }
  //skip to data area
  count += 4 + sizeof(chunkSize);

  //set playing parameter
  delayus = 1000000/sampleRate;  // 8000Hz = 125 , 16kHz = 62 ...
  //sigmaDeltaSetup(0, sampleRate); // setup channel 0 with sampleRate
  //sigmaDeltaSetup(0, 22050); // setup channel 0 with sampleRate
  //sigmaDeltaWrite(0, 0); //initialize channel 0 to off

  //play wav data
  while (count < length) {
    if (bitsPerSample == 16) {
      memcpy(&data16, (char*)audio + count, sizeof(data16));
      left = ((uint16_t) data16 + 32767) >> 8;
      count += sizeof(data16);
      if (numChannels == 2) count += sizeof(data16);
    } else {
      memcpy(&data8, (char*)audio + count, sizeof(data8));
      left = data8;
      count += sizeof(data8);
      if (numChannels == 2) count += sizeof(data8);
    }
    //sigmaDeltaWrite(0, left);
    dac_output_voltage(DAC_CHANNEL_1, left);
    dac_output_voltage(DAC_CHANNEL_2, left);

    ets_delay_us(delayus);
  }
  //sigmaDeltaWrite(127, 0);
  dac_output_voltage(DAC_CHANNEL_1, 127);
  dac_output_voltage(DAC_CHANNEL_2, 127);

  return 1;
}

#endif


#define ESPDHT11	1
#if ESPDHT11
  #include "dht11.h"
#define DHT_GPIO 14
	uint8_t dhtData[4];

#endif

#define ESPDS18B20	1
#if ESPDS18B20
#include "ds18b20.h" //Include library
//const int DS_PIN = 13; //GPIO where you connected ds18b20

#endif

#define ESPWS2812	0
#if ESPWS2812
#include "ws2812.h" //Include library

#define WS2812_PIN	15
const uint8_t pixel_count = 3; // Number of your "pixels"

#define delay_ms(ms) vTaskDelay((ms) / portTICK_RATE_MS)
#if 0
void rainbow(void *pvParameters)
{
  const uint8_t anim_step = 10;
  const uint8_t anim_max = 255;
//  const uint8_t pixel_count = 3; // Number of your "pixels"
  const uint8_t delay = 25; // duration between color changes
  rgbVal color = makeRGBVal(anim_max, 0, 0);
  uint8_t step = 0;
  rgbVal color2 = makeRGBVal(anim_max, 0, 0);
  uint8_t step2 = 0;

  rgbVal *pixels;
  pixels = malloc(sizeof(rgbVal) * pixel_count);

  while (1)
  {
#if 0
    color = color2;
    step = step2;

    for (uint8_t i = 0; i < pixel_count; i++) {
      pixels[i] = color;

      if (i == 1) {
        color2 = color;
        step2 = step;
      }

      switch (step) {
      case 0:
        color.g += anim_step;
        if (color.g >= anim_max)
          step++;
        break;
      case 1:
        color.r -= anim_step;
        if (color.r == 0)
          step++;
        break;
      case 2:
        color.b += anim_step;
        if (color.b >= anim_max)
          step++;
        break;
      case 3:
        color.g -= anim_step;
        if (color.g == 0)
          step++;
        break;
      case 4:
        color.r += anim_step;
        if (color.r >= anim_max)
          step++;
        break;
      case 5:
        color.b -= anim_step;
        if (color.b == 0)
          step = 0;
        break;
      }
    }

    ws2812_setColors(pixel_count, pixels);

    delay_ms(delay);
#endif
    color=makeRGBVal(0, 0, 0);
    pixels[0] = color;

    color=makeRGBVal(0, 0, 0);
    pixels[1] = color;

    color=makeRGBVal(0, 0, 0);
    pixels[2] = color;
    ws2812_setColors(pixel_count,  pixels);
    delay_ms(1000);


    color=makeRGBVal(anim_max, 0, 0);//rgb
    pixels[0] = color;

    color=makeRGBVal(0, anim_max, 0);
    pixels[1] = color;

    color=makeRGBVal(0, 0, anim_max);//
    pixels[2] = color;
    ws2812_setColors(pixel_count,  pixels);
    delay_ms(1000);

//===================

    color=makeRGBVal(0, 0, anim_max);
    pixels[0] = color;

    color=makeRGBVal(anim_max, 0, 0);
    pixels[1] = color;

    color=makeRGBVal(0, anim_max, 0);
    pixels[2] = color;
    ws2812_setColors(pixel_count,  pixels);
    delay_ms(1000);

//===================

     color=makeRGBVal(0, anim_max, 0);
     pixels[0] = color;

     color=makeRGBVal(0, 0, anim_max);
     pixels[1] = color;

     color=makeRGBVal(anim_max, 0, 0);
     pixels[2] = color;
     ws2812_setColors(pixel_count,  pixels);
     delay_ms(1000);

#if 0
        //============
        color=makeRGBVal(anim_max, anim_max, anim_max);
        pixels[0] = color;

        color=makeRGBVal(anim_max, anim_max, anim_max);
        pixels[1] = color;

        color=makeRGBVal(anim_max, anim_max, anim_max);
        pixels[2] = color;
        ws2812_setColors(pixel_count,  pixels);
        delay_ms(3000);
#endif

  }
}

#endif
#endif

#define ESPBH1750	1
#if ESPBH1750
  #include "bh1750.h"

#endif




#define CAMERA_PIXEL_FORMAT CAMERA_PF_RGB565
//#define CAMERA_PIXEL_FORMAT CAMERA_PF_YUV422
#define CAMERA_FRAME_SIZE CAMERA_FS_QVGA

static const char* TAG = "ESP-CAM";
static EventGroupHandle_t espilicam_event_group;
EventBits_t uxBits;
const int MOVIEMODE_ON_BIT = BIT0;




bool is_moviemode_on()
{
    return (xEventGroupGetBits(espilicam_event_group) & MOVIEMODE_ON_BIT) ? 1 : 0;
}

static void set_moviemode(bool c) {
    if (is_moviemode_on() == c) {
        return;
    } else {
      if (c) {
      xEventGroupSetBits(espilicam_event_group, MOVIEMODE_ON_BIT);
      } else {
      xEventGroupClearBits(espilicam_event_group, MOVIEMODE_ON_BIT);
      }
    }
}

// CAMERA CONFIG
static camera_pixelformat_t s_pixel_format;
static camera_config_t config = {
    .ledc_channel = LEDC_CHANNEL_0,
    .ledc_timer = LEDC_TIMER_0,
    .pin_d0 = CONFIG_D0,
    .pin_d1 = CONFIG_D1,
    .pin_d2 = CONFIG_D2,
    .pin_d3 = CONFIG_D3,
    .pin_d4 = CONFIG_D4,
    .pin_d5 = CONFIG_D5,
    .pin_d6 = CONFIG_D6,
    .pin_d7 = CONFIG_D7,
    .pin_xclk = CONFIG_XCLK,
    .pin_pclk = CONFIG_PCLK,
    .pin_vsync = CONFIG_VSYNC,
    .pin_href = CONFIG_HREF,
    .pin_sscb_sda = CONFIG_SDA,
    .pin_sscb_scl = CONFIG_SCL,
    .pin_reset = CONFIG_RESET,
    .xclk_freq_hz = CONFIG_XCLK_FREQ,
#if  CONFIG_ENABLE_TEST_PATTERN
    .test_pattern_enabled = CONFIG_ENABLE_TEST_PATTERN,//test_pattern_enabled =1   //disable = 0
#endif
    };

static camera_model_t camera_model;

// DISPLAY LOGIC
static inline uint8_t clamp(int n)
{
    n = n>255 ? 255 : n;
    return n<0 ? 0 : n;
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
static inline uint16_t ILI9341_color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint16_t get_grayscale_pixel_as_565(uint8_t pix) {
    // R = (img[n]&248)<<8; // 5 bit cao cua Y
    // G = (img[n]&252)<<3; // 6 bit cao cua Y
    // B = (img[n]&248)>>3; // 5 bit cao cua Y
    uint16_t graypixel=((pix&248)<<8)|((pix&252)<<3)|((pix&248)>>3);
    return graypixel;

}

// integers instead of floating point...
static inline uint16_t fast_yuv_to_rgb565(int y, int u, int v) {
int a0 = 1192 * (y - 16);
int a1 = 1634 * (v - 128);
int a2 = 832 * (v - 128);
int a3 = 400 * (u - 128);
int a4 = 2066 * (u - 128);
int r = (a0 + a1) >> 10;
int g = (a0 - a2 - a3) >> 10;
int b = (a0 + a4) >> 10;
return ILI9341_color565(clamp(r),clamp(g),clamp(b));

}

// fast but uses floating points...
static inline uint16_t fast_pascal_to_565(int Y, int U, int V) {
  uint8_t r, g, b;
  r = clamp(1.164*(Y-16) + 1.596*(V-128));
  g = clamp(1.164*(Y-16) - 0.392*(U-128) - 0.813*(V-128));
  b = clamp(1.164*(Y-16) + 2.017*(U-128));
  return ILI9341_color565(r,g,b);
}

//Warning: This gets squeezed into IRAM.
volatile static uint32_t *currFbPtr __attribute__ ((aligned(4))) = NULL;

inline uint8_t unpack(int byteNumber, uint32_t value) {
    return (value >> (byteNumber * 8));
}

// camera code

const static char http_hdr[] = "HTTP/1.1 200 OK\r\n";
const static char http_stream_hdr[] =
        "Content-type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n\r\n";
const static char http_jpg_hdr[] =
        "Content-type: image/jpg\r\n\r\n";
const static char http_pgm_hdr[] =
        "Content-type: image/x-portable-graymap\r\n\r\n";
const static char http_stream_boundary[] = "--123456789000000000000987654321\r\n";
const static char http_bitmap_hdr[] =
        "Content-type: image/bitmap\r\n\r\n";
const static char http_yuv422_hdr[] =
        "Content-Disposition: attachment; Content-type: application/octet-stream\r\n\r\n";

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;

// command parser...
#include "smallargs.h"

#define UNUSED(x) ((void)x)
#define RESPONSE_BUFFER_LEN 256
#define CMD_BUFFER_LEN 128

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            s_ip_addr = event->event_info.got_ip.ip_info.ip;

           // xTaskCreate(&tcp_server_task, "tcp_server_task", 4096, NULL, 5, NULL);//tcp server task
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
             auto-reassociate. */
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        case  SYSTEM_EVENT_AP_STACONNECTED:
        	ESP_LOGI(TAG, "get Connected");

        	break;
        case      SYSTEM_EVENT_AP_STADISCONNECTED:

        	break;

        default:
            break;
    }
    return ESP_OK;
}

static void init_wifi_sta(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    //printf("CONFIG_WIFI_PASSWORD IS %s\n",WIFI_PASSWORD);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    //ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");
}

static void init_wifi_ap(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

	 ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	 wifi_config_t wifi_config;
	 memcpy(wifi_config.ap.ssid, "WiFiCamera", sizeof("WiFiCamera"));
	 memcpy(wifi_config.ap.password, "12345678", sizeof("12345678"));
	 wifi_config.ap.ssid_len = strlen("WiFiCamera");
	 wifi_config.ap.max_connection = 2;
	 wifi_config.ap.authmode = WIFI_AUTH_WPA_PSK;
	 ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
	 esp_wifi_start();
}

static void init_wifi_staap(void)
{


}
// COMMAND PARSER
uint8_t getHexVal(char c)
{
   if(c >= '0' && c <= '9')
     return (uint8_t)(c - '0');
   else
     return (uint8_t)(c-'A'+10);
}

static void convert_fb32bit_line_to_bmp565(uint32_t *srcline, uint8_t *destline, const camera_pixelformat_t format) {

  uint16_t pixel565 = 0;
  uint16_t pixel565_2 = 0;
  uint32_t long2px = 0;
  uint16_t *sptr;
  int current_src_pos = 0, current_dest_pos = 0;
  for ( int current_pixel_pos = 0; current_pixel_pos < camera_get_fb_width(); current_pixel_pos += 2 )
  {
    current_src_pos = current_pixel_pos / 2;
    long2px = srcline[current_src_pos];
    if (format == CAMERA_PF_YUV422) {
        uint8_t y1, y2, u, v;
        y1 = unpack(0,long2px);
        v  = unpack(1,long2px);
        y2 = unpack(2,long2px);
        u  = unpack(3,long2px);

        pixel565 = fast_yuv_to_rgb565(y1,u,v);
        pixel565_2 = fast_yuv_to_rgb565(y2,u,v);

        sptr = &destline[current_dest_pos];
        *sptr = pixel565;
        sptr = &destline[current_dest_pos+2];
        *sptr = pixel565_2;
        current_dest_pos += 4;

    } else if (format == CAMERA_PF_RGB565) {
      pixel565 =  (unpack(2,long2px) << 8) | unpack(3,long2px);
      pixel565_2 = (unpack(0,long2px) << 8) | unpack(1,long2px);

      sptr = &destline[current_dest_pos];
      *sptr = pixel565;
      sptr = &destline[current_dest_pos+2];
      *sptr = pixel565_2;
      current_dest_pos += 4;
    }
  }
}


// TODO: handle http request while videomode on

static void http_server_netconn_serve(struct netconn *conn)
{
    /*  user data buff, it is designed on pbuf
        struct netbuff{
            struct pbuf *p, *ptr;
            struct ip_addr *addr;
            u16_t port;
        }
        netbuf: it is just a header, data storage in the pbuf.
                p is pointer of the first pbuf struct always,but 
                ptr maybe point other position. 
     */
    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;
    /*
    netconn_recv: Read the data from the port, and data recvive from recvmbox, To TCP
                  ,data in recvmbox is pbuf struct.
                  blocking if nothing yet there.
                  We assume the request (the part we care about) is in one netbuf
    */
    err = netconn_recv(conn, &inbuf);
    if (err == ERR_OK) {
        /*
            API function: let ptr of netbuf record pbuf data address put in dataptr
            buf:    start address of pbuf
            buflen: data length in the pbuf
        */
        netbuf_data(inbuf, (void**) &buf, &buflen);
        /* 
            Is this an HTTP GET command? (only check the first 5 chars, since
            there are other formats for GET, and we're keeping it very simple )
        */
        if (buflen >= 5 && buf[0] == 'G' && buf[1] == 'E' && buf[2] == 'T' && buf[3] == ' ' && buf[4] == '/') {
            printf("000\n");
            // disable videomode (autocapture) to allow streaming...
            bool s_moviemode = is_moviemode_on();
            set_moviemode(false);
            /* Send the HTTP header
             * subtract 1 from the size, since we dont send the \0 in the string
             * NETCONN_NOCOPY: our data is const static, so no need to copy it
             * http_hdr : first address of data
             * sizeof(http_hdr)-1: length of data
             * err_t netconn_write(struct netconn *conn, const void *dataptr, size_t size, u8_t apiflags);
             * #define NETCONN_NOFLAG 0X00
             * #define NETCONN_NOCOPY 0X00 not copy dataptr data to buff, so don't modify data
             * #define NETCONN_COPY   0X01 copy data to core thread space
             * #define NETCONN_MORE   0X02 last TCP package's PSH will be set, and receiver will trans data to upper
             */
            netconn_write(conn, http_hdr, sizeof(http_hdr) - 1,NETCONN_NOCOPY);
            //check if a stream is requested.
            if (buf[5] == 's') {
                printf("00\n");
                //Send mjpeg stream header
                err = netconn_write(conn, http_stream_hdr, sizeof(http_stream_hdr) - 1,NETCONN_NOCOPY);
                ESP_LOGD(TAG, "Stream started.");
                ESP_LOGD(TAG, "write http_stream hdr  = %d", err);
                //Run while everyhting is ok and connection open.
                while(err == ERR_OK) {
                    printf("01\n");
                    ESP_LOGD(TAG, "Capture frame");
                    //PAUSE_DISPLAY = true;
                    //err = camera_run();
                    //PAUSE_DISPLAY = false;
                    if (err != ESP_OK) {
                        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                    } else {
                        ESP_LOGD(TAG, "Done");
                        //stream an image..
                        if((s_pixel_format == CAMERA_PF_RGB565) || (s_pixel_format == CAMERA_PF_YUV422)) {
                            printf("02\n");
                            // write mime boundary start
                            err = netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1,NETCONN_NOCOPY);
                            // write bitmap header
                            char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
                            err = netconn_write(conn, bmp, sizeof(bitmap565), NETCONN_NOCOPY);
                            free(bmp);
                            // convert framebuffer on the fly...
                            // only rgb and yuv...
                            uint32_t *fbl;
                            uint8_t s_line[camera_get_fb_width()*2];
                            for (int i = 0; i < camera_get_fb_height(); i++) {
                                fbl = &currFbPtr[(i*camera_get_fb_width())/2];  //(i*(320*2)/4); // 4 bytes for each 2 pixel / 2 byte read..
                                convert_fb32bit_line_to_bmp565(fbl, s_line,s_pixel_format);
                                err = netconn_write(conn, s_line, camera_get_fb_width()*2,NETCONN_COPY);
                            }

                        }else {
                            printf("03\n");
                            // stream jpeg
                            err = netconn_write(conn, http_jpg_hdr, sizeof(http_jpg_hdr) - 1,NETCONN_NOCOPY);
                            if(err == ERR_OK)
                                err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),NETCONN_COPY);
                        }
                        if(err == ERR_OK){
                            //Send boundary to next jpeg
                            printf("04\n");
                            err = netconn_write(conn, http_stream_boundary,sizeof(http_stream_boundary) -1, NETCONN_NOCOPY);
                        }
                        vTaskDelay(30 / portTICK_RATE_MS);
                    }
                }
                ESP_LOGD(TAG, "Stream ended.");
//                ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
            } else {

                //CAMER_PF_JPEG
                if (s_pixel_format == CAMERA_PF_JPEG) {
                printf("0\n");
                    netconn_write(conn, http_jpg_hdr, sizeof(http_jpg_hdr) - 1, NETCONN_NOCOPY);

                //CAMERA_PF_GRAYSCALE
                } else if (s_pixel_format == CAMERA_PF_GRAYSCALE) {
                printf("1\n");
                    netconn_write(conn, http_pgm_hdr, sizeof(http_pgm_hdr) - 1, NETCONN_NOCOPY);
                    if (memcmp(&buf[5], "pgm", 3) == 0) {
                        char pgm_header[32];
                        snprintf(pgm_header, sizeof(pgm_header), "P5 %d %d %d\n", camera_get_fb_width(), camera_get_fb_height(), 255);
                        netconn_write(conn, pgm_header, strlen(pgm_header), NETCONN_COPY);
                    }else {
                        char outstr[120];
                        get_image_mime_info_str(outstr);
                        netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                        //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                    }

                //CAMERA_PF_RGB565
                } else if (s_pixel_format == CAMERA_PF_RGB565) {//bmp  000---2----2-a
                //printf("2\n");
                    netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
                    if (memcmp(&buf[5], "bmp", 3) == 0) {
                        printf("2-a\n");
                        char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
                        err = netconn_write(conn, bmp, sizeof(bitmap565), NETCONN_COPY);
                        free(bmp);
                    } else {
                        printf("2-b\n");
                        char outstr[120];
                        get_image_mime_info_str(outstr);
                        netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                        //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                    }

                //CAMERA_PF_YUV422
                } else if (s_pixel_format == CAMERA_PF_YUV422) {//000 --->  3 --->  a
                //printf("3\n");
                    if (memcmp(&buf[5], "bmp", 3) == 0) {
                        printf("3a\n");
                        //PAUSE_DISPLAY = true;
                        // send YUV converted to 565 2bpp for now...
                        netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
                        char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
                        err = netconn_write(conn, bmp, sizeof(bitmap565), NETCONN_COPY);
                        free(bmp);
                    } else {
                        printf("3b\n");
                        char outstr[120];
                        get_image_mime_info_str(outstr);
                        netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                        printf("3c\n");
                        //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                    }

                //other
                } else {
                printf("4\n");
                    char outstr[120];
                    get_image_mime_info_str(outstr);
                    netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                }

                // handle non streaming images (http../get and http:../bmp )
                ESP_LOGD(TAG, "Image requested.");
                //ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
                bool s_moviemode = is_moviemode_on();
                set_moviemode(false);
                set_moviemode(s_moviemode);
                //ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
                if (err != ESP_OK) {
                    ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                } else {
                    ESP_LOGD(TAG, "Done");
                    //Send jpeg
                    if ((s_pixel_format == CAMERA_PF_RGB565) || (s_pixel_format == CAMERA_PF_YUV422)) {
                        ESP_LOGD(TAG, "Converting framebuffer to RGB565 requested, sending...");
                        uint32_t *fbl;

if(CAMERA_FRAME_SIZE==CAMERA_FS_QQVGA)
{
                        uint8_t s_line[160*2];
                        for (int i = 0; i < 120; i++) {
                            printf("sending=%d\n", i);
                            fbl = &currFbPtr[(i*160)/2];  //(i*(320*2)/4); // 4 bytes for each 2 pixel / 2 byte read..
                            convert_fb32bit_line_to_bmp565(fbl, s_line, s_pixel_format);
                            err = netconn_write(conn, s_line, 160*2, NETCONN_COPY);
                        }
}
if(CAMERA_FRAME_SIZE==CAMERA_FS_QVGA)
{
                        uint8_t s_line[320*2];
                        for (int i = 0; i < 240; i++) {
                            printf("sending %d\n", i);
                            fbl = &currFbPtr[(i*320)/2];  //(i*(320*2)/4); // 4 bytes for each 2 pixel / 2 byte read..
                            convert_fb32bit_line_to_bmp565(fbl, s_line, s_pixel_format);
                            err = netconn_write(conn, s_line, 320*2, NETCONN_COPY);
                        }
}
                        //    ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
                    } else
                        err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),NETCONN_NOCOPY);
                } // handle .bmp and std gets...
            }// end GET request:
        set_moviemode(s_moviemode);
        }
    }
    netconn_close(conn); /* Close the connection (server closes in HTTP) */
    netbuf_delete(inbuf);/* Delete the buffer (netconn_recv gives us ownership,so we have to make sure to deallocate the buffer) */
}

static void http_server(void *pvParameters)
{
    /* in the lwip/api.h
       struct netconn{
        enum netconn_type type;
        enum netconn_state state;
        union {
            struct ip_pcb *ip;  // ip control block
            struct tcp_pcb *tcp; //tcp control block
            struct udp_pcb *udp; //udp control block
            struct raw_pcb *raw; // raw control block
        } pcb;
        err_t err;  //err flag
        sys_sem_t op_completed; //semphare
        sys_mbox_t recvmbox; //receve message box, buff queue
        sys_mbox_t acceptmbox; //connect accept queue
        int socket; //socket handle
        s16_t recv_avail; //buffed len in the receive message box
        struct api_msg_msg *write_msg; //send buff full, data storage here
        size_t write_offset; //next times send index
        netconn_callback callback; //about connect callback function 
       }
    */
    struct netconn *conn, *newconn;  
    err_t err,ert;
    /*
        alloc netconn space for netconn struct     
    */
    conn = netconn_new(NETCONN_TCP);  /* creat TCP connector */
    /*
    bind netconn&localIP&localPORT
    */
    netconn_bind(conn, NULL, 80);  /* bind HTTP port */
    /*
    SERVER listen state , register accept_function, if new conn come 
    */
    netconn_listen(conn);  /* server listen connect */
    do {
        /*
        netconn_accept: get a new conn from acceptmbox, if acceptmbox is NULL,
                        the thread will be blocked until new conn come. it is diffent of the API 
                        from IDF,as idf return esp log

        return:         address of new conn struct
        include:        err_t   netconn_accept(struct netconn *conn, struct netconn **new_conn);
        */
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {    /* new conn is coming */
            ert = camera_run();
            printf("%s\n",ert == ERR_OK ? "camer run success" : "camera run failed" );
            vTaskDelay(3000 / portTICK_RATE_MS);
            http_server_netconn_serve(newconn);
            /*
            netconn_delete: if status is connecting, after call this function, do active close.
             , delete newconn struct in the end.
            */
            netconn_delete(newconn);
        }
    } while (err == ERR_OK);
    /*
        netconn_close: function is close TCP conn, means that send a FIN package and return.
        notes: did not delete the netconn, if user want to delete the struct, call netconn_delete  
    */
    netconn_close(conn);
    netconn_delete(conn);
}

#if 0
int mystrcmp(const char *str1,const char *str2)
{
    /*不可用while(*str1++==*str2++)来比较，当不相等时仍会执行一次++，
    return返回的比较值实际上是下一个字符。应将++放到循环体中进行。*/
    while(*str1 == *str2)
    {
        if(*str1 == '\0')
            return 0;

        str1++;
        str2++;
    }
    return *str1 - *str2;
}
#endif


#if ESPDS18B20
#define	TCP_SERVER_PORT	8086
void tcp_server_task(void* arg)
{
#if ESPWS2812
	rgbVal *pixels;
	pixels = malloc(sizeof(rgbVal) * pixel_count);
	rgbVal color;

	ws2812_init(WS2812_PIN);
#endif

	struct sockaddr_in server_addr;
	struct sockaddr_in conn_addr;
	int sock_fd;             /* server socked */
	int sock_conn;          /* request socked */
	socklen_t addr_len;
	int err;
	int length;
	int count = 0;
	uint8_t rxbuffer[100];

SERVER_SOCK_BEGIN:
	    printf("tcp server task......\r\n");
	    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
	    if (sock_fd == -1)
	    {
	        printf("failed to create sock_fd!\n");
	        close(sock_fd);
	       // RAW_ASSERT(0);
	        goto SERVER_SOCK_BEGIN;
	    }

	    memset(&server_addr, 0, sizeof(server_addr));
	    server_addr.sin_family = AF_INET;
	    server_addr.sin_addr.s_addr =htonl(INADDR_ANY);//or
	    server_addr.sin_port = htons(TCP_SERVER_PORT);  //server port

	    err = bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
	    if (err < 0) {
	       // RAW_ASSERT(0);
	    	close(sock_fd);
			printf("TCP server ask bind error!!!\n");
			vTaskDelay(1000/portTICK_RATE_MS);
			goto SERVER_SOCK_BEGIN;
	    }

	    err = listen(sock_fd, 1);
	    if (err < 0) {
	       // RAW_ASSERT(0);
	    	close(sock_fd);
			printf("ESP8266	TCP	server task failed	to	set	listen	queue!\n");
			vTaskDelay(1000/portTICK_RATE_MS);
			goto SERVER_SOCK_BEGIN;

	    }

	    addr_len = sizeof(struct sockaddr_in);

	    printf("before accept!\n");
	    sock_conn = accept(sock_fd, (struct sockaddr *)&conn_addr, &addr_len);
	    printf("after accept!\n");

	    uint8_t colour_r=0,colour_g=0,colour_b=0;

	    while (1)
	    {
	        //char	*data_buffer	=	(char	*)zalloc(128);  //free(recv_buf);
	        length = recv(sock_conn, (unsigned int *)rxbuffer, 100, 0);
	        rxbuffer[length]=0x00;

	        printf("received string: %s,length is %d\n", rxbuffer,length);

#if ESPWS2812
	        if(rxbuffer[0]=='{'&&rxbuffer[1]=='"'&&rxbuffer[2]=='t'&&rxbuffer[3]=='"'&&rxbuffer[4]==':')
	        {
//{"t":4,"ss":0}
	        	if(rxbuffer[5]=='4')//switch
	        	{

	        		switch(rxbuffer[12])
	        		{
	        			case'0'://off
							color=makeRGBVal(0, 0, 0);
							pixels[0] = color;
							pixels[1] = color;
							pixels[2] = color;
							ws2812_setColors(pixel_count,  pixels);
	        			break;


	        			case'1'://on
							color=makeRGBVal(255, 255, 255);
							pixels[0] = color;
							pixels[1] = color;
							pixels[2] = color;
							ws2812_setColors(pixel_count,  pixels);
	        			break;

	        			default:break;
	        		}

	        	}
	        	if(rxbuffer[5]=='2')//colour
	        	{

	        		if(rxbuffer[13]==',')//{"t":2,"cr":8,"cg":7,"cb":255}
	        		{
	        			colour_r=rxbuffer[12]-'0';//r 1 bit
	        			if(rxbuffer[20]==',')
	        			{
	        				colour_g=rxbuffer[19]-'0';//{"t":2,"cr":8,"cg":7,"cb":2}
		        			if(rxbuffer[27]=='}')
		        			{
		        				colour_b=rxbuffer[26]-'0';
		        			}
		        			else if(rxbuffer[28]=='}')//{"t":2,"cr":8,"cg":7,"cb":20}
		        			{
		        				colour_b=(rxbuffer[26]-'0')*10+(rxbuffer[27]-'0');
		        			}
		        			else if(rxbuffer[29]=='}')//{"t":2,"cr":8,"cg":7,"cb":210}
		        			{
		        				colour_b=(rxbuffer[26]-'0')*100+(rxbuffer[27]-'0')*10+(rxbuffer[28]-'0');
		        			}

	        			}
	        			else if(rxbuffer[21]==',')//{"t":2,"cr":8,"cg":70,"cb":2}
	        			{
	        				colour_g=(rxbuffer[19]-'0')*10+(rxbuffer[20]-'0');
	              			if(rxbuffer[28]=='}')//{"t":2,"cr":8,"cg":70,"cb":2}
	        		        {
	        		        	colour_b=rxbuffer[27]-'0';
	        		        }
	        		        else if(rxbuffer[29]=='}')//{"t":2,"cr":8,"cg":70,"cb":20}
	        		        {
	        		        	colour_b=(rxbuffer[27]-'0')*10+(rxbuffer[28]-'0');
	        		        }
	        		        else if(rxbuffer[30]=='}')//{"t":2,"cr":8,"cg":17,"cb":210}
	        		        {
	        		        	colour_b=(rxbuffer[27]-'0')*100+(rxbuffer[28]-'0')*10+(rxbuffer[29]-'0');
	        		        }


	        			}
	        			else if(rxbuffer[22]==',')//{"t":2,"cr":8,"cg":170,"cb":2}
	        			{
	        				colour_g=(rxbuffer[19]-'0')*100+(rxbuffer[20]-'0')*10+(rxbuffer[21]-'0');
	              			if(rxbuffer[29]=='}')//{"t":2,"cr":8,"cg":170,"cb":2}
	        		        {
	        		        	colour_b=rxbuffer[28]-'0';
	        		        }
	        		        else if(rxbuffer[30]=='}')//{"t":2,"cr":8,"cg":170,"cb":20}
	        		        {
	        		        	colour_b=(rxbuffer[28]-'0')*10+(rxbuffer[29]-'0');//2 bit
	        		        }
	        		        else if(rxbuffer[31]=='}')//{"t":2,"cr":8,"cg":117,"cb":210}
	        		        {
	        		        	colour_b=(rxbuffer[28]-'0')*100+(rxbuffer[29]-'0')*10+(rxbuffer[30]-'0');
	        		        }
	        			}
	        		}
	        		else if(rxbuffer[14]==',')
	        		{
	        			colour_r=(rxbuffer[12]-'0')*10+(rxbuffer[13]-'0');//r 2 bit
	        			if(rxbuffer[21]==',')
	        			{
	        				colour_g=rxbuffer[20]-'0';//g 1 bit    {"t":2,"cr":80,"cg":7,"cb":2}
		        			if(rxbuffer[28]=='}')
		        			{
		        				colour_b=rxbuffer[27]-'0';//b 1 bit
		        			}
		        			else if(rxbuffer[29]=='}')//{"t":2,"cr":80,"cg":7,"cb":20}
		        			{
		        				colour_b=(rxbuffer[27]-'0')*10+(rxbuffer[28]-'0');
		        			}
		        			else if(rxbuffer[30]=='}')//{"t":2,"cr":80,"cg":7,"cb":210}
		        			{
		        				colour_b=(rxbuffer[27]-'0')*100+(rxbuffer[28]-'0')*10+(rxbuffer[29]-'0');
		        			}

	        			}
	        			else if(rxbuffer[22]==',')//{"t":2,"cr":80,"cg":70,"cb":2}
	        			{
	        				colour_g=(rxbuffer[20]-'0')*10+(rxbuffer[21]-'0');//b 2 bit
	              			if(rxbuffer[29]=='}')//{"t":2,"cr":80,"cg":70,"cb":2}
	        		        {
	        		        	colour_b=rxbuffer[28]-'0';
	        		        }
	        		        else if(rxbuffer[30]=='}')//{"t":2,"cr":80,"cg":70,"cb":20}
	        		        {
	        		        	colour_b=(rxbuffer[28]-'0')*10+(rxbuffer[29]-'0');
	        		        }
	        		        else if(rxbuffer[31]=='}')//{"t":2,"cr":80,"cg":17,"cb":210}
	        		        {
	        		        	colour_b=(rxbuffer[28]-'0')*100+(rxbuffer[29]-'0')*10+(rxbuffer[30]-'0');
	        		        }
	        			}
	        			else if(rxbuffer[23]==',')//{"t":2,"cr":80,"cg":170,"cb":2}
	        			{
	        				colour_g=(rxbuffer[20]-'0')*100+(rxbuffer[21]-'0')*10+(rxbuffer[22]-'0');//g 3
	              			if(rxbuffer[30]=='}')//{"t":2,"cr":80,"cg":170,"cb":2}
	        		        {
	        		        	colour_b=rxbuffer[29]-'0';
	        		        }
	        		        else if(rxbuffer[31]=='}')//{"t":2,"cr":80,"cg":170,"cb":20}
	        		        {
	        		        	colour_b=(rxbuffer[29]-'0')*10+(rxbuffer[30]-'0');//2 bit
	        		        }
	        		        else if(rxbuffer[32]=='}')//{"t":2,"cr":80,"cg":117,"cb":210}
	        		        {
	        		        	colour_b=(rxbuffer[29]-'0')*100+(rxbuffer[30]-'0')*10+(rxbuffer[31]-'0');
	        		        }
	        			}
	        		}
	        		else if(rxbuffer[15]==',')
	        		{
	        			colour_r=(rxbuffer[12]-'0')*100+(rxbuffer[13]-'0')*10+rxbuffer[14]-'0';//r  3 bit
	        			if(rxbuffer[22]==',')
	        			{
	        				colour_g=rxbuffer[21]-'0';//g 1 bit    {"t":2,"cr":180,"cg":7,"cb":2}
		        			if(rxbuffer[29]=='}')
		        			{
		        				colour_b=rxbuffer[28]-'0';//b 1 bit
		        			}
		        			else if(rxbuffer[30]=='}')//{"t":2,"cr":180,"cg":7,"cb":20}
		        			{
		        				colour_b=(rxbuffer[28]-'0')*10+(rxbuffer[29]-'0');
		        			}
		        			else if(rxbuffer[31]=='}')//{"t":2,"cr":180,"cg":7,"cb":210}
		        			{
		        				colour_b=(rxbuffer[28]-'0')*100+(rxbuffer[29]-'0')*10+(rxbuffer[30]-'0');
		        			}

	        			}
	        			else if(rxbuffer[23]==',')//{"t":2,"cr":180,"cg":70,"cb":2}
	        			{
	        				colour_g=(rxbuffer[21]-'0')*10+(rxbuffer[22]-'0');//g 2 bit
	              			if(rxbuffer[30]=='}')//{"t":2,"cr":180,"cg":70,"cb":2}
	        		        {
	        		        	colour_b=rxbuffer[29]-'0';
	        		        }
	        		        else if(rxbuffer[31]=='}')//{"t":2,"cr":180,"cg":70,"cb":20}
	        		        {
	        		        	colour_b=(rxbuffer[29]-'0')*10+(rxbuffer[30]-'0');
	        		        }
	        		        else if(rxbuffer[32]=='}')//{"t":2,"cr":180,"cg":17,"cb":210}
	        		        {
	        		        	colour_b=(rxbuffer[29]-'0')*100+(rxbuffer[30]-'0')*10+(rxbuffer[31]-'0');
	        		        }


	        			}
	        			else if(rxbuffer[24]==',')//{"t":2,"cr":180,"cg":170,"cb":2}
	        			{
	        				colour_g=(rxbuffer[21]-'0')*100+(rxbuffer[22]-'0')*10+(rxbuffer[23]-'0');//g 3
	              			if(rxbuffer[31]=='}')//{"t":2,"cr":180,"cg":170,"cb":2}
	        		        {
	        		        	colour_b=rxbuffer[30]-'0';
	        		        }
	        		        else if(rxbuffer[32]=='}')//{"t":2,"cr":180,"cg":170,"cb":20}
	        		        {
	        		        	colour_b=(rxbuffer[30]-'0')*10+(rxbuffer[31]-'0');//2 bit
	        		        }
	        		        else if(rxbuffer[33]=='}')//{"t":2,"cr":180,"cg":117,"cb":210}
	        		        {
	        		        	colour_b=(rxbuffer[30]-'0')*100+(rxbuffer[31]-'0')*10+(rxbuffer[32]-'0');
	        		        }
	        			}
	        		}
					color=makeRGBVal(colour_r,colour_g, colour_b);
					printf("r=%d, g=%d, b=%d\r\n",colour_r,colour_g, colour_b);
					pixels[0] = color;
					pixels[1] = color;
					pixels[2] = color;
					ws2812_setColors(pixel_count,  pixels);
	        	}

	        	if(rxbuffer[5]=='3')//breath
	        	{
	        		uint8_t anim_max=255;
	        		switch(rxbuffer[12])
	        		{
	        			case'0'://slow
	        			    color=makeRGBVal(anim_max, 0, 0);//rgb
	        			    pixels[0] = color;

	        			    color=makeRGBVal(0, anim_max, 0);
	        			    pixels[1] = color;

	        			    color=makeRGBVal(0, 0, anim_max);//
	        			    pixels[2] = color;
	        			    ws2812_setColors(pixel_count,  pixels);
	        			    delay_ms(5000);

	        			//===================

	        			    color=makeRGBVal(0, 0, anim_max);
	        			    pixels[0] = color;

	        			    color=makeRGBVal(anim_max, 0, 0);
	        			    pixels[1] = color;

	        			    color=makeRGBVal(0, anim_max, 0);
	        			    pixels[2] = color;
	        			    ws2812_setColors(pixel_count,  pixels);
	        			    delay_ms(5000);

	        			//===================

	        			     color=makeRGBVal(0, anim_max, 0);
	        			     pixels[0] = color;

	        			     color=makeRGBVal(0, 0, anim_max);
	        			     pixels[1] = color;

	        			     color=makeRGBVal(anim_max, 0, 0);
	        			     pixels[2] = color;
	        			     ws2812_setColors(pixel_count,  pixels);
	        			     delay_ms(5000);

	        			break;


	        			case'1'://medi

	        			    color=makeRGBVal(anim_max, 0, 0);//rgb
	        			    pixels[0] = color;

	        			    color=makeRGBVal(0, anim_max, 0);
	        			    pixels[1] = color;

	        			    color=makeRGBVal(0, 0, anim_max);//
	        			    pixels[2] = color;
	        			    ws2812_setColors(pixel_count,  pixels);
	        			    delay_ms(1000);

	        			//===================

	        			    color=makeRGBVal(0, 0, anim_max);
	        			    pixels[0] = color;

	        			    color=makeRGBVal(anim_max, 0, 0);
	        			    pixels[1] = color;

	        			    color=makeRGBVal(0, anim_max, 0);
	        			    pixels[2] = color;
	        			    ws2812_setColors(pixel_count,  pixels);
	        			    delay_ms(1000);

	        			//===================

	        			    color=makeRGBVal(0, anim_max, 0);
	        			    pixels[0] = color;

	        			    color=makeRGBVal(0, 0, anim_max);
	        			    pixels[1] = color;

	        			    color=makeRGBVal(anim_max, 0, 0);
	        			    pixels[2] = color;
	        			    ws2812_setColors(pixel_count,  pixels);
	        			    delay_ms(1000);

	        			break;

	        			case'2'://fast
	        			    color=makeRGBVal(anim_max, 0, 0);//rgb
	        			    pixels[0] = color;

	        			    color=makeRGBVal(0, anim_max, 0);
	        			    pixels[1] = color;

	        			    color=makeRGBVal(0, 0, anim_max);//
	        			    pixels[2] = color;
	        			    ws2812_setColors(pixel_count,  pixels);
	        			    delay_ms(500);

	        			//===================

	        			    color=makeRGBVal(0, 0, anim_max);
	        			    pixels[0] = color;

	        			    color=makeRGBVal(anim_max, 0, 0);
	        			    pixels[1] = color;

	        			    color=makeRGBVal(0, anim_max, 0);
	        			    pixels[2] = color;
	        			    ws2812_setColors(pixel_count,  pixels);
	        			    delay_ms(500);

	        			//===================

	        			     color=makeRGBVal(0, anim_max, 0);
	        			     pixels[0] = color;

	        			     color=makeRGBVal(0, 0, anim_max);
	        			     pixels[1] = color;

	        			     color=makeRGBVal(anim_max, 0, 0);
	        			     pixels[2] = color;
	        			     ws2812_setColors(pixel_count,  pixels);
	        			     delay_ms(500);

	        			break;

	        			default:break;
	        		}

	        	}
	        	if(rxbuffer[5]=='1')//light
	        	{
	        		switch(rxbuffer[12])
	        		{
	        			case'0'://
							color=makeRGBVal((uint8_t)(colour_r*0.1),(uint8_t)(colour_g*0.1), (uint8_t)(colour_b*0.1));
							printf("Low Light:r=%d, g=%d, b=%d\r\n",(uint8_t)(colour_r*0.1),(uint8_t)(colour_g*0.1), (uint8_t)(colour_b*0.1));
							pixels[0] = color;
							pixels[1] = color;
							pixels[2] = color;
							ws2812_setColors(pixel_count,  pixels);
	        			break;


	        			case'1'://
							color=makeRGBVal((uint8_t)(colour_r*0.6),(uint8_t)(colour_g*0.6), (uint8_t)(colour_b*0.6));
							printf("Medium Light:r=%d, g=%d, b=%d\r\n",(uint8_t)(colour_r*0.6),(uint8_t)(colour_g*0.6), (uint8_t)(colour_b*0.6));
							pixels[0] = color;
							pixels[1] = color;
							pixels[2] = color;
							ws2812_setColors(pixel_count,  pixels);

	        			break;
	        			case'2'://
							color=makeRGBVal((uint8_t)(colour_r),(uint8_t)(colour_g), (uint8_t)(colour_b));
							printf("High Light:r=%d, g=%d, b=%d\r\n",colour_r,colour_g, colour_b);
							pixels[0] = color;
							pixels[1] = color;
							pixels[2] = color;
							ws2812_setColors(pixel_count,  pixels);
	        			break;

	        			default:break;
	        		}
	        	}
	        	memset(rxbuffer, 0, length);
	        }
#endif
	        //send(sock_conn, "good", 5, 0);
	//        send(sock_conn, data_buffer, length, 0);
	        if(length<=0)
	        {
	    		printf("ESP8266 TCP server task read data fail!\n");
	    		close(sock_conn);

	    		goto SERVER_SOCK_BEGIN;
	    		//sock_conn = accept(sock_fd, (struct sockaddr *)&conn_addr, &addr_len);
	        }
	    }

	/**
	while(1){

		//printf("test2...%d...\r\n",system_get_time()-g_user_x);
		if(uart0ReceiveNum>10) uart0ReceivedFlag=1;
		if(uart0ReceivedFlag==1)
		{
			uart0_tx_buffer(uart0RxBuf,uart0ReceiveNum);
			uart0ReceiveNum=0;
		}
	}
	**/
}
#endif


void sendpic(void)
{
    bool s_moviemode = is_moviemode_on();
    set_moviemode(false);
    char err;
    char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
    err=send(g_iSock_fd, bmp, sizeof(bitmap565), 0);
    free(bmp);

    ESP_LOGD(TAG, "-Image requested.");
    //ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
    uint32_t *fbl;
    s_moviemode = is_moviemode_on();
    set_moviemode(false);
    set_moviemode(s_moviemode);

//    if(CAMERA_FRAME_SIZE==CAMERA_FS_QVGA)
    {
    	uint8_t s_line[camera_get_fb_width()*2];
    	for (int i = 0; i < camera_get_fb_height(); i++)
    	{
    		printf("-sending %d\n", i);
    		fbl = &currFbPtr[(i*camera_get_fb_width())/2];  //(i*(320*2)/4); // 4 bytes for each 2 pixel / 2 byte read..
    		convert_fb32bit_line_to_bmp565(fbl, s_line, s_pixel_format);
    		err = send(g_iSock_fd, s_line, camera_get_fb_width()*2, 0);
    	}
    }
    //free(s_line);

}


void printJson(cJSON * root)//以递归的方式打印json的最内层键值对
{
    for(int i=0; i<cJSON_GetArraySize(root); i++)   //遍历最外层json键值对
    {
        cJSON * item = cJSON_GetArrayItem(root, i);
        if(cJSON_Object == item->type)      //如果对应键的值仍为cJSON_Object就递归调用printJson
        {
        	printJson(item);
        }
        else if(cJSON_String == item->type)                              //值不为json对象就直接打印出键和值
        {
            printf("%s->", item->string);
            printf("%s\n", cJSON_Print(item));
        }
        else if((cJSON_Number == item->type))
	    {
        	printf("%s=", item->string);
	        printf("%d->",item->valueint);
	        printf("%s\n",cJSON_Print(item));
	    }
        else if((cJSON_Array == item->type))
	    {
        	printf("%s=", item->string);
	        //printf("%d->",item->valueint);
	        printf("%s\n",cJSON_Print(item));
	    }
        else if((cJSON_True == item->type)||(cJSON_False == item->type))
	    {
        	printf("%s:", item->string);
	        printf("%d->",item->valueint);
	        printf("%s\n",cJSON_Print(item));
	    }
        else if((cJSON_NULL == item->type))
	    {
        	printf("%s:", item->string);
	    }



    }
}

TimerHandle_t xTimerUser; // 定义句柄
// 定时器回调函数格式
void vTimerCallback( TimerHandle_t xTimer )
{
    // do something no block
    // 获取溢出次数
    uint32_t ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
    // 累积溢出次数
    ++ulCount;
    // 更新溢出次数
   // vTimerSetTimerID( xTimer, ( void * ) ulCount );

    if (ulCount >= 10) {
        // 停止定时器
        xTimerStop( xTimer, 0 );
        printf("Stop xTimer/r/n");
    }
}


static void tcp_send_task(void *pvParameters)
{
	uint32_t esp_timer_count =system_get_time();
    //esp_timer_count=system_get_time();
    while(1)
    {
    	//esp_task_wdt_feed();
    	//if((system_get_time()-esp_timer_count)>=20000000)  //20s
    	{
    		esp_timer_count=system_get_time();
    		cJSON * root =  cJSON_CreateObject();//NULL
    	    //cJSON * item =  cJSON_CreateObject();
    	    //cJSON * next =  cJSON_CreateObject();

    		int Num=1;
    		//cJSON_AddItemToObject(root, "AUTH", cJSON_CreateString("TDP10"));
    	    cJSON_AddItemToObject(root, "pid", cJSON_CreateNumber(Num));//根节点下添加  或者cJSON_AddNumberToObject(root, "rc",Num);
    	    Num=1;
    	    cJSON_AddItemToObject(root, "tid", cJSON_CreateNumber(Num));
    	    cJSON_AddItemToObject(root, "type", cJSON_CreateString("WIFI0"));//或者  cJSON_AddStringToObject(root, "operation", "CALL");
    		//cJSON_AddItemToObject(root, "swVer", cJSON_CreateString("1.0"));
    		//cJSON_AddItemToObject(root, "hwVER", cJSON_CreateString("1.0"));

    	    cJSON_AddItemToObject(root, "srcMac",cJSON_CreateString("A4-E9-75-3D-DC-DF"));
//    		cJSON_AddItemToObject(root, "dstMac",cJSON_CreateString("04-12-56-7E-2A-38"));


#if(ESPDHT11==1)
    	    setDHTPin(DHT_GPIO);
    	    if(ReadDHT11(dhtData))  //get temprature
    	    {
    	    	uint8_t outstr[10];
    	    	DHT11_NumToString(dhtData[0],outstr);

    	    	printf("Relative Humidity   :%s%%\r\n",outstr);

    	    	DHT11_NumToString(dhtData[1],outstr);

    	    	DHT11_NumToString(dhtData[2],outstr);
    	    	printf("Current Temperature :%sC\r\n",outstr);
    	    	DHT11_NumToString(dhtData[3],outstr);

    	    	 cJSON_AddItemToObject(root, "temp", cJSON_CreateNumber(dhtData[2]));
    	    	 cJSON_AddItemToObject(root, "humidity", cJSON_CreateNumber(dhtData[0]));

    	    }
    	    else
    	    {
    	    	printf("--Read DHT11 Error!--\n");

    	    }

#endif
#if(ESPBH1750==1)

				Init_BH1750(33, 27);
				printf("--Ambient Light[%d]==%0.2flx\r\n",Read_BH1750(),Convert_BH1750());
				cJSON_AddItemToObject(root, "illum", cJSON_CreateNumber(Read_BH1750()));
#endif

    	    Num=28;

    	    cJSON_AddItemToObject(root, "waterLevel", cJSON_CreateNumber(90));
    	    //cJSON_AddItemToObject(root, "illum", cJSON_CreateNumber(65535));
    	    cJSON_AddItemToObject(root, "red", cJSON_CreateNumber(255));
    	    cJSON_AddItemToObject(root, "green", cJSON_CreateNumber(255));
    	    cJSON_AddItemToObject(root, "blue", cJSON_CreateNumber(255));
    	    cJSON_AddItemToObject(root, "bright", cJSON_CreateNumber(100));
    	    cJSON_AddItemToObject(root, "led", cJSON_CreateNumber(1));
    	    cJSON_AddItemToObject(root, "ledMode", cJSON_CreateNumber(8));
    	    cJSON_AddItemToObject(root, "fan", cJSON_CreateNumber(1));
    	    cJSON_AddItemToObject(root, "pump", cJSON_CreateNumber(0));
    	    cJSON_AddItemToObject(root, "sound", cJSON_CreateNumber(0));
    	    cJSON_AddItemToObject(root, "live", cJSON_CreateNumber(1));
    	    printf("%s\n", cJSON_Print(root));
    	    int ret=send(g_iSock_fd, cJSON_Print(root), strlen(cJSON_Print(root)), 0);//canot use sizeof(cJSON_Print(root))

    	}
    	vTaskDelay(20000 / portTICK_RATE_MS);
    }
}

static void tcp_cli_task(void *pvParameters)
{
	struct sockaddr_in server_addr;
	//uint8_t
	char data_buffer[512];
	uint8_t xbuffer[]="Temparature: 30.00C ";
//	uint8_t *xbuffer=malloc(512);
	int length;
	int err;

#if(ESPWS2812==1)
	rgbVal *pixels;
	pixels = malloc(sizeof(rgbVal) * pixel_count);
	ws2812_init(WS2812_PIN);
#endif

SOCKBEGIN:

	do
	{
		g_iSock_fd = socket(AF_INET, SOCK_STREAM, 0);//socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (g_iSock_fd == -1)
		{
			close(g_iSock_fd);
			printf("failed to create cli socket %d!\n",g_iSock_fd);
			vTaskDelay(1000/portTICK_RATE_MS);
		}
	}while(g_iSock_fd==-1);

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr =inet_addr(SERVER_IP); /* 将目标服务器的IP写入一个结构体 */
    server_addr.sin_port = htons(REMOTE_PORT);

	do
    {
       err=connect(g_iSock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr));//if success is 0
       if(err!=0)
       {
        	printf("connect error %d....\r\n",err); //-1
        	close(g_iSock_fd);
        	vTaskDelay(1000/portTICK_RATE_MS);
        	goto SOCKBEGIN;
       }
    }while(err!=0);

    printf("connect server success %s:%d....\r\n",SERVER_IP,REMOTE_PORT);

//    send(g_iSock_fd,"The more efforts you make, the more fortunes you get.",sizeof("The more efforts you make, the more fortunes you get."), 0);



	cJSON * root =  cJSON_CreateObject();//NULL
    cJSON * item =  cJSON_CreateObject();
    cJSON * next =  cJSON_CreateObject();

	int Num=1;
	cJSON_AddItemToObject(root, "auth", cJSON_CreateString("TDP10"));
    cJSON_AddItemToObject(root, "pid", cJSON_CreateNumber(Num));//根节点下添加  或者cJSON_AddNumberToObject(root, "rc",Num);
    Num=1;
    cJSON_AddItemToObject(root, "tid", cJSON_CreateNumber(Num));
    cJSON_AddItemToObject(root, "type", cJSON_CreateString("WIFI"));//或者  cJSON_AddStringToObject(root, "operation", "CALL");
	cJSON_AddItemToObject(root, "swVer", cJSON_CreateString("1.0"));
	cJSON_AddItemToObject(root, "hwVer", cJSON_CreateString("1.0"));

   // cJSON_AddItemToObject(root, "Mac", item);//root节点下添加节点MAC
    cJSON_AddItemToObject(root, "srcMac",cJSON_CreateString("A4-E9-75-3D-DC-DF"));
	cJSON_AddItemToObject(root, "dstMac",cJSON_CreateString("04-12-56-7E-2A-38"));

    printf("%s\n", cJSON_Print(root));
    send(g_iSock_fd, cJSON_Print(root), strlen(cJSON_Print(root)), 0);//canot use sizeof(cJSON_Print(root))

//	cJSON_Delete(root); // 释放内存

    while(1)
	{

		length = recv(g_iSock_fd, data_buffer, sizeof(data_buffer), 0); //>0 返回收到的字节数目   =0断开连接  <0错误

		if(length>0)//
		{
			data_buffer[length]='\0';//0x00;//字符串结束
			printf("tcp cli rcv: %s/%d\r\n",data_buffer,length);
			//send(g_iSock_fd, data_buffer,length, 0);

			if(data_buffer[0]!='{'&&data_buffer[length]!='}') return;  //cjson
				cJSON * root = NULL;
				cJSON * item = NULL;//cjson对象
				root = cJSON_Parse(data_buffer);
				if (!root)
				{
					printf("Error before: [%s]\n",cJSON_GetErrorPtr());
				}
				else
				{
					  printf("%s\n", "有格式的方式打印Json:");
					  printf("%s\n\n", cJSON_Print(root));
					  printf("%s\n", "无格式方式打印json：");
					  printf("%s\n\n", cJSON_PrintUnformatted(root));

					  printf("%s\n", "获取auth下的cjson对象:");
					  item = cJSON_GetObjectItem(root, "auth");//
#if 0
					  switch(item->type)
					  {
					      case cJSON_String:
					      //printf("%s\n",item->valuestring); //TDP
					      break;
					      case cJSON_Number:
					      //printf("%d\n", item->valueint);
					      break;
					      case cJSON_NULL:
					      //printf("%d\n", cJSON_Print(item));
					      break;
					      case cJSON_True:
					      //printf("%d\n", cJSON_Print(item));
					      break;
					      case cJSON_False:
					      //printf("%d\n", cJSON_Print(item));
					      break;

					      default:break;
					   }
#endif
					   printf("%s\n", cJSON_Print(item));//"TDP"
					   //printf("%d\n", item->type);//4——cJSON_String
					   //printf("%s\n", item->string);//auth
					  // printf("%s\n", item->valuestring);//TDP

					   printf("%s\n", "获取pid下的cjson对象");
					   item = cJSON_GetObjectItem(root, "pid");
					   printf("%s\n", cJSON_Print(item));

					   //printf("%s:", item->string);   //看一下cjson对象的结构体中这两个成员的意思  //pid:
					   //printf("%s\n", item->valuestring);// 123456789

					   printf("%s\n", "获取TYPE下的cjson对象");
					   item = cJSON_GetObjectItem(root, "type");
					   printf("%s\n", cJSON_Print(item));
					   //printf("%d\n", item->valueint);//56
					   //printf("%06f\n", item->valuedouble);//???

					   printf("%s\n", "获取swVer下的cjson对象");
					   item = cJSON_GetObjectItem(root, "swVer");
					   printf("%s\n", cJSON_Print(item));

					   printf("%s\n", "获取Mac下的cjson对象");
					   item = cJSON_GetObjectItem(root, "srcMac");
					   printf("%s\n", cJSON_Print(item));

					   item = cJSON_GetObjectItem(root , "dstMac");
					   printf("%s\n", cJSON_Print(item));

					   //printf("%d\n", item->type);//get error why?
					   //printf("%s\n", item->valuestring);

					   printf("\n%s\n", "打印json所有最内层键值对:");
					   printJson(root);


#if 0
					   int i=0;
					   for(i=0; i<cJSON_GetArraySize(root); i++)   //遍历最外层json键值对
					   {
						   cJSON * item = cJSON_GetArrayItem(root, i);
					       if(cJSON_Object == item->type)      //如果对应键的值仍为cJSON_Object就递归调用printJson
					       {  //printJson(item);
					    	   //cJSON * next = cJSON_GetArrayItem(item, i);
					       }
					        else if((cJSON_String == item->type))                                //值不为json对象就直接打印出键和值
					        {
					            printf("%s->", item->string);
					            printf("%s\n", cJSON_Print(item));
					        }
					        else if((cJSON_Number == item->type))
						    {
					        	printf("%s=", item->string);
						        printf("%d->", item->valueint);
						        printf("%s\n", cJSON_Print(item));

						    }

					    }
#endif
				if(root!=NULL)
				{
					cJSON_Delete(root);
				}

				}




#if 0
			if(data_buffer[0]=='b' &&data_buffer[1]=='m'&&data_buffer[2]=='p')
			{

		        err = camera_run();
		        printf("%s\n",err == ERR_OK ? "--camer run success--" : "==camera run failed==" );
		        vTaskDelay(1000 / portTICK_RATE_MS);

	            if(err==ERR_OK)
				sendpic();
			}


#if(ESPDS18B20==1)
			else if(data_buffer[0]=='d' &&data_buffer[1]=='s'&&data_buffer[2]=='1'&&data_buffer[3]=='8')
			{
				ds18b20_init(DS_GPIO);
				printf("Temperature: %0.2f\n",ds18b20_get_temp());
	//			data_buffer[0]='T';data_buffer[1]='e';data_buffer[2]='m';data_buffer[3]=':';
				xbuffer[13]=(((int)ds18b20_get_temp())/10)+'0';
				xbuffer[14]=(((int)ds18b20_get_temp())%10)+'0';
				xbuffer[16]=((int)(ds18b20_get_temp()*10)%10)+'0';
				xbuffer[17]=((int)(ds18b20_get_temp()*100)%10)+'0';
				xbuffer[18]='\n';
				int err=send(g_iSock_fd, xbuffer, sizeof(xbuffer), 0);

//				ds18b20_init(DS_PIN);
//				int temp=ds18b20ReadTemp();//ds18b20ReadTemp
//				printf("--Temperature:%d.%d℃\r\n",temp/100,temp%100);

			}
#endif
#if(ESPADC==1)
			else if(data_buffer[0]=='a' &&data_buffer[1]=='d'&&data_buffer[2]=='c'&&data_buffer[3]=='0')
			{
			    // initialize ADC
			    adc1_config_width(ADC_WIDTH_12Bit);
			    adc1_config_channel_atten(ADC1_TEST_CHANNEL,ADC_ATTEN_11db);


			}

#endif

#if(ESPWS2812==1)
			else if(data_buffer[0]=='r' &&data_buffer[1]=='g'&&data_buffer[2]=='b')
			{

				rgbVal color=makeRGBVal(255, 255, 255);
				switch(data_buffer[3])
				{

					case '0':
						color=makeRGBVal(255, 255, 255);
						pixels[0] = color;

						color=makeRGBVal(255, 255, 255);
						pixels[1] = color;

						color=makeRGBVal(255, 255, 255);
						pixels[2] = color;
						ws2812_setColors(pixel_count,  pixels);
					break;
					case '1'://red
					    color=makeRGBVal(255, 0, 0);
					    pixels[0] = color;

					    color=makeRGBVal(255, 0, 0);
					    pixels[1] = color;

					    color=makeRGBVal(255, 0, 0);
					    pixels[2] = color;
					    ws2812_setColors(pixel_count,  pixels);
					break;
					case '2'://green
				    color=makeRGBVal(0, 255, 0);
				    pixels[0] = color;

				    color=makeRGBVal(0, 255, 0);
				    pixels[1] = color;

				    color=makeRGBVal(0, 255, 0);
				    pixels[2] = color;
				    ws2812_setColors(pixel_count,  pixels);
					break;
					case '3'://blue
				    color=makeRGBVal(0, 0, 255);
				    pixels[0] = color;

				    color=makeRGBVal(0, 0, 255);
				    pixels[1] = color;

				    color=makeRGBVal(0, 0, 255);
				    pixels[2] = color;
				    ws2812_setColors(pixel_count,  pixels);
					break;
					case '4':
				    color=makeRGBVal(255, 20, 147);//DeepPink
				    pixels[0] = color;
				    pixels[1] = color;
				    pixels[2] = color;

				    ws2812_setColors(pixel_count,  pixels);
					break;
					case '5':
				    color=makeRGBVal(160, 32, 240);//Purple
				    pixels[0] = color;
				    pixels[1] = color;
				    pixels[2] = color;

				    ws2812_setColors(pixel_count,  pixels);
					break;
					case '6':
				    color=makeRGBVal(124,252, 0);//LawnGreen
				    pixels[0] = color;
				    pixels[1] = color;
				    pixels[2] = color;

				    ws2812_setColors(pixel_count,  pixels);
					break;
					case '7':
				    color=makeRGBVal(192,62, 255);//DarkOrchid1
				    pixels[0] = color;
				    pixels[1] = color;
				    pixels[2] = color;

				    ws2812_setColors(pixel_count,  pixels);
					break;
					case '8':
				    color=makeRGBVal(28,28, 28);//grey11
				    pixels[0] = color;
				    pixels[1] = color;
				    pixels[2] = color;

				    ws2812_setColors(pixel_count,  pixels);
					break;
					case '9':
				    color=makeRGBVal(224,255, 255);//LightCyan1
				    pixels[0] = color;
				    pixels[1] = color;
				    pixels[2] = color;

				    ws2812_setColors(pixel_count,  pixels);
					break;
					case 'a':
				    color=makeRGBVal(123, 234, 56);
				    pixels[0] = color;
				    pixels[1] = color;
				    pixels[2] = color;
				    ws2812_setColors(pixel_count,  pixels);
					break;

					default:
					    color=makeRGBVal(0, 0, 0);
					    pixels[0] = color;
					    pixels[1] = color;
					    pixels[2] = color;
					    ws2812_setColors(pixel_count,  pixels);
					break;
				}
			}
#endif
#endif
		memset(data_buffer,0,length);
		}
        if(length <=0)
        {
        	close(g_iSock_fd);
        	goto SOCKBEGIN;
        }

	}
}

#if 0
#define EX_UART_NUM UART_NUM_0

#define BUF_SIZE (500)
static QueueHandle_t uart0_queue;

static void uart_task(void *pvParameters)
{
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    do {
        int len = uart_read_bytes(EX_UART_NUM, data, BUF_SIZE, 100 / portTICK_RATE_MS);
        if(len > 0) {
 //           ESP_LOGI(TAG, "uart read : %d", len);
 //           uart_write_bytes(EX_UART_NUM, (const char*)data, len);
        	send(g_iSock_fd, (const char*)data,len, 0);
        }
    } while(1);
}


static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.
                in this example, we don't process data in event, but read data outside.*/
                case UART_DATA:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    ESP_LOGI(TAG, "data, len: %d; buffered len: %d", event.size, buffered_size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow\n");
                    //If fifo overflow happened, you should consider adding flow control for your application.
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(EX_UART_NUM);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full\n");
                    //If buffer full happened, you should consider encreasing your buffer size
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(EX_UART_NUM);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break\n");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error\n");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error\n");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGI(TAG, "uart pattern detected\n");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d\n", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void uart0_init(void)
{
    uart_config_t uart_config = {
       .baud_rate = 115200,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .rx_flow_ctrl_thresh = 122,
    };
    //Set UART parameters
    uart_param_config(EX_UART_NUM, &uart_config);
    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart0_queue, 0);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_intr(EX_UART_NUM, '+', 3, 10000, 10, 10);
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    //process data
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

}

#endif






void app_main()
{
	size_t free8start=0, free32start=0, free8=0, free32=0;
//	heap_mem_log();
    ESP_LOGI(TAG,"get free size of 32BIT heap : %d\n",xPortGetFreeHeapSizeCaps(MALLOC_CAP_32BIT));


#if ESPIDFV21RC
    free8=xPortGetFreeHeapSizeCaps(MALLOC_CAP_8BIT);
    free32=xPortGetFreeHeapSizeCaps(MALLOC_CAP_32BIT);
    free8start=xPortGetMinimumEverFreeHeapSizeCaps(MALLOC_CAP_8BIT);
    free32start=xPortGetMinimumEverFreeHeapSizeCaps(MALLOC_CAP_32BIT);
#else
    free32=heap_caps_get_largest_free_block(MALLOC_CAP_32BIT);
    free8=heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    free8start=heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    free32start=heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT);
#endif


    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());//Free heap: 273368b
    ESP_LOGI(TAG, "Free (largest free blocks) 8bit-capable memory : %db, 32-bit capable memory %db\n", free8, free32);
    //ree (largest free blocks) 8bit-capable memory : 144272b, 32-bit capable memory 144272b
    ESP_LOGI(TAG, "Free (min free size) 8bit-capable memory : %db, 32-bit capable memory %db\n", free8start, free32start);
//    Free (min free size) 8bit-capable memory : 272396b, 32-bit capable memory 338868b

 //   currFbPtr = heap_caps_malloc(160*120*2, MALLOC_CAP_32BIT);
 //   currFbPtr = (volatile uint32_t)heap_caps_malloc(160*120*2, MALLOC_CAP_32BIT);//NULL fail why? 320*240*2=153600>144272

    if(CAMERA_FRAME_SIZE==CAMERA_FS_QQVGA)
    	currFbPtr = (volatile uint32_t)pvPortMallocCaps(160*120*2, MALLOC_CAP_32BIT);
    else if(CAMERA_FRAME_SIZE==CAMERA_FS_QVGA)
    	currFbPtr = (volatile uint32_t)pvPortMallocCaps(320*240*2, MALLOC_CAP_32BIT);

    vTaskDelay(1000 / portTICK_RATE_MS);

//    if (currFbPtr == NULL) {
//        ESP_LOGE(TAG, "Not enough memory to allocate 1");
//        return;
//   }

    ESP_LOGI(TAG,"%s=%p\n",currFbPtr == NULL ? "------currFbPtr is NULL------" : "==========currFbPtr not NULL======",currFbPtr );

    ESP_LOGI(TAG,"get free size of 32BIT heap : %d\n",xPortGetFreeHeapSizeCaps(MALLOC_CAP_32BIT));

    ESP_LOGI(TAG,"Starting nvs_flash_init ...");
    nvs_flash_init();

    vTaskDelay(3000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());

    ESP_LOGI(TAG, "Wifi Initialized...");
    init_wifi_sta();
    //init_wifi_ap();


    ESP_LOGI(TAG,"get free size of 32BIT heap : %d\n",xPortGetFreeHeapSizeCaps(MALLOC_CAP_32BIT));

    // VERY UNSTABLE without this delay after init'ing wifi...
    // however, much more stable with a new Power Supply
    vTaskDelay(5000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());

    // camera init
    esp_err_t err = camera_probe(&config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        //return;
    }
    if (camera_model == CAMERA_OV7725) {
        ESP_LOGI(TAG, "Detected OV7725 camera, using grayscale bitmap format");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        config.frame_size = CAMERA_FRAME_SIZE;
    } else if (camera_model == CAMERA_OV7670) {
        ESP_LOGI(TAG, "Detected OV7670 camera");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        config.frame_size = CAMERA_FRAME_SIZE;
    } else if (camera_model == CAMERA_OV2640) {
        ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
        s_pixel_format = CAMERA_PF_JPEG;
        config.frame_size = CAMERA_FS_VGA;
        config.jpeg_quality = 15;
    } else {
        ESP_LOGE(TAG, "Camera not supported");
        //return;
    }

#if ESPIDFV21RC
    free8=xPortGetFreeHeapSizeCaps(MALLOC_CAP_8BIT);
    free32=xPortGetFreeHeapSizeCaps(MALLOC_CAP_32BIT);
    free8start=xPortGetMinimumEverFreeHeapSizeCaps(MALLOC_CAP_8BIT);
    free32start=xPortGetMinimumEverFreeHeapSizeCaps(MALLOC_CAP_32BIT);
#else
    free32=heap_caps_get_largest_free_block(MALLOC_CAP_32BIT);
    free8=heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    free8start=heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    free32start=heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT);
#endif

    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Free (largest free blocks) 8bit-capable memory : %d, 32-bit capable memory %d\n", free8, free32);
    ESP_LOGI(TAG, "Free (min free size) 8bit-capable memory : %d, 32-bit capable memory %d\n", free8start, free32start);


    espilicam_event_group = xEventGroupCreate();
    config.displayBuffer = currFbPtr;
    config.pixel_format = s_pixel_format;

    err = camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        //return;
    }

    vTaskDelay(2000 / portTICK_RATE_MS);

    ESP_LOGD(TAG, "Starting http_server task...");
    // keep an eye on stack... 5784 min with 8048 stck size last count..
    xTaskCreatePinnedToCore(&http_server, "http_server", 4096, NULL, 6, NULL,1);

    ESP_LOGI(TAG, "open http://" IPSTR "/bmp for single image/bitmap image", IP2STR(&s_ip_addr));
    ESP_LOGI(TAG, "open http://" IPSTR "/stream for multipart/x-mixed-replace stream of bitmaps", IP2STR(&s_ip_addr));
    ESP_LOGI(TAG, "open http://" IPSTR "/get for raw image as stored in framebuffer ", IP2STR(&s_ip_addr));

#if ESPIDFV21RC
    free8=xPortGetFreeHeapSizeCaps(MALLOC_CAP_8BIT);
    free32=xPortGetFreeHeapSizeCaps(MALLOC_CAP_32BIT);
    free8start=xPortGetMinimumEverFreeHeapSizeCaps(MALLOC_CAP_8BIT);
    free32start=xPortGetMinimumEverFreeHeapSizeCaps(MALLOC_CAP_32BIT);
#else
    free32=heap_caps_get_largest_free_block(MALLOC_CAP_32BIT);
    free8=heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    free8start=heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    free32start=heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT);
#endif

	xTaskCreate(&tcp_cli_task, "tcp_cli_task", 4096, NULL, 10, NULL);//TCP Client task

	xTaskCreate(&tcp_send_task, "tcp_send_task", 2048, NULL, 8, NULL);//TCP Client Send Task

   //    uart0_init();
//    xTaskCreatePinnedToCore(&uart_task, "uart_task", 512, NULL, 7, NULL,1);

#if(ESPWS2812==1)
 //   ws2812_init(WS2812_PIN);
//    xTaskCreate(rainbow, "ws2812 rainbow demo", 4096, NULL, 10, NULL);

#endif


    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Free (largest free blocks) 8bit-capable memory : %d, 32-bit capable memory %d\n", free8, free32);
    ESP_LOGI(TAG, "Free (min free size) 8bit-capable memory : %d, 32-bit capable memory %d\n", free8start, free32start);

    ESP_LOGI(TAG,"get free size of 32BIT heap : %d\n",xPortGetFreeHeapSizeCaps(MALLOC_CAP_32BIT));
//  ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
    ESP_LOGI(TAG, "Camera demo ready.");


#if ESPI2SOUT
    //for 36Khz sample rates, we create 100Hz sine wave, every cycle need 36000/100 = 360 samples (4-bytes or 8-bytes each sample)
     //depend on bits_per_sample
     //using 6 buffers, we need 60-samples per buffer
     //if 2-channels, 16-bit each channel, total buffer is 360*4 = 1440 bytes
     //if 2-channels, 24/32-bit each channel, total buffer is 360*8 = 2880 bytes
     i2s_config_t i2s_config = {
#if I2S_DAC_BUILT_IN
         .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,     // Only TX
#else
		 .mode = I2S_MODE_MASTER | I2S_MODE_TX,
#endif
         .sample_rate = SAMPLE_RATE,
         .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,//16,    /*if using DAC, the DAC module will only take the 8bits from MSB */
         .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,        //2-channels
         .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB,
         .dma_buf_count = 6,
         .dma_buf_len = 60,                                                     //
         .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
     };
     i2s_driver_install((i2s_port_t)I2S_NUM, &i2s_config, 0, NULL);

#if I2S_DAC_BUILT_IN
     i2s_set_pin((i2s_port_t)I2S_NUM, NULL);
#else
     i2s_pin_config_t pin_config = {
         .bck_io_num = 26,
         .ws_io_num = 25,//lrck
         .data_out_num = 22,//I2S_DATA_OUT
         .data_in_num = -1     //Not used   //I2S_DATA_IN
     };
     i2s_set_pin((i2s_port_t)I2S_NUM, &pin_config);
#endif

     int test_bits = 16;

#endif
#if I2S_DAC_BUILT_IN
     dac_output_enable(DAC_CHANNEL_1);// use DAC_CHANNEL_1 (pin 25 fixed)
     dac_output_enable(DAC_CHANNEL_2);// use DAC_CHANNEL_1 (pin 26 fixed)
     dac_output_voltage(DAC_CHANNEL_1, 127);
     dac_output_voltage(DAC_CHANNEL_2, 127);
#endif
     while (1)
     {
#if ESPI2SOUT
         setup_triangle_sine_waves(test_bits);


    //	 i2s_set_clk(I2S_NUM, SAMPLE_RATE, test_bits, 2);
     //    i2s_write_bytes(I2S_NUM, (const char *)pacman,  sizeof(pacman), 1000000/SAMPLE_RATE);
     //    delay(5000);
         //vTaskDelay(5000/portTICK_RATE_MS);
//         test_bits += 8;
//         if(test_bits > 32)
//             test_bits = 16;

#if I2S_DAC_BUILT_IN
            //play(pacman , sizeof(pacman));
            //delay(2000);

           // play(pacmanDeath , sizeof(pacmanDeath));
           // delay(2000);
//            play( music1 , sizeof( music1));
 //           delay(2000);

            play(music1 , sizeof( music1));
            delay(2000);
#endif
#endif
//===========================================



     }


}
