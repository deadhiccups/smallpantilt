/*********
Pan/Tilt system that works with ESP32 Camera and a M5 Servo Controller to control 2 servos  
connects to wifi - starts website where you can control the servos and laser

VCC = 5 volts
SDA = Data Pin  = 20 
SCL = Clock Pin = 21
GND = Ground Pin
*********/
/**
  
#define TARGET_LASER 47

*********/  
#include "esp_camera.h"
#include "SD_MMC.h"
#include "WiFi.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems
#include "esp_http_server.h"
#include <Wire.h>
#include "M5_UNIT_8SERVO.h"

// Replace with your network credentials
//const char* ssid = "username";
//const char* password = "password";
// Replace with your WAP credentials
const char* ssid = "ESP32-PANTILT";
const char* password = "password";

M5_UNIT_8SERVO unit_8servo;

#define PART_BOUNDARY "123456789000000000000987654321"
uint8_t servo1 = 6;
uint8_t servo2 = 7;
#define TARGET_LASER 47

//#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM_B
#define CAMERA_MODEL_FREENOVE_ESP32S3_CAM
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP32S3_EYE
//#define CAMERA_MODEL_M5STACK_CAMS3_UNIT


#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM_B)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     22
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21                         

#elif defined(CAMERA_MODEL_M5STACK_CAMS3_UNIT)
  #define CAM_BOARD "CAMERA_MODEL_M5STACK_CAMS3_UNIT"
  #define PWDN_GPIO_NUM  -1
  #define RESET_GPIO_NUM 21
  #define XCLK_GPIO_NUM  11
  #define SIOD_GPIO_NUM  17
  #define SIOC_GPIO_NUM  41

  #define Y9_GPIO_NUM    13
  #define Y8_GPIO_NUM    4
  #define Y7_GPIO_NUM    10
  #define Y6_GPIO_NUM    5
  #define Y5_GPIO_NUM    7
  #define Y4_GPIO_NUM    16
  #define Y3_GPIO_NUM    15
  #define Y2_GPIO_NUM    6
  #define VSYNC_GPIO_NUM 42
  #define HREF_GPIO_NUM  18
  #define PCLK_GPIO_NUM  12

#define LED_GPIO_NUM 14
#elif defined(CAMERA_MODEL_ESP32S3_EYE) || defined(CAMERA_MODEL_FREENOVE_ESP32S3_CAM)
  #define CAM_BOARD "CAMERA_MODEL_ESP32S3_EYE_FREENOVE"
  #define PWDN_GPIO_NUM -1
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM 15
  #define SIOD_GPIO_NUM 4
  #define SIOC_GPIO_NUM 5

  #define Y2_GPIO_NUM 11
  #define Y3_GPIO_NUM 9
  #define Y4_GPIO_NUM 8
  #define Y5_GPIO_NUM 10
  #define Y6_GPIO_NUM 12
  #define Y7_GPIO_NUM 18
  #define Y8_GPIO_NUM 17
  #define Y9_GPIO_NUM 16

  #define VSYNC_GPIO_NUM 6
  #define HREF_GPIO_NUM 7
  #define PCLK_GPIO_NUM 13

  #if defined(CAMERA_MODEL_FREENOVE_ESP32S3_CAM)
  #define USE_WS2812 // Use WS2812 rgb led
  #endif
  #ifdef USE_WS2812 
  #define LED_GPIO_NUM 48 // WS2812 rgb led
  #else
  #define LED_GPIO_NUM 2 // blue signal led    
  #endif
// Define SD Pins
  #define SD_MMC_CLK 39 
  #define SD_MMC_CMD 38
  #define SD_MMC_D0 40
// Define I2C Pins
  #define I2C_SDA 20
  #define I2C_SCL 21
  #if defined(CAMERA_MODEL_ESP32S3_EYE)
// Define Mic Pins
  #define I2S_SD 2  // I2S Microphone
  #define I2S_WS 42
  #define I2S_SCK 41
  #endif
#else

  #error "Camera model not selected"
#endif

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
  <head>
    <title>Pan Tilt</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; background-color: #000000; color: green; margin:0px auto; padding-top: 30px;}
      table { margin-left: auto; border: 2px solid green; margin-right: auto; }
      td { padding: 8 px; }
      .button {
        background-color: #000000;
        border: 2px solid green;
        color: green;
        padding: 5px 10px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 20px;
        margin: 3px 2px;
        cursor: pointer;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
        -webkit-tap-highlight-color: rgba(0,0,0,0);
      }
      img {  width: auto ;
        max-width: 50% ;
        height: auto ; 
      }
      .row{
        display: flex;
        justify-content: center;
        align-items; center;
      }
      .column :{
        flex: 50%;
        padding; 5px
      }
    </style>
  </head>
  <body>
    <h1>Pan Tilt</h1>
    <p>&nbsp</p>
    <p>&nbsp</p>
    <img src="" id="photo">
    <p>&nbsp</p>
    <p>&nbsp</p>
    <table>
      <tr><td colspan="3" align="center"><button class="button" onmousedown="toggleCheckbox('up');" ontouchstart="toggleCheckbox('up');">Up</button></td></tr>
      <tr><td align="center"><button class="button" onmousedown="toggleCheckbox('left');" ontouchstart="toggleCheckbox('left');">Left</button></td><td align="center"></td><td align="center"><button class="button" onmousedown="toggleCheckbox('right');" ontouchstart="toggleCheckbox('right');">Right</button></td></tr>
      <tr><td colspan="3" align="center"><button class="button" onmousedown="toggleCheckbox('down');" ontouchstart="toggleCheckbox('down');">Down</button></td></tr>
      <tr><td colspan="3" align="center"><button class="button" onmousedown="toggleCheckbox('light');" ontouchstart="toggleCheckbox('light');">Laser</button></td></tr>
    </table>
  <script>
   function toggleCheckbox(x) {
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=" + x, true);
     xhr.send();
   }
   window.onload = document.getElementById("photo").src = window.location.href.slice(0, -1) + ":81/stream";
  </script>
  </body>
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

static esp_err_t cmd_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
  
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if(!buf){
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  sensor_t * s = esp_camera_sensor_get();
  //flip the camera vertically
  //s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  // mirror effect
  //s->set_hmirror(s, 1);          // 0 = disable , 1 = enable

  int res = 0;
//stops at 1500, less than 1500 rotates clockwise, and greater 
//than 1500 rotates counterclockwise. The closer the number is to 1500, the slower it rotates.
//
  if(!strcmp(variable, "up")) {
    Serial.println("Up");
    unit_8servo.setServoPulse(servo1, 1350);
    delay(100);
    unit_8servo.setServoPulse(servo1, 1500);
  }
  else if(!strcmp(variable, "left")) {
    Serial.println("Left");
    unit_8servo.setServoPulse(servo2, 1650);
    delay(100);
    unit_8servo.setServoPulse(servo2, 1500);
  }
  else if(!strcmp(variable, "right")) {
    Serial.println("Right");
    unit_8servo.setServoPulse(servo2, 1350);
    delay(100);
    unit_8servo.setServoPulse(servo2, 1500);
  }
  else if(!strcmp(variable, "down")) {
    Serial.println("Down");
    unit_8servo.setServoPulse(servo1, 1650);
    delay(100);
    unit_8servo.setServoPulse(servo1, 1500);

  }
    else if(!strcmp(variable, "stopservo1")) {
    Serial.println("stopservo1");
    unit_8servo.setServoPulse(servo1, 1500);

  }
    else if(!strcmp(variable, "stopservo2")) {
    Serial.println("stopservo2");
    unit_8servo.setServoPulse(servo2, 1500);

  }
    else if(!strcmp(variable, "light")) {
      digitalWrite(TARGET_LASER, HIGH); //Turn on
      delay (1000); //Wait 2 sec    
      digitalWrite(TARGET_LASER, LOW);
      delay (500); //Wait 1 sec
      Serial.println("laser");
  }
  else {
    res = -1;
  }

  if(res){
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri       = "/action",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
  };
  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
  }
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup() {
     
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println("setting up"); 
  pinMode (TARGET_LASER, OUTPUT);
  while (!unit_8servo.begin(&Wire, 20, 21, M5_UNIT_8SERVO_DEFAULT_ADDR)) {
      Serial.println("extio Connect Error");
        delay(100);
  }
  // unit_8servo.setAllPinMode(DIGITAL_INPUT_MODE);
  // unit_8servo.setAllPinMode(DIGITAL_OUTPUT_MODE);
  // unit_8servo.setAllPinMode(ADC_INPUT_MODE);
  unit_8servo.setAllPinMode(SERVO_CTL_MODE);
  // unit_8servo.setAllPinMode(RGB_LED_MODE);
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  // init with high specs to pre-allocate larger buffers
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 20;
  config.fb_count = 4;

  if(psramFound()){
    Serial.println("psram");
    config.jpeg_quality = 20;
    config.fb_count = 4;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  } else {
    // Limit the frame size when PSRAM is not available
    Serial.println("no psram");
    config.frame_size = FRAMESIZE_VGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  s->set_vflip(s, 0); // flip it back
  s->set_brightness(s, 1); // up the brightness just a bit
  s->set_saturation(s, 0); // lower the saturation
 /* 
 /* 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  while (WiFi.STA.hasIP() != true) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(WiFi.localIP());
  */
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  startCameraServer();
}

void loop() {
  
}
