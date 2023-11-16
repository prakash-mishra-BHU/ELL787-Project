/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <demo_4_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h
// ##################### Line, Wi-Fi settings (Preferences) #####################
String clientId         = "110779100661-hvit1dv77p35aqjbjgt1vinfvd0g9htu.apps.googleusercontent.com"; // $$$ CHANGE REQUIRED $$$
String clientSecret     = "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";                        // $$$ CHANGE REQUIRED $$$
String refreshToken     = "1//0gqHDXQkszUkICgYIARAAGBASNwF-L9Ir3sGj19o2vG606OhSyE_OzF2OWIFWBf7ujmkomPsMtbp-bIazZn2FebFf84Q-7Yzcv0M";                        // $$$ CHANGE REQUIRED $$$
String driveFolder      = "1FHZ6_E7jh41om1s1EvcGYy8XBTTknZ-0";                        // $$$ CHANGE REQUIRED $$$

const char *ssid        = "OPPOF19Pro"; // $$$ CHANGE REQUIRED $$$
const char *password    = "12345678"; // $$$ CHANGE REQUIRED $$$

const int   Interval    = 20;                 // Image save interval (minutes) [OFF:-1]
const int   SavaTime    = 15;                 // Image save time (0 to 24 hours exactly) [OFF:-1]
// ###################################################################
const char* refreshServer = "oauth2.googleapis.com";
const char* refreshUri    = "/token";
const char* apiServer     = "www.googleapis.com";
const char* apiUri        = "/upload/drive/v3/files?uploadType=multipart";
String accessToken        = "";

int preMin  = -1;     // last run time(for Interval)
int preHour = -1;     // last run time(for Time)

//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

// LED Pin Setting
const byte LED_PIN      = 2;  // Green LED
#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
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

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 10000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
// Global Values
WiFiClientSecure httpsClient;
bool ledFlag          = true; // LED Control Flag
int waitingTime      = 30000; // Wait 30 seconds to google response.
camera_fb_t * fb;
/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    //comment out the below line to start inference immediately after upload
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
      // ####### LED Pin setting #######
  pinMode ( LED_PIN, OUTPUT );

  // ####### Wireless Wi-Fi connection #######
  WiFi.begin ( ssid, password );
  while ( WiFi.status() != WL_CONNECTED ) { // infinite loop until connected
    // LED flashes every second while connected
    ledFlag = !ledFlag;
    digitalWrite(LED_PIN, ledFlag);
    delay ( 1000 );
    Serial.print ( "." );
  }
  // Wi-Fi connection completed (IP address display)
  Serial.print ( "Wi-Fi Connected! IP address: " );
  Serial.println ( WiFi.localIP() );
  Serial.println ( );
  // LED lights when Wi-Fi is connected (Wi-Fi connection status)
  digitalWrite ( LED_PIN, true );

  // ####### HTTPS certificate check setting #######
  // Skip Server certificate check (required since 1.0.5)
  httpsClient.setInsecure();//skip verification
  //httpsClient.setCACert(rootCA);// It is also possible to obtain a root certificate in advance using a web browser and set rootCA

  // ####### NTP setting #######
  configTime(9 * 3600L, 0, "ntp.nict.jp", "time.google.com", "ntp.jst.mfeed.ad.jp");

  Serial.println("Setup Completed!!!");
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    bool doFlag = false;
    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }
        doFlag= false;
        if (bb.value > 0.9){
          doFlag= true;
          }
        // Do Save Image to Google Drive
  if ( doFlag ) {
    // ####### Get JPEG picture #######
    Serial.println("Start get JPG");
    getCameraJPEG();
    // ####### get Access Token #######
    Serial.println("Start get AccessToken");
    getAccessToken();
    // ####### Save JPEG to GoogleDrive #######
    Serial.println("Start Post GoogleDrive");
    postGoogleDriveByAPI();
  }
        ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
    if (!bb_found) {
        ei_printf("    No objects found\n");
    }
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label,
                                    result.classification[ix].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif


    free(snapshot_buf);
  
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }


    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so 
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif

// Send JPEG by Http POST
void postGoogleDriveByAPI() {

  Serial.println("Connect to " + String(apiServer));
  if (httpsClient.connect(apiServer, 443)) {
    Serial.println("Connection successful");

    // Get Time for save file name
    struct tm timeInfo;
    char preFilename[16];
    getLocalTime(&timeInfo);
    sprintf(preFilename, "%04d%02d%02d_%02d%02d%02d",
          timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
          timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
    String saveFilename = "M5TimerCam_" + String(preFilename) + ".jpg";

    String metadata = "--foo_bar_baz\r\n"
                      "Content-Type: application/json; charset=UTF-8\r\n\r\n"
                      "{\"name\":\"" + saveFilename + "\",\"parents\":[\"" + driveFolder + "\"]}\r\n\r\n"; // parents:save folder
                      //"{\"name\":\"" + saveFilename + "\"}\r\n\r\n"; // parerents is Optional
    String startBoundry = "--foo_bar_baz\r\n"
                          "Content-Type:image/jpeg\r\n\r\n";
    String endBoundry   = "\r\n--foo_bar_baz--";

    unsigned long contentsLength = metadata.length() + startBoundry.length() + fb->len + endBoundry.length();
    String header = "POST " + String(apiUri) + " HTTP/1.1\r\n" +
                    "HOST: " + String(apiServer) + "\r\n" +
                    "Connection: close\r\n" +
                    "content-type: multipart/related; boundary=foo_bar_baz\r\n" +
                    "content-length: " + String(contentsLength) + "\r\n" +
                    "authorization: Bearer " + accessToken + "\r\n\r\n";

    Serial.println("Send JPEG DATA by API");
    httpsClient.print(header);
    httpsClient.print(metadata);
    httpsClient.print(startBoundry);
    // JPEG data is separated into 1000 bytes and POST
    unsigned long dataLength = fb->len;
    uint8_t*      bufAddr    = fb->buf;
    for(unsigned long i = 0; i < dataLength ;i=i+1000) {
      if ( (i + 1000) < dataLength ) {
        httpsClient.write(( bufAddr + i ), 1000);
      } else if (dataLength%1000 != 0) {
        httpsClient.write(( bufAddr + i ), dataLength%1000);
      }
    }
    httpsClient.print(endBoundry);

    Serial.println("Waiting for response.");
    long int StartTime=millis();
    while (!httpsClient.available()) {
      Serial.print(".");
      delay(100);
      if ((StartTime+waitingTime) < millis()) {
        Serial.println();
        Serial.println("No response.");
        break;
      }
    }
    Serial.println();
    while (httpsClient.available()) {
      Serial.print(char(httpsClient.read()));
    }
    /*Serial.println("Recieving Reply");
    while (httpsClient.connected()) {
      String retLine = httpsClient.readStringUntil('\n');
      //Serial.println("retLine:" + retLine);
      int okStartPos = retLine.indexOf("200 OK");
      if (okStartPos >= 0) {
        Serial.println("200 OK");
        break;
      }
    }*/
  } else {
    Serial.println("Connected to " + String(refreshServer) + " failed.");
  }
  httpsClient.stop();
}

// Update access token with refresh token (access token valid time is 1 hour)
void getAccessToken() {
  accessToken = "None";
  // ####### Get Access Token #######
  Serial.println("Connect to " + String(refreshServer));
  if (httpsClient.connect(refreshServer, 443)) {
    Serial.println("Connection successful");

    String body = "client_id="     + clientId     + "&" +
                  "client_secret=" + clientSecret + "&" +
                  "refresh_token=" + refreshToken + "&" +
                  "grant_type=refresh_token";

    // Send Header
    httpsClient.println("POST "  + String(refreshUri) + " HTTP/1.1");
    httpsClient.println("Host: " + String(refreshServer));
    httpsClient.println("content-length: " + (String)body.length());
    httpsClient.println("Content-Type: application/x-www-form-urlencoded");
    httpsClient.println();
    // Send Body
    httpsClient.println(body);

    Serial.println("Recieving Token");
    while (httpsClient.connected()) {
      String retLine = httpsClient.readStringUntil('\n');
      //Serial.println("retLine:" + retLine);
      int tokenStartPos = retLine.indexOf("access_token");
      if (tokenStartPos >= 0) {
        tokenStartPos     = retLine.indexOf("\"", tokenStartPos) + 1;
        tokenStartPos     = retLine.indexOf("\"", tokenStartPos) + 1;
        int tokenEndPos   = retLine.indexOf("\"", tokenStartPos);
        accessToken       = retLine.substring(tokenStartPos, tokenEndPos); 
        Serial.println("AccessToken:"+accessToken);
        break;
      }
    }
  } else {
    Serial.println("Connected to " + String(refreshServer) + " failed.");
  }
  httpsClient.stop();
  if (accessToken == "None") {
    Serial.println("Get AccessToken Failed. Restart ESP32!");
    delay(3000);
    ESP.restart();
  }
}

// Get JPEG image with OV3660
void getCameraJPEG(){
  fb = esp_camera_fb_get();  // Get JPEG image
  if (!fb) {
    Serial.printf("Camera capture failed");
  }
  Serial.printf("JPG: %uB ", (uint32_t)(fb->len));
  Serial.println();
  // Shooting end processing
  esp_camera_fb_return(fb);
}
