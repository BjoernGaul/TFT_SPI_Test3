//Mac Adresse Remote: 30:C9:22:FF:71:F4

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Arduino.h>
#include <lvgl.h>
#include <ui.h>
#include <funktions.h>


/*Don't forget to set Sketchbook location in File/Preferences to the path of your UI project (the parent foder of this INO file)*/

//Set REPEAT_CAL to false to stop calibrating again
#define CALIBRATION_FILE "/calibrationData"
#define REPEAT_CAL false

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

void* lv_mem_pool;
#define LV_BUF_SIZE (screenWidth * screenHeight / 10)  // Define the size of the screen buffer

void* buf1;
void* buf2;


TFT_eSPI tft = TFT_eSPI(screenHeight, screenWidth); /* TFT instance */

//Joystick

#define xPinA D3  // xPin Joystick A
#define yPinA D5  // yPin Joystick A
#define yPinB D6  // yPin Joystick B
#define xPinB D7  // xPin Joystick B

uint16_t lastxA = 0;
uint16_t lastyA = 0;
uint16_t lastxB = 0;
uint16_t lastyB = 0;
uint8_t dirB = 0;
uint8_t dirA = 0;
unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 100; // Interval in milliseconds

//Variables
uint16_t posFLS = 0;
uint16_t posFLT = 0;
uint16_t posFLB = 0;
uint16_t posFRS = 0;
uint16_t posFRT = 0;
uint16_t posFRB = 0;
uint16_t posBRS = 0;
uint16_t posBRT = 0;
uint16_t posBRB = 0;
uint16_t posBLS = 0;
uint16_t posBLT = 0;
uint16_t posBLB = 0;
uint16_t lastPosCheck = 0;
static const uint16_t isStanding = 69;

//Variables ESP-Now
const uint8_t sit = 0;
const uint8_t stand = 1;
const uint8_t crab = 2;
const uint8_t FLS = 10;
const uint8_t FLT = 11;
const uint8_t FLB = 12;
const uint8_t FRS = 20;
const uint8_t FRT = 21;
const uint8_t FRB = 22;
const uint8_t BRS = 30;
const uint8_t BRT = 31;
const uint8_t BRB = 32;
const uint8_t BLS = 40;
const uint8_t BLT = 41;
const uint8_t BLB = 42;
uint16_t dataSend[2] = {0, 0};


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX = 0, touchY = 0;

    if( !tft.getTouch( &touchX, &touchY, 600 ) )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;

        Serial.print( "Data x " );
        Serial.println( touchX );

        Serial.print( "Data y " );
        Serial.println( touchY );
        Serial.printf("posFLS: %d\n", posFLS);
        Serial.printf("posFLT: %d\n", posFLT);
        Serial.printf("posFLB: %d\n", posFLB);
    }
}


//Funktions
void touch_calibrate();

//ESP-NOW

int answer = 0;

const uint8_t remoteMac[6] = {0x30, 0xC9, 0x22, 0xEC, 0xB9, 0x80}; //30:C9:22:FF:71:F4
void readMacAdress();
void onDataReceive(const uint8_t * mac, const uint8_t * data, int len) {
  Serial.print("Received data from: ");
  int receivedNumber;
  memcpy(&receivedNumber, data, sizeof(receivedNumber));
  Serial.println(receivedNumber);
  if (receivedNumber == 3) {
    esp_err_t result = esp_now_send(remoteMac, (uint8_t *) &answer, sizeof(answer));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
  }
  if (receivedNumber == 1) {
    answer = 1;
  }else{
    answer = 0;
  }
}




void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */

    // Check if PSRAM is available
    if (!psramFound()) {
        Serial.println("PSRAM not found");
        while (true); // Halt execution if PSRAM is not found
    }

    // Allocate memory from PSRAM
    lv_mem_pool = ps_malloc(LV_MEM_SIZE);
    if (lv_mem_pool == NULL) {
        Serial.println("Failed to allocate memory from PSRAM");
        while (true); // Halt execution if allocation fails
    } else {
        Serial.println("Memory allocated from PSRAM");
    }

    // Allocate screen buffers from PSRAM
    buf1 = ps_malloc(LV_BUF_SIZE * sizeof(lv_color_t));
    buf2 = ps_malloc(LV_BUF_SIZE * sizeof(lv_color_t));
    if (buf1 == NULL || buf2 == NULL) {
        Serial.println("Failed to allocate screen buffers from PSRAM");
        while (true); // Halt execution if allocation fails
    } else {
        Serial.println("Screen buffers allocated from PSRAM");
    }


    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

  tft.begin();          /* TFT init */
  tft.setRotation( 1 ); /* Landscape orientation, flipped */

  touch_calibrate();
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(20, 0);
  tft.setTextFont(2);
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("Starting...");

  /*Initialize the display buffer*/
  static lv_disp_draw_buf_t draw_buf;
  lv_disp_draw_buf_init( &draw_buf, buf1, buf2, screenWidth * screenHeight / 10 );

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register( &indev_drv );


  ui_init();

  Serial.println( " LVGL Setup done, starting ESP-Now" );

  //ESP-NOW Setup
  //readMacAdress();
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onDataReceive);
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, remoteMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_err_t addPeerResult = esp_now_add_peer(&peerInfo);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    Serial.println(addPeerResult);
    return;
  }else{
    Serial.println("Peer added");
  }

  //Joystick setup
  pinMode(xPinA, INPUT);
  pinMode(yPinA, INPUT);
  pinMode(xPinB, INPUT);
  pinMode(yPinB, INPUT);
  lastxA = analogRead(xPinA);
  lastyA = analogRead(yPinA);
  lastxB = analogRead(xPinB);
  lastyB = analogRead(yPinB);

  Serial.println("Setup done");
}

//LOOP /////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  //Joystick
  unsigned long currentTime = millis();
  if (currentTime - lastCheckTime >= checkInterval) 
  {
    lastCheckTime = currentTime;
    dirA = readJoystick(xPinA, yPinA);
    lastxA = analogRead(xPinA);
    lastyA = analogRead(yPinA);
    dirB = readJoystick(xPinB, yPinB);
    lastxB = analogRead(xPinB);
    lastyB = analogRead(yPinB);
  }
  //ESP-NOW
  if(millis() - lastPosCheck > 10000){
    Serial.println("CheckisStanding");
    lastPosCheck = millis();
    esp_now_send(remoteMac, (uint8_t *) &isStanding, sizeof(isStanding));
  }
  if(answer == 0){
    lv_obj_add_state(ui_LimbControl1, LV_STATE_DISABLED);
    lv_obj_add_state(ui_Walk, LV_STATE_DISABLED);
  }else if(answer == 1){
    lv_obj_clear_state(ui_LimbControl1, LV_STATE_DISABLED);
    lv_obj_clear_state(ui_Walk, LV_STATE_DISABLED);
  }
  //LVGL
  lv_timer_handler(); /* let the GUI do its work */
  delay(5);
}

//Touch Calibration/////////////////////////////////////////////////////////////////////////////////////
void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  tft.fillScreen(TFT_BLACK);

  // check file system
  if (!SPIFFS.begin()) {
    Serial.println("formatting file system");

    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

//ESP-NOW/////////////////////////////////////////////////////////////////////////////////////////////////////
void readMacAdress(){
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

//Event functions/////////////////////////////////////////////////////////////////////////////////////////////

void getPositionLegs(lv_event_t * e)
{
  esp_now_send(remoteMac, (uint8_t *) &isStanding, sizeof(isStanding));
}

//Positions/////////////////////////////////////////////////////////////////////////////////////////////////////
void sendSit1(lv_event_t * e){
  Serial.println("Sit1");
  esp_now_send(remoteMac, (uint8_t *) &sit, sizeof(answer));
}

void standSend1(lv_event_t * e){
  Serial.println("Stand1");
  esp_now_send(remoteMac, (uint8_t *) &stand, sizeof(stand));
  answer = 1;
}

void crabSend1(lv_event_t * e){
  Serial.println("Crab1");
}

void resetPositionDog(lv_event_t * e){
  Serial.println("Reset");
  esp_now_send(remoteMac, (uint8_t *) &stand, sizeof(stand));
}


//LEGS////////////////////////////////////////////////////////////////////////////////////////////////////////
//Front left leg /////////////////////////////////////////////////////////////////////////////////////////////
void flSideChangeVal(lv_event_t * e)
{
  Serial.printf("posFLS: %d\n", lv_slider_get_value(e->target));
  posFLS = lv_slider_get_value(e->target);
  dataSend[0] = FLS;
  dataSend[1] = posFLS;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));

}

void flTopChangeVal(lv_event_t * e)
{
  Serial.printf("posFLT: %d\n", lv_slider_get_value(e->target));
  posFLT = lv_slider_get_value(e->target);
  dataSend[0] = FLT;
  dataSend[1] = posFLT;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}

void flBotChangeVal(lv_event_t * e)
{
  Serial.printf("posFLB: %d\n", lv_slider_get_value(e->target));
  posFLB = lv_slider_get_value(e->target);
  dataSend[0] = FLB;
  dataSend[1] = posFLB;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}

//Front right leg /////////////////////////////////////////////////////////////////////////////////////////////
void frSideChangeVal(lv_event_t * e)
{
  Serial.printf("posFRS: %d\n", lv_slider_get_value(e->target));
  posFRS = lv_slider_get_value(e->target);
  dataSend[0] = FRS;
  dataSend[1] = posFRS;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}

void frTopChangeVal(lv_event_t * e)
{
  Serial.printf("posFRT: %d\n", lv_slider_get_value(e->target));
  posFRT = lv_slider_get_value(e->target);
  dataSend[0] = FRT;
  dataSend[1] = posFRT;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}

void frBotChangeVal(lv_event_t * e)
{
  Serial.printf("posFRB: %d\n", lv_slider_get_value(e->target));
  posFRB = lv_slider_get_value(e->target);
  dataSend[0] = FRB;
  dataSend[1] = posFRB;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}


//Back left leg/////////////////////////////////////////////////////////////////////////////////////////////

void blSideChangeVal(lv_event_t * e)
{
  Serial.printf("posBLS: %d\n", lv_slider_get_value(e->target));
  posBLS = lv_slider_get_value(e->target);
  dataSend[0] = BLS;
  dataSend[1] = posBLS;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}

void blTopChangeVal(lv_event_t * e)
{
  Serial.printf("posBLT: %d\n", lv_slider_get_value(e->target));
  posBLT = lv_slider_get_value(e->target);
  dataSend[0] = BLT;
  dataSend[1] = posBLT;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}

void blBotChangeVal(lv_event_t * e)
{
  Serial.printf("posBLB: %d\n", lv_slider_get_value(e->target));
  posBLB = lv_slider_get_value(e->target);
  dataSend[0] = BLB;
  dataSend[1] = posBLB;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}


//Back right leg/////////////////////////////////////////////////////////////////////////////////////////////
void brSideChangeVal(lv_event_t * e)
{
  Serial.printf("posBRS: %d\n", lv_slider_get_value(e->target));
  posBRS = lv_slider_get_value(e->target);
  dataSend[0] = BRS;
  dataSend[1] = posBRS;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}

void brTopChangeVal(lv_event_t * e)
{
  Serial.printf("posBRT: %d\n", lv_slider_get_value(e->target));
  posBRT = lv_slider_get_value(e->target);
  dataSend[0] = BRT;
  dataSend[1] = posBRT;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}

void brBotChangeVal(lv_event_t * e)
{
  Serial.printf("posBRB: %d\n", lv_slider_get_value(e->target));
  posBRB = lv_slider_get_value(e->target);
  dataSend[0] = BRB;
  dataSend[1] = posBRB;
  esp_now_send(remoteMac, (uint8_t *) &dataSend, sizeof(dataSend));
}