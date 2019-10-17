#include <FastLED.h>
#include <LEDMatrix.h>
#include <LEDText.h>
#include <FontMatrise.h>
#include <WiFi.h>

#define MATRIX_TYPE HORIZONTAL_ZIGZAG_MATRIX
const uint16_t  MATRIX_WIDTH =  35;
const uint16_t  MATRIX_HEIGHT = 30;
const uint16_t  NUM_LEDS = 791;
const uint8_t DATA_PIN = 12;
const int BAUDRATE = 1497600;

const uint8_t verticalMarker = 252;
const uint8_t textMarker = 253;
const uint8_t startMarker = 254;
const uint8_t endMarker = 255;

CRGB leds[NUM_LEDS];
cLEDMatrix<MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_TYPE> textledsHorizontal;
cLEDMatrix<MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_TYPE> textledsVertical;

cLEDText ScrollingMsgHorizontal;
cLEDText ScrollingMsgVertical;

typedef struct{
  bool textmode;
  bool verticalMode;
  uint8_t length;
  uint8_t data[NUM_LEDS*3];
} MessageStruct;

static MessageStruct messageBuffer[2];

xQueueHandle xMessageQueue;
xQueueHandle xFreeQueue;

void setup(){
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.begin(BAUDRATE, SERIAL_8N1);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

    //init horizontal text
    ScrollingMsgHorizontal.Init(&textledsHorizontal, textledsHorizontal.Width(), ScrollingMsgHorizontal.FontHeight()+1,0,16);
    ScrollingMsgHorizontal.SetFont(MatriseFontData);
    ScrollingMsgHorizontal.SetTextColrOptions(COLR_RGB |  COLR_SINGLE, 0xff, 0x00, 0xff);
    ScrollingMsgHorizontal.SetScrollDirection(SCROLL_RIGHT);
    ScrollingMsgHorizontal.SetFrameRate(2);
    ScrollingMsgHorizontal.SetTextDirection(CHAR_DOWN);
    //init vertical text
    ScrollingMsgVertical.Init(&textledsVertical, 6, MATRIX_HEIGHT,15,0);
    ScrollingMsgVertical.SetFont(MatriseFontData);
    ScrollingMsgVertical.SetScrollDirection(SCROLL_UP);
    ScrollingMsgVertical.SetTextDirection(CHAR_DOWN);
    ScrollingMsgVertical.SetTextColrOptions(COLR_RGB |  COLR_SINGLE, 0xff, 0x00, 0xff);
    ScrollingMsgVertical.SetFrameRate(2);

  BaseType_t xIndex;
  xMessageQueue = xQueueCreate(2,sizeof(MessageStruct*));
  xFreeQueue = xQueueCreate(2, sizeof(MessageStruct*));

 for (xIndex = 0; xIndex <2; xIndex++){
      MessageStruct *messageStructPointer = messageBuffer + xIndex;
      xQueueSendToBack(xFreeQueue, (void*) &messageStructPointer, 0);
    }

    xTaskCreatePinnedToCore(
            ReceiveSerialDataTask, //task function
            "ReceiveSerialDataTask", //name of task
            10000, //stack size
            NULL, //parameter of task
            2,     //priority of task
            NULL,  //task handle
            0);    //core assigned to
        delay(10);

    xTaskCreatePinnedToCore(
        VisualizationTask, //task function
        "VisualizationTask", //name of task
        10000, //stack size
        NULL, //parameter of task
        3,     //priority of task
        NULL,  //task handle
        1);  //core assigned to    
        delay(10);

// brightness  0-255
  FastLED.setBrightness(10);
  FastLED.showColor(CRGB::Red);
  delay(20);
  FastLED.showColor(CRGB::Lime);
  delay(20);
  FastLED.showColor(CRGB::Blue);
  delay(20);
  FastLED.setBrightness(255);
  FastLED.show();
}


void ReceiveSerialDataTask(void * parameter){
  static boolean recvInProgress = false;
  static boolean recvComplete  = false;
  static uint16_t index =0;
  static uint8_t rb;
  MessageStruct *messageStructPointer;
  
  xQueueReceive(xFreeQueue, (void*) &messageStructPointer, 0);
  for (;;)
  {
      while (Serial.available() > 0 && recvComplete==false) 
        { 
            rb = Serial.read();
            if (recvInProgress==true)
            {
                if (rb != endMarker)
                {    
                        (*messageStructPointer).data[index] = rb;
                        index++;
                }
                else 
                { //end of message
                    (*messageStructPointer).length = index;
                    xQueueSendToBack(xMessageQueue, (void*)&messageStructPointer, portMAX_DELAY);
                    xQueueReceive(xFreeQueue, (void*)&messageStructPointer, portMAX_DELAY);
                    index = 0;
                    recvComplete = true;
                    recvInProgress = false;
                }
            }  
            else if (rb == startMarker) 
            {   //new message
                recvInProgress = true;
                receiveMessageInfo(messageStructPointer);               
            }
       }
        recvComplete = false;
        delay(1);
  }
}

void receiveMessageInfo(MessageStruct  *message)
{
static uint8_t rb;
static bool textVertical = false;
static bool textmode = false;
static uint8_t infoIndex = 0;
  while (Serial.available() > 0 &&  infoIndex <2)
  {
    rb = Serial.read();
    if (rb == textMarker)
    {
      textmode = true;
    }
    else if (rb == verticalMarker)
    {
      textVertical = true;
    }
    infoIndex++;
  }
  (*message).textmode = textmode;
  (*message).verticalMode = textVertical;
  infoIndex = 0;
  textVertical = false;
  textmode = false;
}


void VisualizationTask(void * parameter)
{
  MessageStruct *messageStructPointer;
  cLEDText * cLEDTextPointer;
  cLEDMatrixBase *cLEDMatrixBasePointer;
  for(;;)
  {
    //block and wait for message
    xQueueReceive(xMessageQueue, (void*) &messageStructPointer, portMAX_DELAY);
      if ( (*messageStructPointer).textmode == false)
      {   //MEA mode
        for (int i=0; i<NUM_LEDS*3; i+=3){
            leds[i/3].r = (*messageStructPointer).data[i];
            leds[i/3].g = (*messageStructPointer).data[i+1];
            leds[i/3].b = (*messageStructPointer).data[i+2];
            }
        FastLED.show();
        xQueueSendToBack(xFreeQueue, (void *) &messageStructPointer , portMAX_DELAY);
      }
      else
      {      //text mode
        if ((*messageStructPointer).verticalMode ==false)
        { //horizontal text
         cLEDTextPointer = &ScrollingMsgHorizontal;
         cLEDMatrixBasePointer = &textledsHorizontal;
        }
        else
        { //vertical text
          cLEDTextPointer = &ScrollingMsgVertical;
          cLEDMatrixBasePointer = &textledsVertical;
        }
          (*cLEDTextPointer).SetText((unsigned char*)(*messageStructPointer).data,(*messageStructPointer).length);
          while (uxQueueMessagesWaiting(xMessageQueue)==0) //check if new elements in queue)
          {
             if ( (*cLEDTextPointer).UpdateText() == -1 )
             { //text finished scrolling, looping back
                (*cLEDTextPointer).SetText((unsigned char*)(*messageStructPointer).data,(*messageStructPointer).length);
             }
             else
             {     
             remapLeds(leds, *cLEDMatrixBasePointer);
             FastLED.show();
             }       
         }
        xQueueSendToBack(xFreeQueue, (void *) &messageStructPointer , portMAX_DELAY);
      }
    delay(1);
  }
}

void remapLeds(CRGB * output, cLEDMatrixBase &input){
  for(int i = 0;i<input.Size(); i++){
      //strip 1
    if (i>12 && i<21)
      output[20-i]  = input(i);
      //strip 2
    if(i>43 && i<60)
      output[i-37] = input(i);
      //strip 3
    if(i>77 && i<99)
      output[i-53] = input(i);
      //strip 4
    if(i>108 && i<134)
     output[i-65] = input(i);
      //strip 5
    if(i>143 && i<172)
     output[i-73] = input(i);
      //strip 6
    if(i>176 && i<207)
      output[i-80] = input(i);
      //strip 7
    if(i>211 && i<244)
   output[i-83] = input(i);
      //strip 8 
    if(i>245 && i<278)
      output[i-86] = input(i);
      //strip 9-11
    if(i>279 && i<385)
     output[i-87] = input(i);
      //strip 12-13
    if(i>385 && i<454)
     output[i-88] = input(i);
      //strip 14-15
    if(i>455 && i<524)
      output[i-90] = input(i);
      //strip 16
    if(i>525 && i<560)
      output[i-92] = input(i);
      //strip 17
    if(i>560 && i<593)
     output[i-93] = input(i);
      //strip 18
    if(i>596 && i<629)
      output[i-97] = input(i);
      //strip 19
    if(i>630 && i<662)
     output[i-100] = input(i);
      //strip 20
    if(i>667 && i<698)
     output[i-105] = input(i);
      //strip 21
    if(i>701 && i<731)
    output[i-110] = input(i);
      //strip 22
    if(i>738 && i<767)
     output[i-117] = input(i);
      //strip 23
    if(i>773 && i<800)
    output[i-124] = input(i);
      //strip 24
    if(i>810 && i<835)
      output[i-135] = input(i);
      //strip 25
    if(i>845 && i<868)
      output[i-146] = input(i);
      //strip 26
    if(i>882 && i<903)
      output[i-161] = input(i);
      //strip 27
    if(i>917 && i<936)
      output[i-176] = input(i);
      //strip 28
    if(i>955 && i<970)
      output[i-196] = input(i);
      //strip 29
    if(i>991 && i<1002)
      output[i-218] = input(i);
      //strip 30
    if(i>1029 && i<1037)
      output[i-245] = input(i);
  }
}

void loop(){
    vTaskSuspend(NULL);
}
