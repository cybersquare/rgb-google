#include <FastLED.h>
#define LED_PIN     3
#define NUM_LEDS    88
#define LED_TYPE    WS2812B
#define COLOR_ORDER RGB
#define BRIGHTNESS  255
#define UPDATES_PER_SECOND 100

#define kMatrixWidth  17
#define kMatrixHeight 6
#define kMatrixSerpentineLayout  true

CRGBArray<NUM_LEDS> leds;

uint32_t x,y,v_time,hue_time,hxy;
uint8_t octaves=1;
uint8_t hue_octaves=3;
int xscale=57771;
int yscale=57771;
int hue_scale=1;
int time_speed=1111;
int hue_speed=31;
int x_speed=331;
int y_speed=1111;

CRGBPalette16 currentPalette;
TBlendType    currentBlending;
extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

void setup() 
{
    Serial.begin(9600);
    delay(1000);
   FastLED.addLeds<NEOPIXEL,LED_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);

    currentPalette = RainbowColors_p;
    currentBlending = LINEARBLEND;
    
  random16_set_seed(8934);
  random16_add_entropy(analogRead(3));
    hxy = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
  x = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
  y = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
  v_time = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
  hue_time = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
}

void fadeall() { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(250); } }


char ch;
void loop()
{
  FastLED.clear();
  FastLED.show();
  if(Serial.available())
    {
      ch=Serial.read();
      Serial.println(ch);
       switch (ch)
        {
          case '1':
          for(int i=0;i<10;i++)
          {
             leds[37] = CRGB(255, 0, 0);
             leds[44] = CRGB(255, 0, 0);
             FastLED.show();
             delay(90);
             FastLED.clear();
             leds[20] = CRGB(255, 0, 0);
leds[30] = CRGB(255, 0, 0);
leds[36] = CRGB(255, 0, 0);
leds[38] = CRGB(255, 0, 0);
leds[43] = CRGB(255, 0, 0);
leds[45] = CRGB(255, 0, 0);
leds[51] = CRGB(255, 0, 0);
leds[58] = CRGB(255, 0, 0);
FastLED.show();
delay(90);
FastLED.clear();

leds[19] = CRGB(255, 0, 0);
leds[21] = CRGB(255, 0, 0);
leds[29] = CRGB(255, 0, 0);
leds[31] = CRGB(255, 0, 0);
leds[35] = CRGB(255, 0, 0);
leds[36] = CRGB(255, 0, 0);
leds[37] = CRGB(255, 0, 0);
leds[38] = CRGB(255, 0, 0);
leds[39] = CRGB(255, 0, 0);
leds[42] = CRGB(255, 0, 0);
leds[43] = CRGB(255, 0, 0);
leds[44] = CRGB(255, 0, 0);
leds[45] = CRGB(255, 0, 0);
leds[46] = CRGB(255, 0, 0);
leds[50] = CRGB(255, 0, 0);
leds[51] = CRGB(255, 0, 0);
leds[52] = CRGB(255, 0, 0);
leds[57] = CRGB(255, 0, 0);
leds[58] = CRGB(255, 0, 0);
leds[59] = CRGB(255, 0, 0);
leds[65] = CRGB(255, 0, 0);
leds[72] = CRGB(255, 0, 0);
FastLED.show();

delay(90);
FastLED.clear(); 

leds[1] = CRGB(255, 0, 0);
leds[2] = CRGB(255, 0, 0);
leds[4] = CRGB(255, 0, 0);
leds[5] = CRGB(255, 0, 0);
leds[6] = CRGB(0, 0, 0);
leds[11] = CRGB(255, 0, 0);
leds[12] = CRGB(255, 0, 0);
leds[14] = CRGB(255, 0, 0);
leds[15] = CRGB(255, 0, 0);
leds[17] = CRGB(255, 0, 0);
leds[18] = CRGB(255, 0, 0);
leds[19] = CRGB(255, 0, 0);
leds[20] = CRGB(255, 0, 0);
leds[21] = CRGB(255, 0, 0);
leds[22] = CRGB(255, 0, 0);
leds[23] = CRGB(255, 0, 0);
leds[27] = CRGB(255, 0, 0);
leds[28] = CRGB(255, 0, 0);
leds[29] = CRGB(255, 0, 0);
leds[30] = CRGB(255, 0, 0);
leds[31] = CRGB(255, 0, 0);
leds[32] = CRGB(255, 0, 0);
leds[33] = CRGB(255, 0, 0);
leds[34] = CRGB(255, 0, 0);
leds[35] = CRGB(255, 0, 0);
leds[36] = CRGB(255, 0, 0);
leds[37] = CRGB(255, 0, 0);
leds[38] = CRGB(255, 0, 0);
leds[39] = CRGB(255, 0, 0);
leds[40] = CRGB(255, 0, 0);
leds[41] = CRGB(255, 0, 0);
leds[42] = CRGB(255, 0, 0);
leds[43] = CRGB(255, 0, 0);
leds[44] = CRGB(255, 0, 0);
leds[45] = CRGB(255, 0, 0);
leds[46] = CRGB(255, 0, 0);
leds[47] = CRGB(255, 0, 0);
leds[49] = CRGB(255, 0, 0);
leds[50] = CRGB(255, 0, 0);
leds[51] = CRGB(255, 0, 0);
leds[52] = CRGB(255, 0, 0);
leds[53] = CRGB(255, 0, 0);
leds[56] = CRGB(255, 0, 0);
leds[57] = CRGB(255, 0, 0);
leds[58] = CRGB(255, 0, 0);
leds[59] = CRGB(255, 0, 0);
leds[60] = CRGB(255, 0, 0);
leds[64] = CRGB(255, 0, 0);
leds[65] = CRGB(255, 0, 0);
leds[66] = CRGB(255, 0, 0);
leds[71] = CRGB(255, 0, 0);
leds[72] = CRGB(255, 0, 0);
leds[73] = CRGB(255, 0, 0);
leds[79] = CRGB(255, 0, 0);
leds[84] = CRGB(255, 0, 0);
FastLED.show();
delay(90);
FastLED.clear();

leds[19] = CRGB(255, 0, 0);
leds[21] = CRGB(255, 0, 0);
leds[29] = CRGB(255, 0, 0);
leds[31] = CRGB(255, 0, 0);
leds[35] = CRGB(255, 0, 0);
leds[36] = CRGB(255, 0, 0);
leds[37] = CRGB(255, 0, 0);
leds[38] = CRGB(255, 0, 0);
leds[39] = CRGB(255, 0, 0);
leds[42] = CRGB(255, 0, 0);
leds[43] = CRGB(255, 0, 0);
leds[44] = CRGB(255, 0, 0);
leds[45] = CRGB(255, 0, 0);
leds[46] = CRGB(255, 0, 0);
leds[50] = CRGB(255, 0, 0);
leds[51] = CRGB(255, 0, 0);
leds[52] = CRGB(255, 0, 0);
leds[57] = CRGB(255, 0, 0);
leds[58] = CRGB(255, 0, 0);
leds[59] = CRGB(255, 0, 0);
leds[65] = CRGB(255, 0, 0);
leds[72] = CRGB(255, 0, 0);
FastLED.show();

delay(90);
FastLED.clear(); 

leds[20] = CRGB(255, 0, 0);
leds[30] = CRGB(255, 0, 0);
leds[36] = CRGB(255, 0, 0);
leds[38] = CRGB(255, 0, 0);
leds[43] = CRGB(255, 0, 0);
leds[45] = CRGB(255, 0, 0);
leds[51] = CRGB(255, 0, 0);
leds[58] = CRGB(255, 0, 0);
FastLED.show();
delay(90);
FastLED.clear();
          }
break; 

case '2':
for(int i=0;i<5;i++)
{
leds[36] = CRGB(255, 0, 187);
leds[37] = CRGB(255, 0, 187);
leds[44] = CRGB(255, 0, 187);
leds[45] = CRGB(255, 0, 187);
leds[50] = CRGB(255, 0, 187);
leds[51] = CRGB(255, 0, 187);
leds[58] = CRGB(255, 0, 187);
leds[59] = CRGB(255, 0, 187);
FastLED.show();
delay(90);
FastLED.clear();

leds[19] = CRGB(255, 0, 187);
leds[20] = CRGB(255, 0, 187);
leds[30] = CRGB(255, 0, 187);
leds[31] = CRGB(255, 0, 187);
leds[35] = CRGB(255, 0, 187);
leds[38] = CRGB(255, 0, 187);
leds[43] = CRGB(255, 0, 187);
leds[46] = CRGB(255, 0, 187);
leds[49] = CRGB(255, 0, 187);
leds[52] = CRGB(255, 0, 187);
leds[57] = CRGB(255, 0, 187);
leds[60] = CRGB(255, 0, 187);
leds[64] = CRGB(255, 0, 187);
leds[65] = CRGB(255, 0, 187);
leds[72] = CRGB(255, 0, 187);
leds[73] = CRGB(255, 0, 187);
FastLED.show();
delay(90);
FastLED.clear();

leds[0] = CRGB(0, 0, 0);
leds[1] = CRGB(255, 0, 187);
leds[2] = CRGB(255, 0, 187);
leds[3] = CRGB(255, 0, 187);
leds[4] = CRGB(255, 0, 187);
leds[12] = CRGB(255, 0, 187);
leds[13] = CRGB(255, 0, 187);
leds[14] = CRGB(255, 0, 187);
leds[15] = CRGB(255, 0, 187);
leds[17] = CRGB(255, 0, 187);
leds[22] = CRGB(255, 0, 187);
leds[28] = CRGB(255, 0, 187);
leds[33] = CRGB(255, 0, 187);
leds[34] = CRGB(255, 0, 187);
leds[39] = CRGB(255, 0, 187);
leds[42] = CRGB(255, 0, 187);
leds[47] = CRGB(255, 0, 187);
leds[48] = CRGB(255, 0, 187);
leds[53] = CRGB(255, 0, 187);
leds[56] = CRGB(255, 0, 187);
leds[61] = CRGB(255, 0, 187);
leds[62] = CRGB(255, 0, 187);
leds[67] = CRGB(255, 0, 187);
leds[70] = CRGB(255, 0, 187);
leds[75] = CRGB(255, 0, 187);
leds[77] = CRGB(255, 0, 187);
leds[78] = CRGB(255, 0, 187);
leds[79] = CRGB(255, 0, 187);
leds[80] = CRGB(255, 0, 187);
leds[83] = CRGB(255, 0, 187);
leds[84] = CRGB(255, 0, 187);
leds[85] = CRGB(255, 0, 187);
leds[86] = CRGB(255, 0, 187);
FastLED.show();
delay(90);
FastLED.clear();

leds[0] = CRGB(0, 0, 0);
leds[1] = CRGB(255, 0, 187);
leds[2] = CRGB(255, 0, 187);
leds[3] = CRGB(255, 0, 187);
leds[4] = CRGB(255, 0, 187);
leds[12] = CRGB(255, 0, 187);
leds[13] = CRGB(255, 0, 187);
leds[14] = CRGB(255, 0, 187);
leds[15] = CRGB(255, 0, 187);
leds[17] = CRGB(255, 0, 187);
leds[22] = CRGB(255, 0, 187);
leds[28] = CRGB(255, 0, 187);
leds[33] = CRGB(255, 0, 187);
leds[34] = CRGB(255, 0, 187);
leds[39] = CRGB(255, 0, 187);
leds[42] = CRGB(255, 0, 187);
leds[47] = CRGB(255, 0, 187);
leds[48] = CRGB(255, 0, 187);
leds[53] = CRGB(255, 0, 187);
leds[56] = CRGB(255, 0, 187);
leds[61] = CRGB(255, 0, 187);
leds[62] = CRGB(255, 0, 187);
leds[67] = CRGB(255, 0, 187);
leds[70] = CRGB(255, 0, 187);
leds[75] = CRGB(255, 0, 187);
leds[77] = CRGB(255, 0, 187);
leds[78] = CRGB(255, 0, 187);
leds[79] = CRGB(255, 0, 187);
leds[80] = CRGB(255, 0, 187);
leds[83] = CRGB(255, 0, 187);
leds[84] = CRGB(255, 0, 187);
leds[85] = CRGB(255, 0, 187);
leds[86] = CRGB(255, 0, 187);
FastLED.show();
delay(90);
FastLED.clear();

leds[19] = CRGB(255, 0, 187);
leds[20] = CRGB(255, 0, 187);
leds[30] = CRGB(255, 0, 187);
leds[31] = CRGB(255, 0, 187);
leds[35] = CRGB(255, 0, 187);
leds[38] = CRGB(255, 0, 187);
leds[43] = CRGB(255, 0, 187);
leds[46] = CRGB(255, 0, 187);
leds[49] = CRGB(255, 0, 187);
leds[52] = CRGB(255, 0, 187);
leds[57] = CRGB(255, 0, 187);
leds[60] = CRGB(255, 0, 187);
leds[64] = CRGB(255, 0, 187);
leds[65] = CRGB(255, 0, 187);
leds[72] = CRGB(255, 0, 187);
leds[73] = CRGB(255, 0, 187);
FastLED.show();
delay(90);
FastLED.clear();

leds[36] = CRGB(255, 0, 187);
leds[37] = CRGB(255, 0, 187);
leds[44] = CRGB(255, 0, 187);
leds[45] = CRGB(255, 0, 187);
leds[50] = CRGB(255, 0, 187);
leds[51] = CRGB(255, 0, 187);
leds[58] = CRGB(255, 0, 187);
leds[59] = CRGB(255, 0, 187);
FastLED.show();
delay(90);
FastLED.clear();
}
break;

case '3':
for(int i=0;i<5;i++)
{
leds[0] = CRGB(255, 0, 187);
leds[1] = CRGB(255, 0, 187);
leds[2] = CRGB(255, 0, 187);
leds[3] = CRGB(255, 0, 187);
leds[4] = CRGB(255, 0, 187);
leds[5] = CRGB(255, 0, 187);
leds[6] = CRGB(255, 0, 187);
leds[7] = CRGB(255, 0, 187);
leds[8] = CRGB(255, 0, 187);
leds[9] = CRGB(255, 0, 187);
leds[10] = CRGB(255, 0, 187);
leds[11] = CRGB(255, 0, 187);
leds[12] = CRGB(255, 0, 187);
leds[13] = CRGB(255, 0, 187);
leds[14] = CRGB(255, 0, 187);
leds[15] = CRGB(255, 0, 187);
leds[16] = CRGB(255, 0, 187);
leds[17] = CRGB(255, 0, 187);
leds[18] = CRGB(255, 161, 0);
leds[19] = CRGB(255, 161, 0);
leds[20] = CRGB(255, 161, 0);
leds[21] = CRGB(255, 161, 0);
leds[22] = CRGB(255, 161, 0);
leds[23] = CRGB(255, 0, 187);
leds[24] = CRGB(255, 0, 187);
leds[25] = CRGB(255, 0, 187);
leds[26] = CRGB(255, 0, 187);
leds[27] = CRGB(255, 0, 187);
leds[28] = CRGB(255, 161, 0);
leds[29] = CRGB(255, 161, 0);
leds[30] = CRGB(255, 161, 0);
leds[31] = CRGB(255, 161, 0);
leds[32] = CRGB(255, 161, 0);
leds[33] = CRGB(255, 0, 187);
leds[34] = CRGB(255, 0, 187);
leds[35] = CRGB(255, 161, 0);
leds[36] = CRGB(0, 255, 255);
leds[37] = CRGB(0, 255, 255);
leds[38] = CRGB(0, 255, 255);
leds[39] = CRGB(255, 161, 0);
leds[40] = CRGB(255, 0, 187);
leds[41] = CRGB(255, 0, 187);
leds[42] = CRGB(255, 161, 0);
leds[43] = CRGB(0, 255, 255);
leds[44] = CRGB(0, 255, 255);
leds[45] = CRGB(0, 255, 255);
leds[46] = CRGB(255, 161, 0);
leds[47] = CRGB(255, 0, 187);
leds[48] = CRGB(255, 0, 187);
leds[49] = CRGB(255, 161, 0);
leds[50] = CRGB(0, 255, 255);
leds[51] = CRGB(0, 255, 255);
leds[52] = CRGB(0, 255, 255);
leds[53] = CRGB(255, 161, 0);
leds[54] = CRGB(255, 0, 187);
leds[55] = CRGB(255, 0, 187);
leds[56] = CRGB(255, 161, 0);
leds[57] = CRGB(0, 255, 255);
leds[58] = CRGB(0, 255, 255);
leds[59] = CRGB(0, 255, 255);
leds[60] = CRGB(255, 161, 0);
leds[61] = CRGB(255, 0, 187);
leds[62] = CRGB(255, 0, 187);
leds[63] = CRGB(255, 161, 0);
leds[64] = CRGB(255, 161, 0);
leds[65] = CRGB(255, 161, 0);
leds[66] = CRGB(255, 161, 0);
leds[67] = CRGB(255, 161, 0);
leds[68] = CRGB(255, 0, 187);
leds[69] = CRGB(255, 0, 187);
leds[70] = CRGB(255, 161, 0);
leds[71] = CRGB(255, 161, 0);
leds[72] = CRGB(255, 161, 0);
leds[73] = CRGB(255, 161, 0);
leds[74] = CRGB(255, 161, 0);
leds[75] = CRGB(255, 0, 187);
leds[76] = CRGB(255, 0, 187);
leds[77] = CRGB(255, 0, 187);
leds[78] = CRGB(255, 0, 187);
leds[79] = CRGB(255, 0, 187);
leds[80] = CRGB(255, 0, 187);
leds[81] = CRGB(255, 0, 187);
leds[82] = CRGB(255, 0, 187);
leds[83] = CRGB(255, 0, 187);
leds[84] = CRGB(255, 0, 187);
leds[85] = CRGB(255, 0, 187);
leds[86] = CRGB(255, 0, 187);
leds[87] = CRGB(255, 0, 187);
FastLED.show();
delay(90);
FastLED.clear();

leds[0] = CRGB(255, 164, 0);
leds[1] = CRGB(255, 164, 0);
leds[2] = CRGB(255, 164, 0);
leds[3] = CRGB(255, 164, 0);
leds[4] = CRGB(255, 164, 0);
leds[5] = CRGB(255, 164, 0);
leds[6] = CRGB(255, 164, 0);
leds[7] = CRGB(255, 164, 0);
leds[8] = CRGB(255, 164, 0);
leds[9] = CRGB(255, 164, 0);
leds[10] = CRGB(255, 164, 0);
leds[11] = CRGB(255, 164, 0);
leds[12] = CRGB(255, 164, 0);
leds[13] = CRGB(255, 164, 0);
leds[14] = CRGB(255, 164, 0);
leds[15] = CRGB(255, 164, 0);
leds[16] = CRGB(255, 164, 0);
leds[17] = CRGB(255, 164, 0);
leds[18] = CRGB(0, 255, 255);
leds[19] = CRGB(0, 255, 255);
leds[20] = CRGB(0, 255, 255);
leds[21] = CRGB(0, 255, 255);
leds[22] = CRGB(0, 255, 255);
leds[23] = CRGB(255, 164, 0);
leds[24] = CRGB(255, 164, 0);
leds[25] = CRGB(255, 164, 0);
leds[26] = CRGB(255, 164, 0);
leds[27] = CRGB(255, 164, 0);
leds[28] = CRGB(0, 255, 255);
leds[29] = CRGB(0, 255, 255);
leds[30] = CRGB(0, 255, 255);
leds[31] = CRGB(0, 255, 255);
leds[32] = CRGB(0, 255, 255);
leds[33] = CRGB(255, 164, 0);
leds[34] = CRGB(255, 164, 0);
leds[35] = CRGB(0, 255, 255);
leds[36] = CRGB(255, 0, 177);
leds[37] = CRGB(255, 0, 177);
leds[38] = CRGB(255, 0, 177);
leds[39] = CRGB(0, 255, 255);
leds[40] = CRGB(255, 164, 0);
leds[41] = CRGB(255, 146, 0);
leds[42] = CRGB(0, 255, 255);
leds[43] = CRGB(255, 0, 177);
leds[44] = CRGB(255, 0, 177);
leds[45] = CRGB(255, 0, 177);
leds[46] = CRGB(0, 255, 255);
leds[47] = CRGB(255, 146, 0);
leds[48] = CRGB(255, 146, 0);
leds[49] = CRGB(0, 255, 255);
leds[50] = CRGB(255, 0, 177);
leds[51] = CRGB(255, 0, 177);
leds[52] = CRGB(255, 0, 177);
leds[53] = CRGB(0, 255, 255);
leds[54] = CRGB(255, 164, 0);
leds[55] = CRGB(255, 164, 0);
leds[56] = CRGB(0, 255, 255);
leds[57] = CRGB(255, 0, 177);
leds[58] = CRGB(255, 0, 177);
leds[59] = CRGB(255, 0, 177);
leds[60] = CRGB(0, 255, 255);
leds[61] = CRGB(255, 164, 0);
leds[62] = CRGB(255, 164, 0);
leds[63] = CRGB(0, 255, 255);
leds[64] = CRGB(0, 255, 255);
leds[65] = CRGB(0, 255, 255);
leds[66] = CRGB(0, 255, 255);
leds[67] = CRGB(0, 255, 255);
leds[68] = CRGB(255, 164, 0);
leds[69] = CRGB(255, 164, 0);
leds[70] = CRGB(0, 255, 255);
leds[71] = CRGB(0, 255, 255);
leds[72] = CRGB(0, 255, 255);
leds[73] = CRGB(0, 255, 255);
leds[74] = CRGB(0, 255, 255);
leds[75] = CRGB(255, 164, 0);
leds[76] = CRGB(255, 164, 0);
leds[77] = CRGB(255, 164, 0);
leds[78] = CRGB(255, 164, 0);
leds[79] = CRGB(255, 164, 0);
leds[80] = CRGB(255, 164, 0);
leds[81] = CRGB(255, 164, 0);
leds[82] = CRGB(255, 164, 0);
leds[83] = CRGB(255, 164, 0);
leds[84] = CRGB(255, 164, 0);
leds[85] = CRGB(255, 164, 0);
leds[86] = CRGB(255, 164, 0);
leds[87] = CRGB(255, 164, 0);
FastLED.show();
delay(90);
FastLED.clear();


leds[0] = CRGB(0, 255, 255);
leds[1] = CRGB(0, 255, 255);
leds[2] = CRGB(0, 255, 255);
leds[3] = CRGB(0, 255, 255);
leds[4] = CRGB(0, 255, 255);
leds[5] = CRGB(0, 255, 255);
leds[6] = CRGB(0, 255, 255);
leds[7] = CRGB(0, 255, 255);
leds[8] = CRGB(0, 255, 255);
leds[9] = CRGB(0, 255, 255);
leds[10] = CRGB(0, 255, 255);
leds[11] = CRGB(0, 255, 255);
leds[12] = CRGB(0, 255, 255);
leds[13] = CRGB(0, 255, 255);
leds[14] = CRGB(0, 255, 255);
leds[15] = CRGB(0, 255, 255);
leds[16] = CRGB(0, 255, 255);
leds[17] = CRGB(0, 255, 255);
leds[18] = CRGB(255, 0, 208);
leds[19] = CRGB(255, 0, 208);
leds[20] = CRGB(255, 0, 180);
leds[21] = CRGB(255, 0, 180);
leds[22] = CRGB(255, 0, 180);
leds[23] = CRGB(0, 255, 255);
leds[24] = CRGB(0, 255, 255);
leds[25] = CRGB(0, 255, 255);
leds[26] = CRGB(0, 255, 255);
leds[27] = CRGB(0, 255, 255);
leds[28] = CRGB(255, 0, 208);
leds[29] = CRGB(255, 0, 208);
leds[30] = CRGB(255, 0, 180);
leds[31] = CRGB(255, 0, 180);
leds[32] = CRGB(255, 0, 180);
leds[33] = CRGB(0, 255, 255);
leds[34] = CRGB(0, 255, 255);
leds[35] = CRGB(255, 0, 180);
leds[36] = CRGB(255, 148, 0);
leds[37] = CRGB(255, 148, 0);
leds[38] = CRGB(255, 148, 0);
leds[39] = CRGB(255, 0, 208);
leds[40] = CRGB(0, 255, 255);
leds[41] = CRGB(0, 255, 255);
leds[42] = CRGB(255, 0, 180);
leds[43] = CRGB(255, 148, 0);
leds[44] = CRGB(255, 148, 0);
leds[45] = CRGB(255, 148, 0);
leds[46] = CRGB(255, 0, 180);
leds[47] = CRGB(0, 255, 255);
leds[48] = CRGB(0, 255, 255);
leds[49] = CRGB(255, 0, 208);
leds[50] = CRGB(255, 148, 0);
leds[51] = CRGB(255, 148, 0);
leds[52] = CRGB(255, 148, 0);
leds[53] = CRGB(255, 0, 180);
leds[54] = CRGB(0, 255, 255);
leds[55] = CRGB(0, 255, 255);
leds[56] = CRGB(255, 0, 180);
leds[57] = CRGB(255, 148, 0);
leds[58] = CRGB(255, 148, 0);
leds[59] = CRGB(255, 148, 0);
leds[60] = CRGB(255, 0, 180);
leds[61] = CRGB(0, 255, 255);
leds[62] = CRGB(0, 255, 255);
leds[63] = CRGB(255, 0, 180);
leds[64] = CRGB(255, 0, 180);
leds[65] = CRGB(255, 0, 180);
leds[66] = CRGB(255, 0, 180);
leds[67] = CRGB(255, 0, 180);
leds[68] = CRGB(0, 255, 255);
leds[69] = CRGB(0, 255, 255);
leds[70] = CRGB(255, 0, 180);
leds[71] = CRGB(255, 0, 180);
leds[72] = CRGB(255, 0, 180);
leds[73] = CRGB(255, 0, 180);
leds[74] = CRGB(255, 0, 180);
leds[75] = CRGB(0, 255, 255);
leds[76] = CRGB(0, 255, 255);
leds[77] = CRGB(0, 255, 255);
leds[78] = CRGB(0, 255, 255);
leds[79] = CRGB(0, 255, 255);
leds[80] = CRGB(0, 255, 255);
leds[81] = CRGB(0, 255, 255);
leds[82] = CRGB(0, 255, 255);
leds[83] = CRGB(0, 255, 255);
leds[84] = CRGB(0, 255, 255);
leds[85] = CRGB(0, 255, 255);
leds[86] = CRGB(0, 255, 255);
leds[87] = CRGB(0, 255, 255);
FastLED.show();
delay(90);
FastLED.clear();

leds[0] = CRGB(0, 255, 255);
leds[1] = CRGB(0, 255, 255);
leds[2] = CRGB(0, 255, 255);
leds[3] = CRGB(0, 255, 255);
leds[4] = CRGB(0, 255, 255);
leds[5] = CRGB(0, 255, 255);
leds[6] = CRGB(0, 255, 255);
leds[7] = CRGB(0, 255, 255);
leds[8] = CRGB(0, 255, 255);
leds[9] = CRGB(0, 255, 255);
leds[10] = CRGB(0, 255, 255);
leds[11] = CRGB(0, 255, 255);
leds[12] = CRGB(0, 255, 255);
leds[13] = CRGB(0, 255, 255);
leds[14] = CRGB(0, 255, 255);
leds[15] = CRGB(0, 255, 255);
leds[16] = CRGB(0, 255, 255);
leds[17] = CRGB(0, 255, 255);
leds[18] = CRGB(255, 0, 208);
leds[19] = CRGB(255, 0, 208);
leds[20] = CRGB(255, 0, 180);
leds[21] = CRGB(255, 0, 180);
leds[22] = CRGB(255, 0, 180);
leds[23] = CRGB(0, 255, 255);
leds[24] = CRGB(0, 255, 255);
leds[25] = CRGB(0, 255, 255);
leds[26] = CRGB(0, 255, 255);
leds[27] = CRGB(0, 255, 255);
leds[28] = CRGB(255, 0, 208);
leds[29] = CRGB(255, 0, 208);
leds[30] = CRGB(255, 0, 180);
leds[31] = CRGB(255, 0, 180);
leds[32] = CRGB(255, 0, 180);
leds[33] = CRGB(0, 255, 255);
leds[34] = CRGB(0, 255, 255);
leds[35] = CRGB(255, 0, 180);
leds[36] = CRGB(255, 148, 0);
leds[37] = CRGB(255, 148, 0);
leds[38] = CRGB(255, 148, 0);
leds[39] = CRGB(255, 0, 208);
leds[40] = CRGB(0, 255, 255);
leds[41] = CRGB(0, 255, 255);
leds[42] = CRGB(255, 0, 180);
leds[43] = CRGB(255, 148, 0);
leds[44] = CRGB(255, 148, 0);
leds[45] = CRGB(255, 148, 0);
leds[46] = CRGB(255, 0, 180);
leds[47] = CRGB(0, 255, 255);
leds[48] = CRGB(0, 255, 255);
leds[49] = CRGB(255, 0, 208);
leds[50] = CRGB(255, 148, 0);
leds[51] = CRGB(255, 148, 0);
leds[52] = CRGB(255, 148, 0);
leds[53] = CRGB(255, 0, 180);
leds[54] = CRGB(0, 255, 255);
leds[55] = CRGB(0, 255, 255);
leds[56] = CRGB(255, 0, 180);
leds[57] = CRGB(255, 148, 0);
leds[58] = CRGB(255, 148, 0);
leds[59] = CRGB(255, 148, 0);
leds[60] = CRGB(255, 0, 180);
leds[61] = CRGB(0, 255, 255);
leds[62] = CRGB(0, 255, 255);
leds[63] = CRGB(255, 0, 180);
leds[64] = CRGB(255, 0, 180);
leds[65] = CRGB(255, 0, 180);
leds[66] = CRGB(255, 0, 180);
leds[67] = CRGB(255, 0, 180);
leds[68] = CRGB(0, 255, 255);
leds[69] = CRGB(0, 255, 255);
leds[70] = CRGB(255, 0, 180);
leds[71] = CRGB(255, 0, 180);
leds[72] = CRGB(255, 0, 180);
leds[73] = CRGB(255, 0, 180);
leds[74] = CRGB(255, 0, 180);
leds[75] = CRGB(0, 255, 255);
leds[76] = CRGB(0, 255, 255);
leds[77] = CRGB(0, 255, 255);
leds[78] = CRGB(0, 255, 255);
leds[79] = CRGB(0, 255, 255);
leds[80] = CRGB(0, 255, 255);
leds[81] = CRGB(0, 255, 255);
leds[82] = CRGB(0, 255, 255);
leds[83] = CRGB(0, 255, 255);
leds[84] = CRGB(0, 255, 255);
leds[85] = CRGB(0, 255, 255);
leds[86] = CRGB(0, 255, 255);
leds[87] = CRGB(0, 255, 255);
FastLED.show();
delay(90);
FastLED.clear();

leds[0] = CRGB(255, 164, 0);
leds[1] = CRGB(255, 164, 0);
leds[2] = CRGB(255, 164, 0);
leds[3] = CRGB(255, 164, 0);
leds[4] = CRGB(255, 164, 0);
leds[5] = CRGB(255, 164, 0);
leds[6] = CRGB(255, 164, 0);
leds[7] = CRGB(255, 164, 0);
leds[8] = CRGB(255, 164, 0);
leds[9] = CRGB(255, 164, 0);
leds[10] = CRGB(255, 164, 0);
leds[11] = CRGB(255, 164, 0);
leds[12] = CRGB(255, 164, 0);
leds[13] = CRGB(255, 164, 0);
leds[14] = CRGB(255, 164, 0);
leds[15] = CRGB(255, 164, 0);
leds[16] = CRGB(255, 164, 0);
leds[17] = CRGB(255, 164, 0);
leds[18] = CRGB(0, 255, 255);
leds[19] = CRGB(0, 255, 255);
leds[20] = CRGB(0, 255, 255);
leds[21] = CRGB(0, 255, 255);
leds[22] = CRGB(0, 255, 255);
leds[23] = CRGB(255, 164, 0);
leds[24] = CRGB(255, 164, 0);
leds[25] = CRGB(255, 164, 0);
leds[26] = CRGB(255, 164, 0);
leds[27] = CRGB(255, 164, 0);
leds[28] = CRGB(0, 255, 255);
leds[29] = CRGB(0, 255, 255);
leds[30] = CRGB(0, 255, 255);
leds[31] = CRGB(0, 255, 255);
leds[32] = CRGB(0, 255, 255);
leds[33] = CRGB(255, 164, 0);
leds[34] = CRGB(255, 164, 0);
leds[35] = CRGB(0, 255, 255);
leds[36] = CRGB(255, 0, 177);
leds[37] = CRGB(255, 0, 177);
leds[38] = CRGB(255, 0, 177);
leds[39] = CRGB(0, 255, 255);
leds[40] = CRGB(255, 164, 0);
leds[41] = CRGB(255, 146, 0);
leds[42] = CRGB(0, 255, 255);
leds[43] = CRGB(255, 0, 177);
leds[44] = CRGB(255, 0, 177);
leds[45] = CRGB(255, 0, 177);
leds[46] = CRGB(0, 255, 255);
leds[47] = CRGB(255, 146, 0);
leds[48] = CRGB(255, 146, 0);
leds[49] = CRGB(0, 255, 255);
leds[50] = CRGB(255, 0, 177);
leds[51] = CRGB(255, 0, 177);
leds[52] = CRGB(255, 0, 177);
leds[53] = CRGB(0, 255, 255);
leds[54] = CRGB(255, 164, 0);
leds[55] = CRGB(255, 164, 0);
leds[56] = CRGB(0, 255, 255);
leds[57] = CRGB(255, 0, 177);
leds[58] = CRGB(255, 0, 177);
leds[59] = CRGB(255, 0, 177);
leds[60] = CRGB(0, 255, 255);
leds[61] = CRGB(255, 164, 0);
leds[62] = CRGB(255, 164, 0);
leds[63] = CRGB(0, 255, 255);
leds[64] = CRGB(0, 255, 255);
leds[65] = CRGB(0, 255, 255);
leds[66] = CRGB(0, 255, 255);
leds[67] = CRGB(0, 255, 255);
leds[68] = CRGB(255, 164, 0);
leds[69] = CRGB(255, 164, 0);
leds[70] = CRGB(0, 255, 255);
leds[71] = CRGB(0, 255, 255);
leds[72] = CRGB(0, 255, 255);
leds[73] = CRGB(0, 255, 255);
leds[74] = CRGB(0, 255, 255);
leds[75] = CRGB(255, 164, 0);
leds[76] = CRGB(255, 164, 0);
leds[77] = CRGB(255, 164, 0);
leds[78] = CRGB(255, 164, 0);
leds[79] = CRGB(255, 164, 0);
leds[80] = CRGB(255, 164, 0);
leds[81] = CRGB(255, 164, 0);
leds[82] = CRGB(255, 164, 0);
leds[83] = CRGB(255, 164, 0);
leds[84] = CRGB(255, 164, 0);
leds[85] = CRGB(255, 164, 0);
leds[86] = CRGB(255, 164, 0);
leds[87] = CRGB(255, 164, 0);
FastLED.show();
delay(90);
FastLED.clear();
leds[0] = CRGB(255, 0, 187);
leds[1] = CRGB(255, 0, 187);
leds[2] = CRGB(255, 0, 187);
leds[3] = CRGB(255, 0, 187);
leds[4] = CRGB(255, 0, 187);
leds[5] = CRGB(255, 0, 187);
leds[6] = CRGB(255, 0, 187);
leds[7] = CRGB(255, 0, 187);
leds[8] = CRGB(255, 0, 187);
leds[9] = CRGB(255, 0, 187);
leds[10] = CRGB(255, 0, 187);
leds[11] = CRGB(255, 0, 187);
leds[12] = CRGB(255, 0, 187);
leds[13] = CRGB(255, 0, 187);
leds[14] = CRGB(255, 0, 187);
leds[15] = CRGB(255, 0, 187);
leds[16] = CRGB(255, 0, 187);
leds[17] = CRGB(255, 0, 187);
leds[18] = CRGB(255, 161, 0);
leds[19] = CRGB(255, 161, 0);
leds[20] = CRGB(255, 161, 0);
leds[21] = CRGB(255, 161, 0);
leds[22] = CRGB(255, 161, 0);
leds[23] = CRGB(255, 0, 187);
leds[24] = CRGB(255, 0, 187);
leds[25] = CRGB(255, 0, 187);
leds[26] = CRGB(255, 0, 187);
leds[27] = CRGB(255, 0, 187);
leds[28] = CRGB(255, 161, 0);
leds[29] = CRGB(255, 161, 0);
leds[30] = CRGB(255, 161, 0);
leds[31] = CRGB(255, 161, 0);
leds[32] = CRGB(255, 161, 0);
leds[33] = CRGB(255, 0, 187);
leds[34] = CRGB(255, 0, 187);
leds[35] = CRGB(255, 161, 0);
leds[36] = CRGB(0, 255, 255);
leds[37] = CRGB(0, 255, 255);
leds[38] = CRGB(0, 255, 255);
leds[39] = CRGB(255, 161, 0);
leds[40] = CRGB(255, 0, 187);
leds[41] = CRGB(255, 0, 187);
leds[42] = CRGB(255, 161, 0);
leds[43] = CRGB(0, 255, 255);
leds[44] = CRGB(0, 255, 255);
leds[45] = CRGB(0, 255, 255);
leds[46] = CRGB(255, 161, 0);
leds[47] = CRGB(255, 0, 187);
leds[48] = CRGB(255, 0, 187);
leds[49] = CRGB(255, 161, 0);
leds[50] = CRGB(0, 255, 255);
leds[51] = CRGB(0, 255, 255);
leds[52] = CRGB(0, 255, 255);
leds[53] = CRGB(255, 161, 0);
leds[54] = CRGB(255, 0, 187);
leds[55] = CRGB(255, 0, 187);
leds[56] = CRGB(255, 161, 0);
leds[57] = CRGB(0, 255, 255);
leds[58] = CRGB(0, 255, 255);
leds[59] = CRGB(0, 255, 255);
leds[60] = CRGB(255, 161, 0);
leds[61] = CRGB(255, 0, 187);
leds[62] = CRGB(255, 0, 187);
leds[63] = CRGB(255, 161, 0);
leds[64] = CRGB(255, 161, 0);
leds[65] = CRGB(255, 161, 0);
leds[66] = CRGB(255, 161, 0);
leds[67] = CRGB(255, 161, 0);
leds[68] = CRGB(255, 0, 187);
leds[69] = CRGB(255, 0, 187);
leds[70] = CRGB(255, 161, 0);
leds[71] = CRGB(255, 161, 0);
leds[72] = CRGB(255, 161, 0);
leds[73] = CRGB(255, 161, 0);
leds[74] = CRGB(255, 161, 0);
leds[75] = CRGB(255, 0, 187);
leds[76] = CRGB(255, 0, 187);
leds[77] = CRGB(255, 0, 187);
leds[78] = CRGB(255, 0, 187);
leds[79] = CRGB(255, 0, 187);
leds[80] = CRGB(255, 0, 187);
leds[81] = CRGB(255, 0, 187);
leds[82] = CRGB(255, 0, 187);
leds[83] = CRGB(255, 0, 187);
leds[84] = CRGB(255, 0, 187);
leds[85] = CRGB(255, 0, 187);
leds[86] = CRGB(255, 0, 187);
leds[87] = CRGB(255, 0, 187);
FastLED.show();
delay(90);
FastLED.clear();
}
break;

case '4':

leds[2] = CRGB(0, 183, 255);
leds[3] = CRGB(0, 183, 255);
leds[13] = CRGB(0, 183, 255);
leds[14] = CRGB(0, 183, 255);
leds[17] = CRGB(0, 183, 255);
leds[18] = CRGB(0, 183, 255);
leds[19] = CRGB(0, 183, 255);
leds[20] = CRGB(0, 183, 255);
leds[21] = CRGB(0, 183, 255);
leds[28] = CRGB(0, 183, 255);
leds[29] = CRGB(0, 183, 255);
leds[30] = CRGB(0, 183, 255);
leds[31] = CRGB(0, 183, 255);
leds[32] = CRGB(0, 183, 255);
leds[34] = CRGB(0, 183, 255);
leds[36] = CRGB(0, 183, 255);
leds[37] = CRGB(0, 183, 255);
leds[42] = CRGB(0, 183, 255);
leds[44] = CRGB(0, 183, 255);
leds[45] = CRGB(0, 183, 255);
leds[48] = CRGB(0, 183, 255);
leds[49] = CRGB(0, 183, 255);
leds[50] = CRGB(0, 183, 255);
leds[51] = CRGB(0, 183, 255);
leds[52] = CRGB(0, 183, 255);
leds[53] = CRGB(0, 183, 255);
leds[56] = CRGB(0, 183, 255);
leds[57] = CRGB(0, 183, 255);
leds[58] = CRGB(0, 183, 255);
leds[59] = CRGB(0, 183, 255);
leds[60] = CRGB(0, 183, 255);
leds[64] = CRGB(0, 183, 255);
leds[65] = CRGB(0, 183, 255);
leds[67] = CRGB(0, 183, 255);
leds[72] = CRGB(0, 183, 255);
leds[73] = CRGB(0, 183, 255);
leds[75] = CRGB(0, 183, 255);
leds[77] = CRGB(0, 183, 255);
leds[78] = CRGB(0, 183, 255);
leds[79] = CRGB(0, 183, 255);
leds[80] = CRGB(0, 183, 255);
leds[81] = CRGB(0, 183, 255);
leds[83] = CRGB(0, 183, 255);
leds[84] = CRGB(0, 183, 255);
leds[85] = CRGB(0, 183, 255);
leds[86] = CRGB(0, 183, 255);
leds[87] = CRGB(0, 183, 255);
FastLED.show();
delay(3000);
FastLED.clear();

break;
case '5':
for(int i=0;i<4;i++)
{
 static uint8_t hue;
  for(int i = 0; i < NUM_LEDS/2; i++) {   
    // fade everything out
    leds.fadeToBlackBy(40);

    // let's set an led value
    leds[i] = CHSV(hue++,255,255);

    // now, let's first 20 leds to the top 20 leds, 
    leds(NUM_LEDS/2,NUM_LEDS-1) = leds(NUM_LEDS/2 - 1 ,0);
    FastLED.delay(33);
  }
}
  break;

case '6':
{
  for(int i=0;i<3;i++)
  {
    static uint8_t hue = 0;
  Serial.print("x");
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(hue++, 255, 255);
    FastLED.show();
    fadeall();
    delay(10);
  }
  Serial.print("x");
  for(int i = (NUM_LEDS)-1; i >= 0; i--) {
    leds[i] = CHSV(hue++, 255, 255);
    FastLED.show();
    fadeall();
    delay(10);
  }
}
}
break;

case '7':

for(int i=0;i<150;i++)
{
  fill_2dnoise16(LEDS.leds(), kMatrixWidth, kMatrixHeight, kMatrixSerpentineLayout,
                octaves,x,xscale,y,yscale,v_time,
                hue_octaves,hxy,hue_scale,hxy,hue_scale,hue_time, false);

  LEDS.show();
  x += x_speed;
  y += y_speed;
  v_time += time_speed;
  hue_time += hue_speed;
}
break;
case '8':
for(int i=0;i<3000;i++)
{
    ChangePalettePeriodically();
    
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; /* motion speed */
    
    FillLEDsFromPaletteColors( startIndex);
    
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
}
break;

case '9':
  while(1)
      {
        ChangePalettePeriodically();
    
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; /* motion speed */
    
    FillLEDsFromPaletteColors( startIndex);
    
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
      }
    }
  }
}
void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
    uint8_t brightness = 255;
    
    for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        colorIndex += 3;
    }
}
void ChangePalettePeriodically()
{
    uint8_t secondHand = (millis() / 1000) % 60;
    static uint8_t lastSecond = 99;
    
    if( lastSecond != secondHand) {
        lastSecond = secondHand;
        if( secondHand ==  0)  { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
        if( secondHand == 10)  { currentPalette = RainbowStripeColors_p;   currentBlending = NOBLEND;  }
        if( secondHand == 15)  { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
        if( secondHand == 20)  { SetupPurpleAndGreenPalette();             currentBlending = LINEARBLEND; }
        if( secondHand == 25)  { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
        if( secondHand == 30)  { SetupBlackAndWhiteStripedPalette();       currentBlending = NOBLEND; }
        if( secondHand == 35)  { SetupBlackAndWhiteStripedPalette();       currentBlending = LINEARBLEND; }
        if( secondHand == 40)  { currentPalette = CloudColors_p;           currentBlending = LINEARBLEND; }
        if( secondHand == 45)  { currentPalette = PartyColors_p;           currentBlending = LINEARBLEND; }
        if( secondHand == 50)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = NOBLEND;  }
        if( secondHand == 55)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = LINEARBLEND; }
    }
}
void SetupTotallyRandomPalette()
{
    for( int i = 0; i < 16; i++) {
        currentPalette[i] = CHSV( random8(), 255, random8());
    }
}
void SetupBlackAndWhiteStripedPalette()
{
    // 'black out' all 16 palette entries...
    fill_solid( currentPalette, 16, CRGB::Black);
    // and set every fourth one to white.
    currentPalette[0] = CRGB::White;
    currentPalette[4] = CRGB::White;
    currentPalette[8] = CRGB::White;
    currentPalette[12] = CRGB::White;
    
}
void SetupPurpleAndGreenPalette()
{
    CRGB purple = CHSV( HUE_PURPLE, 255, 255);
    CRGB green  = CHSV( HUE_GREEN, 255, 255);
    CRGB black  = CRGB::Black;
    
    currentPalette = CRGBPalette16(
                                   green,  green,  black,  black,
                                   purple, purple, black,  black,
                                   green,  green,  black,  black,
                                   purple, purple, black,  black );
}
const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
{
    CRGB::Red,
    CRGB::Gray, // 'white' is too bright compared to red and blue
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Red,
    CRGB::Gray,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Blue,
    CRGB::Black,
    CRGB::Black
};
