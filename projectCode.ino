// CS 362 Project 2, Marcos Echevarria, Nicholas Tryba
// Hot-Cold game program
// References: Elegoo TFT Touch Screen User Manual
//             Elegoo TFT Touch Screen Sample Code

#include <Elegoo_GFX.h>    // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

int currX1, currX2, currX3, currY1, currY2, currY3;  // Current player position

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

void setup(void) {
  Serial.begin(9600);

#ifdef USE_Elegoo_SHIELD_PINOUT
  Serial.println(F("Using Elegoo 2.8\" TFT Arduino Shield Pinout"));
#else
  Serial.println(F("Using Elegoo 2.8\" TFT Breakout Board Pinout"));
#endif

  Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());

  tft.reset();

  uint16_t identifier = tft.readID();
   if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x4535) {
    Serial.println(F("Found LGDP4535 LCD driver"));
  }else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else if(identifier==0x0101)
  {     
      identifier=0x9341;
       Serial.println(F("Found 0x9341 LCD driver"));
  }else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Elegoo 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_Elegoo_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Elegoo_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    identifier=0x9341;
  
  }

  tft.begin(identifier);
  drawGrid(WHITE);
}

void loop(void) {
  // for(uint8_t rotation=0; rotation<4; rotation++) {
  //   tft.setRotation(rotation);
  //   testText();
  //   delay(2000);
  // }
  // TODO: blinking triangle
  while (Serial.available() <= 0) {
    system("Pause");  
  }
  String userChoice = Serial.readString();
  if (userChoice.equals("right")) {
    currX1 -= 36;
    currX2 -= 36;
    currX3 -= 36;
    moveRight();
  }
}

void moveRight() {
    unsigned long start = micros();
  fillScreen();
  unsigned long t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);

  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();

  for (int i = 0; i< w; i+=36) {
    tft.drawLine(i, 0, i, h, WHITE);
  }
  for (int i = 0; i< h; i+=36) {
    tft.drawLine(0, i, w, i, WHITE);
  }
  // 108 144 180
  tft.drawTriangle(
      currX1, 144, // peak
      currX2, 180, // bottom left
      currX3, 144, // bottom right
      GREEN);
  tft.drawRect(180, 72, 36, 36, RED);
  tft.fillRect(180, 72, 36, 36, RED);
  return micros() - start;
}

// -------------------
unsigned long fillScreen() {
  unsigned long start = micros();
  tft.fillScreen(BLACK);
  return micros() - start;
}
// ------------

unsigned long drawGrid(uint16_t color) {
  unsigned long start = micros();
  fillScreen();
  unsigned long t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);

  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();

  for (int i = 0; i< w; i+=36) {
    tft.drawLine(i, 0, i, h, color);
  }
  for (int i = 0; i< h; i+=36) {
    tft.drawLine(0, i, w, i, color);
  }
  // 108 144 180
  tft.drawTriangle(  // Static player position for now
      108, 144, // peak
      126, 180, // bottom left
      144, 144, // bottom right
      GREEN);
  currX1 = 108;
  currX2 = 126;
  currX3 = 144;
  tft.drawRect(180, 72, 36, 36, RED);
  tft.fillRect(180, 72, 36, 36, RED);
  return micros() - start;
}

