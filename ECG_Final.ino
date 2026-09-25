// ============================================================
// Arduino UNO + AD8232 ECG + 1.8" ST7735S TFT
// Real-time ECG waveform display
// ============================================================

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>


// ============================================================
// TFT PIN DEFINITIONS
// ============================================================

#define TFT_RST   A4
#define TFT_CS    A5
#define TFT_DC    A3

// Hardware SPI:
// TFT DIN -> D11
// TFT CLK -> D13


// ============================================================
// AD8232 PIN DEFINITIONS
// ============================================================

#define ECG_PIN       A0
#define LO_PLUS_PIN   7
#define LO_MINUS_PIN  8


// ============================================================
// TFT OBJECT
// ============================================================

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);


// ============================================================
// ECG DISPLAY PARAMETERS
// ============================================================

// We want to visualize ECG values from 0 to 700
const int ECG_MIN = 0;
const int ECG_MAX = 700;

// Top part of screen reserved for title
const int GRAPH_TOP = 18;

// Landscape screen:
// width  = 160
// height = 128

int xPos = 0;
int previousY = 64;


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);


  // ----------------------------------------------------------
  // AD8232 setup
  // ----------------------------------------------------------

  pinMode(LO_PLUS_PIN, INPUT);
  pinMode(LO_MINUS_PIN, INPUT);


  // ----------------------------------------------------------
  // TFT setup
  // ----------------------------------------------------------

  tft.initR(INITR_BLACKTAB);

  // Landscape mode
  tft.setRotation(1);

  // Black background
  tft.fillScreen(ST77XX_BLACK);


  // ----------------------------------------------------------
  // Display heading
  // ----------------------------------------------------------

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);

  tft.setCursor(4, 4);
  tft.print("AD8232 ECG");


  // Horizontal separator line
  tft.drawLine(
    0,
    GRAPH_TOP - 1,
    tft.width() - 1,
    GRAPH_TOP - 1,
    ST77XX_WHITE
  );


  // Start waveform from left side
  xPos = 0;
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  int ecgValue;


  // ----------------------------------------------------------
  // Check electrode connection
  // ----------------------------------------------------------

  if ((digitalRead(LO_PLUS_PIN) == HIGH) ||
      (digitalRead(LO_MINUS_PIN) == HIGH)) {

    ecgValue = 0;
  }

  else {

    // Read ORIGINAL ECG value
    ecgValue = analogRead(ECG_PIN);
  }


  // ----------------------------------------------------------
  // Serial Monitor output
  // ----------------------------------------------------------

  Serial.print("ECG:");
  Serial.println(ecgValue);


  // ----------------------------------------------------------
  // Convert ECG value into TFT Y coordinate
  //
  // ECG value itself is NOT changed.
  // Only its screen position is calculated.
  // ----------------------------------------------------------

  int displayValue = constrain(
    ecgValue,
    ECG_MIN,
    ECG_MAX
  );


  int currentY = map(
    displayValue,
    ECG_MIN,
    ECG_MAX,
    tft.height() - 1,
    GRAPH_TOP
  );


  // ----------------------------------------------------------
  // Clear current vertical column
  // ----------------------------------------------------------

  tft.drawFastVLine(
    xPos,
    GRAPH_TOP,
    tft.height() - GRAPH_TOP,
    ST77XX_BLACK
  );


  // ----------------------------------------------------------
  // Draw ECG waveform
  // ----------------------------------------------------------

  if (xPos > 0) {

    tft.drawLine(
      xPos - 1,
      previousY,
      xPos,
      currentY,
      ST77XX_GREEN
    );
  }


  previousY = currentY;


  // ----------------------------------------------------------
  // Move to next X position
  // ----------------------------------------------------------

  xPos++;


  // ----------------------------------------------------------
  // When right edge is reached,
  // start plotting again from left
  // ----------------------------------------------------------

  if (xPos >= tft.width()) {

    xPos = 0;

    // Clear only waveform area
    tft.fillRect(
      0,
      GRAPH_TOP,
      tft.width(),
      tft.height() - GRAPH_TOP,
      ST77XX_BLACK
    );
  }


  // ----------------------------------------------------------
  // Sampling delay
  // ----------------------------------------------------------

  delay(4);
}