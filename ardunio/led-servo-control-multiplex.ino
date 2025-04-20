#include <Servo.h>

#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
#include <avr/power.h> 
#endif

#define LED_PIN    2
#define LED_COUNT 145
const unsigned int ESC_SIGNAL_STOP = 1500;
const unsigned long BAUD_RATE = 115200;

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

byte PIN_ESC_BR = 40;
byte PIN_ESC_FR = 42;
byte PIN_ESC_BL = 46;
byte PIN_ESC_FL = 44;

Servo motor_br;
Servo motor_fr;
Servo motor_bl;
Servo motor_fl;

const byte ESC_DATA_MAXLEN = 32;
char esc_data[ESC_DATA_MAXLEN];
char esc_buffer[ESC_DATA_MAXLEN]; // temporary array for use when parsing

//hold the parsed data
int input_br = 0;
int input_fr = 0;
int input_bl = 0;
int input_fl = 0;

void setup() {
    Serial.begin(BAUD_RATE);
    motor_br.attach(PIN_ESC_BR);
    motor_fr.attach(PIN_ESC_FR);
    motor_bl.attach(PIN_ESC_BL);
    motor_fl.attach(PIN_ESC_FL);

    // Send "stop" signal
    motor_br.writeMicroseconds(ESC_SIGNAL_STOP);
    motor_fr.writeMicroseconds(ESC_SIGNAL_STOP);
    motor_bl.writeMicroseconds(ESC_SIGNAL_STOP);
    motor_fl.writeMicroseconds(ESC_SIGNAL_STOP);

    #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
    #endif

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();            // Turn OFF all pixels ASAP
    strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

    delay(5000); // Allow ESCs to recognize signal and settle down
    colorWipe(strip.Color(0,0,255),5); // Blue
}

void loop() {
  if (recvWithStartEndMarkers()) {
    strcpy(esc_data, esc_buffer);
    parseData();
    sendData();
  }
}

const char ESC_DATA_START = '<', ESC_DATA_END = '>';
bool recvWithStartEndMarkers() {
  static byte i = 0;
  char recieved;

  while (
    Serial.available() > 0 
    && Serial.read() != ESC_DATA_START
  );

  if (Serial.available() <= 0) {
    return false;
  }

  while (
    Serial.available() > 0 
    && (recieved = Serial.read()) != ESC_DATA_END
  ) {
    if (i < ESC_DATA_MAXLEN)
      esc_buffer[i++] = recieved;
  }

  if (Serial.available() <= 0) {
    return false;
  }

  esc_buffer[i] = '\0'; // terminate the string
  i = 0;

  return true;
}

// split the data into its parts
void parseData() {
    char* i; // this is used by strtok() as an index

    i = strtok(esc_data, ","); // get the first part
    input_br = i != NULL ? atoi(i) : ESC_SIGNAL_TOP;
 
    i = strtok(NULL, ","); // continue where it left off
    input_fr = i != NULL ? atoi(i) : ESC_SIGNAL_STOP;
    
    i = strtok(NULL, ",");
    input_bl = i != NULL ? atoi(i) : ESC_SIGNAL_STOP;
    
    i = strtok(NULL, ",");
    input_fl = i != NULL ? atoi(i) : ESC_SIGNAL_STOP;
}

void sendData(){
  // Send signals to ESCs
  motor_br.writeMicroseconds(input_br);
  motor_fr.writeMicroseconds(input_fr); 
  motor_bl.writeMicroseconds(input_bl); 
  motor_fl.writeMicroseconds(input_fl);
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int hue = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
