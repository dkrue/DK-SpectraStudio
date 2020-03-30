#include <avr/pgmspace.h>
#include <ffft.h>
#include <math.h>
#include <Bounce2.h>
#include <Adafruit_NeoPixel.h>

// NOTE: for all neopixels to work, memory usage can't be much over 1560 bytes of dynamic memory
#define pixelRingPIN 15 //(pin 15 = A1)
#define pixelRingCount 24
Adafruit_NeoPixel pixelRing = Adafruit_NeoPixel(24, 15, NEO_GRB  + NEO_KHZ800);

#define neopixelPIN 5
#define neopixelCount 31
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(neopixelCount, neopixelPIN, NEO_GRB  + NEO_KHZ800);

#define eightpixelLeftPIN 3
#define eightpixelRightPIN 6
Adafruit_NeoPixel eightpixelsL = Adafruit_NeoPixel(8, eightpixelLeftPIN, NEO_GRB  + NEO_KHZ800);
Adafruit_NeoPixel eightpixelsR = Adafruit_NeoPixel(8, eightpixelRightPIN, NEO_GRB  + NEO_KHZ800);

#ifdef __AVR_ATmega32U4__
 #define ADC_CHANNEL 7
#else
 #define ADC_CHANNEL 0
#endif

#define COLUMNS 16
#define buttonPin A2
#define buttonLEDPin A3
Bounce debouncer = Bounce(); 
uint8_t buttonMode = 0;

unsigned long previousMillis = 0;
unsigned long interval = 16000;
unsigned long keyStartMillis = 0;
uint8_t keyFrame = 0, keySpeed = 62, keyForward = 1;

uint8_t reactiveMode = 0;
uint8_t reactiveDirection = 0;
uint8_t reactiveSparkle = 0;
uint8_t blankingModulo[3] = {2,3,5};
int8_t pixelR[neopixelCount];
int8_t pixelG[neopixelCount];
int8_t pixelB[neopixelCount];
int8_t pixelSlew = 12;

int8_t eightpixelBand = 0;
int8_t eightpixelMode = 0;


int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer
volatile byte samplePos = 0;     // Buffer position counter

byte
  peak[COLUMNS],      // Peak level of each column; used for falling dots
  dotCount = 0, // Frame counter for delaying dot-falling speed
  colCount = 0; // Frame counter for storing past column data
int
  col[COLUMNS][10],   // Column levels for the prior 10 frames
  minLvlAvg[COLUMNS], // For dynamic adjustment of low & high ends of graph,
  maxLvlAvg[COLUMNS], // pseudo rolling averages for the prior few frames.
  colDiv[COLUMNS];    // Used when filtering FFT output to 16 columns

/*
These tables were arrived at through testing, modeling and trial and error,
exposing the unit to assorted music and sounds.  But there's no One Perfect
EQ Setting to Rule Them All, and the graph may respond better to some
inputs than others.  The software works at making the graph interesting,
but some columns will always be less lively than others, especially
comparing live speech against ambient music of varying genres.
*/
static const uint8_t PROGMEM
  // This is low-level noise that's subtracted from each FFT output column:
  noise[64]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
  // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
  eq[64]={
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
  // When filtering down to 16 columns, these tables contain indexes
  // and weightings of the FFT spectrum output values to use.  Not all
  // buckets are used -- the bottom-most and several at the top are
  // either noisy or out of range or generally not good for a graph.
  // DK Change: Include lowest bucket for line-in sub bass and expanded led display
  //    Interpolate column weights
  col0data[] = {  2,  0,  // # of spectrum bins to merge, index of first
    111,  51 },           // Weights for each bin
  col1data[] = {  3,  0,  // DK add
     35, 120, 11 },             
  col2data[] = {  4,  1,  // 4 bins, starting at index 1
     19, 186,  38,   2 }, // Weights for 4 bins.  Got it now?
  col3data[] = {  4,  1,  // DK add
      6,  69,  121,   6 },      
  col4data[] = {  5,  2,
     11, 156, 118,  16,   1 },
  col5data[] = {  6,  2,  // DK add
      7,  32, 156, 118,  16,   1 },     
  col6data[] = {  8,  3,
      5,  55, 165, 164,  71,  18,   4,   1 },
  col7data[] = {  8,  3,
      1,  22, 56, 111,  166,  56,   16,   5 }, // DK add
  col8data[] = { 11,  5,
      3,  24,  89, 169, 178, 114,  51,  20,   6,   2,   1 },
  col9data[] = { 11,  6,
      3,  24,  50,  85, 156, 169, 128,  28,   6,   2,   1 }, // DK add 
  col10data[] = { 17,  7,
      2,   9,  29,  70, 125, 172, 185, 162, 118, 81,
     51,  21,  10,   5,   2,   1,   1 },
  col11data[] = { 21,  9,
      2,   9,  39,  83, 125, 172, 185, 162, 118, 74, // DK add
     55,  29,  21,  14,  10,   5,   2,   1,   1 },     
  col12data[] = { 25, 11,
      1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
    174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
      4,   2,   1,   1,   1 },
  col13data[] = { 31, 13,
      1,   4,  11,  25,  49,  73, 111, 166, 185, 180, // DK add
    169, 135, 110,  77,  55,  37,  19,  10,  6,   2,
      4,   2,   1,   1,   1 },      
  col14data[] = { 37, 16,
      1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  13,   12,   12,   10,
      10,   9,   9,   8,   8,   12,   12 },
  col15data[] = { 44, 13,
      1,   2,   5,  10,  18,  30,  46,  67,  92, 118, // DK add
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  17,   16,   15,   14,
     13,   13,   12,   12},      
            
  // And then this points to the start of the data for each of the columns:
  * const colData[]  = {
    col0data, col1data, col2data, col3data,
    col4data, col5data, col6data, col7data,
    col8data, col9data, col10data, col11data,
    col12data, col13data, col14data, col15data};

void setup() {
  
  randomSeed(analogRead(A5));

  uint8_t i, j, nBins, binNum, *data;

  memset(peak, 0, sizeof(peak));
  memset(col , 0, sizeof(col));

  for(i=0; i<COLUMNS; i++) {
    minLvlAvg[i] = 0;
    maxLvlAvg[i] = 512;
    data         = (uint8_t *)pgm_read_word(&colData[i]);
    nBins        = pgm_read_byte(&data[0]) + 2;
    binNum       = pgm_read_byte(&data[1]);
    for(colDiv[i]=0, j=2; j<nBins; j++)
      colDiv[i] += pgm_read_byte(&data[j]);
  }

  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion 
  ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, use AREF pin
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
  ADCSRB = 0;                // Free run mode, no high MUX bit
  DIDR0  = 1 << ADC_CHANNEL; // Turn off digital input for ADC pin
  //TIMSK0 = 0;                // Timer0 off // DK disable

  sei(); // Enable interrupts

  pinMode(buttonLEDPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  debouncer.attach(buttonPin);
  debouncer.interval(10);

  pixelRing.setBrightness(25);
  pixelRing.begin();

  pixels.setBrightness(100);
  pixels.begin(); 

  eightpixelsL.setBrightness(40); // tame neopixel brightness
  eightpixelsL.begin();
  eightpixelsR.setBrightness(40); // tame neopixel brightness  
  eightpixelsR.begin(); 

  reactiveMode = random(16)+1;
  eightpixelMode = random(8);
}

void loop() {
  uint8_t  i, x, L, *data, nBins, binNum, weighting, c;
  uint16_t minLvl, maxLvl;
  int      level, sum;
  bool forward;

  while(ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish

  fft_input(capture, bfly_buff);   // Samples -> complex #s
  samplePos = 0;                   // Reset sample counter
  ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
  fft_execute(bfly_buff);          // Process complex data
  fft_output(bfly_buff, spectrum); // Complex -> spectrum

  // Remove noise and apply EQ levels
  for(x=0; x<FFT_N/2; x++) {
    L = pgm_read_byte(&noise[x]);
    spectrum[x] = (spectrum[x] <= L) ? 0 :
      (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
  }

  // Fill background of both eightpixel bars w/colors, then idle part of column amplitude will erase
  for(i=0; i< 8; i++) {
    switch(eightpixelMode) {
      case 0: //random rainbow
          setEightPixel(i, (i*i+1)*random(12), (i*i+2)*random(6), (i*i+2)*random(3));
      break; 
      case 1: // classic eq: green, yellow, red peak
          setEightPixel(i, (i*i+1)*(i<4 ? 12 : i<7 ? 4 : 6), (i*i+2)*(i<4 ? 6 : i<7 ? 5 : 0), 0);
      break;
      case 2: // red to blue, white peak
          setEightPixel(i, (i*i+1)*(i<4 ? 12 : i<7 ? 4 : 6), i<7 ? 0 : 250, (i*i+1)*(i<4 ? 1 : 5));
      break; 
      case 3: // alternating bright white
          setEightPixel(i, 255*(i%2), 255*(i%2), 255*(i%2));
      break;
      case 4: // alternating blue with yellow, white peak
          setEightPixel(i, i<7 ? (i*i+1)*4 : 255, i<7 ? (i*i+1)*4 : 255, (i*i+1)*4*(i%2) + (i*5));
      break;   
      case 5: // red to random double peak
          setEightPixel(i, (i*i+1)*(i<4 ? 12 : i<7 ? 4 : 6), i<6 ? 0 : random(255), i<6 ? 0 : random(255));
      break;         
      case 6: // alternating bright blue
          setEightPixel(i, 0, 0, 200*(i%2)+(i*i)+5);
      break;  
      case 7: // yellow to random peak
          setEightPixel(i, (i*i+1)*(i<4 ? 12 : i<7 ? 4 : 6), (i*i+2)*2, i<7 ? 0 : random(255));
      break;     
      case 8: // green to blue, red peak (modern mixer eq style)  
        setEightPixel(i, i<7 ? 0 : 255, (i*i+1)*(i<4 ? 10 : i<7 ? 1 : 0), (i*i+1)*(i<4 ? 1 : i<7 ? 5 : 0));
        break;     
      default:
        //eightpixelMode = 0;  // reset to zero when incrementing by one 
      break;     
    }
  }

  // Downsample spectrum output to 16 columns:
  for(x=0; x<COLUMNS; x++) {
    data   = (uint8_t *)pgm_read_word(&colData[x]);
    nBins  = pgm_read_byte(&data[0]) + 2;
    binNum = pgm_read_byte(&data[1]);
    for(sum=0, i=2; i<nBins; i++)
      sum += spectrum[binNum++] * pgm_read_byte(&data[i]); // Weighted
    col[x][colCount] = sum / colDiv[x];                    // Average
    minLvl = maxLvl = col[x][0];
    for(i=1; i<10; i++) { // Get range of prior 10 frames
      if(col[x][i] < minLvl)      minLvl = col[x][i];
      else if(col[x][i] > maxLvl) maxLvl = col[x][i];
    }
    // minLvl and maxLvl indicate the extents of the FFT output, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
    if((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
    minLvlAvg[x] = (minLvlAvg[x] * 7 + minLvl) >> 3; // Dampen min/max levels
    maxLvlAvg[x] = (maxLvlAvg[x] * 7 + maxLvl) >> 3; // (fake rolling average)

    // Second fixed-point scale based on dynamic min/max levels:
    level = 10L * (col[x][colCount] - minLvlAvg[x]) /
      (long)(maxLvlAvg[x] - minLvlAvg[x]);

    // Clip output and convert to byte:
    if(level < 0L)      c = 0;
    else if(level > 10) c = 10; // Allow dot to go a couple pixels off top
    else                c = (uint8_t)level;

    if(c > peak[x]) peak[x] = c; // Keep dot on top

    // Update single line led band    
    if(x == eightpixelBand) {
      for(i=c; i<8; i++) {
        eightpixelsL.setPixelColor(i, 0, 0, 0);
        eightpixelsR.setPixelColor(i, 0, 0, 0);        
      }
    }
  }

  eightpixelsL.show();
  eightpixelsR.show();

  // Every 2nd frame, make the peak pixels drop by 1: // every 10 for "peak mode"
  if(++dotCount >= 2) { // DK orig >= 2: smoother
    dotCount = 0;
    for(x=0; x<COLUMNS; x++) {
      if(peak[x] >= 3) peak[x]-=3; // DK orig --1 THIS CAN BE -1 for smooth mode. -3 for intense blinky mode
      else peak[x] = 0;
    }
  }

  // Store 10 frames of data for weighted average
  if(++colCount >= 10) colCount = 0;



  // Update pixel mode timer
  unsigned long currentMillis = millis();
  if(reactiveMode > 0) {
    /// Animation / blanking / chasing speed
    if((unsigned long)(currentMillis - keyStartMillis) >= interval / keySpeed) { // lower divisor = slower keyframes 62.5 spans entire 16 sec scene
      keyStartMillis = millis();
      if(keyForward == 1) {
        keyFrame++;
      } else {
        keyFrame--;
      }
    }
    
    if((unsigned long)(currentMillis - previousMillis) >= interval)  {
      reactiveMode = random(15) + 1;
      previousMillis = millis();
      keyFrame = 0; //not needed?

      reactiveDirection = random(2);
      reactiveSparkle = random(12);
      if(reactiveSparkle > 4) reactiveSparkle = 0;

      blankingModulo[0] = random(10)+1;
      if(blankingModulo[0] > 8) blankingModulo[0] = 2;

      blankingModulo[1] = random(10)+1;
      if(blankingModulo[1] > 8) blankingModulo[1] = 2;

      blankingModulo[2] = random(10)+1;
      if(blankingModulo[2] > 8) blankingModulo[2] = 3;

      keyForward = random(1);
      keySpeed = random(60 * peak[0] * (blankingModulo[0] > blankingModulo[1] ? blankingModulo[0] : blankingModulo[1]));
      if(keySpeed > 200) keySpeed = 0;      

      pixelSlew = random(24)+1;

      eightpixelBand = random(24);
      if(eightpixelBand > 15) eightpixelBand = random(1);

      // one quarter chance of a eightpixel mode changing to a new one, if on
      if(eightpixelMode > 8 || random(4) == 0) {
        eightpixelMode = random(70);
      }
    }
  }

  //for(i=0; i<8; i+=8) { // per jewel // orig <= 8
  //  for(x=0; x<=16; x+=random(4)) { // per pixel (orig. x=0, x<8, i++)
  switch(reactiveDirection) {
    case 0: // mirror towards center
      i=0; forward=true;
      break;
    case 1: // mirror away from center
      i=neopixelCount/2; forward=true;
      break;      
    case 2: // forward
      i=random(neopixelCount/2); forward=true;
      break;
    case 3: // reverse
      i=neopixelCount-1 - random(neopixelCount/2); forward=false;
      break;
  } 

  x=0;
  for(c=0; c<neopixelCount; c++) {
    // Only set value if detected, otherwise let fade out
    //if(peak[x]>4) {
      switch (reactiveMode) {
        case 0: // OFF
          setPixel(forward? i+x : i-x, 0, 0, 0); 
          break;
        case 1: // Green & blue hyper sparkle
          setPixel(forward? i+x : i-x, peak[2] % 20 * x, peak[0] % 20 * x, peak[4] * x * 5);
          break;          
        case 2: // Full color flower
          setPixel(forward? i+x : i-x, peak[x] > 3 ? 255 : peak[1] * x * 2, peak[3] > 4 ? 255 : peak[x] * x * 3, peak[3] > 4 ? 255 : peak[x] * x * 3);    
          break;    
        case 3: // Blue green pulsar
          setPixel(forward? i+x : i-x, peak[1] > 7 ? peak[3] * 20 : peak[x] * x * 2, (x % 2) * 10 * peak[x], peak[0] * (x+1) * 3);
          break;      
        case 4 :// Purple berry pulsar
          setPixel(forward? i+x : i-x, peak[0] * 25, 0, peak[0] * (x+1) * 3);
          break;      
        case 5: // Red & white sparkle raspberry
          setPixel(forward? i+x : i-x, x%2 == 0 ? peak[2] * 25 : 0, peak[x] % 2 * 50, peak[x] % 2 * 50);
          break;      
        case 6: // Pink flicker
          setPixel(forward? i+x : i-x, peak[x] * peak[x] * 3, 0, peak[x] * x * 3);     
          break;
        case 7: // Green & white intensity
          setPixel(forward? i+x : i-x, (x % 2) * peak[0], peak[x] * (x+1), (x % 2) * peak[0]);
          break;
        case 8: // High velocity blue & white super blast
          setPixel(forward? i+x : i-x, peak[0] > 7 ? 255 : peak[0] > 6 ? 127 : 0, peak[0] > 7 ? 255 : peak[0] > 6 ? 127 : 0, peak[x] * x * 2);
          break;  
        case 9: // White bass threshold bass flash, red undertones
          setPixel(forward? i+x : i-x, (x % 2) * peak[0] * 2, 0, peak[1] * (x+1) * 3);    
          break;
        case 10: // Blueberry brilliance
          setPixel(forward? i+x : i-x, 0, 0, peak[2] * (x%4) * peak[0]);
          break;
        case 11: // Green & blue white soft bass blast
          setPixel(forward? i+x : i-x, peak[x] * peak[x] * 2, peak[x%2] * x * 2, peak[x%2] * x * 2);
          break;
        case 12: // Green ninja turtle blast
          setPixel(forward? i+x : i-x, x%2 == 0 ? peak[9]*3 : 0, peak[x] * 2 * peak[0], x%2 == 0 ? peak[9]*3 : 0);
          break;
        case 13: // Phantom color flower
          setPixel(forward? i+x : i-x, peak[x] * x * peak[2], peak[x] > 4 ? 255 : peak[x] * x * 2, peak[x] * peak[1] + peak[0]);
          break;
        case 14: // Random rainbow color sparkle
          setPixel(forward? i+x : i-x, peak[random(0,3)] % 2 * random(0,256), peak[random(0,2)] % 2 * random(0,100), peak[random(0,3)] % 2 * random(0,256));
          break;    
        case 15: // Purple stomp bass flash
          setPixel(forward? i+x : i-x, peak[0] * ((4-x) + 4) * (peak[0] / 2) + peak[8] % x, 0, peak[0] * ((4-x) + 4) * (peak[0] / 2) + peak[x]);
          break;                    
      }
    //}

    // Fade out when quiet
    if(pixelR[c] > 0) pixelR[c]--;
    if(pixelG[c] > 0) pixelG[c]--;
    if(pixelB[c] > 0) pixelB[c]--;
    pixels.setPixelColor(c, pixelR[c], pixelG[c], pixelB[c]);
    pixelRing.setPixelColor(c, pixelR[c], pixelG[c], pixelB[c]);

    // Set next pixel
    if(reactiveSparkle > 1) {
      x+=random(reactiveSparkle)+1;
    } else {
      x++;
    }

    // Flip mirror mode half way through pixel array
    if(reactiveDirection == 0 && forward == true && x>=neopixelCount/2) {
      i=neopixelCount-1;
      forward=false;
      x=0;
    }
    if(reactiveDirection == 1 && forward == true && x>=neopixelCount) {
      i=(neopixelCount/2) - 1;
      forward=false;
      x=0;
    }    

    if(x > COLUMNS) {
      x=-COLUMNS;
    }
  }
  //}

  if(blankingModulo[0] > 1) {
    for(c=0; c<neopixelCount; c++) {
      if(c % blankingModulo[0] == 0) {
        pixels.setPixelColor((c+keyFrame) % neopixelCount, 0, 0, 0);
      }
    }
    if(random(32000/(keySpeed+1)) == 0) {
      keyForward=random(1);
      keySpeed = random(60 * peak[0] * (blankingModulo[0] > blankingModulo[1] ? blankingModulo[0] : blankingModulo[1]));
      if(keySpeed > 200) keySpeed = 0;
    }
  }
  if(blankingModulo[1] > 1) {
    for(c=0; c<neopixelCount; c++) {
      if(c % blankingModulo[1] == 0) { 
        pixels.setPixelColor((c+keyFrame) % neopixelCount, 0, 0, 0);
        pixelRing.setPixelColor((c+keyFrame) % neopixelCount, 32, 16, 12); // neutral white       
      }
    }
  }  
  if(blankingModulo[2] > 1) {
    for(c=0; c<neopixelCount; c++) {
      if(c % blankingModulo[2] == 0) {
        pixels.setPixelColor((c+keyFrame) % neopixelCount, 0, 0, 0);
        pixelRing.setPixelColor((c+keyFrame) % neopixelCount, 32, 16, 12); // neutral white
      }
    }
  } 

  pixels.show();
  pixelRing.show();

  // Update the debounced button state
  debouncer.update();

  // Onboard LED button press
  if(debouncer.fell()) {
    if(buttonMode == 0) {
      buttonMode = 1;
      digitalWrite(buttonLEDPin, HIGH);
    } else {
      buttonMode = 0;
      digitalWrite(buttonLEDPin, LOW);
    }
  }
}

// Set pixel softened with slew amount to emulate high power lamps
void setPixel(uint8_t pixel, int8_t red, int8_t green, int8_t blue) {
  
  // Don't refresh pixel if value is the same so it can fade out
  if(pixelR[pixel] == red && pixelG[pixel] == green && pixelB[pixel] == blue) 
    return;
  
  pixelR[pixel] -= (pixelR[pixel] - red) / pixelSlew;
  pixelG[pixel] -= (pixelG[pixel] - green) / pixelSlew;
  pixelB[pixel] -= (pixelB[pixel] - blue) / pixelSlew;

  pixels.setPixelColor(pixel, pixelR[pixel], pixelG[pixel], pixelB[pixel]);
  pixelRing.setPixelColor(pixel, pixelR[pixel], pixelG[pixel], pixelB[pixel]);
}

void setEightPixel(uint8_t pixel, int8_t red, int8_t green, int8_t blue) {
  eightpixelsL.setPixelColor(pixel, red, green, blue);
  eightpixelsR.setPixelColor(pixel, red, green, blue); 
}

ISR(ADC_vect) { // Audio-sampling interrupt
  static const int16_t noiseThreshold = 4;
  int16_t              sample         = ADC; // 0-1023

  capture[samplePos] =
    ((sample > (512-noiseThreshold)) &&
     (sample < (512+noiseThreshold))) ? 0 :
    sample - 512; // Sign-convert for FFT; -512 to +511

  if(++samplePos >= FFT_N) ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
}
