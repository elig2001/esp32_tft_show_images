//  background light pin
#define ENABLE_SCREEN  \
  pinMode(27, OUTPUT); \
  digitalWrite(27, HIGH);

#include <Arduino.h>
// #include "1.h"
// #include "2.h"
// #include "3.h"
// #include "4.h"
#include "5.h"
// #include "6.h"
#include <TFT_eSPI.h>

// JPEG decoder library
#include <JPEGDecoder.h>

// Return the minimum of two values a and b
#define minimum(a, b) (((a) < (b)) ? (a) : (b))
void draw_image(const uint32_t *image_as_array, uint32_t image_length, uint16_t image_height, uint16_t image_width);
void jpegInfo();
void drawArrayJpeg(const uint8_t arrayname[], uint32_t array_size, int xpos, int ypos);
void renderJPEG(int xpos, int ypos);

TFT_eSPI tft = TFT_eSPI();

// put function declarations here:

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting...");

  // Initialise the screen
  tft.init();

  // Ideally set orientation for good viewing angle range because
  // the anti-aliasing effectiveness varies with screen viewing angle
  // g_tft.setRotation();

  tft.fillScreen(TFT_BLACK);

  ENABLE_SCREEN
  delay(2000);

  tft.setRotation(0); // portrait
  tft.fillScreen(TFT_BLACK);

  // drawArrayJpeg(image, sizeof(image), 0, 0); // Draw a jpeg image stored in memory at x,y
  // drawArrayJpeg(EagleEye, sizeof(EagleEye), 0, 0); // Draw a jpeg image stored in memory at x,y
  drawArrayJpeg(aa, sizeof(aa), 0, 0); // Draw a jpeg image stored in memory at x,y
  // drawArrayJpeg(bb, sizeof(bb), 0, 0); // Draw a jpeg image stored in memory at x,y
  // draw_image(bb, sizeof(bb), 340, 174);
}

void loop()
{
}

void draw_image(const uint32_t *image_as_array, uint32_t image_length, uint16_t image_height, uint16_t image_width)
{
  // uint8_t red = 0;
  // uint8_t green = 0;
  // uint8_t blue = 0;
  // uint32_t color = 0;
  for (size_t height = 0; height < image_height; height++)
  {
    for (size_t width = 0; width < image_width; width++)
    {
      // blue = (image_as_array[height + width] & 0b11) << 6;
      // red = (image_as_array[height + width] & 0b11100000);
      // green = (image_as_array[height + width] & 0b00011100) << 3;
      // color = (red << 16) | (green << 8) |  blue;
      // tft.drawPixel(width, height, tft.color16to24(tft.color8to16(image_as_array[height + width])));
      tft.drawPixel(width, height, image_as_array[width + height]);

      // tft.drawPixel(width, height, color);

      // tft.drawPixel(width, height, tft.color565(red, green, blue));
      // tft.color8to16()
      // tft.color
      // Serial.printf("x - %lu, y - %lu (%lu, %lu, %lu)\n",width, height, red,green,blue );
    }
  }
}

// ####################################################################################################
//  Draw a JPEG on the TFT pulled from a program memory array
// ####################################################################################################
void drawArrayJpeg(const uint8_t arrayname[], uint32_t array_size, int xpos, int ypos)
{

  int x = xpos;
  int y = ypos;

  JpegDec.decodeArray(arrayname, array_size);

  jpegInfo(); // Print information from the JPEG file (could comment this line out)

  renderJPEG(x, y);

  Serial.println("#########################");
}

// ####################################################################################################
//  Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
// ####################################################################################################
//  This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
//  fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void renderJPEG(int xpos, int ypos)
{

  // retrieve information about the image
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // read each MCU block until there are no more
  while (JpegDec.readSwappedBytes())
  {

    // save a pointer to the image block
    pImg = JpegDec.pImage;

    // calculate where the image block should be drawn on the screen
    int mcu_x = JpegDec.MCUx * mcu_w + xpos; // Calculate coordinates of top left corner of current MCU
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x)
      win_w = mcu_w;
    else
      win_w = min_w;

    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y)
      win_h = mcu_h;
    else
      win_h = min_h;

    // copy pixels into a contiguous block
    if (win_w != mcu_w)
    {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++)
      {
        p += mcu_w;
        for (int w = 0; w < win_w; w++)
        {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    // draw image MCU block only if it will fit on the screen
    if ((mcu_x + win_w) <= tft.width() && (mcu_y + win_h) <= tft.height())
    {
      tft.pushRect(mcu_x, mcu_y, win_w, win_h, pImg);
    }
    else if ((mcu_y + win_h) >= tft.height())
      JpegDec.abort(); // Image has run off bottom of screen so abort decoding
  }

  // calculate how long it took to draw the image
  drawTime = millis() - drawTime;

  // print the results to the serial port
  Serial.print(F("Total render time was    : "));
  Serial.print(drawTime);
  Serial.println(F(" ms"));
  Serial.println(F(""));
}

// ####################################################################################################
//  Print image information to the serial port (optional)
// ####################################################################################################
void jpegInfo()
{
  Serial.println(F("==============="));
  Serial.println(F("JPEG image info"));
  Serial.println(F("==============="));
  Serial.print(F("Width      :"));
  Serial.println(JpegDec.width);
  Serial.print(F("Height     :"));
  Serial.println(JpegDec.height);
  Serial.print(F("Components :"));
  Serial.println(JpegDec.comps);
  Serial.print(F("MCU / row  :"));
  Serial.println(JpegDec.MCUSPerRow);
  Serial.print(F("MCU / col  :"));
  Serial.println(JpegDec.MCUSPerCol);
  Serial.print(F("Scan type  :"));
  Serial.println(JpegDec.scanType);
  Serial.print(F("MCU width  :"));
  Serial.println(JpegDec.MCUWidth);
  Serial.print(F("MCU height :"));
  Serial.println(JpegDec.MCUHeight);
  Serial.println(F("==============="));
}
