#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
#define BTN_PIN 32
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;

#define NUM_SAMPLES 128
#define VREF 4.096
#define LSB (VREF / 32767.0)

int16_t samples[NUM_SAMPLES];
unsigned long timestamps[NUM_SAMPLES];
int bufIndex = 0;
bool paused = false;
bool lastButtonState = HIGH;
unsigned long lastDisplayMs = 0;
const unsigned long DISPLAY_INTERVAL = 40;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(BTN_PIN, INPUT_PULLUP);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) while (1);
  if (!ads.begin(0x48)) while (1);
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(10, 25);
  display.println("Portable Oscilloscope");
  display.display();
  delay(1000);
  for (int i = 0; i < NUM_SAMPLES; ++i) {
    samples[i] = 0;
    timestamps[i] = micros();
  }
}

void loop() {
  handleButton();
  if (!paused) takeSample();
  if (millis() - lastDisplayMs > DISPLAY_INTERVAL) {
    drawScope();
    lastDisplayMs = millis();
  }
}

void handleButton() {
  bool state = digitalRead(BTN_PIN);
  if (!state && lastButtonState) {
    paused = !paused;
    delay(180);
  }
  lastButtonState = state;
}

void takeSample() {
  static unsigned long lastMicros = 0;
  unsigned long now = micros();
  if (now - lastMicros < 1200) return;
  lastMicros = now;
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += ads.readADC_SingleEnded(0);
  samples[bufIndex] = (int16_t)(sum / 4);
  timestamps[bufIndex] = now;
  bufIndex = (bufIndex + 1) % NUM_SAMPLES;
}

float computeSampleRate() {
  unsigned long newest = timestamps[(bufIndex + NUM_SAMPLES - 1) % NUM_SAMPLES];
  int oldestIndex = bufIndex;
  unsigned long oldest = timestamps[oldestIndex];
  unsigned long dt = (newest >= oldest) ? (newest - oldest) : (ULONG_MAX - oldest + newest);
  if (dt == 0) return 0;
  return (float)(NUM_SAMPLES - 1) * 1e6f / (float)dt;
}

float estimateFrequencyFromZeroCrossings(int16_t minVal, int16_t maxVal) {
  float mid = (minVal + maxVal) * 0.5f;
  const int maxCrossings = NUM_SAMPLES / 2;
  float crossingTimes[maxCrossings]; // in seconds
  int crossCount = 0;

  int start = bufIndex; // newest index is bufIndex - 1, oldest is bufIndex
  int prevIdx = (start + NUM_SAMPLES - 1) % NUM_SAMPLES;
  int curIdx;

  for (int i = 0; i < NUM_SAMPLES; ++i) {
    curIdx = (start + i) % NUM_SAMPLES;
    int16_t v1 = samples[prevIdx];
    int16_t v2 = samples[curIdx];
    unsigned long t1 = timestamps[prevIdx];
    unsigned long t2 = timestamps[curIdx];

    if ((v1 <= mid) && (v2 > mid)) {
      if (v2 != v1) {
        float frac = (mid - v1) / (float)(v2 - v1);
        float t_cross_s = (t1 + frac * (float)(t2 - t1)) * 1e-6f;
        if (crossCount < maxCrossings) crossingTimes[crossCount++] = t_cross_s;
      }
    }
    prevIdx = curIdx;
  }

  if (crossCount < 2) return 0.0f;

  double sumPeriods = 0.0;
  int periodsFound = 0;
  for (int k = 1; k < crossCount; ++k) {
    double period = crossingTimes[k] - crossingTimes[k - 1];
    if (period > 0.0005 && period < 1.0) { // ignore absurd values (keep 0.5ms..1s)
      sumPeriods += period;
      periodsFound++;
    }
  }
  if (periodsFound == 0) return 0.0f;
  double avgPeriod = sumPeriods / periodsFound;
  if (avgPeriod <= 0.0) return 0.0f;
  return (float)(1.0 / avgPeriod);
}

void drawScope() {
  int16_t minVal = samples[0], maxVal = samples[0];
  for (int i = 1; i < NUM_SAMPLES; i++) {
    if (samples[i] < minVal) minVal = samples[i];
    if (samples[i] > maxVal) maxVal = samples[i];
  }

  float mid = (minVal + maxVal) * 0.5f;
  float vpp = (maxVal - minVal) * LSB;
  double sumSq = 0.0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    float v = (samples[i] - mid) * LSB;
    sumSq += v * v;
  }
  float vrms = sqrt(sumSq / NUM_SAMPLES);
  float dc = mid * LSB;

  float sampleRate = computeSampleRate();
  float freq = estimateFrequencyFromZeroCrossings(minVal, maxVal);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("Vpp: %.2fV   Vrms: %.2fV", vpp, vrms);

  int midY = SCREEN_HEIGHT / 2;
  display.drawFastHLine(0, midY, SCREEN_WIDTH, SSD1306_WHITE);

  int start = (bufIndex + 1) % NUM_SAMPLES;
  int prevX = 0;
  int prevIdx = start;
  float prevFraction = 0.0f;
  float span = (float)(maxVal - minVal);
  for (int i = 0; i < NUM_SAMPLES; ++i) {
    int idx = (start + i) % NUM_SAMPLES;
    float fraction = 0.0f;
    if (span != 0.0f) fraction = (samples[idx] - mid) / span;
    int x = i;
    int y = midY - (int)roundf(fraction * (float)(SCREEN_HEIGHT / 2 - 8));
    y = constrain(y, 10, SCREEN_HEIGHT - 12);
    if (i > 0) display.drawLine(prevX, constrain(prevIdx,10, SCREEN_HEIGHT-12), x, y, SSD1306_WHITE);
    prevX = x;
    prevIdx = y;
  }

  display.setCursor(0, 54);
  display.printf("Freq: %.1fHz  DC: %.2fV", freq, dc);
  if (paused) {
    display.setCursor(90, 0);
    display.print("[PAUSED]");
  }
  display.display();
}
