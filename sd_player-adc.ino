#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>

#define SD_CS         5
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRCK      26

#define BUFFER_SIZE   1024
#define BLOCK_SAMPLES 256

File wavFile;
int16_t audioBufferA[BUFFER_SIZE / 2];
int16_t audioBufferB[BUFFER_SIZE / 2];

int samplesInBufferA = 0;
int samplesInBufferB = 0;
bool useBufferA = true;
bool wavFinished = false;

float playhead = 0.0f;
float speedRatio = 1.0f;

int16_t outBlock[BLOCK_SAMPLES * 2]; // stereo output

Adafruit_ADS1115 ads;
unsigned long lastAdcRead = 0;
const int adcInterval = 50;

#define I2S_NUM       I2S_NUM_0
#define FIXED_SAMPLE_RATE 22050

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = FIXED_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM);
}

bool initSDandOpenWav() {
  if (!SD.begin(SD_CS)) return false;
  wavFile = SD.open("/sound.wav");
  if (!wavFile) return false;
  wavFile.seek(44); // skip WAV header
  return true;
}

int readAudioBlock(int16_t* dest) {
  if (!wavFile || !wavFile.available()) return 0;
  uint8_t temp[BUFFER_SIZE];
  int bytesRead = wavFile.read(temp, BUFFER_SIZE);
  for (int i = 0; i < bytesRead / 2; i++) {
    dest[i] = temp[i * 2] | (temp[i * 2 + 1] << 8);
  }
  return bytesRead / 2;
}

void updateSpeedRatio() {
  if (millis() - lastAdcRead > adcInterval) {
    lastAdcRead = millis();
    int16_t adc = ads.readADC_SingleEnded(0);
    float voltage = ads.computeVolts(adc);
    voltage = constrain(voltage, 0.0, 4.096);
    speedRatio = 0.25 + (voltage / 4.096) * 2.0; // 0.25x ~ 2.25x
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();
  ads.setGain(GAIN_ONE);
  if (!ads.begin()) while (1);
  if (!initSDandOpenWav()) while (1);

  setupI2S();

  samplesInBufferA = readAudioBlock(audioBufferA);
  samplesInBufferB = readAudioBlock(audioBufferB);

  Serial.println("初始化完成，开始播放...");
}

void loop() {
  if (wavFinished) {
    delay(10);
    return;
  }

  updateSpeedRatio();

  int16_t* curBuf = useBufferA ? audioBufferA : audioBufferB;
  int samplesInCur = useBufferA ? samplesInBufferA : samplesInBufferB;

  for (int i = 0; i < BLOCK_SAMPLES; ++i) {
    float idx = playhead;
    int i0 = (int)idx;
    float frac = idx - i0;

    // 边界保护
    if (i0 < 0 || i0 >= samplesInCur - 1) {
      outBlock[2 * i] = 0;
      outBlock[2 * i + 1] = 0;
      continue;
    }

    int16_t s1 = curBuf[i0];
    int16_t s2 = curBuf[i0 + 1];
    int32_t interp = s1 + (int32_t)((s2 - s1) * frac);
    if (interp > 32767) interp = 32767;
    if (interp < -32768) interp = -32768;
    int16_t o = (int16_t)interp;

    outBlock[2 * i] = o;
    outBlock[2 * i + 1] = o;

    playhead += speedRatio;

    // 切换缓冲区逻辑
    while (playhead >= samplesInCur - 1) {
      playhead -= samplesInCur;
      useBufferA = !useBufferA;

      if (useBufferA) samplesInBufferA = readAudioBlock(audioBufferA);
      else            samplesInBufferB = readAudioBlock(audioBufferB);

      if ((useBufferA && samplesInBufferA == 0) || (!useBufferA && samplesInBufferB == 0)) {
        wavFinished = true;
        wavFile.close();
        break;
      }

      curBuf = useBufferA ? audioBufferA : audioBufferB;
      samplesInCur = useBufferA ? samplesInBufferA : samplesInBufferB;

      if (playhead >= samplesInCur - 1) {
        playhead = samplesInCur - 2; // 强行拉回
      }
    }
  }

  size_t bytesWritten = 0;
  i2s_write(I2S_NUM, outBlock, sizeof(outBlock), &bytesWritten, portMAX_DELAY);
}
