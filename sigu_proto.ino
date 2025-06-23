#include <Arduino.h>
#include <SdFat.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <math.h>

// ===== 引脚定义 =====
#define SD_CS         10
#define BUTTON_PIN    4
#define LED_PIN       2
#define I2S_BCLK_PIN  17
#define I2S_WS_PIN    16
#define I2S_DIN_PIN   18
#define I2S_DOUT_PIN  21
#define I2S_MCLK_RX   15
#define I2S_MCLK_TX   14
#define ADC_SPEED_CH  ADC1_CHANNEL_1

// ===== 音频参数 =====
#define SAMPLE_RATE       44100
#define CHANNELS          1
#define RECORD_BUF_SZ     8192
#define PLAY_BUF_SZ       8192
#define BLOCK_SIZE        512
#define FADE_SAMPLES      512
#define MAX_RECORD_TIME   60

// ===== 全局变量 =====
SdFat sd;
FsFile wavFile;
int32_t recBuf[RECORD_BUF_SZ / 4];
int16_t playBuf[PLAY_BUF_SZ], playOut[BLOCK_SIZE * 2];

volatile bool isRecording = false;
volatile bool isPlaying = false;
volatile uint32_t bytesRecorded = 0;

unsigned long bufStart = 0;
unsigned long fileSize = 0, dataStart = 44, maxSamples = 0;
double playhead = 0;
float speedRatio = 1.0;
float smoothSpeed = 2048.0f, speedSmooth = 0.5f;

bool txInstalled = false, sdInited = false, btnLast = false;
unsigned long recStartTime = 0, lastBtnTime = 0;

// ===== 初始化 =====
void setupADC() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_SPEED_CH, ADC_ATTEN_DB_11);
}

void setupI2S_RX() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = true
  };
  i2s_pin_config_t pins = {I2S_MCLK_RX, I2S_BCLK_PIN, I2S_WS_PIN, -1, I2S_DIN_PIN};
  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void setupI2S_TX() {
  if (txInstalled) {
    i2s_driver_uninstall(I2S_NUM_1);
    txInstalled = false;
  }
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = true
  };
  i2s_pin_config_t pins = {I2S_MCLK_TX, I2S_BCLK_PIN, I2S_WS_PIN, I2S_DOUT_PIN, -1};
  i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &pins);
  i2s_zero_dma_buffer(I2S_NUM_1);
  txInstalled = true;
}

void initSD() {
  if (!sdInited) {
    sdInited = sd.begin(SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(20)));
    Serial.printf("SD卡初始化：%s\n", sdInited ? "成功" : "失败");
  }
}

void writeWavHeader(FsFile &f, uint32_t dataLen) {
  uint32_t byteRate = SAMPLE_RATE * CHANNELS * 2;
  uint16_t blockAlign = CHANNELS * 2;
  uint32_t chunkSize = 36 + dataLen;
  uint8_t hdr[44] = {
    'R','I','F','F',
    (uint8_t)(chunkSize), (uint8_t)(chunkSize>>8), (uint8_t)(chunkSize>>16), (uint8_t)(chunkSize>>24),
    'W','A','V','E','f','m','t',' ',
    16,0,0,0, 1,0,CHANNELS,0,
    (uint8_t)(SAMPLE_RATE), (uint8_t)(SAMPLE_RATE>>8), (uint8_t)(SAMPLE_RATE>>16), (uint8_t)(SAMPLE_RATE>>24),
    (uint8_t)(byteRate), (uint8_t)(byteRate>>8), (uint8_t)(byteRate>>16), (uint8_t)(byteRate>>24),
    (uint8_t)(blockAlign), 0, 16,0,
    'd','a','t','a',
    (uint8_t)(dataLen), (uint8_t)(dataLen>>8), (uint8_t)(dataLen>>16), (uint8_t)(dataLen>>24)
  };
  f.seek(0); f.write(hdr, 44);
}

// ===== 音频处理 =====
void recordAudio() {
  size_t readBytes = 0;
  i2s_read(I2S_NUM_0, recBuf, RECORD_BUF_SZ, &readBytes, portMAX_DELAY);
  if (!readBytes) return;

  uint32_t samples = readBytes / 4;
  for (uint32_t i = 0; i < samples; i++) {
    int16_t s = recBuf[i] >> 16;
    wavFile.write((uint8_t*)&s, 2);
    bytesRecorded += 2;
  }
}

int16_t readSample(FsFile &f) {
  uint8_t b[2];
  if (f.read(b, 2) != 2) return 0;
  return (int16_t)(b[0] | (b[1] << 8));
}

void fillPlayBuffer(FsFile &f) {
  bufStart = max(0L, min((long)(playhead - PLAY_BUF_SZ/2), (long)(maxSamples - PLAY_BUF_SZ)));
  f.seek(dataStart + bufStart * 2);
  for (int i = 0; i < PLAY_BUF_SZ; i++) {
    playBuf[i] = (f.position() < fileSize) ? readSample(f) : 0;
  }
}

int16_t interpolateSamp(double pos) {
  if (pos < 0 || pos >= maxSamples-1) return 0;
  int idx = (int)floor(pos);
  double frac = pos - idx;
  if (idx < bufStart || idx >= bufStart + PLAY_BUF_SZ - 1) fillPlayBuffer(wavFile);
  int bufIdx = idx - bufStart;
  return playBuf[bufIdx] + (playBuf[bufIdx+1] - playBuf[bufIdx]) * frac;
}

void playbackLoop() {
  for (int i = 0; i < BLOCK_SIZE; i++) {
    int16_t s = interpolateSamp(playhead);
    if (playhead < FADE_SAMPLES) s *= (playhead / FADE_SAMPLES);
    if (playhead > maxSamples - FADE_SAMPLES) s *= ((maxSamples - playhead) / FADE_SAMPLES);
    playOut[2*i] = s;
    playOut[2*i+1] = s;
    playhead += speedRatio;
    if (playhead >= maxSamples) playhead = 0;
    if (playhead < 0) playhead = maxSamples - 1;
  }
  size_t w;
  i2s_write(I2S_NUM_1, playOut, sizeof(playOut), &w, portMAX_DELAY);
}

// ===== 主程序 =====
void setup() {
  Serial.begin(115200); delay(500);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  setupADC(); setupI2S_RX(); initSD();
  Serial.println("系统初始化完成");
}

void loop() {
  bool btn = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  if (btn && !btnLast && now - lastBtnTime > 300) {
    lastBtnTime = now;

    if (!isRecording && !isPlaying) {
      if (wavFile && wavFile.isOpen()) wavFile.close();
      if (sd.exists("/record.wav")) sd.remove("/record.wav");

      wavFile = sd.open("/record.wav", O_WRITE | O_CREAT | O_TRUNC);
      if (!wavFile) {
        Serial.println("创建 WAV 文件失败！");
        return;
      }

      // 🔧 每次录音前，重启 I2S_RX，清空 DMA 缓冲
      i2s_driver_uninstall(I2S_NUM_0);
      delay(100);
      setupI2S_RX();

      uint8_t empty[44] = {0}; wavFile.write(empty, 44);
      bytesRecorded = 0;
      isRecording = true;
      recStartTime = now;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("开始录音...");
    }

    else if (isRecording) {
      isRecording = false;
      writeWavHeader(wavFile, bytesRecorded);
      wavFile.close();
      delay(100);

      // ✅ 关闭 RX，准备播放用 TX
      i2s_driver_uninstall(I2S_NUM_0);
      wavFile = sd.open("/record.wav", FILE_READ);
      if (!wavFile) { Serial.println("WAV 文件打开失败"); return; }

      fileSize = wavFile.size();
      maxSamples = (fileSize - dataStart) / 2;
      setupI2S_TX();
      playhead = bufStart = 0;
      fillPlayBuffer(wavFile);
      isPlaying = true;
      Serial.printf("播放开始，共 %lu samples\n", maxSamples);
    }

    else if (isPlaying) {
      isPlaying = false;
      if (txInstalled) {
        i2s_driver_uninstall(I2S_NUM_1);
        txInstalled = false;
      }
      wavFile.close();
      digitalWrite(LED_PIN, LOW);
      Serial.println("播放结束");

      // ✅ 播放结束后，重新初始化 I2S RX，准备下一次录音
      delay(100);
      setupI2S_RX();
    }
  }
  btnLast = btn;

  if (isRecording) {
    recordAudio();
    if (now - recStartTime > MAX_RECORD_TIME * 1000) {
      isRecording = false;
      writeWavHeader(wavFile, bytesRecorded);
      wavFile.close();
      digitalWrite(LED_PIN, LOW);
      Serial.println("录音超时，已自动停止");

      // 防止未释放
      i2s_driver_uninstall(I2S_NUM_0);
    }
  }

  if (isPlaying) {
    int raw = adc1_get_raw(ADC_SPEED_CH);
    smoothSpeed = smoothSpeed * (1.0f - speedSmooth) + raw * speedSmooth;
    float s = (smoothSpeed - 2048.0f) / 2048.0f;
    speedRatio = constrain(s * 4.0f, -4.0f, 4.0f);
    playbackLoop();
  }

  static unsigned long lastBlink = 0;
  if ((isRecording && now - lastBlink > 100) || (isPlaying && now - lastBlink > 500)) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); lastBlink = now;
  }
}

