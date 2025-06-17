#include <SdFat.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <math.h>

// 硬件配置
#define SD_CS         5
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRCK      26
#define SPEED_PIN     34      // 速度控制引脚(ADC1_CH6)

// ADC配置
#define ADC_WIDTH     ADC_WIDTH_BIT_12
#define ADC_ATTEN     ADC_ATTEN_DB_11  // 0-3.3V测量范围
#define ADC_DEADZONE  2     // 中心死区范围(0-4095)
#define ADC_READ_INTERVAL 2  // ADC读取间隔(ms)

// 音频参数 - 增大缓冲区
#define SAMPLE_RATE   44100
#define BUFFER_SIZE   32768   // 增大缓冲区 (32K样本)
#define BLOCK_SIZE    256
#define FADE_SAMPLES  512

SdFat sd;
FsFile wavFile;

// 音频缓冲区
int16_t audioBuffer[BUFFER_SIZE];
unsigned long bufferStart = 0;
unsigned int bufferFill = 0;

double playhead = 0.0;        // 播放头精度改为double
float speedRatio = 0.0f;
float volume = 1.0f;
bool isPlaying = false;

unsigned long fileSize = 0;
unsigned long dataStart = 0;
unsigned long maxSamples = 0;

int16_t outputBlock[BLOCK_SIZE * 2];

// 文件读取缓冲区 - 增大缓冲区
uint8_t fileBuffer[2048];    // 增大文件缓冲区 (2KB)
unsigned int fileBufferPos = 0;
unsigned int fileBufferSize = 0;

// ADC平滑处理变量
float smoothedSpeedValue = 2048.0f; // 初始中值
const float smoothingFactor = 0.1f; // 平滑系数(0.0-1.0)

void setupADC() {
  adc1_config_width(ADC_WIDTH);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN); // GPIO34对应ADC1_CH6
}

float readSpeedRaw() {
  static unsigned long lastReadTime = 0;
  static float filteredValue = 2048.0f; // 初始中值
  
  if (millis() - lastReadTime >= ADC_READ_INTERVAL) {
    lastReadTime = millis();
    
    // 快速读取3次取中值
    int raw1 = adc1_get_raw(ADC1_CHANNEL_6);
    int raw2 = adc1_get_raw(ADC1_CHANNEL_6);
    int raw3 = adc1_get_raw(ADC1_CHANNEL_6);
    
    // 取中值
    int rawValue = (raw1 + raw2 + raw3) / 3;
    
    // 一阶低通滤波
    filteredValue = filteredValue * (1.0f - smoothingFactor) + rawValue * smoothingFactor;
    
    // 更新平滑值
    smoothedSpeedValue = filteredValue;
  }
  
  return smoothedSpeedValue;
}

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 32,
    .dma_buf_len = 256,
    .use_apll = true,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

bool initSDandOpenWav() {
  if (!sd.begin(SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(10)))) {
    Serial.println("SD卡初始化失败!");
    sd.initErrorHalt(&Serial);
    return false;
  }

  wavFile = sd.open("/sound.wav", O_READ);
  if (!wavFile) {
    Serial.println("无法打开 sound.wav");
    return false;
  }
  
  fileSize = wavFile.size();
  
  if (fileSize > 44) {
    dataStart = 44;
    wavFile.seek(dataStart);
    maxSamples = (fileSize - dataStart) / 2;
  } else {
    Serial.println("文件太小，不是有效的WAV文件");
    return false;
  }
  
  return true;
}

int16_t readSample() {
  if (fileBufferPos >= fileBufferSize) {
    fileBufferSize = wavFile.read(fileBuffer, sizeof(fileBuffer));
    fileBufferPos = 0;
    if (fileBufferSize == 0) {
      if (wavFile.available()) {
        wavFile.seek(dataStart); // 循环时回到文件开头
        fileBufferSize = wavFile.read(fileBuffer, sizeof(fileBuffer));
      } else {
        return 0;
      }
    }
  }
  
  uint8_t low = fileBuffer[fileBufferPos++];
  uint8_t high = fileBuffer[fileBufferPos++];
  return (int16_t)((high << 8) | low);
}

void fillBuffer() {
  double bufferPos = playhead - bufferStart;  // 使用double精度
  
  // 增大缓冲区后降低填充频率
  if (bufferPos < BUFFER_SIZE / 8 || bufferPos > BUFFER_SIZE * 7/8 || 
      playhead < bufferStart || playhead >= bufferStart + bufferFill) {
    
    bufferStart = playhead - BUFFER_SIZE / 2;
    
    // 处理循环缓冲区的边界条件
    if (bufferStart < 0) {
      bufferStart += maxSamples; // 从末尾开始
    } else if (bufferStart >= maxSamples) {
      bufferStart -= maxSamples; // 回到开头
    }
    
    bufferStart = constrain(bufferStart, 0, maxSamples - 1);
    
    wavFile.seek(dataStart + bufferStart * 2);
    
    bufferFill = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      uint32_t samplePos = bufferStart + i;
      
      // 处理循环位置
      if (samplePos >= maxSamples) {
        samplePos -= maxSamples;
        if (i == 0) wavFile.seek(dataStart); // 回到文件开头
      }
      
      audioBuffer[i] = readSample();
      bufferFill++;
    }
  }
}

// 改进的播放头边界处理 - 确保循环播放
void handlePlayheadBoundary() {
  // 使用模运算确保播放头在有效范围内
  if (playhead >= maxSamples) {
    playhead = fmod(playhead, maxSamples);
    // 当播放头循环时，可能需要刷新缓冲区
    bufferFill = 0;
    wavFile.seek(dataStart); // 回到文件开头
  } 
  else if (playhead < 0) {
    playhead = maxSamples + fmod(playhead, maxSamples);
    // 当播放头循环时，可能需要刷新缓冲区
    bufferFill = 0;
    wavFile.seek(dataStart + (maxSamples - 1) * 2); // 跳到文件末尾
  }
}

void updateSpeedRatio() {
  float rawValue = readSpeedRaw(); // 读取经过平滑处理的ADC值
  
  // 将0-4095映射到-1.0到+1.0，中心有死区
  if (rawValue < 2048 - ADC_DEADZONE) {
    speedRatio = -1.0 * (1.0 - rawValue/(2048 - ADC_DEADZONE));
  } else if (rawValue > 2048 + ADC_DEADZONE) {
    speedRatio = (rawValue - (2048 + ADC_DEADZONE)) / (4095 - (2048 + ADC_DEADZONE));
  } else {
    speedRatio = 0.0;
  }
  
  // 应用指数曲线使控制更加平滑
  if (speedRatio > 0) {
    speedRatio = pow(speedRatio, 1.5);
  } else if (speedRatio < 0) {
    speedRatio = -pow(-speedRatio, 1.5);
  }
  
  speedRatio = constrain(speedRatio * 2.0, -2.0, 2.0);
  
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf("速度: %.3f | ADC值: %.1f | 播放头: %.1f/%lu\n", 
                 speedRatio, rawValue, playhead, maxSamples);
  }
}

int16_t interpolate(double position) {  // 参数改为double
  // 确保位置在有效范围内
  if (position < 0 || position >= maxSamples) {
    // 当位置超出范围时，先处理边界
    handlePlayheadBoundary();
    position = playhead; // 使用处理后的位置
  }
  
  double bufferPos = position - bufferStart;  // 使用double精度
  if (bufferPos < 0 || bufferPos >= bufferFill - 1) {
    fillBuffer();
    bufferPos = position - bufferStart;
    // 如果仍然超出范围，返回0
    if (bufferPos < 0 || bufferPos >= bufferFill - 1) {
      return 0;
    }
  }
  
  unsigned int idx = (unsigned int)bufferPos;
  double frac = bufferPos - idx;  // 使用double精度
  
  if (idx >= BUFFER_SIZE - 1) {
    idx = BUFFER_SIZE - 2;
    frac = 1.0;
  }
  
  int16_t s1 = audioBuffer[idx];
  int16_t s2 = audioBuffer[idx + 1];
  
  return s1 + (s2 - s1) * frac;
}

void flushI2SSilence(int times = 10) {
  int16_t silence[BLOCK_SIZE * 2] = {0};
  size_t bytesWritten;
  for (int i = 0; i < times; i++) {
    i2s_write(I2S_NUM_0, silence, sizeof(silence), &bytesWritten, portMAX_DELAY);
  }
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // 初始化ADC
  setupADC();
  
  Serial.println("初始化SD卡...");
  if (!initSDandOpenWav()) {
    Serial.println("SD卡初始化失败，请检查:");
    Serial.println("1. SD卡格式(FAT32/exFAT)");
    Serial.println("2. 文件是否存在(sound.wav)");
    Serial.println("3. 硬件连接(CS:5)");
    while(1);
  }

  // 初始化优化 - 填充完整缓冲区
  Serial.println("填充音频缓冲区...");
  bufferStart = 0;
  wavFile.seek(dataStart);
  bufferFill = 0;
  
  // 确保填满整个缓冲区
  for (int i = 0; i < BUFFER_SIZE; i++) {
    audioBuffer[i] = readSample();
    bufferFill++;
  }
  
  // 设置播放头在缓冲区中心
  playhead = BUFFER_SIZE / 2;
  
  setupI2S();
  flushI2SSilence();

  Serial.println("系统初始化完成");
  Serial.println("电位器控制: 左=倒放 中=暂停 右=正放");
  Serial.printf("音频文件: %lu 采样点\n", maxSamples);
  Serial.printf("缓冲区大小: %d 采样 (%.1f秒)\n", BUFFER_SIZE, (float)BUFFER_SIZE/SAMPLE_RATE);
}

void loop() {
  updateSpeedRatio();
  
  // 降低填充频率 - 仅在需要时填充
  static unsigned long lastFillTime = 0;
  if (millis() - lastFillTime > 20) { // 每20ms检查一次
    fillBuffer();
    lastFillTime = millis();
  }
  
  // 删除淡出功能，直接停止
  if (abs(speedRatio) < 0.01) {
    if (isPlaying) {
      isPlaying = false;
      flushI2SSilence(); // 立即静音
    }
    delay(1);
    return;
  }
  
  if (!isPlaying) {
    isPlaying = true;
    volume = 0.0f; // 保留淡入功能
  }
  volume = min(1.0f, volume + 0.01f);

  for (int i = 0; i < BLOCK_SIZE; i++) {
    int16_t sample = interpolate(playhead);
    
    // 应用淡入和音量
    sample = (int16_t)(sample * volume);
    
    // 边界淡入淡出
    if (playhead < FADE_SAMPLES) {
      sample = (int16_t)(sample * (playhead / FADE_SAMPLES));
    } else if (playhead > maxSamples - FADE_SAMPLES) {
      sample = (int16_t)(sample * ((maxSamples - playhead) / FADE_SAMPLES));
    }
    
    outputBlock[i * 2] = sample;
    outputBlock[i * 2 + 1] = sample;

    playhead += speedRatio;
    handlePlayheadBoundary(); // 处理播放头边界
  }

  size_t bytesWritten;
  esp_err_t err = i2s_write(I2S_NUM_0, outputBlock, sizeof(outputBlock), &bytesWritten, portMAX_DELAY);
  
  if (err != ESP_OK) {
    Serial.println("I2S写入错误，尝试恢复...");
    setupI2S();
  }
}