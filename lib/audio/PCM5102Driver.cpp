#include "PCM5102Driver.h"
#include <math.h>

#ifndef I2S_COMM_FORMAT_STAND_I2S
#define I2S_COMM_FORMAT_STAND_I2S I2S_COMM_FORMAT_I2S
#endif

void PCM5102Driver::begin(uint32_t sampleRateHz) {
  _sampleRate = sampleRateHz;

  // Install I2S driver
  i2s_config_t i2s_cfg = {};
  i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_cfg.sample_rate = (int)_sampleRate;
  i2s_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2s_cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT; // stereo
  i2s_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_cfg.dma_buf_count = 8;
  i2s_cfg.dma_buf_len = 256;
  #ifdef CONFIG_IDF_TARGET_ESP32
  i2s_cfg.use_apll = true;           // use APLL for exact clocks
  i2s_cfg.tx_desc_auto_clear = true; // clear DMA on underflow
  i2s_cfg.fixed_mclk = 0;
  #endif

  i2s_driver_install(_port, &i2s_cfg, 0, nullptr);

  i2s_pin_config_t pin_cfg = {};
  pin_cfg.bck_io_num = _pin_bck;
  pin_cfg.ws_io_num = _pin_lrck;   // LRCK
  pin_cfg.data_out_num = _pin_data;
  pin_cfg.data_in_num = I2S_PIN_NO_CHANGE;
  i2s_set_pin(_port, &pin_cfg);
  // Ensure clock matches our intended SR and format
  i2s_set_clk(_port, _sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  i2s_zero_dma_buffer(_port);
  i2s_start(_port);

  // Writer task (high priority, core 0)
  xTaskCreatePinnedToCore(writerTaskThunk, "pcm5102_tx", 4096, this, 6, &_task, 0);
}

void IRAM_ATTR PCM5102Driver::writeFromISR(uint8_t sample8) {
  // Convert 8-bit centered to 16-bit signed, duplicate to stereo (little-endian)
  int16_t s = (int16_t(int(sample8) - 128) << 8);
  uint8_t b0 = uint8_t(s & 0xFF);
  uint8_t b1 = uint8_t((s >> 8) & 0xFF);
  // Check free space to avoid overflow
  size_t h = _head;
  size_t t = _tail;
  size_t used = h - t;
  if (used > RB_SIZE - 8) {
    // drop sample to protect continuity
    _dropsFromISR++;
    return;
  }
  // push 4 bytes: L0,L1,R0,R1
  _rb[h & (RB_SIZE - 1)] = b0; h++;
  _rb[h & (RB_SIZE - 1)] = b1; h++;
  _rb[h & (RB_SIZE - 1)] = b0; h++;
  _rb[h & (RB_SIZE - 1)] = b1; h++;
  _head = h;
  // Wake writer task promptly to reduce latency
  if (_task) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(_task, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
  }
}

void PCM5102Driver::write(uint8_t sample8) {
  // Convert 8-bit centered to 16-bit signed, duplicate to stereo (little-endian)
  int16_t s = (int16_t(int(sample8) - 128) << 8);
  uint8_t b0 = uint8_t(s & 0xFF);
  uint8_t b1 = uint8_t((s >> 8) & 0xFF);
  // Check free space to avoid overflow
  size_t h = _head;
  size_t t = _tail;
  size_t used = h - t;
  if (used > RB_SIZE - 8) {
    return;
  }
  // push 4 bytes: L0,L1,R0,R1
  _rb[h & (RB_SIZE - 1)] = b0; h++;
  _rb[h & (RB_SIZE - 1)] = b1; h++;
  _rb[h & (RB_SIZE - 1)] = b0; h++;
  _rb[h & (RB_SIZE - 1)] = b1; h++;
  _head = h;
  if (_task) xTaskNotifyGive(_task);
}

void PCM5102Driver::writerTask() {
  const size_t CHUNK = 2048; // bytes per write to I2S (~8 ms @48kHz)
  uint8_t buf[CHUNK];
  for (;;) {
    if (_sineActive) {
      // Generate pure sine directly in the writer task
      const size_t frames = CHUNK / 4; // stereo 16-bit
      const float twoPi = 6.28318530717958647692f;
      const float dph = twoPi * _sineFreq / (float)_sampleRate;
      const int16_t amp = (int16_t)(fmaxf(0.0f, fminf(_sineAmp, 1.0f)) * 32767.0f);
      size_t idx = 0;
      for (size_t i = 0; i < frames; ++i) {
        int16_t s = (int16_t)(sinf(_sinePhase) * amp);
        buf[idx++] = (uint8_t)(s & 0xFF);
        buf[idx++] = (uint8_t)((s >> 8) & 0xFF);
        buf[idx++] = (uint8_t)(s & 0xFF);
        buf[idx++] = (uint8_t)((s >> 8) & 0xFF);
        _sinePhase += dph;
        if (_sinePhase >= twoPi) _sinePhase -= twoPi;
      }
      size_t written = 0;
      i2s_write(_port, buf, CHUNK, &written, portMAX_DELAY);
      continue;
    } else if (_cb) {
      // Pull-mode: ask external source for samples in this clock domain
      const size_t frames = CHUNK / 4;
      size_t idx = 0;
      for (size_t i = 0; i < frames; ++i) {
        uint8_t s8 = _cb(_cbCtx);
        int16_t s = (int16_t(int(s8) - 128) << 8);
        buf[idx++] = (uint8_t)(s & 0xFF);
        buf[idx++] = (uint8_t)((s >> 8) & 0xFF);
        buf[idx++] = (uint8_t)(s & 0xFF);
        buf[idx++] = (uint8_t)((s >> 8) & 0xFF);
      }
      size_t written = 0;
      i2s_write(_port, buf, CHUNK, &written, portMAX_DELAY);
      continue;
    } else {
      // Wait for data or timeout
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(4));

      // snapshot available
      size_t h = _head;
      size_t t = _tail;
      size_t available = h - t; // modulo 2^n since RB_SIZE is power of 2
      if (available == 0) {
        // keep link alive with silence
        int16_t s = 0;
        uint8_t frame[4] = { uint8_t(s & 0xFF), uint8_t((s >> 8) & 0xFF), uint8_t(s & 0xFF), uint8_t((s >> 8) & 0xFF) };
        size_t written = 0;
        i2s_write(_port, frame, sizeof(frame), &written, pdMS_TO_TICKS(2));
        vTaskDelay(pdMS_TO_TICKS(1));
        _underruns++;
        continue;
      }

      // Track available range
      if (available < _minAvail) _minAvail = available;
      if (available > _maxAvail) _maxAvail = available;

      size_t to_copy = (available > CHUNK) ? CHUNK : available;
      // Maintain whole frames (4 bytes per sample: 16-bit stereo)
      to_copy &= ~size_t(3);
      if (to_copy == 0) {
        // not enough data for a whole frame; feed a frame of silence
        int16_t s = 0;
        uint8_t frame[4] = { uint8_t(s & 0xFF), uint8_t((s >> 8) & 0xFF), uint8_t(s & 0xFF), uint8_t((s >> 8) & 0xFF) };
        size_t written = 0;
        i2s_write(_port, frame, sizeof(frame), &written, pdMS_TO_TICKS(2));
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }
      // copy with wrap handling
      for (size_t i = 0; i < to_copy; ++i) {
        buf[i] = _rb[(t + i) & (RB_SIZE - 1)];
      }
      _tail = t + to_copy;

      size_t written = 0;
      i2s_write(_port, buf, to_copy, &written, portMAX_DELAY);
    }
  }
}

void PCM5102Driver::resetStats() {
  _dropsFromISR = 0;
  _underruns = 0;
  _minAvail = (size_t)-1;
  _maxAvail = 0;
}

void PCM5102Driver::getStats(uint32_t& dropsFromISR, uint32_t& underruns, size_t& minAvail, size_t& maxAvail) const {
  dropsFromISR = _dropsFromISR;
  underruns = _underruns;
  minAvail = _minAvail;
  maxAvail = _maxAvail;
}

void PCM5102Driver::testMinimal(uint32_t freqHz, float seconds, float amplitude) {
  // Suspend writer to avoid concurrent writes
  if (_task) vTaskSuspend(_task);
  i2s_zero_dma_buffer(_port);

  const uint32_t totalSamples = (uint32_t)((double)_sampleRate * (double)seconds);
  const size_t framesPerChunk = 512; // stereo frames per chunk
  const size_t bytesPerChunk = framesPerChunk * 4; // 16-bit stereo
  uint8_t buf[bytesPerChunk];

  const float twoPi = 6.28318530717958647692f;
  const float dph = twoPi * (float)freqHz / (float)_sampleRate;
  const int16_t amp = (int16_t)(fmaxf(0.0f, fminf(amplitude, 1.0f)) * 32767.0f);
  float phase = 0.0f;

  uint32_t generated = 0;
  while (generated < totalSamples) {
    size_t frames = framesPerChunk;
    if (generated + frames > totalSamples) frames = totalSamples - generated;
    size_t idx = 0;
    for (size_t i = 0; i < frames; ++i) {
      int16_t s = (int16_t)(sinf(phase) * amp);
      // little-endian L then R
      buf[idx++] = (uint8_t)(s & 0xFF);
      buf[idx++] = (uint8_t)((s >> 8) & 0xFF);
      buf[idx++] = (uint8_t)(s & 0xFF);
      buf[idx++] = (uint8_t)((s >> 8) & 0xFF);
      phase += dph;
      if (phase >= twoPi) phase -= twoPi;
    }
    size_t written = 0;
    i2s_write(_port, buf, idx, &written, portMAX_DELAY);
    (void)written;
    generated += (uint32_t)frames;
  }

  // small tail of silence
  uint8_t z[4] = {0,0,0,0};
  size_t w = 0;
  i2s_write(_port, z, sizeof(z), &w, pdMS_TO_TICKS(10));

  if (_task) vTaskResume(_task);
}

void PCM5102Driver::startPureSine(uint32_t freqHz, float amplitude) {
  _sineFreq = (float)freqHz;
  _sineAmp = fmaxf(0.0f, fminf(amplitude, 1.0f));
  _sinePhase = 0.0f;
  i2s_zero_dma_buffer(_port);
  _sineActive = true;
  if (_task) xTaskNotifyGive(_task);
}

void PCM5102Driver::stopPureSine() {
  _sineActive = false;
  i2s_zero_dma_buffer(_port);
}
