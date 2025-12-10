/*
 * Audio Delay Effect for File Playback using Teensy 4.1
 * Version 1.0
 *
 * Author: Rafael Sabe
 * Email: rafaelmsabe@gmail.com
 */

/*
 * DESCRIPTION AND WIRING:
 * 
 * 20x4 Alphanumeric LCD display.
 * 4 push buttons
 * 1 switch button
 * 
 * Micro SD Card Slot
 * I2S Codec.
 * 
 * LCD will show information about the feedforward and feedback delay time (in number of samples) and amplitude.
 * 
 * 4 push buttons:
 * DELAYDEC: Decrement selected delay time
 * DELAYINC: Increment selected delay time
 * AMPDEC: Decrement selected delay amplitude
 * AMPINC: Increment selected delay amplitude
 * 
 * switch button:
 * OPEN: Select feedforward delay.
 * CLOSED: Select feedback delay.
 * 
 * WIRING:
 * 
 * LCD display:
 * DB4: Pin 34
 * DB5: Pin 35
 * DB6: Pin 36
 * DB7: Pin 37
 * RS: Pin 38
 * E: Pin 41
 * 
 * 4 push buttons:
 * DELAYDEC: Pin 24 and GND
 * DELAYINC: Pin 25 and GND
 * AMPDEC: Pin 26 and GND
 * AMPINC: Pin 27 and GND
 * 
 * switch button: Pin 28 and GND
 * 
 * Micro SD Card: Teensy board built in slot.
 * 
 * I2S Codec: I2S Port 2
 * 
 * DATAOUT: Pin 2
 * WORDCLK: Pin 3
 * BITCLK: Pin 4
 * DATAIN: Pin 5 (unused)
 * MASTERCLK: Pin 33 (unused)
 * 
 * (My codec has internal master clock generator, so I didn't use MCLK. If you wish to use MCLK, uncomment the GPIO_33 pin assignment in "hw_init_i2s_dma()")
 */

#include "audiodelaypb_def.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include <util.h>
#include <lcd.hpp>
#include <SD.h>
#include <DMAChannel.h>

#include "AudioDelay.hpp"

#define LCD_DB4 34U
#define LCD_DB5 35U
#define LCD_DB6 36U
#define LCD_DB7 37U
#define LCD_RS 38U

#define LCD1_E 41U
#define LCD1_NCHARS 20U
#define LCD1_NLINES 4U

#define BUTTON_FFFB_SEL_PIN 28U
#define BUTTON_DELAYDEC_PIN 24U
#define BUTTON_DELAYINC_PIN 25U
#define BUTTON_AMPDEC_PIN 26U
#define BUTTON_AMPINC_PIN 27U

#define UI_DELAYVAL_MIN 0
#define UI_DELAYVAL_MAX 16000

#define UI_AMPVAL_MIN -1.0F
#define UI_AMPVAL_MAX 1.0F

#define UI_AMP_STEP 0.05F

#define LOOP_DELAYTIME_MS 8U

/*
 * REMINDER: I2S PORT 2 PINS:
 * 
 * PIN 2: DATA OUT
 * PIN 3: LRCLK
 * PIN 4: BCLK
 * PIN 5: DATA IN
 * PIN 33: MCLK
 */

#define __ERROR_GENERIC -1
#define __ERROR_NOFILE -2
#define __ERROR_FILENOTSUPPORTED -3
#define __ERROR_FORMATNOTSUPPORTED -4
#define __ERROR_BROKENHEADER -5
#define __ERROR_SDHW -6

#define FILEIN_DIR ("TestFile.wav")

__attribute__((aligned(PTR_SIZE_BITS))) uintptr_t AUDIO_SAMPLERATE = 0u;
__attribute__((aligned(PTR_SIZE_BITS))) uint64_t AUDIO_DATABEGIN = 0u;
__attribute__((aligned(PTR_SIZE_BITS))) uint64_t AUDIO_DATAEND = 0u;

__attribute__((aligned(PTR_SIZE_BITS))) void (*p_bufferload)(void) = NULL;

__attribute__((aligned(PTR_SIZE_BITS))) LCD lcd1(LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7, LCD_RS, LCD1_E, LCD1_NCHARS, LCD1_NLINES);
__attribute__((aligned(PTR_SIZE_BITS))) DMAChannel dma_i2s;

__attribute__((aligned(PTR_SIZE_BITS))) FsBaseFile filein_obj;
__attribute__((aligned(PTR_SIZE_BITS))) uint64_t filein_size = 0u;
__attribute__((aligned(PTR_SIZE_BITS))) uint64_t filein_pos = 0u;

__attribute__((aligned(PTR_SIZE_BITS))) uintptr_t dspbuffer_nseg = 0u;
__attribute__((aligned(PTR_SIZE_BITS))) uintptr_t bufferoutput_nseg_load = 0u;
__attribute__((aligned(PTR_SIZE_BITS))) uintptr_t bufferoutput_nseg_play = 0u;

__attribute__((aligned(32))) int16_t p_bufferinput[AUDIOBUFFER_SIZE_SAMPLES]; /*This buffer is a lot bigger than it has to be, but this is good for doing format conversion.*/
__attribute__((aligned(32))) int16_t p_bufferoutput[AUDIOBUFFER_SIZE_SAMPLES];
__attribute__((aligned(32))) int16_t* pp_bufferoutput_segments[AUDIOBUFFER_N_SEGMENTS] = {NULL};

__attribute__((aligned(PTR_SIZE_BITS))) AudioDelay auddelay;

extern void sys_stop(intptr_t stop_code, const char *stop_msg) __attribute__((__noreturn__)) FLASHMEM;

extern void sd_deinit(void) FLASHMEM;

extern bool filein_open(void) FLASHMEM;
extern void filein_close(void) FLASHMEM;

extern intptr_t filein_get_params(void) FLASHMEM;
extern bool compare_signature(const char *auth, const uint8_t *buf) FLASHMEM;

extern void buffer_init(void) FLASHMEM;

extern void playback_init(void) FLASHMEM;

extern void bufferload_proc(void) FLASHMEM;

extern void _buffer_load_i16_1ch(void) FLASHMEM;
extern void _buffer_load_i16_2ch(void) FLASHMEM;
extern void _buffer_load_i24_1ch(void) FLASHMEM;
extern void _buffer_load_i24_2ch(void) FLASHMEM;

extern void buffer_loadout(void) FLASHMEM;

extern void buffer_segment_update(void) FLASHMEM;

extern void dma_i2s_isr(void);

extern void hw_init_i2s_dma(void) FLASHMEM;
extern void hw_deinit_i2s_dma(void) FLASHMEM;

extern void lcd_data_update(void) FLASHMEM;
extern void button_wait_release(uint8_t pin) FLASHMEM;

void setup(void)
{
  intptr_t n_ret = 0;
  
  lcd1.begin();
  auddelay.begin();

  pinMode(BUTTON_DELAYDEC_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DELAYINC_PIN, INPUT_PULLUP);
  pinMode(BUTTON_AMPDEC_PIN, INPUT_PULLUP);
  pinMode(BUTTON_AMPINC_PIN, INPUT_PULLUP);
  pinMode(BUTTON_FFFB_SEL_PIN, INPUT_PULLUP);

  if(!SD.begin(BUILTIN_SDCARD)) sys_stop(__ERROR_SDHW, "Err: SD failed");

  if(!filein_open()) sys_stop(__ERROR_NOFILE, "Err: No File");

  n_ret = filein_get_params();
  if(n_ret < 0) sys_stop(n_ret, textbuf);

  buffer_init();
  playback_init();

  lcd_data_update();
  return;
}

void loop(void)
{
  intptr_t n_delay;
  float f_amp;
  bool ff_fb_sel;
  
_l_loop_runtimeloop:

  if(!digitalRead(BUTTON_DELAYDEC_PIN))
  {
    ff_fb_sel = !((bool) digitalRead(BUTTON_FFFB_SEL_PIN));

    if(ff_fb_sel) n_delay = (intptr_t) auddelay.getFBDelay();
    else n_delay = (intptr_t) auddelay.getFFDelay();

    if(n_delay <= 500) n_delay -= 10;
    else if(n_delay <= 1000) n_delay -= 50;
    else if(n_delay <= 5000) n_delay -= 100;
    else if(n_delay <= 10000) n_delay -= 500;
    else n_delay -= 1000;

    if(n_delay < UI_DELAYVAL_MIN) n_delay = UI_DELAYVAL_MIN;

    if(ff_fb_sel) auddelay.setFBDelay((uintptr_t) n_delay);
    else auddelay.setFFDelay((uintptr_t) n_delay);

    lcd_data_update();
    button_wait_release(BUTTON_DELAYDEC_PIN);
  }

  if(!digitalRead(BUTTON_DELAYINC_PIN))
  {
    ff_fb_sel = !((bool) digitalRead(BUTTON_FFFB_SEL_PIN));

    if(ff_fb_sel) n_delay = (intptr_t) auddelay.getFBDelay();
    else n_delay = (intptr_t) auddelay.getFFDelay();

    if(n_delay >= 10000) n_delay += 1000;
    else if(n_delay >= 5000) n_delay += 500;
    else if(n_delay >= 1000) n_delay += 100;
    else if(n_delay >= 500) n_delay += 50;
    else n_delay += 10;

    if(n_delay > UI_DELAYVAL_MAX) n_delay = UI_DELAYVAL_MAX;

    if(ff_fb_sel) auddelay.setFBDelay((uintptr_t) n_delay);
    else auddelay.setFFDelay((uintptr_t) n_delay);

    lcd_data_update();
    button_wait_release(BUTTON_DELAYINC_PIN);
  }

  if(!digitalRead(BUTTON_AMPDEC_PIN))
  {
    ff_fb_sel = !((bool) digitalRead(BUTTON_FFFB_SEL_PIN));

    if(ff_fb_sel) f_amp = auddelay.getFBAmp();
    else f_amp = auddelay.getFFAmp();

    f_amp -= UI_AMP_STEP;
    if(f_amp < UI_AMPVAL_MIN) f_amp = UI_AMPVAL_MIN;

    if(ff_fb_sel) auddelay.setFBAmp(f_amp);
    else auddelay.setFFAmp(f_amp);
    
    lcd_data_update();
    button_wait_release(BUTTON_AMPDEC_PIN);
  }

  if(!digitalRead(BUTTON_AMPINC_PIN))
  {
    ff_fb_sel = !((bool) digitalRead(BUTTON_FFFB_SEL_PIN));

    if(ff_fb_sel) f_amp = auddelay.getFBAmp();
    else f_amp = auddelay.getFFAmp();

    f_amp += UI_AMP_STEP;
    if(f_amp > UI_AMPVAL_MAX) f_amp = UI_AMPVAL_MAX;

    if(ff_fb_sel) auddelay.setFBAmp(f_amp);
    else auddelay.setFFAmp(f_amp);
    
    lcd_data_update();
    button_wait_release(BUTTON_AMPINC_PIN);
  }

  delay(LOOP_DELAYTIME_MS);
  goto _l_loop_runtimeloop;
  return;
}

FLASHMEM void __attribute__((__noreturn__)) sys_stop(intptr_t stop_code, const char *stop_msg)
{
  sd_deinit();
  hw_deinit_i2s_dma();

  lcd1.clear();
  lcd1.setCursorPosition(0u, 0u);
  lcd1.printText("SYSTEM STOPPED");

  snprintf(textbuf, TEXTBUF_SIZE_CHARS, "STOP CODE: %d", stop_code);
  lcd1.setCursorPosition(0u, 1u);
  lcd1.printText(textbuf);

  if(stop_msg != NULL)
  {
    lcd1.setCursorPosition(0u, 2u);
    lcd1.printText(stop_msg);
  }
  
_l_sys_stop_loop:

  delay(4096u);
  goto _l_sys_stop_loop;
}

FLASHMEM void sd_deinit(void)
{
  filein_close();
  SD.sdfs.end();
  return;
}

FLASHMEM bool filein_open(void)
{
  filein_close();

  filein_obj = SD.sdfs.open(FILEIN_DIR, O_RDONLY);
  if(!filein_obj.isOpen()) return false;

  filein_size = filein_obj.fileSize();
  return true;
}

FLASHMEM void filein_close(void)
{
  filein_obj.close();
  filein_size = 0u;
  return;
}

FLASHMEM intptr_t filein_get_params(void)
{
  const uintptr_t BUFFER_SIZE = 4096u;
  uintptr_t buffer_index = 0u;

  uint8_t p_headerinfo[BUFFER_SIZE];

  uint32_t u32;
  uint16_t u16;

  uint16_t bit_depth;
  uint16_t n_channels;

  memset(p_headerinfo, 0, BUFFER_SIZE);

  filein_obj.seekSet(0u);
  filein_obj.read(p_headerinfo, BUFFER_SIZE);

  if(!compare_signature("RIFF", p_headerinfo))
  {
    snprintf(textbuf, TEXTBUF_SIZE_CHARS, "Err: File Inv");
    return __ERROR_FILENOTSUPPORTED;
  }

  if(!compare_signature("WAVE", (const uint8_t*) (((uintptr_t) p_headerinfo) + 8u)))
  {
    snprintf(textbuf, TEXTBUF_SIZE_CHARS, "Err: File Inv");
    return __ERROR_FILENOTSUPPORTED;
  }

  buffer_index = 12u;

  while(true)
  {
    if(buffer_index > (BUFFER_SIZE - 8u))
    {
      snprintf(textbuf, TEXTBUF_SIZE_CHARS, "Err: Brk Head");
      return __ERROR_BROKENHEADER;
    }

    if(compare_signature("fmt ", (const uint8_t*) (((uintptr_t) p_headerinfo) + buffer_index))) break;

    u32 = *((uint32_t*) (((uintptr_t) p_headerinfo) + buffer_index + 4u));
    buffer_index += (uintptr_t) (u32 + 8u);
  }

  if(buffer_index > (BUFFER_SIZE - 24u))
  {
    snprintf(textbuf, TEXTBUF_SIZE_CHARS, "Err: Brk Head");
    return __ERROR_BROKENHEADER;
  }

  u16 = *((uint16_t*) (((uintptr_t) p_headerinfo) + buffer_index + 8u));
  if(u16 != 1u)
  {
    snprintf(textbuf, TEXTBUF_SIZE_CHARS, "Err: Bad Fmt");
    return __ERROR_FORMATNOTSUPPORTED;
  }

  n_channels = *((uint16_t*) (((uintptr_t) p_headerinfo) + buffer_index + 10u));

  AUDIO_SAMPLERATE = (uintptr_t) *((uint32_t*) (((uintptr_t) p_headerinfo) + buffer_index + 12u));

  bit_depth = *((uint16_t*) (((uintptr_t) p_headerinfo) + buffer_index + 22u));

  u32 = *((uint32_t*) (((uintptr_t) p_headerinfo) + buffer_index + 4u));
  buffer_index += (uintptr_t) (u32 + 8u);

  while(true)
  {
    if(buffer_index > (BUFFER_SIZE - 8u))
    {
      snprintf(textbuf, TEXTBUF_SIZE_CHARS, "Err: Brk Head");
      return __ERROR_BROKENHEADER;
    }

    if(compare_signature("data", (const uint8_t*) (((uintptr_t) p_headerinfo) + buffer_index))) break;

    u32 = *((uint32_t*) (((uintptr_t) p_headerinfo) + buffer_index + 4u));
    buffer_index += (uintptr_t) (u32 + 8u);
  }

  u32 = *((uint32_t*) (((uintptr_t) p_headerinfo) + buffer_index + 4u));

  AUDIO_DATABEGIN = (uint64_t) (buffer_index + 8u);
  AUDIO_DATAEND = AUDIO_DATABEGIN + ((uint64_t) u32);

  if((bit_depth == 16u) && (n_channels == 1u))
  {
    p_bufferload = &_buffer_load_i16_1ch;
    return 0;
  }

  if((bit_depth == 16u) && (n_channels == 2u))
  {
    p_bufferload = &_buffer_load_i16_2ch;
    return 0;
  }

  if((bit_depth == 24u) && (n_channels == 1u))
  {
    p_bufferload = &_buffer_load_i24_1ch;
    return 0;
  }

  if((bit_depth == 24u) && (n_channels == 2u))
  {
    p_bufferload = &_buffer_load_i24_2ch;
    return 0;
  }

  return __ERROR_FORMATNOTSUPPORTED;
}

FLASHMEM bool compare_signature(const char *auth, const uint8_t *buf)
{
  if(auth == NULL) return false;
  if(buf == NULL) return false;

  if(auth[0] != ((char) buf[0])) return false;
  if(auth[1] != ((char) buf[1])) return false;
  if(auth[2] != ((char) buf[2])) return false;
  if(auth[3] != ((char) buf[3])) return false;

  return true;
}

FLASHMEM void buffer_init(void)
{
  uintptr_t n_seg = 0u;
  
  memset(p_bufferinput, 0, AUDIOBUFFER_SIZE_BYTES);
  memset(p_bufferoutput, 0, AUDIOBUFFER_SIZE_BYTES);

  for(n_seg = 0u; n_seg < AUDIOBUFFER_N_SEGMENTS; n_seg++) pp_bufferoutput_segments[n_seg] = (int16_t*) (((uintptr_t) p_bufferoutput) + n_seg*AUDIOBUFFER_SEGMENT_SIZE_BYTES);
  
  return;
}

FLASHMEM void playback_init(void)
{
  dspbuffer_nseg = 0u;
  bufferoutput_nseg_load = 0u;
  bufferoutput_nseg_play = AUDIOBUFFER_N_SEGMENTS/2u;

  filein_pos = AUDIO_DATABEGIN;

  bufferload_proc();
  hw_init_i2s_dma();

  return;
}

FLASHMEM void bufferload_proc(void)
{
  (*p_bufferload)();
  auddelay.runDSP(dspbuffer_nseg);
  buffer_loadout();
  
  return;
}

FLASHMEM void _buffer_load_i16_1ch(void)
{
  uintptr_t n_frame;
  uintptr_t n_sample;
  float *p_dspbuf_segin = NULL;

  float factor;
  float f32;

  p_dspbuf_segin = auddelay.getInputBufferSegment(dspbuffer_nseg);
  if(p_dspbuf_segin == NULL) sys_stop(__ERROR_GENERIC, "audiodelay error");
  
  memset(p_bufferinput, 0, AUDIOBUFFER_SIZE_BYTES);

  if(filein_pos >= AUDIO_DATAEND) filein_pos = AUDIO_DATABEGIN;

  filein_obj.seekSet(filein_pos);
  filein_obj.read(p_bufferinput, (AUDIOBUFFER_SEGMENT_SIZE_FRAMES*2u));
  filein_pos += (AUDIOBUFFER_SEGMENT_SIZE_FRAMES*2u);

  factor = AUDIO_SAMPLEFACTOR;

  n_sample = 0u;
  for(n_frame = 0u; n_frame < AUDIOBUFFER_SEGMENT_SIZE_FRAMES; n_frame++)
  {
    f32 = (float) p_bufferinput[n_frame];
    f32 /= factor;
    p_dspbuf_segin[n_sample] = f32;
    p_dspbuf_segin[n_sample + 1u] = f32;

    n_sample += 2u;
  }
  
  return;
}

FLASHMEM void _buffer_load_i16_2ch(void)
{
  uintptr_t n_sample;
  float *p_dspbuf_segin = NULL;

  float factor;
  float f32;

  p_dspbuf_segin = auddelay.getInputBufferSegment(dspbuffer_nseg);
  if(p_dspbuf_segin == NULL) sys_stop(__ERROR_GENERIC, "audiodelay error");
  
  memset(p_bufferinput, 0, AUDIOBUFFER_SIZE_BYTES);

  if(filein_pos >= AUDIO_DATAEND) filein_pos = AUDIO_DATABEGIN;

  filein_obj.seekSet(filein_pos);
  filein_obj.read(p_bufferinput, AUDIOBUFFER_SEGMENT_SIZE_BYTES);
  filein_pos += AUDIOBUFFER_SEGMENT_SIZE_BYTES;

  factor = AUDIO_SAMPLEFACTOR;

  for(n_sample = 0u; n_sample < AUDIOBUFFER_SEGMENT_SIZE_SAMPLES; n_sample++)
  {
    f32 = (float) p_bufferinput[n_sample];
    f32 /= factor;
    p_dspbuf_segin[n_sample] = f32;
  }
  
  return;
}

FLASHMEM void _buffer_load_i24_1ch(void)
{
  uintptr_t n_frame;
  uintptr_t n_sample;
  uintptr_t n_byte;
  float *p_dspbuf_segin = NULL;

  float factor;
  float f32;

  int16_t sample_val;

  p_dspbuf_segin = auddelay.getInputBufferSegment(dspbuffer_nseg);
  if(p_dspbuf_segin == NULL) sys_stop(__ERROR_GENERIC, "audiodelay error");
  
  memset(p_bufferinput, 0, AUDIOBUFFER_SIZE_BYTES);

  if(filein_pos >= AUDIO_DATAEND) filein_pos = AUDIO_DATABEGIN;

  filein_obj.seekSet(filein_pos);
  filein_obj.read(p_bufferinput, (AUDIOBUFFER_SEGMENT_SIZE_FRAMES*3u));
  filein_pos += (AUDIOBUFFER_SEGMENT_SIZE_FRAMES*3u);

  factor = AUDIO_SAMPLEFACTOR;

  n_sample = 0u;
  n_byte = 1u;
  for(n_frame = 0u; n_frame < AUDIOBUFFER_SEGMENT_SIZE_FRAMES; n_frame++)
  {
    sample_val = *((int16_t*) (((uintptr_t) p_bufferinput) + n_byte));
    
    f32 = (float) sample_val;
    f32 /= factor;
    p_dspbuf_segin[n_sample] = f32;
    p_dspbuf_segin[n_sample + 1u] = f32;

    n_sample += 2u;
    n_byte += 3u;
  }
  
  return;
}

FLASHMEM void _buffer_load_i24_2ch(void)
{
  uintptr_t n_sample;
  uintptr_t n_byte;
  float *p_dspbuf_segin = NULL;

  float factor;
  float f32;

  int16_t sample_val;

  p_dspbuf_segin = auddelay.getInputBufferSegment(dspbuffer_nseg);
  if(p_dspbuf_segin == NULL) sys_stop(__ERROR_GENERIC, "audiodelay error");
  
  memset(p_bufferinput, 0, AUDIOBUFFER_SIZE_BYTES);

  if(filein_pos >= AUDIO_DATAEND) filein_pos = AUDIO_DATABEGIN;

  filein_obj.seekSet(filein_pos);
  filein_obj.read(p_bufferinput, (AUDIOBUFFER_SEGMENT_SIZE_SAMPLES*3u));
  filein_pos += (AUDIOBUFFER_SEGMENT_SIZE_SAMPLES*3u);

  factor = AUDIO_SAMPLEFACTOR;

  n_byte = 1u;
  for(n_sample = 0u; n_sample < AUDIOBUFFER_SEGMENT_SIZE_SAMPLES; n_sample++)
  {
    sample_val = *((int16_t*) (((uintptr_t) p_bufferinput) + n_byte));
    
    f32 = (float) sample_val;
    f32 /= factor;
    p_dspbuf_segin[n_sample] = f32;

    n_byte += 3u;
  }
  
  return;
}

FLASHMEM void buffer_loadout(void)
{
  uintptr_t n_sample;
  float *p_dspbuf_segout = NULL;
  int16_t *p_bufout_loadseg = NULL;
  
  float factor;
  float f32;

  p_dspbuf_segout = auddelay.getOutputBufferSegment(dspbuffer_nseg);
  if(p_dspbuf_segout == NULL) sys_stop(__ERROR_GENERIC, "audiodelay error");

  p_bufout_loadseg = pp_bufferoutput_segments[bufferoutput_nseg_load];

  factor = AUDIO_SAMPLEFACTOR - 1.0f;

  for(n_sample = 0u; n_sample < AUDIOBUFFER_SEGMENT_SIZE_SAMPLES; n_sample++)
  {
    f32 = p_dspbuf_segout[n_sample];

    if(f32 > 1.0f) f32 = 1.0f;
    else if(f32 < -1.0f) f32 = -1.0f;

    f32 *= factor;

    p_bufout_loadseg[n_sample] = (int16_t) roundf(f32);
  }
  
  return;
}

FLASHMEM void buffer_segment_update(void)
{
  dspbuffer_nseg++;
  dspbuffer_nseg %= DSPBUFFER_N_SEGMENTS;
  
  bufferoutput_nseg_load++;
  bufferoutput_nseg_load %= AUDIOBUFFER_N_SEGMENTS;

  bufferoutput_nseg_play++;
  bufferoutput_nseg_play %= AUDIOBUFFER_N_SEGMENTS;
  
  return;
}

void dma_i2s_isr(void)
{
  buffer_segment_update();

  dma_i2s.TCD->SADDR = pp_bufferoutput_segments[bufferoutput_nseg_play];

  dma_i2s.clearError();
  dma_i2s.clearComplete();
  dma_i2s.clearInterrupt();

  bufferload_proc();
  return;
}

FLASHMEM void hw_init_i2s_dma(void)
{
  /*
   * I didn't want to use the Teensy's I2S library because it does a lot of things automatically and 
   * I wanted to have more manual control over DMA and so on...
   * 
   * However, I couldn't find the microcontroller's datasheet, or any helpful source explaining the I2S port register architecture in this MCU.
   * 
   * So what I did was looking on the I2S library source files and trying to implement the same code here.
   * It's very confusing and I myself don't totally understand what's going on here, I just know it worked...
   * 
   * If you have any questions, don't ask me because I don't know either...
   */
  
  static constexpr uint32_t I2S_FRAMESIZE = AUDIO_NCHANNELS - 1u;
  static constexpr uint32_t I2S_SYNCWIDTH = AUDIO_NCHANNELS*AUDIO_BITDEPTH - 1u;
  static constexpr uint32_t I2S_WORDSIZE = AUDIO_NCHANNELS*AUDIO_BITDEPTH - 1u;

  double f64 = 0.0;
  uint32_t n1 = 0u;
  uint32_t n2 = 0u;
  uint32_t c0 = 0u;
  uint32_t c1 = 0u;
  uint32_t c2 = 0u;

  uint32_t reg = 0u;

  CCM_CCGR5 |= CCM_CCGR5_SAI2(CCM_CCGR_ON);

  n1 = 4u;
  n2 = 1u + (24000000u*27u)/(AUDIO_SAMPLERATE*n1*256u);

  f64 = ((double) AUDIO_SAMPLERATE)*((double) n1)*((double) n2)*256.0/24000000.0;

  c0 = (uint32_t) f64;
  c2 = 10000u;
  c1 = ((uint32_t) f64)*c2 - c0*c2;

  CCM_ANALOG_PLL_AUDIO = (CCM_ANALOG_PLL_AUDIO_BYPASS | CCM_ANALOG_PLL_AUDIO_ENABLE | CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2u) | CCM_ANALOG_PLL_AUDIO_DIV_SELECT(c0));
  CCM_ANALOG_PLL_AUDIO_NUM = (c1 & CCM_ANALOG_PLL_AUDIO_NUM_MASK);
  CCM_ANALOG_PLL_AUDIO_DENOM = (c2 & CCM_ANALOG_PLL_AUDIO_DENOM_MASK);

  CCM_ANALOG_PLL_AUDIO &= ~(CCM_ANALOG_PLL_AUDIO_POWERDOWN);

  while(!(CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_LOCK)) delayMicroseconds(1u);

  CCM_ANALOG_MISC2 &= ~(CCM_ANALOG_MISC2_DIV_MSB | CCM_ANALOG_MISC2_DIV_LSB);

  CCM_ANALOG_PLL_AUDIO &= ~(CCM_ANALOG_PLL_AUDIO_BYPASS);

  reg = CCM_CSCMR1;
  reg &= ~(CCM_CSCMR1_SAI2_CLK_SEL_MASK);
  reg |= CCM_CSCMR1_SAI2_CLK_SEL(2u);
  CCM_CSCMR1 = reg;

  reg = CCM_CS2CDR;
  reg &= ~(CCM_CS2CDR_SAI2_CLK_PRED_MASK | CCM_CS2CDR_SAI2_CLK_PODF_MASK);
  reg |= (CCM_CS2CDR_SAI2_CLK_PRED(n1 - 1u) | CCM_CS2CDR_SAI2_CLK_PODF(n2 - 1u));
  CCM_CS2CDR = reg;

  reg = IOMUXC_GPR_GPR1;
  reg &= ~(IOMUXC_GPR_GPR1_SAI2_MCLK3_SEL_MASK);
  reg |= (IOMUXC_GPR_GPR1_SAI2_MCLK_DIR | IOMUXC_GPR_GPR1_SAI2_MCLK3_SEL(0u));
  IOMUXC_GPR_GPR1 = reg;

  I2S2_TMR = 0u;
  I2S2_TCR1 = I2S_TCR1_RFW(0u);
  I2S2_TCR2 = (I2S_TCR2_SYNC(0u) | I2S_TCR2_BCP | I2S_TCR2_BCD | I2S_TCR2_DIV(1u) | I2S_TCR2_MSEL(1u));
  I2S2_TCR3 = I2S_TCR3_TCE_2CH;
  I2S2_TCR4 = (I2S_TCR4_FRSZ(I2S_FRAMESIZE) | I2S_TCR4_SYWD(I2S_SYNCWIDTH) | I2S_TCR4_MF | I2S_TCR4_FSD | I2S_TCR4_FSE | I2S_TCR4_FSP);
  I2S2_TCR5 = (I2S_TCR5_WNW(I2S_WORDSIZE) | I2S_TCR5_W0W(I2S_WORDSIZE) | I2S_TCR5_FBT(I2S_WORDSIZE));

  I2S2_RMR = 0u;
  I2S2_RCR1 = I2S_RCR1_RFW(0u);
  I2S2_RCR2 = (I2S_RCR2_SYNC(1u) | I2S_RCR2_BCP | I2S_RCR2_BCD | I2S_RCR2_DIV(1u) | I2S_RCR2_MSEL(1u));
  I2S2_RCR3 = I2S_RCR3_RCE_2CH;
  I2S2_RCR4 = (I2S_RCR4_FRSZ(I2S_FRAMESIZE) | I2S_RCR4_SYWD(I2S_SYNCWIDTH) | I2S_RCR4_MF | I2S_RCR4_FSD | I2S_RCR4_FSE | I2S_RCR4_FSP);
  I2S2_RCR5 = (I2S_RCR5_WNW(I2S_WORDSIZE) | I2S_RCR5_W0W(I2S_WORDSIZE) | I2S_RCR5_FBT(I2S_WORDSIZE));

  /*CORE_PIN33_CONFIG = 2u;*/ /*UNUSED*/
  CORE_PIN4_CONFIG = 2u;
  CORE_PIN3_CONFIG = 2u;
  CORE_PIN2_CONFIG = 2u;
  /*CORE_PIN5_CONFIG = 2u;*/ /*UNUSED*/

  I2S2_TCSR = 0u;
  while(I2S2_TCSR & I2S_TCSR_TE) delayMicroseconds(1u);

  I2S2_RCSR = 0u;
  while(I2S2_RCSR & I2S_RCSR_RE) delayMicroseconds(1u);

  dma_i2s.begin(true);

  dma_i2s.TCD->DADDR = (void*) (((uintptr_t) &I2S2_TDR0) + 2u);
  dma_i2s.TCD->DOFF = 0u;
  dma_i2s.TCD->SADDR = pp_bufferoutput_segments[bufferoutput_nseg_play];
  dma_i2s.TCD->SOFF = 2u;
  dma_i2s.TCD->NBYTES_MLNO = 2u;
  dma_i2s.TCD->SLAST = -((int32_t) DMA_I2S_TRANSFER_SIZE_BYTES);
  dma_i2s.TCD->DLASTSGA = 0u;
  dma_i2s.TCD->CSR = (DMA_TCD_CSR_INTMAJOR);
  dma_i2s.TCD->ATTR = (DMA_TCD_ATTR_SSIZE(1u) | DMA_TCD_ATTR_DSIZE(1u));
  dma_i2s.TCD->CITER_ELINKNO = DMA_I2S_TRANSFER_SIZE_SAMPLES;
  dma_i2s.TCD->BITER_ELINKNO = DMA_I2S_TRANSFER_SIZE_SAMPLES;

  dma_i2s.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI2_TX);
  dma_i2s.attachInterrupt(&dma_i2s_isr);

  dma_i2s.clearError();
  dma_i2s.clearComplete();
  dma_i2s.clearInterrupt();
  dma_i2s.enable();

  I2S2_RCSR = (I2S_RCSR_RE | I2S_RCSR_BCE);
  I2S2_TCSR = (I2S_TCSR_TE | I2S_TCSR_BCE | I2S_TCSR_FRDE | I2S_TCSR_FR);

  return;
}

FLASHMEM void hw_deinit_i2s_dma(void)
{
  dma_i2s.disable();
  dma_i2s.clearError();
  dma_i2s.clearComplete();
  dma_i2s.clearInterrupt();

  *((uint16_t*) (((uintptr_t) &I2S2_TDR0) + 2u)) = 0u;
  *((uint16_t*) (((uintptr_t) &I2S2_TDR0) + 2u)) = 0u;
  *((uint16_t*) (((uintptr_t) &I2S2_TDR0) + 2u)) = 0u;
  *((uint16_t*) (((uintptr_t) &I2S2_TDR0) + 2u)) = 0u;

  delay(1u);

  I2S2_TCSR = 0u;
  while(I2S2_TCSR & I2S_TCSR_TE) delayMicroseconds(1u);

  I2S2_RCSR = 0u;
  while(I2S2_RCSR & I2S_RCSR_RE) delayMicroseconds(1u);

  /*pinMode(33u, INPUT_PULLUP);*/ /*UNUSED*/
  pinMode(4u, INPUT_PULLUP);
  pinMode(3u, INPUT_PULLUP);
  pinMode(2u, INPUT_PULLUP);
  /*pinMode(5u, INPUT_PULLUP);*/ /*UNUSED*/

  CCM_ANALOG_PLL_AUDIO |= CCM_ANALOG_PLL_AUDIO_BYPASS;
  CCM_ANALOG_PLL_AUDIO |= CCM_ANALOG_PLL_AUDIO_POWERDOWN;
  CCM_ANALOG_PLL_AUDIO &= ~(CCM_ANALOG_PLL_AUDIO_ENABLE);
  
  return;
}

FLASHMEM void lcd_data_update(void)
{
  lcd1.clear();

  snprintf(textbuf, TEXTBUF_SIZE_CHARS, "FF Dly: %u", auddelay.getFFDelay());
  lcd1.setCursorPosition(0u, 0u);
  lcd1.printText(textbuf);

  snprintf(textbuf, TEXTBUF_SIZE_CHARS, "FF Amp: %f", auddelay.getFFAmp());
  lcd1.setCursorPosition(0u, 1u);
  lcd1.printText(textbuf);

  snprintf(textbuf, TEXTBUF_SIZE_CHARS, "FB Dly: %u", auddelay.getFBDelay());
  lcd1.setCursorPosition(0u, 2u);
  lcd1.printText(textbuf);

  snprintf(textbuf, TEXTBUF_SIZE_CHARS, "FB Amp: %f", auddelay.getFBAmp());
  lcd1.setCursorPosition(0u, 3u);
  lcd1.printText(textbuf);
  
  return;
}

FLASHMEM void button_wait_release(uint8_t pin)
{
  do{
    delay(8u);
  }while(!digitalRead(pin));
  
  return;
}
