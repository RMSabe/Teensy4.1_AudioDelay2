/*
 * Audio Delay Effect for File Playback using Teensy 4.1
 * Version 1.0
 *
 * Author: Rafael Sabe
 * Email: rafaelmsabe@gmail.com
 */

#ifndef AUDIODELAY_HPP
#define AUDIODELAY_HPP

#include "audiodelaypb_def.h"
#include "util.h"

#include <Arduino.h>

class AudioDelay {
	public:
		bool begin(void) FLASHMEM;
		bool runDSP(uintptr_t n_segment) FLASHMEM;

		float* getInputBuffer(void) FLASHMEM;
		float* getOutputBuffer(void) FLASHMEM;

		float* getInputBufferSegment(uintptr_t n_segment) FLASHMEM;
		float* getOutputBufferSegment(uintptr_t n_segment) FLASHMEM;

		bool setDryInAmp(float amp) FLASHMEM;
		bool setOutAmp(float amp) FLASHMEM;

		float getDryInAmp(void) FLASHMEM;
		float getOutAmp(void) FLASHMEM;

		bool setFFDelay(uintptr_t n_delay) FLASHMEM;
		bool setFFAmp(float amp) FLASHMEM;

		uintptr_t getFFDelay(void) FLASHMEM;
		float getFFAmp(void) FLASHMEM;

		bool setFBDelay(uintptr_t n_delay) FLASHMEM;
		bool setFBAmp(float amp) FLASHMEM;

		uintptr_t getFBDelay(void) FLASHMEM;
		float getFBAmp(void) FLASHMEM;

		bool resetParams(void) FLASHMEM;

		intptr_t getStatus(void) FLASHMEM;

		enum Status {
			STATUS_ERROR = -1,
			STATUS_UNINITIALIZED = 0,
			STATUS_INITIALIZED = 1
		};

	private:
		__attribute__((aligned(PTR_SIZE_BITS))) float _dryin_amp = 0.0f;
		__attribute__((aligned(PTR_SIZE_BITS))) float _out_amp = 0.0f;

		__attribute__((aligned(PTR_SIZE_BITS))) uintptr_t _ff_delay = 0u;
		__attribute__((aligned(PTR_SIZE_BITS))) float _ff_amp = 0.0f;

		__attribute__((aligned(PTR_SIZE_BITS))) uintptr_t _fb_delay = 0u;
		__attribute__((aligned(PTR_SIZE_BITS))) float _fb_amp = 0.0f;

		__attribute__((aligned(PTR_SIZE_BITS))) intptr_t _status = this->STATUS_UNINITIALIZED;

		__attribute__((aligned(PTR_SIZE_BITS))) float _p_bufferinput[DSPBUFFER_SIZE_SAMPLES];
		__attribute__((aligned(PTR_SIZE_BITS))) float _p_bufferoutput[DSPBUFFER_SIZE_SAMPLES];

		__attribute__((aligned(PTR_SIZE_BITS))) float* _pp_bufferinput_segments[DSPBUFFER_N_SEGMENTS] = {NULL};
		__attribute__((aligned(PTR_SIZE_BITS))) float* _pp_bufferoutput_segments[DSPBUFFER_N_SEGMENTS] = {NULL};

		bool _buffer_init(void) FLASHMEM;
		bool _retrieve_prev_nframe(uintptr_t curr_buf_nframe, uintptr_t n_delay, uintptr_t *p_prev_buf_nframe, uintptr_t *p_prev_nseg, uintptr_t *p_prev_seg_nframe) FLASHMEM;
		bool _retrieve_prev_nframe(uintptr_t curr_nseg, uintptr_t curr_seg_nframe, uintptr_t n_delay, uintptr_t *p_prev_buf_nframe, uintptr_t *p_prev_nseg, uintptr_t *p_prev_seg_nframe) FLASHMEM;
};

#endif /*AUDIODELAY_HPP*/

