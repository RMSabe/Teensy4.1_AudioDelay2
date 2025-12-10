/*
 * Audio Delay Effect for File Playback using Teensy 4.1
 * Version 1.0
 *
 * Author: Rafael Sabe
 * Email: rafaelmsabe@gmail.com
 */

#include "AudioDelay.hpp"
#include <string.h>

FLASHMEM bool AudioDelay::begin(void)
{
	if(this->_status > 0) return true;

	this->_status = this->STATUS_UNINITIALIZED;

	if(!this->_buffer_init())
	{
		this->_status = this->STATUS_ERROR;
		return false;
	}

	this->_status = this->STATUS_INITIALIZED;
	this->resetParams();

	return true;
}

FLASHMEM bool AudioDelay::runDSP(uintptr_t n_segment)
{
	float *p_curr_seg_in = NULL;
	float *p_curr_seg_out = NULL;

	uintptr_t n_currsample;
	uintptr_t n_prevsample;
	uintptr_t n_channel;

	uintptr_t n_frame;
	uintptr_t prev_buf_nframe;

	if(this->_status < 1) return false;
	if(n_segment >= DSPBUFFER_N_SEGMENTS) return false;

	p_curr_seg_in = this->_pp_bufferinput_segments[n_segment];
	p_curr_seg_out = this->_pp_bufferoutput_segments[n_segment];

	for(n_frame = 0u; n_frame < DSPBUFFER_SEGMENT_SIZE_FRAMES; n_frame++)
	{
		n_currsample = n_frame*AUDIO_NCHANNELS;

		for(n_channel = 0u; n_channel < AUDIO_NCHANNELS; n_channel++)
		{
			p_curr_seg_out[n_currsample] = (this->_dryin_amp)*(p_curr_seg_in[n_currsample]);
			n_currsample++;
		}

		this->_retrieve_prev_nframe(n_segment, n_frame, this->_ff_delay, &prev_buf_nframe, NULL, NULL);

		n_currsample = n_frame*AUDIO_NCHANNELS;
		n_prevsample = prev_buf_nframe*AUDIO_NCHANNELS;

		for(n_channel = 0u; n_channel < AUDIO_NCHANNELS; n_channel++)
		{
			p_curr_seg_out[n_currsample] += (this->_ff_amp)*(this->_p_bufferinput[n_prevsample]);
			n_currsample++;
			n_prevsample++;
		}

		this->_retrieve_prev_nframe(n_segment, n_frame, this->_fb_delay, &prev_buf_nframe, NULL, NULL);

		n_currsample = n_frame*AUDIO_NCHANNELS;
		n_prevsample = prev_buf_nframe*AUDIO_NCHANNELS;

		for(n_channel = 0u; n_channel < AUDIO_NCHANNELS; n_channel++)
		{
			p_curr_seg_out[n_currsample] += (this->_fb_amp)*(this->_p_bufferoutput[n_prevsample]);
			n_currsample++;
			n_prevsample++;
		}

		n_currsample = n_frame*AUDIO_NCHANNELS;

		for(n_channel = 0u; n_channel < AUDIO_NCHANNELS; n_channel++)
		{
			p_curr_seg_out[n_currsample] *= (this->_out_amp);
			n_currsample++;
		}
	}

	return true;
}

FLASHMEM float* AudioDelay::getInputBuffer(void)
{
	return this->_p_bufferinput;
}

FLASHMEM float* AudioDelay::getOutputBuffer(void)
{
	return this->_p_bufferoutput;
}

FLASHMEM float* AudioDelay::getInputBufferSegment(uintptr_t n_segment)
{
	if(this->_status < 1) return NULL;
	if(n_segment >= DSPBUFFER_N_SEGMENTS) return NULL;

	return this->_pp_bufferinput_segments[n_segment];
}

FLASHMEM float* AudioDelay::getOutputBufferSegment(uintptr_t n_segment)
{
	if(this->_status < 1) return NULL;
	if(n_segment >= DSPBUFFER_N_SEGMENTS) return NULL;

	return this->_pp_bufferoutput_segments[n_segment];
}

FLASHMEM bool AudioDelay::setDryInAmp(float amp)
{
	this->_dryin_amp = amp;
	return true;
}

FLASHMEM bool AudioDelay::setOutAmp(float amp)
{
	this->_out_amp = amp;
	return true;
}

FLASHMEM float AudioDelay::getDryInAmp(void)
{
	return this->_dryin_amp;
}

FLASHMEM float AudioDelay::getOutAmp(void)
{
	return this->_out_amp;
}

FLASHMEM bool AudioDelay::setFFDelay(uintptr_t n_delay)
{
	if(n_delay >= DSPBUFFER_SIZE_FRAMES) return false;

	this->_ff_delay = n_delay;
	return true;
}

FLASHMEM bool AudioDelay::setFFAmp(float amp)
{
	this->_ff_amp = amp;
	return true;
}

FLASHMEM uintptr_t AudioDelay::getFFDelay(void)
{
	return this->_ff_delay;
}

FLASHMEM float AudioDelay::getFFAmp(void)
{
	return this->_ff_amp;
}

FLASHMEM bool AudioDelay::setFBDelay(uintptr_t n_delay)
{
	if(n_delay >= DSPBUFFER_SIZE_FRAMES) return false;

	this->_fb_delay = n_delay;
	return true;
}

FLASHMEM bool AudioDelay::setFBAmp(float amp)
{
	this->_fb_amp = amp;
	return true;
}

FLASHMEM uintptr_t AudioDelay::getFBDelay(void)
{
	return this->_fb_delay;
}

FLASHMEM float AudioDelay::getFBAmp(void)
{
	return this->_fb_amp;
}

FLASHMEM bool AudioDelay::resetParams(void)
{
	this->_dryin_amp = 1.0f;
	this->_out_amp = 1.0f;
	this->_ff_amp = 0.0f;
	this->_fb_amp = 0.0f;
	this->_ff_delay = 0u;
	this->_fb_delay = 0u;

	return true;
}

FLASHMEM intptr_t AudioDelay::getStatus(void)
{
	return this->_status;
}

FLASHMEM bool AudioDelay::_buffer_init(void)
{
	uintptr_t n_seg;

	for(n_seg = 0u; n_seg < DSPBUFFER_N_SEGMENTS; n_seg++)
	{
		this->_pp_bufferinput_segments[n_seg] = (float*) (((uintptr_t) this->_p_bufferinput) + n_seg*DSPBUFFER_SEGMENT_SIZE_BYTES);
		this->_pp_bufferoutput_segments[n_seg] = (float*) (((uintptr_t) this->_p_bufferoutput) + n_seg*DSPBUFFER_SEGMENT_SIZE_BYTES);
	}

	memset(this->_p_bufferinput, 0, DSPBUFFER_SIZE_BYTES);
	memset(this->_p_bufferoutput, 0, DSPBUFFER_SIZE_BYTES);

	return true;
}

FLASHMEM bool AudioDelay::_retrieve_prev_nframe(uintptr_t curr_buf_nframe, uintptr_t n_delay, uintptr_t *p_prev_buf_nframe, uintptr_t *p_prev_nseg, uintptr_t *p_prev_seg_nframe)
{
	uintptr_t prev_buf_nframe;
	uintptr_t prev_nseg;
	uintptr_t prev_seg_nframe;

	if(curr_buf_nframe >= DSPBUFFER_SIZE_FRAMES) return false;
	if(n_delay >= DSPBUFFER_SIZE_FRAMES) return false;

	if(n_delay > curr_buf_nframe) prev_buf_nframe = DSPBUFFER_SIZE_FRAMES - (n_delay - curr_buf_nframe);
	else prev_buf_nframe = curr_buf_nframe - n_delay;

	prev_nseg = prev_buf_nframe/DSPBUFFER_SEGMENT_SIZE_FRAMES;
	prev_seg_nframe = prev_buf_nframe%DSPBUFFER_SEGMENT_SIZE_FRAMES;

	if(p_prev_buf_nframe != NULL) *p_prev_buf_nframe = prev_buf_nframe;
	if(p_prev_nseg != NULL) *p_prev_nseg = prev_nseg;
	if(p_prev_seg_nframe != NULL) *p_prev_seg_nframe = prev_seg_nframe;

	return true;
}

FLASHMEM bool AudioDelay::_retrieve_prev_nframe(uintptr_t curr_nseg, uintptr_t curr_seg_nframe, uintptr_t n_delay, uintptr_t *p_prev_buf_nframe, uintptr_t *p_prev_nseg, uintptr_t *p_prev_seg_nframe)
{
	uintptr_t curr_buf_nframe;

	if(curr_nseg >= DSPBUFFER_N_SEGMENTS) return false;
	if(curr_seg_nframe >= DSPBUFFER_SEGMENT_SIZE_FRAMES) return false;

	curr_buf_nframe = curr_nseg*DSPBUFFER_SEGMENT_SIZE_FRAMES + curr_seg_nframe;

	return this->_retrieve_prev_nframe(curr_buf_nframe, n_delay, p_prev_buf_nframe, p_prev_nseg, p_prev_seg_nframe);
}

