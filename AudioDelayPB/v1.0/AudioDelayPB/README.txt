Audio Delay for File Playback using Teensy 4.1
Version 1.0

This is a simplified version of my AudioDelay2 project for GNU-Linux & Windows.

AudioDelay2 for GNU-Linux: https://github.com/RMSabe/GNU-Linux_AudioDelay2
AudioDelay2 for Windows: https://github.com/RMSabe/Windows_AudioDelay2

It will play a .wav file, 16bit or 24bit, mono or stereo, running the effect in real-time.

The file needs to be named "TestFile.wav" and must be placed at the root directory of an microsd card, which will be placed in the microsd card slot on the Teensy board.
FAT32 Partitioning recommended.

This is just a prototype. I plan to expand this project to be an audio streaming delay unit, sampling an input, running the DSP and routing to the output.
For now, this is what I've got.

Libraries used:
"util.h" v1.0 : https://github.com/RMSabe/ArduinoIDE_Lib/tree/main/Util/v1.0
"lcd.hpp" v1.0 : https://github.com/RMSabe/LCD_Driver/tree/main/ArduinoIDE/v1.0

Author: Rafael Sabe
Email: rafaelmsabe@gmail.com

