/* *************************************************************************************************
 *  Sketch: sineWave
 *  Author: Tanya Vasilevitsky
 *  Date: 07/19/2016
 *  Description: This sketch produces a sine wave output to DAC0 of the arduino DUE board. 
 *  There are several options for The frequency & granularity of the wave , which can be used
 *  To choose a wave simply uncomment (only!) one of the WAVE_#
 *
 * *************************************************************************************************/

//////////////////////////////////////////////////////////////////////////////////////////////////
 /* Uncomment the desired wave, leave the rest commented*/ 

//#define WAVE_1 //A sine wave of 120 samples, in 12 bit DAC resolution, Maximmal frequency 14 KHZ
//#define WAVE_2 //A sine wave of 60 samples, in 12 bit DAC resolution, Maximmal frequency 28 KHZ
//#define WAVE_3 //A sine wave of 24 samples, in 10 bit DAC resolution, Maximmal frequency 70 KHZ
//#define WAVE_4 //A sine wave of 24 samples, in 12 bit DAC resolution, Maximmal frequency 70 KHZ
#define WAVE_5 //A sine wave of 20 samples, in 12 bit DAC resolution, Maximmal frequency 84 KHZ
/////////////////////////////////////////////////////////////////////////////////////////////////



/*Determine the resolution*/
#if defined(WAVE_1) || defined(WAVE_2) || defined(WAVE_4) || defined(WAVE_5) 
#define DAC_RESOLUTION 12
#elif  defined WAVE_3
#define DAC_RESOLUTION 10
#endif

int F = 10; //frequency in HZ
int waitTime = 1000/F;


int i = 0; //index to iterate the wave values

// Sin wave values
#ifdef WAVE_1 
#define MAX_SAMPLES 120
static int sineTable[MAX_SAMPLES] = 
{
  0x7ff, 0x86a, 0x8d5, 0x93f, 0x9a9, 0xa11, 0xa78, 0xadd, 0xb40, 0xba1,
  0xbff, 0xc5a, 0xcb2, 0xd08, 0xd59, 0xda7, 0xdf1, 0xe36, 0xe77, 0xeb4,
  0xeec, 0xf1f, 0xf4d, 0xf77, 0xf9a, 0xfb9, 0xfd2, 0xfe5, 0xff3, 0xffc,
  0xfff, 0xffc, 0xff3, 0xfe5, 0xfd2, 0xfb9, 0xf9a, 0xf77, 0xf4d, 0xf1f,
  0xeec, 0xeb4, 0xe77, 0xe36, 0xdf1, 0xda7, 0xd59, 0xd08, 0xcb2, 0xc5a,
  0xbff, 0xba1, 0xb40, 0xadd, 0xa78, 0xa11, 0x9a9, 0x93f, 0x8d5, 0x86a,
  0x7ff, 0x794, 0x729, 0x6bf, 0x655, 0x5ed, 0x586, 0x521, 0x4be, 0x45d,
  0x3ff, 0x3a4, 0x34c, 0x2f6, 0x2a5, 0x257, 0x20d, 0x1c8, 0x187, 0x14a,
  0x112, 0xdf, 0xb1, 0x87, 0x64, 0x45, 0x2c, 0x19, 0xb, 0x2,
  0x0, 0x2, 0xb, 0x19, 0x2c, 0x45, 0x64, 0x87, 0xb1, 0xdf,
  0x112, 0x14a, 0x187, 0x1c8, 0x20d, 0x257, 0x2a5, 0x2f6, 0x34c, 0x3a4,
  0x3ff, 0x45d, 0x4be, 0x521, 0x586, 0x5ed, 0x655, 0x6bf, 0x729, 0x794
};

#elif defined WAVE_2 
#define MAX_SAMPLES 60
static int sineTable[MAX_SAMPLES] = 
{
  0x7ff,  0x8d5,  0x9a9,  0xa78,  0xb40, 
  0xbff,  0xcb2,  0xd59,  0xdf1,  0xe77, 
  0xeec,  0xf4d,  0xf9a,  0xfd2,  0xff3, 
  0xfff,  0xff3,  0xfd2,  0xf9a,  0xf4d, 
  0xeec,  0xe77,  0xdf1,  0xd59,  0xcb2, 
  0xbff,  0xb40,  0xa78,  0x9a9,  0x8d5, 
  0x7ff,  0x729,  0x655,  0x586,  0x4be, 
  0x3ff,  0x34c,  0x2a5,  0x20d,  0x187, 
  0x112,  0xb1,  0x64,  0x2c,  0xb,
  0x0,  0xb,  0x2c,  0x64,  0xb1, 
  0x112,  0x187,  0x20d,  0x2a5,  0x34c, 
  0x3ff,  0x4be,  0x586,  0x655,  0x729, 
};

#elif defined WAVE_3
#define MAX_SAMPLES 24
static int sineTable[MAX_SAMPLES] = 
{
  0x200, 0x284, 0x2FF, 0x369, 0x3BA, 0x3EE, 0x3FF, 0x3EE,
  0x3BA, 0x369, 0x2FF, 0x284, 0x1FF, 0x17B, 0x100, 0x096, 
  0x045, 0x011, 0x000, 0x011, 0x045, 0x096, 0x100, 0x17B,
};

#elif defined WAVE_4
#define MAX_SAMPLES 24
static int sineTable[MAX_SAMPLES] = 
{
  0x800,0xA11,0xBFF,0xDA7,0xEED,0xFB9,0xFFF,0xFB9,
  0xEED,0xDA7,0xBFF,0xA11,0x7FF,0x5EE,0x400,0x258,
  0x112,0x046,0x000,0x046,0x112,0x258,0x400,0x5EE,
};

#elif defined WAVE_5
#define MAX_SAMPLES 20
static int sineTable[MAX_SAMPLES] = 
{
  0x800,0xA78,0xCB3,0xE78,0xF9B,
  0xFFF,0xF9B,0xE78,0xCB3,0xA78,
  0x800,0x587,0x34C,0x187,0x064,
  0x000,0x064,0x187,0x34C,0x587,
};

#endif


void setup() {
  
  // put your setup code here, to run once:
  analogWriteResolution(DAC_RESOLUTION);  // set the analog output resolution to the desired amount of bits
  analogReadResolution(DAC_RESOLUTION);   // set the analog input resolution to desired amount of bits
}

void loop() {
  
  /*increment wave index cyclicly*/
   i = (i+1) % MAX_SAMPLES;

  //write wave data to DAC0
//  analogWrite(DAC0, sineTable[i]);  // write the selected waveform on DAC0
  analogWrite(DAC0, sineTable[i]);  // write the selected waveform on DAC0

 /*The frequency of the Wave depends on the number of samples used
 More samples --> more loops used to create a single cycle --> slower frequency 
 From data obtained from the sam3x spec, the DAC maximal possible frequncy is 1.68 MGZ,
 The Wave Frequency can be determined by: 
 F = 1.68/(number of samples in wave)
 
 You can uncomment the following line and add delays for a smaller frequency */
 //delayMicroseconds(55);    


}
