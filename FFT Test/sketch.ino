#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2

/* 
These values are just for demonstration purpose
*/
double signalFrequency = 1000;
const double samplingFrequency = 10000;
const uint8_t amplitude = 100;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vRealBuffer[samples];
double vImagBuffer[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

uint16_t loopStep = 0;
uint16_t timeStep = 0;

double recordedFrequencies[10];
double actualFrequencies[10];

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  loopStep++;
  loopStep = loopStep % samples;

  vRealBuffer[loopStep] = readMicrophone(loopStep);
  vImagBuffer[loopStep] = 0;

  if (loopStep==0){
    double frequency = computeFrequency(vRealBuffer, vImagBuffer);

    actualFrequencies[timeStep] = signalFrequency;
    recordedFrequencies[timeStep] = frequency;

    Serial.print("Real: "); Serial.print(signalFrequency);
    Serial.print(" Hz  \t    Measured: "); Serial.print(frequency);
    Serial.print(" Hz\n");

    signalFrequency = rand() % 2000 + 10;
    timeStep++;
  }
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}


double computeFrequency(double *vReal, double *vImag)
{
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); 
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  return FFT.MajorPeak(vReal, samples, samplingFrequency);
}


int8_t readMicrophone(uint16_t time)
{
  double cycles = (((samples-1) * signalFrequency) / samplingFrequency);
  return int8_t((amplitude * (sin((time * (twoPi * cycles)) / samples))) / 2.0);
}


 //Number of signal cycles that the sampling will read
//for (uint16_t i = 0; i < samples; i++)
//{
//  vRealBuffer[i] = int8_t(;/* Build data with positive and negative values*/
  //vReal[i] = uint8_t((amplitude * (sin((i * (twoPi * cycles)) / samples) + 1.0)) / 2.0);/* Build data displaced on the Y axis to include only positive values*/
//  vImagBuffer[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
//}
