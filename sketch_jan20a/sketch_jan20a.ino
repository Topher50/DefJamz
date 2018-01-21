#include "arduinoFFT.h"
#include "chord.h"
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
#define CHANNEL A0
const uint16_t samples = 256;

double vReal[samples];
double vImag[samples];
double avg[] = {19605.96, 8925.05, 3846.65, 1413.25, 1139.36, 675.13, 771.84, 620.47, 363.81, 297.74, 371.13, 418.96, 443.79, 519.79, 666.62, 862.72, 918.99, 733.66, 675.19, 579.28, 670.84, 633.10, 620.16, 592.25, 556.59, 661.11, 739.50, 684.19, 651.22, 591.10, 625.32, 622.69, 504.09, 602.57, 495.87, 447.86, 442.44, 435.03, 398.43, 500.44, 662.78, 926.23, 522.96, 491.52, 534.13, 865.02, 874.64, 697.40, 665.80, 644.04, 511.29, 482.88, 436.89, 439.75, 383.18, 416.13, 481.54, 417.10, 391.40, 429.16, 409.99, 408.62, 382.59, 265.50, 235.82, 262.90, 643.63, 686.54, 215.21, 221.01, 243.85, 227.15, 220.43, 178.06, 161.18, 158.36, 160.12, 202.44, 169.12, 141.75, 163.20, 328.62, 628.15, 259.54, 148.46, 133.32, 139.58, 165.05, 158.61, 142.77, 156.42, 143.37, 176.69, 160.45, 160.47, 163.30, 174.56, 268.76, 217.39, 153.18, 144.25, 123.43, 163.32, 178.56, 161.38, 135.30, 177.56, 777.73, 811.67, 180.02, 125.28, 130.67, 196.99, 215.19, 187.32, 154.96, 152.63, 164.76, 187.36, 142.54, 172.91, 161.10, 159.50, 277.68, 167.64, 133.54, 135.53, 145.82};
double stdD[] = {2337.95, 2613.84, 992.89, 914.50, 535.11, 312.70, 458.76, 386.22, 248.87, 167.72, 181.84, 261.32, 210.32, 255.64, 379.80, 502.02, 569.65, 392.20, 405.79, 396.54, 502.00, 389.92, 333.00, 317.07, 293.87, 337.48, 406.14, 462.66, 426.04, 585.46, 563.58, 382.04, 356.53, 387.11, 279.30, 280.16, 278.48, 275.23, 281.96, 295.72, 318.33, 350.32, 337.66, 323.19, 360.57, 620.56, 792.14, 466.96, 445.88, 478.45, 482.52, 272.96, 349.00, 330.92, 255.62, 286.01, 423.47, 413.53, 519.24, 583.80, 420.39, 331.78, 290.40, 169.93, 148.69, 161.10, 207.10, 218.90, 128.18, 196.75, 242.50, 178.12, 193.69, 90.42, 98.61, 85.48, 95.66, 88.26, 83.63, 73.61, 75.32, 103.95, 99.49, 87.15, 73.35, 74.26, 70.88, 79.78, 82.74, 69.63, 76.63, 66.39, 92.50, 83.38, 80.80, 97.72, 76.42, 99.59, 85.39, 83.54, 65.71, 54.05, 89.84, 97.48, 69.63, 79.55, 87.85, 154.91, 137.44, 87.34, 84.00, 75.74, 103.90, 116.48, 98.74, 81.02, 71.26, 78.03, 97.61, 55.21, 93.46, 99.88, 70.00, 110.36, 93.57, 79.87, 57.02, 73.65};
int G[] = {0,0,10,20,40,80};
int B[] = {12,13,25,50,51,101};
int D[] = {0, 0, 15,30,60,120};
int C[] = {13,14,27,53,54,107};
int E[] = {0, 0, 17,34,67,68};
int FS[] = {0,0,19,38,75,76};
int A[] = {0,0,22,23,45,90};
int CS[] = {0,14,28,29,57,103};
int N[] = {0,0,0,0,0,0};
int arrLen = 6;
int beatsPerMin = 60;

Chord Gmajor; 

Chord Cmajor;

Chord Dmajor;

Chord chords[3];

const double samplingFrequency = 2500;

unsigned int delayTime = 0;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

const int redLight = 7;
const int blueLight = 8;
const int greenLight = 4;
const int whiteLight = 13;

const int mic = 0;

void setup() {
  delayTime = 1000000/samplingFrequency;
  pinMode(redLight, OUTPUT);
  pinMode(blueLight, OUTPUT);
  pinMode(greenLight, OUTPUT);
  pinMode(whiteLight, OUTPUT);
  
  pinMode(mic, INPUT);

  Gmajor.notes[0] = G;
  Gmajor.notes[1] = D;
  Gmajor.notes[2] = B;
  Gmajor.notes[3] = N;
  Gmajor.len = 1;

  Cmajor.notes[0] = C;
  Cmajor.notes[1] = E;
  Cmajor.notes[2] = G;
  Cmajor.notes[3] = N;
  Cmajor.len = 1;
  
  Dmajor.notes[0] = D;
  Dmajor.notes[1] = FS;
  Dmajor.notes[2] = A;
  Dmajor.notes[3] = CS;
  Dmajor.len = 1;

  chords[0] = Gmajor;
  chords[1] = Cmajor;
  chords[2] = Dmajor;
  Serial.begin(9600);
}

void loop() {
  for(int j=0; j<3; ++j) {
    switch (j) {
      case 0: {
        digitalWrite(redLight, HIGH);
        digitalWrite(blueLight, LOW);
        digitalWrite(greenLight, LOW);
        break;
      }
      case 1: {
        digitalWrite(blueLight, HIGH);
        digitalWrite(redLight, LOW);
        digitalWrite(greenLight, LOW);
        break;
      }
      case 2: {
        digitalWrite(greenLight, HIGH);
        digitalWrite(blueLight, LOW);
        digitalWrite(redLight, LOW);
        break;
      }
    }
    int noteTime = (beatsPerMin*4*1000/60)/chords[j].len;
    delay(noteTime/2);
    for(uint16_t i =0;i<samples;i++)
    {
      vReal[i] = double(analogRead(CHANNEL));
      vImag[i] = 0.0;
      delayMicroseconds(delayTime);
    }
    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
    delay(noteTime/2);
    checkChord(chords[j],vReal);
  }
}

void checkChord(Chord chord, double *vData) {
  bool flag = false;
  for(int i=0; i<4; ++i) {
    if(chord.notes[i] == N) {
      continue;
    }
    for(int j=0; j<6; ++j) {
      if(chord.notes[i][j] == 0) {
        continue;
      }

      int bucket = chord.notes[i][j];
      if(vData[bucket] >= (avg[bucket] + 10*stdD[bucket])) {
        flag = true;
        break;
      }
    }

    if(!flag) {
      digitalWrite(whiteLight, LOW);
      Serial.print("\nFAIL\n");
      return;
    }
  }
    digitalWrite(whiteLight, HIGH);
    Serial.print("\nSUCCESS\n");
}

void PrintVector(double *vData, uint16_t bufferSize)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    Serial.print(i, 6);
    Serial.print(" ");
    Serial.print(vData[i], 4);
    Serial.print(" ");
    Serial.print(i * samplingFrequency/samples);
    Serial.println();
  }
  Serial.println();
}
