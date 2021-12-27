/***********************************************
  Includes.
***********************************************/
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <movingAvg.h>
#include <filters.h>
#include <filters_defs.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ecg_project_test_1_inferencing.h>


/***********************************************
  Defines.
***********************************************/
#define SAMPLE_RATE_US 10000 //Limitado a 128 pela biblioteca Adafruit_ADS1X15 para alterar use a fução ConfigSampleRate(uint rate) com os valores disponíveis no enum sampleRate.
#define DEBOUNCE_TIME_MS 25
#define MULTIPLIER_VALUE 0.1875F
#define MOVING_AVG_WINDOW_SIZE 3
#define BUTTON_PIN 2
#define EXAM_DURATION_US 1000
#define MAX_SIGNAL_POINTS 1000
#define MAX_HEARTBEATS_CAPTURED 15
#define MAX_HEARTBEAT_IMAGE_SIZE 100
/***********************************************
  Variáveis.
***********************************************/
//Variáveis do Timer de leitura do ADC.
volatile bool timerFlagAdc;
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

//Variáveis do Timer de controle do tempo do exame.
volatile bool timerFlagExamFinished;
volatile int count;    // Trigger
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

//Variáveis dos Filtros
const float samplingTime = 0.01;                            //Sampling time em segundos.
const float cutoffFreqHigh   = 0.5;                                              //Frequência de corte em Hz.
const float cutoffFreqLow   = 41.0;                                              //Frequência de corte em Hz.
Filter fhp(cutoffFreqHigh, samplingTime, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
IIR::ORDER  order_low  = IIR::ORDER::OD3;                                           // Order (OD1 to OD4)
Filter f(cutoffFreqLow, samplingTime, order_low);
movingAvg avgValue(1);
const float multiplier = 0.1875F;

//Variáveis do ADC.
Adafruit_ADS1115 ads;
enum sampleRate {
  SPS8 = 0x0000, //< 8 amostras por segundo.
  SPS16 = 0x0020, //< 16 amostras por segundo.
  SPS32 = 0x0040, //< 32 amostras por segundo.
  SPS64 = 0x0060, //< 64 amostras por segundo.
  SPS128 = 0x0080, //< 128 amostras por segundo(padrão).
  SPS250 = 0x00A0, //< 250 samples amostras por segundo.
  SPS475 = 0x00C0, //< 475 amostras por segundo.
  SPS860 = 0x00E0, //< 860 amostras por segundo.
};

//Variáveis do Botão.
const uint button = BUTTON_PIN;         //Botão configurado no pino2.
volatile boolean buttonFlag;

//Variáveis para a criação de HeartBeats.
int SignalFiltred[MAX_SIGNAL_POINTS];
int SignalImages[MAX_HEARTBEATS_CAPTURED][MAX_HEARTBEAT_IMAGE_SIZE];
float SignalImagesNormalized[MAX_HEARTBEATS_CAPTURED][MAX_HEARTBEAT_IMAGE_SIZE];
int SignalMaxPointsIndex[MAX_SIGNAL_POINTS];
int signalIndex;

//Variáveis para controle da máquina de estados.
uint state = 1;
bool examStart;
enum states {ms_wait, ms_readingButton, ms_automaticInitiation, ms_starting, ms_collectingData, ms_examFinished, ms_buldingImage, ms_sendingImageToClassification};

//Variáveis para classificação.
static int AFIB_cont = 0;
static int SR_cont = 0;
int zeroCountClassifcationImage = 0;
static float features[100] = {
  // copy raw features here (for example from the 'Live classification' page)
  // see https://docs.edgeimpulse.com/docs/running-your-impulse-arduino
};

/***********************************************
  Definição de funções.
***********************************************/
//Funções do Timer de leitura do ADC.
void IRAM_ATTR onTimer0();

//Funções do Timer de controle do tempo do exame.
void IRAM_ATTR onTimer1();

//Fuções do ADC.
int16_t ReadADC();
void ConfigSampleRate(uint rate);

//Funções do Filtro.
float FilterSignal(int16_t adcValue, float cutoffHighPassFilter, float cutoffLowPassFilter);

//Funções do botão.
boolean Debounce(uint buttonToRead);
void ButtonInterrupt();

//Funções para a criação de HeartBeats.
float FoundSignalPeaks(int* heartSignal);
void RPeakDetection(int* heartSignal);
void ExtractHeartbeats(int sinal_imagem[MAX_HEARTBEATS_CAPTURED][MAX_HEARTBEAT_IMAGE_SIZE]);
void NormalizeHeartbeats();

//Funções para classificação.
void classification(void);
void ei_printf(const char *format, ...);
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr);

/***********************************************
  Setup.
***********************************************/
void setup() {
  //Configuração Serial.
  Serial.begin(38400);
  while (!Serial)
  {

  };

  Serial.println("CLEARSHEET");
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Time,Classification");

  //Configuração do ADC.
  ads.begin();
  //ConfigSampleRate(SPS128);

  //Configuração do Filtro.
  avgValue.begin();

  //Configuração do Botão.
  buttonFlag = false;
  pinMode(button, INPUT);

}

/***********************************************
  Loop.
***********************************************/
void loop() {

  switch (state) {
    case ms_readingButton:
      {
        attachInterrupt(digitalPinToInterrupt(button), ButtonInterrupt, FALLING);
        if (buttonFlag ==  true) {
          bool buttonValue = Debounce(button);
          if (buttonValue == true) {
            detachInterrupt(digitalPinToInterrupt(button));
            buttonValue = false;
            buttonFlag = false;
            examStart = true;
            state = ms_starting;
          }
          buttonFlag = false;
        }
        break;
      }
    case ms_automaticInitiation:
      {
        examStart = true;
        state = ms_starting;
        break;
      }
    case ms_starting:
      {
        if (examStart == true) {
          examStart = false;
          //delay(5000);

          //Configuração do Timer de controle do tempo do exame.
          timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
          timerAttachInterrupt(timer1, &onTimer1, true); // edge (not level) triggered
          timerAlarmWrite(timer1, 10000000, true); // 250000 * 1 us = 250 ms, autoreload true

          //Configuração do Timer de leitura do ADC.
          timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
          timerAttachInterrupt(timer0, &onTimer0, true); // edge (not level) triggered
          timerAlarmWrite(timer0, SAMPLE_RATE_US, true); // 2000000 * 1 us = 2 s, autoreload true

          timerAlarmEnable(timer1); // habilita timer de tempo do exame.
          timerAlarmEnable(timer0); // habilita timer de leitura de adc.

          timerFlagExamFinished = false;
          signalIndex = 0;
          state = ms_collectingData;
        } else if (examStart == false) {
          state = ms_readingButton;
        }
        break;
      }
    case ms_collectingData:
      {
        if ((timerFlagAdc == true) && (timerFlagExamFinished == false)) {
          if ((signalIndex <= 999)) {
            int leituraAdc = FilterSignal(ReadADC(), 0.5f, 41.0f);
            if (signalIndex < 10) {
              leituraAdc = 0;
            }
            SignalFiltred[signalIndex] = leituraAdc;
            signalIndex++;
            timerFlagAdc =  false;
          }
        }
        break;
      }
    case ms_examFinished:
      {
        timerFlagExamFinished = false;
        signalIndex = 0;
        state = ms_buldingImage;
        break;
      }
    case ms_buldingImage:
      {
        RPeakDetection(SignalFiltred);
        ExtractHeartbeats(SignalImages);
        NormalizeHeartbeats ();
        state = ms_sendingImageToClassification;
        break;
      }
    case ms_sendingImageToClassification:
      {
        for (int imageClassificationIndex = 0; imageClassificationIndex < MAX_HEARTBEATS_CAPTURED - 1; imageClassificationIndex++) {
          for (int i = 0; i < 100; i++) {
            if (SignalImagesNormalized[imageClassificationIndex][i] == 0) {
              zeroCountClassifcationImage++;
            }
            features[i] = SignalImagesNormalized[imageClassificationIndex][i];
          }
          if (zeroCountClassifcationImage >= 30) {
            zeroCountClassifcationImage = 0;
            continue;
          }
          else if (zeroCountClassifcationImage < 30) {
            zeroCountClassifcationImage = 0;
            classification();
          }
        }
        //Serial.print("contagem Afib: \n");
        //Serial.print(AFIB_cont);
        //Serial.print("\n");
        //Serial.print("contagem SR: \n");
        //Serial.print(SR_cont);
        //Serial.print("\n");
        if (AFIB_cont > SR_cont) {
          Serial.print("DATA,TIME,");
          Serial.println("1");
        } else if (AFIB_cont < SR_cont) {
          Serial.print("DATA,TIME,");
          Serial.println("0");
        }
        AFIB_cont = 0;
        SR_cont = 0;
        state = ms_wait;
        break;
      }
    case ms_wait:
      {
        delay(1000);
        state = ms_automaticInitiation;
          break;
      }
    default:
      state = ms_wait;
      break;
  }

}

/***********************************************
  Funções.
***********************************************/
/**

*/
void IRAM_ATTR onTimer0() {
  //portENTER_CRITICAL_ISR(&timerMux0);
  //Serial.println("Timer 0 habilita adc");
  timerFlagAdc = true;
  //portEXIT_CRITICAL_ISR(&timerMux0);
}

/**

*/
void IRAM_ATTR onTimer1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  timerAlarmDisable(timer0); // desabilita timer de leitura de adc.
  //count++;
  //if (count >= 100) {
  //Serial.println("Timer 1 finaliza exame");
  timerFlagExamFinished = true;

  timerAlarmDisable(timer1); // desabilita timer de tempo do exame.
  state = ms_examFinished;
  //}
  portEXIT_CRITICAL_ISR(&timerMux1);
}

/**

*/
int16_t ReadADC() {
  int16_t adc0;
  adc0 = ads.readADC_SingleEnded(0);

  //Printa o valor lido no adc no monitor serial.
  //Serial.print("AIN0: "); Serial.println(adc0);

  return adc0;
}

/**

*/
void ConfigSampleRate(uint rate) {
  ads.setDataRate(rate);
}

/**

*/
float FilterSignal(int16_t adcValue, float cutoffHighPassFilter, float cutoffLowPassFilter) {

  float filteredVal = f.filterIn(adcValue * multiplier);
  float finalFilteredVal = fhp.filterIn(filteredVal);
  int avg = avgValue.reading(finalFilteredVal);

  //Printa o valor lido no adc no monitor serial.
  //Serial.print("Valor filtrado: "); Serial.println(final_filteredval);

  return avg;
}

/**

*/
boolean Debounce(uint buttonToRead) {
  volatile bool valueRead = !digitalRead(buttonToRead);
  unsigned long lastDebounceTime = millis();
  while (1) {
    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME_MS) {
      if (valueRead == !digitalRead(buttonToRead)) {
        return valueRead;
      }
      else {
        return false;
      }
    }
  }
}

/**

*/
void ButtonInterrupt() {
  buttonFlag = true;
}

/**

*/
float FoundSignalPeaks(int* heartSignal) {
  int biggerValue = 0;
  int smallerValue = 1000000;
  for (uint i = 0; i < MAX_SIGNAL_POINTS - 1; i++) {
    if (heartSignal[i] > biggerValue) {
      biggerValue = heartSignal[i];
    }
    if (heartSignal[i] < smallerValue) {
      smallerValue = heartSignal[i];
    }
  }
  //Serial.print("Maior valor: "); Serial.println(biggerValue);
  //Serial.print("Menor valor: "); Serial.println(smallerValue);
  return biggerValue;
}

/**

*/
void RPeakDetection(int* heartSignal) {

  int auxIndex = 0;
  int auxValue = 0;
  int maxValue[MAX_SIGNAL_POINTS] = {0};
  int countFlag = 0;
  int tempIndex = 0;
  int peakIndex = 0;
  int tempValue = 0;
  int n = 0;
  float refValue = 0;

  refValue = FoundSignalPeaks(heartSignal) * 0.5;
  //Serial.print("valor_referencia: "); //Serial.println(refValue);

  while (n < MAX_SIGNAL_POINTS - 1) {
    //Serial.print("sinal lido: "); Serial.println(SignalFiltred[n]);
    if (heartSignal[n] > refValue) {
      auxIndex = n;
      auxValue = heartSignal[n];
      for (uint i = 0; i < 5; i++) {
        if (heartSignal[auxIndex + i] > auxValue) {
          tempIndex = auxIndex + i;
          tempValue = heartSignal[auxIndex + i];
          auxValue = heartSignal[auxIndex + i];
        }
        else {
          countFlag = countFlag + 1;
        }
      }
      SignalMaxPointsIndex[peakIndex] = tempIndex + 1;
      maxValue[peakIndex] = tempValue;
      peakIndex++;
      if (countFlag == 5) {
        SignalMaxPointsIndex[peakIndex] = n;
        maxValue[peakIndex] = SignalFiltred[n];
        peakIndex++;
      }
      countFlag = 0;
      tempIndex = 0;
      tempValue = 0;
      auxIndex = 0;
      auxValue = 0;
      //Serial.print("indice_temp+n: "); Serial.println(tempIndex + n);
      for (uint i = 0; i < MAX_SIGNAL_POINTS - 1 ; i++) {
        if (SignalMaxPointsIndex[i] != 0) {
        }
      }
      n = n + 10;
    }
    n = n + 1;
  }

  uint zeroCount = 0;
  for (uint i = 0; i < MAX_SIGNAL_POINTS - 1 ; i++) {
    if (SignalMaxPointsIndex[i] == 0) {
      zeroCount = zeroCount + 1;
    }
  }
  for (uint i = 0; i < MAX_SIGNAL_POINTS - 1 - zeroCount ; i++) {
    if (SignalMaxPointsIndex[i] == 0) {
      SignalMaxPointsIndex[i] = SignalMaxPointsIndex[i + 1];
      SignalMaxPointsIndex[i + 1] = 0;
    }
  }
}

/**

*/
void ExtractHeartbeats(int builtSignal[MAX_HEARTBEATS_CAPTURED][MAX_HEARTBEAT_IMAGE_SIZE]) {
  for (uint i = 0; i < MAX_HEARTBEATS_CAPTURED - 1; i++) {
    if (SignalMaxPointsIndex[i] != 0) {
      uint qrsPoint = SignalMaxPointsIndex[i];
      //Serial.print("Index: ");    Serial.println(qrsPoint);
      for (int j = (-40); j < 60; j++) {
        if (((qrsPoint + j) >= 0) && ((qrsPoint + j) <= MAX_SIGNAL_POINTS)) {
          builtSignal[i][40 + j] = SignalFiltred[(qrsPoint + j)];
          //Serial.print("Construcao de Imagem: ");    Serial.println(builtSignal[i][40 + j]);
        }
      }
    }

    //Serial.println(" ");
    //Serial.println(" ");
  }
}

/**

*/
void NormalizeHeartbeats () {
  for (uint i = 0; i < MAX_HEARTBEATS_CAPTURED - 1; i++) {
    if (SignalMaxPointsIndex[i] != 0) {
      static double biggerValue = 1.0f;
      for (int j = (-40); j < 60; j++) {
        if (SignalImages[i][40 + j] > biggerValue) {
          biggerValue = SignalImages[i][40 + j] + 0.0f;
        }
      }
      for (int j = (-40); j < 60; j++) {
        SignalImagesNormalized[i][40 + j] = SignalImages[i][40 + j] / biggerValue;
        //Serial.print("Sinal máximo: ");    Serial.println(biggerValue);
        //Serial.print("Construcao de Imagem Normalizada: ");
        // Serial.println(SignalImagesNormalized[i][40 + j], 4);
      }
      biggerValue = 1.0f;
    }
    //  Serial.println(" ");
    //  Serial.println(" ");
  }
}

/**

*/
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

/**

*/
void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    Serial.write(print_buf);
  }
}

/**

*/
void classification(void) {
  //ei_printf("Edge Impulse standalone inferencing (Arduino)\n");

  if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    // ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n"
    //    EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
    delay(1000);
    return;
  }

  ei_impulse_result_t result = { 0 };

  // the features are stored into flash, and we don't want to load everything into RAM
  signal_t features_signal;
  features_signal.total_length = sizeof(features) / sizeof(features[0]);
  features_signal.get_data = &raw_feature_get_data;

  // invoke the impulse
  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
  //ei_printf("run_classifier returned: %d\n", res);

  if (res != 0) return;

  // print the predictions
   //ei_printf("Predictions ");
   //ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)");
  //result.timing.dsp, result.timing.classification, result.timing.anomaly);
   //ei_printf(": \n");
  //ei_printf("[");
   for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    //ei_printf("%.5f", result.classification[ix].value);
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    //ei_printf(", ");
#else
    if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
      //ei_printf(", ");
    }
#endif
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  //ei_printf("%.3f", result.anomaly);
#endif
  //ei_printf("]\n");
  float SR_result = 0;
  float AFIB_result = 0;

  // human-readable predictions
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    //ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    if(ix == 0){
      AFIB_result = result.classification[ix].value;
    }
    else{
      SR_result = result.classification[ix].value;
    }
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  //ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

  //SR_result = result.classification[1].value;
  //AFIB_result = result.classification[0].value;
  //Serial.print("valor AFIB:");
  //Serial.print(AFIB_result);
  //Serial.print("\n");
  //Serial.print("valor SR:");
  //Serial.print(SR_result);
  //Serial.print("\n");
  if (SR_result > AFIB_result) {
    //ei_printf("resultado SR\n");
    SR_cont = SR_cont + 1;
  }
  else if (SR_result < AFIB_result) {
    //ei_printf("resultado AFIB\n");
    AFIB_cont = AFIB_cont + 1;
  }

  delay(1000);
  return;
}
