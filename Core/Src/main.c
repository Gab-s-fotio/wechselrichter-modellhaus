/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Anzahl Sinus-Stuetzstellen pro 50Hz-Vollperiode. Bei 10kHz Traeger
 * (siehe TIM1 ARR=16799 -> 168MHz/16800=10kHz) ergibt 200 Werte genau
 * 10000/200 = 50Hz Ausgangsfrequenz. */
#define SAMPLES_PER_CYCLE   200U

/* Amplitude der Sinus-Ausgangsspannung relativ zur maximal moeglichen
 * Amplitude (0.0 .. 1.0). Spitzenwert am Ausgang = MODULATION_INDEX * 24V.
 * 12V-Lampe: 12V eff. * 1.414 = 17V Spitze -> 17/24 = 0.70. */
#define MODULATION_INDEX    0.70f

/* Messfenster fuer die Effektivwert-Berechnung (RMS) in Millisekunden.
 * 100ms = 5 volle 50Hz-Perioden; mittelt ausserdem das Sensorrauschen. */
#define RMS_WINDOW_MS       100U

/* ACS712-20A: 100mV pro Ampere. Der Nullpunkt (VCC/2) wird nicht fest
 * angenommen, sondern beim Start gemessen (siehe Sensors_CalibrateZero),
 * da die 5V vom Board nie genau 5.00V sind. */
#define ACS712_SENSITIVITY_V_PER_A   0.100f
#define ACS712_CALIB_SAMPLES         64U

/* ZMPT101B-Kalibrierung auf die 50Hz-Grundschwingung (gefiltertes Signal):
 * Referenzmessung mit Analog Discovery 2 / WaveForms (Differenzmessung
 * Klemme A - Klemme B, Ausgangsfilter 1mH + 7uF): 11.801V eff. bei
 * voltageFundRmsRaw = 87.62 -> 0.1347 V pro ADC-Schritt. Die Daempfung des
 * digitalen Filters (ca. 6% bei 50Hz) ist darin enthalten. Gilt nur, solange
 * das Poti auf dem ZMPT101B-Modul nicht verstellt wird! */
#define ZMPT101B_VOLTS_PER_RAW       (11.801f / 87.6176f)

/* Hysterese fuer die Nulldurchgangserkennung (Frequenzmessung) in
 * ADC-Schritten. Rauschen ca. 5 Schritte, Sinus-Amplitude ca. 125 Schritte. */
#define FREQ_HYSTERESIS_RAW          30.0f

/* Grenzfrequenz des digitalen Tiefpasses (2x 1. Ordnung) fuer Frequenzmessung
 * und Kurvenanzeige: entfernt Reste des 10kHz-PWM-Anteils und Schaltstoerungen,
 * die der Sensor aufnimmt. Bei 200Hz bleiben vom 50Hz-Sinus ca. 94% der Amplitude.
 * Der Filterfaktor wird aus der echten Zeit zwischen zwei Messungen berechnet,
 * weil die Abtastrate von der Laufzeit des Codes abhaengt. */
#define FREQ_FILTER_CUTOFF_HZ        200.0f

/* Einschwingzeit des Filters, bevor seine Werte in den Effektivwert der
 * Grundschwingung eingehen (Zeitkonstante je Stufe ca. 0,8ms). */
#define FILTER_SETTLE_MS             10U

/* Sperrzeit nach einem gezaehlten Nulldurchgang (halbe 50Hz-Periode):
 * Restzittern direkt am Nulldurchgang kann so nicht doppelt zaehlen. */
#define FREQ_HOLDOFF_MS              10U

/* Abstand der Werte fuer die Kurvenanzeige (SWV Data Trace Timeline Graph)
 * in Mikrosekunden: 500us -> 40 Punkte pro 20ms-Periode. Deutlich schneller
 * ueberfordert die SWO-Leitung des Debuggers. */
#define SCOPE_INTERVAL_US            500U

/* ADC-Referenz des STM32 (VDDA). Auf dem STM32F4DISCOVERY laeuft der
 * Controller mit 3.0V, nicht mit 3.3V. */
#define ADC_VREF        3.0f
#define ADC_MAX_VALUE   4095.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_up;

/* USER CODE BEGIN PV */
/* Sinus-Lookup-Tabelle: DMA schreibt diese Werte nacheinander (zyklisch)
 * in TIM1->CCR1 - das ist die eigentliche SPWM-Erzeugung. */
static uint16_t sineTable[SAMPLES_PER_CYCLE];

/* Zuletzt gemessene ADC-Rohwerte (0..4095), nur zur Beobachtung im
 * Debugger (Live Expressions) - keine Regelung. */
static volatile uint16_t currentRaw = 0;
static volatile uint16_t voltageRaw = 0;

/* Effektivwert (RMS) des Lampenstroms in Ampere (ACS712-20A), ueber
 * RMS_WINDOW_MS gemittelt und um das Grundrauschen bereinigt. */
static volatile float currentAmps = 0.0f;

/* voltageRmsRaw: Effektivwert des ungefilterten Wechselanteils in ADC-
 * Schritten (nur Diagnose, enthaelt auch Schaltstoerungen).
 * voltageVolts: Effektivwert der 50Hz-Lampenspannung in Volt, berechnet aus
 * voltageFundRmsRaw (Umrechnung siehe ZMPT101B_VOLTS_PER_RAW). */
static volatile float voltageRmsRaw = 0.0f;
static volatile float voltageVolts  = 0.0f;

/* Frequenz der Lampenspannung in Hz, aus den Nulldurchgaengen gemessen
 * (0, solange keine Wechselspannung anliegt). */
static volatile float frequencyHz = 0.0f;

/* Momentane Lampenspannung in Millivolt (gefiltert, ohne 10kHz-Zittern) fuer
 * die Kurvenanzeige im Debugger. Bewusst nicht static und als Ganzzahl, damit
 * der SWV Data Trace sie per Adresse findet und richtig darstellt. */
volatile int32_t scopeMillivolts = 0;

/* Tatsaechliche Anzahl Messpaare (Strom + Spannung) pro Sekunde, zur
 * Kontrolle im Debugger. */
static volatile float adcPairsPerSecond = 0.0f;

/* Effektivwert nur der 50Hz-Grundschwingung der Spannung (gefiltert) in
 * ADC-Schritten. voltageRmsRaw enthaelt dagegen auch das 10kHz-Zittern. */
static volatile float voltageFundRmsRaw = 0.0f;

/* Gleichspannungs-Verschiebung der gemessenen Spannung (Mittelwert des
 * letzten Messfensters) in ADC-Schritten. Wird fuer Nulldurchgaenge und
 * Kurvenanzeige abgezogen; zur Diagnose im Debugger sichtbar. */
static volatile float voltageOffsetRaw = 0.0f;

/* ADC-Rohwerte bei 0A bzw. 0V, beim Start gemessen (Bruecke noch aus). */
static volatile float currentZeroRaw = 0.0f;
static volatile float voltageZeroRaw = 0.0f;

/* Grundrauschen des ACS712 (RMS in ADC-Schritten) ohne Strom. */
static volatile float currentNoiseRaw = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Berechnet die Sinus-Tabelle einmalig beim Start. duty = 0.5*(1 + m*sin(x))
 * ergibt eine bipolare Modulation um 50% Tastverhaeltnis herum, die direkt
 * an der H-Bruecke (CH1/CH1N) eine Wechselspannung erzeugt. */
static void SPWM_ComputeSineTable(void)
{
  const uint32_t period = htim1.Init.Period;

  for (uint32_t i = 0; i < SAMPLES_PER_CYCLE; i++)
  {
    float angle = 2.0f * (float)M_PI * (float)i / (float)SAMPLES_PER_CYCLE;
    float duty  = 0.5f * (1.0f + MODULATION_INDEX * sinf(angle));
    sineTable[i] = (uint16_t)(duty * (float)period);
  }
}

/* Liest einen einzelnen ADC-Kanal per Single-Conversion (Polling, kein DMA -
 * fuer reines Monitoring voellig ausreichend). */
static uint16_t ADC_ReadChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  uint16_t value = 0;
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);

  return value;
}

/* Startet den Zykluszaehler des Cortex-M4 (DWT->CYCCNT). Er zaehlt mit dem
 * CPU-Takt (168MHz) und liefert damit Zeitstempel mit ca. 6ns Aufloesung -
 * HAL_GetTick() waere mit 1ms fuer die Frequenzmessung zu grob. */
static void DWT_InitCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Misst Strom und Spannung ueber RMS_WINDOW_MS abwechselnd so oft wie
 * moeglich und liefert die Effektivwerte (Abweichung vom Nullpunkt) in
 * ADC-Schritten sowie die Frequenz der Spannung. Blockiert fuer
 * RMS_WINDOW_MS. */
static void Sensors_MeasureRms(float zeroI, float zeroU, float *rmsI, float *rmsU,
                               float *freqHz)
{
  float sumSqI = 0.0f;
  float sumSqU = 0.0f;
  uint32_t n = 0;
  const uint32_t start = HAL_GetTick();

  /* Nulldurchgangserkennung: Eine steigende Flanke zaehlt erst, wenn die
   * Spannung vorher unter -Hysterese war und jetzt ueber +Hysterese steigt.
   * So loest das Rauschen um den Nullpunkt keine Fehlzaehlung aus. */
  uint8_t  wasNegative = 0;
  uint32_t crossings = 0;
  uint32_t firstCrossCycles = 0;
  uint32_t lastCrossCycles = 0;
  float    uFilt1 = 0.0f;
  float    uFilt2 = 0.0f;
  float    sumSqF = 0.0f;
  float    sumF = 0.0f;
  float    sumU = 0.0f;
  uint32_t nF = 0;
  const uint32_t holdoffCycles = (SystemCoreClock / 1000U) * FREQ_HOLDOFF_MS;
  const uint32_t scopeCycles = (SystemCoreClock / 1000000U) * SCOPE_INTERVAL_US;
  uint32_t lastScopeCycles = DWT->CYCCNT;
  uint32_t lastSampleCycles = DWT->CYCCNT;
  const float omegaCutoff = 2.0f * (float)M_PI * FREQ_FILTER_CUTOFF_HZ;

  while ((HAL_GetTick() - start) < RMS_WINDOW_MS)
  {
    currentRaw = ADC_ReadChannel(ADC_CHANNEL_0);
    voltageRaw = ADC_ReadChannel(ADC_CHANNEL_1);
    float dI = (float)currentRaw - zeroI;
    float dU = (float)voltageRaw - zeroU;
    sumSqI += dI * dI;
    sumSqU += dU * dU;
    sumU   += dU;
    n++;

    /* Gefiltertes Signal (nur 50Hz-Grundschwingung) fuer Frequenz, Kurve
     * und Grundschwingungs-Effektivwert; der RMS-Wert oben bleibt ungefiltert.
     * Filterfaktor aus der tatsaechlichen Zeit seit der letzten Messung:
     * alpha = w*dt / (1 + w*dt) - entspricht einem RC-Tiefpass. */
    uint32_t nowSample = DWT->CYCCNT;
    float dt = (float)(nowSample - lastSampleCycles) / (float)SystemCoreClock;
    lastSampleCycles = nowSample;
    float alpha = (omegaCutoff * dt) / (1.0f + omegaCutoff * dt);

    uFilt1 += alpha * (dU - uFilt1);
    uFilt2 += alpha * (uFilt1 - uFilt2);

    /* Die ersten ms auslassen, bis der Filter (Start bei 0) eingeschwungen ist. */
    if ((HAL_GetTick() - start) >= FILTER_SETTLE_MS)
    {
      sumSqF += uFilt2 * uFilt2;
      sumF   += uFilt2;
      nF++;
    }

    /* Wechselanteil: gefiltertes Signal ohne die Gleichspannungs-Verschiebung
     * (Mittelwert des vorherigen Messfensters). */
    float uAc = uFilt2 - voltageOffsetRaw;

    /* Im festen Raster einen Kurvenpunkt fuer den Debugger ausgeben. */
    if ((DWT->CYCCNT - lastScopeCycles) >= scopeCycles)
    {
      lastScopeCycles += scopeCycles;
      scopeMillivolts = (int32_t)(uAc * ZMPT101B_VOLTS_PER_RAW * 1000.0f);
    }

    if (uAc < -FREQ_HYSTERESIS_RAW)
    {
      wasNegative = 1;
    }
    else if (wasNegative && (uAc > FREQ_HYSTERESIS_RAW))
    {
      wasNegative = 0;
      uint32_t now = DWT->CYCCNT;
      if ((crossings == 0) || ((now - lastCrossCycles) > holdoffCycles))
      {
        if (crossings == 0)
        {
          firstCrossCycles = now;
        }
        lastCrossCycles = now;
        crossings++;
      }
    }
  }

  *rmsI = sqrtf(sumSqI / (float)n);
  adcPairsPerSecond = (float)n * 1000.0f / (float)RMS_WINDOW_MS;

  /* Spannung: nur den Wechselanteil bewerten. Effektivwert um den Mittelwert
   * herum: RMS_ac = sqrt(mean(x^2) - mean(x)^2). Eine Gleichspannungs-
   * Verschiebung (Nullpunktdrift des Sensors) faelscht so den Wert nicht. */
  float meanU = sumU / (float)n;
  float varU  = sumSqU / (float)n - meanU * meanU;
  *rmsU = (varU > 0.0f) ? sqrtf(varU) : 0.0f;
  if (nF > 0U)
  {
    float meanF = sumF / (float)nF;
    float varF  = sumSqF / (float)nF - meanF * meanF;
    voltageFundRmsRaw = (varF > 0.0f) ? sqrtf(varF) : 0.0f;
    voltageOffsetRaw  = meanF;
  }

  /* Zwischen dem ersten und letzten Nulldurchgang liegen (crossings - 1)
   * volle Perioden. Bei 50Hz und 100ms Fenster sind das 4 Perioden. */
  if (crossings >= 2)
  {
    float seconds = (float)(lastCrossCycles - firstCrossCycles) / (float)SystemCoreClock;
    *freqHz = (float)(crossings - 1) / seconds;
  }
  else
  {
    *freqHz = 0.0f;
  }
}

/* Misst die Nullpunkte beider Sensoren und das Grundrauschen des ACS712.
 * Muss aufgerufen werden, BEVOR die H-Bruecke Strom liefert. */
static void Sensors_CalibrateZero(void)
{
  uint32_t sumI = 0;
  uint32_t sumU = 0;
  for (uint32_t i = 0; i < ACS712_CALIB_SAMPLES; i++)
  {
    sumI += ADC_ReadChannel(ADC_CHANNEL_0);
    sumU += ADC_ReadChannel(ADC_CHANNEL_1);
    HAL_Delay(1);
  }
  currentZeroRaw = (float)sumI / (float)ACS712_CALIB_SAMPLES;
  voltageZeroRaw = (float)sumU / (float)ACS712_CALIB_SAMPLES;

  /* Ohne Strom ist der gemessene "Effektivwert" reines Rauschen. */
  float noiseU;
  float noiseI;
  float noiseF;
  Sensors_MeasureRms(currentZeroRaw, voltageZeroRaw, &noiseI, &noiseU, &noiseF);
  currentNoiseRaw = noiseI;
}

/* Rechnet einen Strom-Effektivwert (ADC-Schritte) in Ampere um und zieht
 * dabei das Grundrauschen ab (Rauschen und Signal addieren sich quadratisch). */
static float ACS712_RmsToAmps(float rmsRaw)
{
  float sq = rmsRaw * rmsRaw - currentNoiseRaw * currentNoiseRaw;
  float signalRaw = (sq > 0.0f) ? sqrtf(sq) : 0.0f;
  float volts = (signalRaw / ADC_MAX_VALUE) * ADC_VREF;
  return volts / ACS712_SENSITIVITY_V_PER_A;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  SPWM_ComputeSineTable();

  /* Zeitbasis fuer die Frequenzmessung starten. */
  DWT_InitCycleCounter();

  /* Nullpunkte der Sensoren messen, solange die Bruecke noch aus ist. */
  Sensors_CalibrateZero();

  /* BTS7960 R_EN/L_EN dauerhaft freigeben (per Schaltplan fest auf den
   * STM32-Pins PC0/PC1, keine Deaktivierung noetig fuer dieses Projekt). */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);

  /* SPWM per DMA starten: TIM1-Update-Event laedt automatisch den naechsten
   * Sinus-Wert aus sineTable[] in TIM1->CCR1, komplett ohne CPU-Eingriff.
   * HAL_TIM_PWM_Start_DMA() passt hier NICHT (die erwartet eine an das
   * Capture/Compare-Event gekoppelte DMA, hdma[TIM_DMA_ID_CC1]) - wir haben
   * aber bewusst die Update-Event-DMA (hdma_tim1_up) verknuepft. Daher DMA
   * und Kanal-Ausgaenge hier direkt manuell starten. */
  HAL_DMA_Start(&hdma_tim1_up, (uint32_t)sineTable, (uint32_t)&htim1.Instance->CCR1, SAMPLES_PER_CYCLE);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Dauert RMS_WINDOW_MS, liefert also alle 100ms neue Effektivwerte. */
    float rmsI;
    float rmsU;
    float freq;
    Sensors_MeasureRms(currentZeroRaw, voltageZeroRaw, &rmsI, &rmsU, &freq);
    currentAmps   = ACS712_RmsToAmps(rmsI);
    voltageRmsRaw = rmsU;
    voltageVolts  = voltageFundRmsRaw * ZMPT101B_VOLTS_PER_RAW;
    frequencyHz   = freq;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 16799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 20;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* BTS7960 sofort abschalten (R_EN/L_EN low), damit die H-Bruecke bei
   * einer fehlgeschlagenen Initialisierung nicht unkontrolliert schaltet.
   * Danach anhalten - im Debugger sieht man so, dass ein Fehler auftrat. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
