#include <Arduino.h>
#include <SimpleFOC.h>

// Simple calc values 
#define DCDC_TO_MV(x) ((29.8-2*(x))*1000)

BLDCMotor motor = BLDCMotor(66);
HallSensor sensor = HallSensor(PD2, PD3, PD4, 66);
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15);
InlineCurrentSense current_sense  = InlineCurrentSense(-2.5, PC1, _NC, PC0);
Commander commander = Commander(Serial);

void doA() { sensor.handleA(); };  // HallSensor A callback
void doB() { sensor.handleB(); };  // HallSensor B callback
void doC() { sensor.handleC(); };  // HallSensor C callback

/*********************************************************************
 * @fn      TIM1_Dead_Time_Init
 *
 * @brief   Initializes TIM1 complementary output and dead time.
 *
 * @param   arr - the period value.
 *          psc - the prescaler value.
 *          ccp - the pulse value.
 *
 * @return  none
 */
void TIM1_Dead_Time_Init( u16 arr, u16 psc )
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_OCInitTypeDef TIM_OCInitStructure={0};
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure={0};

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB | RCC_APB2Periph_TIM1, ENABLE );

	/* TIM1_CH1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	/* TIM1_CH1N */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOB, &GPIO_InitStructure );

	TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

	//TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC1Init( TIM1, &TIM_OCInitStructure );
  TIM_OC2Init( TIM1, &TIM_OCInitStructure );
  TIM_OC3Init( TIM1, &TIM_OCInitStructure );

  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = (uint8_t) ((1.0/29000.0)*0.03 / (1.0/144000000));
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig( TIM1, &TIM_BDTRInitStructure );

  TIM1->CH1CVR = 0;
  TIM1->CH2CVR = 0;
  TIM1->CH3CVR = 0;

	//TIM_CtrlPWMOutputs(TIM1, ENABLE );
	TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Disable );
	TIM_ARRPreloadConfig( TIM1, ENABLE );
	//TIM_Cmd( TIM1, ENABLE );
}

void TIM4_Init( u16 arr, u16 psc )
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	TIM_OCInitTypeDef TIM_OCInitStructure={0};
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* TIM4_CH1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOD, &GPIO_InitStructure );

	TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStructure);

	//TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC1Init( TIM4, &TIM_OCInitStructure );
  TIM_OC2Init( TIM4, &TIM_OCInitStructure );
  TIM_OC3Init( TIM4, &TIM_OCInitStructure );
  TIM_OC4Init( TIM4, &TIM_OCInitStructure );

  TIM4->CH1CVR = 0;
  TIM4->CH2CVR = 0;
  TIM4->CH3CVR = 0;
  TIM4->CH4CVR = 0;

	TIM_CtrlPWMOutputs(TIM4, ENABLE );
	TIM_OC1PreloadConfig( TIM4, TIM_OCPreload_Disable );
	TIM_ARRPreloadConfig( TIM4, ENABLE );
	TIM_Cmd( TIM4, ENABLE );
}

void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){
  _UNUSED(dc_a);
  _UNUSED(dc_b);
  _UNUSED(dc_c);
  _UNUSED(phase_state);
  _UNUSED(params);
  if (((uint8_t) *phase_state) == 0) {
    TIM_CtrlPWMOutputs(TIM1, DISABLE );
    TIM_Cmd( TIM1, DISABLE );
    TIM1->CH1CVR = 0;
    TIM1->CH2CVR = 0;
    TIM1->CH3CVR = 0;
    return;
  }
  // CH1 - V, CH2 - W, CH3 - U
  // A - W, B - V, C - U
  TIM1->CH1CVR = (uint16_t) (dc_b*1000);
  TIM1->CH2CVR = (uint16_t) (dc_a*1000);
  TIM1->CH3CVR = (uint16_t) (dc_c*1000);
  if (TIM1->CTLR1 && TIM_CEN) {
    TIM_CtrlPWMOutputs(TIM1, ENABLE );
    TIM_Cmd( TIM1, ENABLE );
  }
  /*Serial.print(dc_a);
  Serial.print(" ");
  Serial.print(dc_b);
  Serial.print(" ");
  Serial.print(dc_c);
  Serial.print(" ");
  Serial.print(((uint8_t) *phase_state));
  Serial.print(" ");
  Serial.println("Write duty cycle");*/
}

void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

  //if( _isset(pinA) ) pinMode(pinA, INPUT_ANALOG);
  //if( _isset(pinB) ) pinMode(pinB, INPUT_ANALOG);
  //if( _isset(pinC) ) pinMode(pinC, INPUT_ANALOG);

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (3.3f)/(4096.0f)
  };

  return params;
}

#define MV_TO_DAC_VAL(mv) ((mv) * 4096 / 6600)

void Dac_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    DAC_InitTypeDef  DAC_InitType = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    DAC_InitType.DAC_Trigger = DAC_Trigger_None;
    DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
    DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_1, &DAC_InitType);
    DAC_Cmd(DAC_Channel_1, ENABLE);

    DAC_SetChannel1Data(DAC_Align_12b_R, 0);
}

void SetDac(uint16_t value) {
    DAC_SetChannel1Data(DAC_Align_12b_R, value);
}

/* Global Variable */
#define I_CURRENT_INDEX 0
#define HVBUS_INDEX 1
#define HVBUS_CURRENT_INDEX 2
#define LVBUS_CURRENT_INDEX 3
#define TEMP_MG_INDEX 4
#define SUPPLY_INDEX 5
#define TEMP_HVBUS_INDEX 6
#define TEMP_LVBUS_INDEX 7
// U - 9, W - 8
#define FAZEA_INDEX 9
#define FAZEC_INDEX 8
#define LVBUS_INDEX 10
#define TEMP_INV_INDEX 11
#define TEMP_MCU_INDEX 12
u16 TxBuf[13] = {0};
s16 Calibrattion_Val = 0;

/*********************************************************************
 * @fn      ADC_Function_Init
 *
 * @brief   Initializes ADC collection.
 *
 * @return  none
 */
void ADC_Function_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure={0};
	GPIO_InitTypeDef GPIO_InitStructure={0};

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = sizeof(TxBuf)/sizeof(TxBuf[0]);
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	ADC_BufferCmd(ADC1, DISABLE);   //disable buffer
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	Calibrattion_Val = Get_CalibrationValue(ADC1);	

  ADC_TempSensorVrefintCmd(ENABLE);
}

/*********************************************************************
 * @fn      Get_ConversionVal
 *
 * @brief   Get Conversion Value.
 *
 * @param   val - Sampling value
 *
 * @return  val+Calibrattion_Val - Conversion Value.
 */
u16 Get_ConversionVal(s16 val)
{
	if((val+Calibrattion_Val)<0|| val==0) return 0;
	if((Calibrattion_Val+val)>4095||val==4095) return 4095;
	return (val+Calibrattion_Val);
}

/*********************************************************************
 * @fn      DMA_Tx_Init
 *
 * @brief   Initializes the DMAy Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void DMA_Tx_Init( DMA_Channel_TypeDef* DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize)
{
	DMA_InitTypeDef DMA_InitStructure={0};

	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );

	DMA_DeInit(DMA_CHx);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
	DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = bufsize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init( DMA_CHx, &DMA_InitStructure );
}

float _readADCVoltageInline(const int pinA, const void* cs_params){
  //Serial.println("Analog Read");
  //uint32_t raw_adc = analogRead(pinA);
  if (pinA == PC0) {
    return Get_ConversionVal(TxBuf[FAZEA_INDEX]) * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
  } else if (pinA == PC1) {
    return Get_ConversionVal(TxBuf[FAZEC_INDEX]) * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
  } else {
    return 0;
  }
}

void ADCStart() {
  ADC_Function_Init();
  
	DMA_Tx_Init( DMA1_Channel1, (u32)&ADC1->RDATAR, (u32)TxBuf, sizeof(TxBuf)/sizeof(TxBuf[0]) );
	DMA_Cmd( DMA1_Channel1, ENABLE );

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );   // Exitation Current
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5 );   // HVBUS Voltage
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5 );   // 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5 );   // 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_239Cycles5 );   // MG Temp
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 6, ADC_SampleTime_239Cycles5 );   // Supply Voltage
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 7, ADC_SampleTime_239Cycles5 );   // 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 8, ADC_SampleTime_239Cycles5 );   // 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 9, ADC_SampleTime_239Cycles5 );  // current faze 1
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 10, ADC_SampleTime_239Cycles5 ); // current faze 2
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 11, ADC_SampleTime_239Cycles5 ); // LVBUS Voltage
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 12, ADC_SampleTime_239Cycles5 ); // Temp Inverter
  ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 13, ADC_SampleTime_239Cycles5 ); // Temp Sensor
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

#define DCDC_ENABLE PE10
#define DCDC_12V_ENABLE PE11
#define PRECHARGE PE12
#define CONTACTOR PE13
#define LOGIC_ENABLE PE14
#define CABIN1 PD5
#define CABIN2 PD6

void configureGPIOOutputs() {
  //pinMode(PD12, OUTPUT);
  //pinMode(PD13, OUTPUT);
  //pinMode(PD14, OUTPUT);
  //pinMode(PD15, OUTPUT);
  pinMode(PE10, OUTPUT); // DCDC Enable
  pinMode(PE11, OUTPUT); // DCDC 12V Enable
  pinMode(PE12, OUTPUT); // PRECHARGE
  pinMode(PE13, OUTPUT); // CONTACTOR
  pinMode(PE14, OUTPUT); // 5V LOGIC Enable
  pinMode(PD5, INPUT_PULLUP); // CABIN1
  pinMode(PD6, INPUT_PULLUP); // CABIN2
  digitalWrite(LOGIC_ENABLE, HIGH);
  digitalWrite(DCDC_ENABLE, LOW);
  digitalWrite(DCDC_12V_ENABLE, LOW);
  delay(200);
  digitalWrite(PRECHARGE, LOW);
  digitalWrite(CONTACTOR, LOW);
}

void onMotor(char* cmd){commander.motor(&motor, cmd);}

#define F_COFF 50 // Face mul coeff was 100x
#define F_MID 2.5

void setup() {
  //Serial.setRx(PC11);
  //Serial.setTx(PC10);
  Serial.begin(115200);
  pinMode(PD11, OUTPUT);
  pinMode(PD2, INPUT);
  pinMode(PD3, INPUT);
  pinMode(PD4, INPUT);
  //pinMode(PC0, INPUT);
  //pinMode(PC1, INPUT);
  //pinMode(PA8, OUTPUT);
  //pinMode(PB13, OUTPUT);
  //pinMode(PA9, OUTPUT);
  //pinMode(PB14, OUTPUT);
  //pinMode(PA10, OUTPUT);
  //pinMode(PB15, OUTPUT);
  digitalWrite(PD11, HIGH);
  TIM4_Init( 1000, 4 );
  configureGPIOOutputs();
  Dac_Init();
  SetDac(MV_TO_DAC_VAL(5000));
  digitalWrite(DCDC_ENABLE, HIGH);
  delay(100);
  digitalWrite(DCDC_ENABLE, LOW);
  SetDac(MV_TO_DAC_VAL(DCDC_TO_MV(14.2)));
  ADCStart();
  sensor.init();
  attachInterrupt(PD2, GPIO_Mode_IN_FLOATING, doA, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
  attachInterrupt(PD3, GPIO_Mode_IN_FLOATING, doB, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
  attachInterrupt(PD4, GPIO_Mode_IN_FLOATING, doC, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  current_sense.skip_align = true;
  current_sense.gain_a = F_COFF;
  current_sense.gain_c = F_COFF;
  current_sense.offset_ia = F_MID;
  current_sense.offset_ic = F_MID;
  motor.zero_electric_angle = 0; 
  motor.sensor_direction = Direction::CW;
  driver.init();
  TIM1_Dead_Time_Init( 1000, 4 );
  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.init();
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_D | _MON_CURR_Q; 
  motor.monitor_downsample = 26000;
  commander.add('m',onMotor,"my motor");
  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  motor.initFOC();
  motor.enable();
  motor.move(0);
}

#define GET_ADC_VAL(x) ((float)Get_ConversionVal(TxBuf[x])*(3.3f)/(4096.0f))
#define I_MAX 500
#define I_MIN 0
#define I_TARGET 1.0f
#define I_COFF 3.0f
#define I_PERIOD 1
#define I_STEP 1
#define V_TARGET 41.6f
#define HVBUS_COFF 12.0f
char c;
u32 start_time = 0;
u16 counter = 0;
u16 i_val = I_MIN;
u16 i_start_time = 0;

String cmd = "";

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Hello, world!");
  //digitalToggle(PD11);
  //delay(500);
  c = Serial.read();
  if (c != -1){
    if (c == '\n') {
      commander.run((char*)cmd.c_str());
      cmd = "";
    } else if (c != '\r'){
      cmd += c;
    }
  }
  motor.loopFOC();
  motor.move();
  motor.monitor();
  if (millis() - i_start_time > I_PERIOD) {
    i_start_time = millis();
    if (GET_ADC_VAL(I_CURRENT_INDEX)*I_COFF < I_TARGET && GET_ADC_VAL(HVBUS_CURRENT_INDEX)*HVBUS_COFF < V_TARGET) {
      if (i_val < I_MAX) i_val += I_STEP;
    } else {
      if (i_val > I_MIN) i_val -= I_STEP;
    }
    TIM4->CH1CVR = i_val;
  }
  /*counter++;
  if (millis() - start_time > 1000) {
    start_time = millis();
    Serial.print("Counter: ");
    Serial.print(counter);
    Serial.print(" ");
    Serial.print("AIN: ");
    for (u8 i = 0; i < sizeof(TxBuf)/sizeof(TxBuf[0]); i++) {
      Serial.print((float)Get_ConversionVal(TxBuf[i])*(3.3f)/(4096.0f));
      Serial.print(" ");
    }
    Serial.print("Cabin: ");
    Serial.print(digitalRead(PD5));
    Serial.print(" ");
    Serial.print(digitalRead(PD6));
    Serial.print("Shaft: ");
    Serial.print(sensor.getVelocity());
    Serial.print(" ");
    Serial.print(sensor.getAngle());
    Serial.print("Curent: ");
    Serial.print(current_sense.getPhaseCurrents().a);
    Serial.print(" ");
    Serial.print(current_sense.getPhaseCurrents().b);
    Serial.print(" ");
    Serial.print(current_sense.getPhaseCurrents().c);
    Serial.print(" ");
    Serial.println();
    counter = 0;
  }*/
}
