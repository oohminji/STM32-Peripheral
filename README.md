# STM32-Peripheral 신호등 구현
 &ensp; **목표** 
> - Language : C
> - Tool : STM32 Cube IDE
> - Driver : HAL드라이버
> - Timer/Counter, Interrupt, ADC, PWM을 이용한 신호등 제어
 - **PWM** : 3색(R, G, B) LED(신호등)을 밝기 설정 
 - **ADC** : 가변 저항의 값을 ADC변환하여 3색 LED 밝기 조절
 - **Timer/Counter** : 타이머로 3색 LED 색 변경
 - **Interrupt** : 2개의 버튼(+, -) 인터럽트가 발생하면 타이머 속 증가 및 감소

## GPIO Mapping
<img width="380" height="380" alt="blinkgpio" src="https://github.com/user-attachments/assets/47be36be-2a26-4cfc-b13c-8139a314a045" /> <br>
> - TIM4_CH1(PB6) : LED RED
> - TIM4_CH2(PB7) : LED GREEN
> - TIM4_CH3(PB8) : LED BLUE
> - ADC1_IN10(PC0) : 가변저항
> - plus_bt(PC2) : 버튼(+)
> - minus_bt(PC3) : 버튼(-)

## Timer/Counter
- **Peripheral** : TIM4
- **구현** :
1. TIM4는 프리스케일러와 주기를 설정하여 정확히 1ms마다 오버플로우 인터럽트를 생성.
```84MHz 시스템 클럭을 84로 분주(Prescaler=84−1)하여 1MHz의 타이머 클럭을 만들고 Period를 1000으로 설정하여1ms 타이머 인터럽트 생성```

2. HAL_TIM_PeriodElapsedCallback() 함수는 1ms마다 실행.

3. 이 콜백 내부에서 카운터(time4Cnt)가 증가. 이 값이 1000에 도달하면(즉, 1초가 ) 플래그(time4SecFlag)가 설정.

4. 메인 while(1) 루프는 time4SecFlag를 감시. 플래그가 설정되면 1초가 지났음을 알게 되고, 총 경과 시간(time4SecCnt)이 사용자가 정의한 time 변수의 배수인지 확인.
 ```
 if(time4SecCnt % time == 0)
 {
     // 색상을 R -> G -> B 순서로 변경
 }
 ```
5. 이 메커니즘을 통해 버튼 인터럽트에 의해 조정되는 time 변수로 색상 변경 속도를 제어.

## PWM
- **Peripheral** : TIM4
- **Channel** : TIM_CHANNEL_1~3(R,G,B)
- **구현** :
1. TIM4는 PWM 모드로 설정. 타이머의 ARR = 999, Period = (1000-1)로 설정되어 PWM 신호의 주기를 정의

2. 밝기는 0에서 999까지 설정할 수 있는 Pulse 값(CCR)에 의해 결정. Pulse 값이 높을수록 듀티 사이클이 길어져 LED가 더 밝아짐.

3. 메인 루프는 __HAL_TIM_SetCompare() 함수를 호출하여 현재 활성화된 색상 채널의 Pulse 값을 업데이트. Pulse 값은 ADC에 의해 지속적으로 업데이트되는 pwmValue 변수에서 가져옴.

## ADC(아날로그-디지털 변환)
- **Peripheral** : ADC1
- **Channel** : ADC_CHANNEL_10
- **구현** :
1. ADC1은 12비트의 연속 변환 모드로 설정. 이는 입력 전압을 지속적으로 0에서 4095 범위의 디지털 값을 생성.

2. 각 변환이 완료된 후 자동으로 트리거되도록 인터럽트가 활성화(HAL_ADC_Start_IT).

3. HAL_ADC_ConvCpltCallback() 함수는 이 인터럽트 발생 시 실행.<br>
> **함수 내부:**
> - HAL_ADC_GetValue()를 사용하여 디지털 값을 읽음.
> - 이 12비트 값(0-4095)은 다음 공식을 사용하여 PWM의 1000 단계 범위(0-999)로 변환<br>
>   ``` pwmValue = (adcValue * __HAL_TIM_GET_AUTORELOAD(&htim4)) / 4095; ```
> - 결과는 전역 변수 pwmValue에 저장되어, 최신 밝기 설정이 메인 루프에서 사용.


## Interrupt
외부 인터럽트는 메인 루프에서 지속적으로 버튼 상태를 확인할 필요 없이(폴링) 버튼 입력을 즉시 감지
- **Peripheral** : GPIO 및 EXTI (외부 인터럽트 컨트롤러)
- **Pin** : plus_bt_Pin, minus_bt_Pin
- **구현** :

1. GPIO 핀은 GPIO_MODE_IT_RISING 모드로 설정되어, 버튼을 눌렀을 때(신호가 LOW에서 HIGH로 변경될 때) 인터럽트를 트리거.

2. 인터럽트가 발생하면 하드웨어에 의해 HAL_GPIO_EXTI_Callback() 함수가 호출됩.

3. switch 문은 어떤 버튼이 눌렸는지 식별하고 전역 플래그 keyno를 1('+') 또는 2('-')로 설정.

4. 메인 while(1) 루프는 10ms마다 keyno 플래그를 확인합니다(디바운싱 목적). 버튼 입력이 등록된 경우, 색상 변경 지연을 결정하는 time 변수를 조정한 후 keyno를 0으로 재설정.

