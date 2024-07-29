/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
enum states{EMITTING, RECEIVING}; //states for trigger
enum inter_sound{NO_INTER, PARPADEO_OFF, PARPADEO_ON}; //states for the buzzer

enum movement_direction{forwards, backwards, turn_left, turn_right, stop}; //states for directions of the car
enum obstacle_det{OBSTACLE_YES, OBSTACLE_TURN, OBSTACLE_NO}; //for the dynamics of the obstacles


char obstacle, pre_obstacle; //control states of dynamics
char intermitente, pre_intermitente; //control the states of buzzer
char state, pre_state; //control para el trigger
char direction; //var para controlar la dirección, cambia de valor con las funciones del coche

unsigned int pulse_duration = 0; //var que guarda la distancia que el echo recibe
char recv = 0; //var control para el tigger, solo para cambiar de uno a otro

//max value for the wheels
float left_velocityMax = 80;
float right_velocityMax = 80;

//ruedad controladas con estas variables, son con las que se trabajan
float left_velocityAc;
float right_velocityAc;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TS_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void trigger_OFF();
void trigger_ON();
void buzzerSOUND();
void buzzerNO_SOUND();
void sound_on_inter();
void sound_off_inter();

void backards_right();
void forwards_right();
void backwards_left();
void forwards_left();

void moveForwards();
void moveBackwards();

void stop_right();
void stop_left();
void stop_movement();

void turn_LEFT();
void turn_RIGHT();

void new_velocity();
void set_direction(char new_direction);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
////////////////////////////////////////////MOVEMENT FUNCTIONS//////////////////////

    //for velocity of the wheels, we use PWM
    //CH3, para PB8
    //CH4, para PB9

void forwards_right(){
  TIM4->CCR3 = right_velocityAc;     //CH3-PB8,                    CCR3: states the value to be reached by the CNT to obtain the match event
  GPIOA -> BSRR |= (1<<(11+16));  //poner un 0 en el PA11            states the velocity its taking in the PWM, la actual que tengamos en ese momento
}

void backwards_right(){
  TIM4->CCR3 = 100-right_velocityAc;   //CH3                       CCR3: the rest of the velocity remaining, the opossite we had before
  GPIOA -> BSRR |= (1<<11); //poner un 1 en PA11
}


void forwards_left(){
  TIM4->CCR4 = left_velocityAc;     //CH4-PB9,
  GPIOA -> BSRR |= (1<<(12+16));  //poner un 0 en el PA12
}

void backwards_left(){
  TIM4->CCR4 = 100-left_velocityAc;   //CH4
  GPIOA -> BSRR |= (1<<12); //poner un 1 en PA12
}

void stop_right(){
  TIM4->CCR3 = 0;     //CH3                                           CCR3: set to 0 to stop the movement
  GPIOA -> BSRR |= (1<<(11+16));  //poner un 0 en el PA11
}

void stop_left(){
  TIM4->CCR4 = 0;     //CH4
  GPIOA -> BSRR |= (1<<(12+16));  //poner un 0 en el PA12
}

void stop_movement(){
  stop_left();
  stop_right();
  direction=stop;
}

void moveForwards(){
  forwards_right();
  forwards_left();
  direction=forwards;
}

void moveBackwards(){
  backwards_right();
  backwards_left();
  direction=backwards;
}

//to turn, we need one wheel stops, and the other goes or different speeds//////////

void turn_LEFT(){   //para girar izq, paramos izq, y para atrás der
  backwards_right();
  stop_left();
  direction=turn_left; //var direction-la que controla donde se mete en el swtich depués --PROBAR SIN ESTO
}

void turn_RIGHT(){  //para girar der, paramos izq, y para atrás izq
  backwards_left();
  stop_right();
  direction=turn_right;
}

void set_direction(char new_direction){
  direction= new_direction;
  new_velocity();
}

void new_velocity(){
  switch(direction){
  case forwards:
    moveForwards();
    break;
  case backwards:
    moveBackwards();
    break;
  case turn_left:
    turn_LEFT();
    break;
  case turn_right:
    turn_RIGHT();
    break;
  case stop:
    stop_movement();
    break;
  }
}



////////////////////////////////////////////TRIGGER FUNCTIONS////////////////////////
void trigger_ON(){
  GPIOD->BSRR |= (1<<2);
}

void trigger_OFF(){
  GPIOD->BSRR |= (1<<2)<<16;
}


////////////////////////////////////////////BUZZER FUNCTIONS/////////////////////////
void buzzerNO_SOUND(){
  GPIOA->BSRR |= (1<<1);
}


void buzzerSOUND(){
  GPIOA->BSRR |= (1<<1)<<16;
}

void sound_on_inter(){
  if (intermitente == NO_INTER) //si el intermitente esta en el estado, no_inter, tiene que pasar a estar intermitente, no_inter es que activamos la intermitencia del buzzer
  {
    intermitente = PARPADEO_ON;         //se cambia a que sea intermitente
    TIM3->CNT = 0;                      //contador a 0, pq TOC
    TIM3->CCR1 = 1000;                  //valor al que tiene que llegar el CNT, la cuenta, tiempo que va a ser intermitente, el tiempo que va a estar sonando y el que no
    TIM3->CR1 |= (1 << 0);              //indica el counter enable, or disable, its = 1, so counter enable
  }
}

void sound_off_inter(){
  intermitente = NO_INTER; //estado en no intermitente y no tiene que hacer nada
  TIM3->CR1 = 0; //contador 0 pq nada que contar
}



//TIM2//////////////////////////////////////////////////////////////////////////////

    //TIM2CH1 as TIC echo
    //TIM2CH2 counting 10us
    //TIM2CH3 para el tiempo que para y el tiempo que gira el coche

void TIM2_IRQHandler(void)
{
  if (((TIM2->SR & (1 << 1)) != 0)) //si la flag del channel1 esta activada, event is ocurring
  {
    if (recv == 0) //si no estoy recibiendo nada, recv es variable de control solo para esto
    {
      //si no recibo nada el pulso va a estar en falling

      recv = 1; //cambio el estado para que pase al else, para que cambie de estado
      TIM2->CNT = 0; //contador a 0, para la cuenta TI: measuure time lapse between 2 events
      TIM2->CCER = 0X0003; // ponemos el pulso a falling, pq si no esta recibiendo el pulso es bajo

    }
    else  //es cuando recv=1, recibimos pulso(señal)
    {
      recv = 0;                    //para cambiar de estado, para que vaya al if
      TIM2->CCER = 0X0001;         //rising el pulso, si recibo algo, pulso alto, empiezo a contar en el CCR1, para saber cuanto esta durando el pulso arriba y medir dst
      pulse_duration = TIM2->CCR1; //CCR1, store el valor cuando un evento externo ocurre, en este caso, el rising edge, guardamos esto en pulse_duration
                                   //para saber la distancia que hemos medido,

      ADC1->CR2 |= 0X40000000;  //launch adc
                                //start the conversion
    }

    TIM2->SR &= ~(1 << 1); //Clear channel 1 flag

  }

  else if ((TIM2->SR & (1 << 2) != 0) && (TIM2->SR & (1 << 3)) != 0) //timer dos para el toc del trigger, en el channel 2
  {

    if (state == EMITTING) //si estamos emitiendo el pulso lo queremos leer
    {
      state = RECEIVING; //cambiamos el estado para despues recibir
      obstacle = OBSTACLE_YES; //Decimos que hay objeto, para parar el coche cuando se encuentra un objeto
      TIM2->CCR2 += 2500; //25ms, time reading the distance, TOC, cuenta atrás, CNT cuenta hasta que llega a ese valor
      TIM2->CCR3 += 100000000; //0,5S, TIME STOP, tiempo parado del coche, cuando llega a ese valor, avanza


    }
    else if (state == RECEIVING) //Si estamos recibiendo
    {
      state = EMITTING; //cambiamos el estado para despues pasar a recibir
      obstacle = OBSTACLE_TURN; //despues de parar el coche queremos que gire
      TIM2->CCR2 += 10; //10us, tiempo recibiendo, TOC, recibe hasta que el CNT llega a ese valor
      TIM2->CCR3 += 1000000;  //time turning, el coche gira hasta que llega a este valor

    }
    else
    {
      obstacle = OBSTACLE_NO; // no hay obstaculo, seguimos normal

      //TIM2->CCR3 += 1000000;//10us
    }

    TIM2->SR &= ~(1 << 2); //clear the flag
    TIM2->SR &= ~(1 << 3); //clear the flag
  }

}

//TIM3///////////////////////////////////////////////////////////////////////////////

    //TIM3-ch1 buzzer intermitente

void TIM3_IRQHandler(void) //RUTINA DE INTERRUPCION DEL BUZZER
{
  if ((TIM3->SR & (1 << 1)) != 0) //comprobamos que flag del ch1 esta activada
  {
    if (intermitente == PARPADEO_OFF) //si esta apagada
    {
      intermitente = PARPADEO_ON; //la encendemos para cambiar el estado
    }
    else if (intermitente == PARPADEO_ON) //si esta encendida
    {
      intermitente = PARPADEO_OFF; //la apagamos para cambiar el estado
    }

    TIM3->CCR1 += 1000; //tiempo que suena, valor al que tiene que llegar el CNT, la cuenta, la incrementamos
    TIM3->SR &= ~(1 << 1); //clear the flag
  }

}

//ADC//////////////////////////////////////////////////////////////////////////////

void ADC1_IRQHandler(void){ //rutina de interrupción
  if((ADC1->SR & (1<<1)) != 0){ //ver si hay un 1 en el pin del EOC, que lo hay nos indica que la conversion esta completa
    //asignamos valor a la velocidad maxima de la rueda, el 4095 es por la posicion del potenciometro, el +50 para que siempre tenga velocidad
    left_velocityMax = (ADC1->DR) * (99-50)/(4095)+50;
    right_velocityMax = (ADC1->DR) * (99-50)/(4095)+50;       //el ADC1->DR, valor que cambiara la velocidad, adc_value que teniamos antes
  }
  ADC1->DR = 0;
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
  MX_ADC_Init();
  MX_TS_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //iniciar los pins /////////////////////////////////////////////////////////////////////////////////////

   //PB8- as alternate function
   GPIOB->MODER |= (1 << (8 * 2 + 1)); //coloco un 1
   GPIOB->MODER &= ~(1 << (8 * 2)); // coloco un 0

   //PA11- as digital output
   GPIOA->MODER &= ~(1 << (11 * 2 + 1)); //coloco un 0
   GPIOA->MODER |= (1 << (11 * 2)); // coloco un 1

   //PB9- as alternate function
   GPIOB->MODER |= (1 << (9 * 2 + 1)); //coloco un 1
   GPIOB->MODER &= ~(1 << (9 * 2)); // coloco un 0

   //PA12- as digital output
   GPIOA->MODER &= ~(1 << (12 * 2 + 1)); //coloco un 0
   GPIOA->MODER |= (1 << (12 * 2)); // coloco un 1

   //AFR[1]para PB8 y PB9, 0001
   GPIOB->AFR[1] = 0X00000022;



   //buffer PA1 as DO(01)
   GPIOA->MODER &= ~(1 << (1 * 2 + 1));  //coloco un 0
   GPIOA->MODER |= (1 << (1 * 2));   //coloco un 1

   //GPIOA->AFR[0] |=  (0x01 << 1*4);
   //GPIOA->AFR[0] &= ~(0x0E << 1*4);
   //GPIOA -> AFR[0] |= (0x10 << (1*4));     //AFR[0] to associate PA1 with AF1



   //TRIGGER PD2 as digital output(01)
   GPIOD->MODER &= ~(1 << (2 * 2 + 1)); //coloco un 0
   GPIOD->MODER |= (1 << (2 * 2));    //coloco un 1

   //ECHO PA5 as AF(10): we use TIM2_CH1
   GPIOA->MODER |= (1 << (5 * 2 + 1));    //coloco un 1
   GPIOA->MODER &= ~(1 << (5 * 2));    //coloco un 0

   GPIOA->AFR[0] |= (0x01 << 5 * 4);
   GPIOA->AFR[0] &= ~(0x0E << 5 * 4);



   //potentiometer PA4 - as analog mode (11)
   GPIOA->MODER |= (1 << (4 * 2 + 1));
   GPIOA->MODER |= (1 << (4 * 2));




 //TIM4 -CH3, para PB8 -CH4, para PB9//////////////PWM//////////////////////////////

   //PWM - velocity

     TIM4->CR1 = 0x0000;   //ARPE=0 -> no PWM, it is TOC
                           //CEN = 0; counter OFF
     TIM4->CR2 = 0x0000; //Always 0 in this course
     TIM4->SMCR = 0x0000;  //Always 0 in this course

     //counter behaviour setting: PSC, CNT, ARR and CCRx
     TIM4->PSC = 31;   //Tu = 1us
     TIM4->CNT = 0x0000;   //intialized the counter to 0, tendra que llegar a 10micros
     TIM4->ARR = 100-1;   //Recommended value
     //states the value to be recovered by CNT to obtain the match event
     TIM4->CCR3 = left_velocityMax;    //CH3 - PB8 - compare register, value we will compare
     TIM4->CCR4 = right_velocityMax;     //CH4 - PB9

     TIM4->DCR = 0; // no nos hace falta la interrupción
     TIM4->DIER = 0x0000;

     //CCM2-para ambos canales 3 y 4
     TIM4->CCMR2 = 0;
     TIM4->CCMR2 |= (1<<3);  //PE
     TIM4->CCMR2 |= (1<<6);  //1
     TIM4->CCMR2 |= (1<<5);  //1
     TIM4->CCMR2 &= ~(1<<4); //0

     TIM4->CCMR2 |= (1<<11); //PE
     TIM4->CCMR2 |= (1<<14); //1
     TIM4->CCMR2 |= (1<<13); //1
     TIM4->CCMR2 &= ~(1<<12);  //0

     TIM4->CCER |= (1<<8);   //CC3E ch3
     TIM4->CCER |= (1<<12);    //CC4E ch4

     TIM4->CR1 |= (1<<7);    //HW
     TIM4->EGR |= (1<<0);    //UG
     TIM4->CR1 |= (1<<0);    //ON
     TIM4->SR = 0;       //FLAG

     moveForwards();



 //TIM2 //////////////////////////////////////////////////////////////////////////////

     //TIM2CH1 as TIC echo
     //TIM2CH2 counting 10us
     //TIM2CH3 para el tiempo que para y el tiempo que gira el coche

     TIM2->CR1 = 0x0000;
     TIM2->CR2 = 0x0000;
     TIM2->SMCR = 0x0000;

     TIM2->PSC = 31;   //Tu= 1us
     TIM2->CNT = 0x0000; //Initialized counter to 0
     TIM2->ARR = 0xFFFF; //Recommended value


     ////IRQ selection
     TIM2->DIER |= 0x0002; //IRQ enabled for CH1

     //Counter output mode
     TIM2->CCMR1 = 0x0001;   ///OCyM = 000 (always in TIC)
                             // OCyPE = 0 (always in TIC)
                             //OC3M = 011(toggle)
     TIM2->CCER = 0x0001;    //CC1NP:CC1P = 00 (active rising edge)
                             //CC1E = 1
                             //CC2E = 1(enables hardware output)

     //contar 10us en el CH2
     TIM2->CCR2 = 10;
     TIM2->DIER |= 0x0004;  // interrupción del CH2

     //contar 10us en el ch3
     TIM2->CCR3 = 10;
     TIM2->DIER |= 0x0008; //interruocion del CH3

     //Enabling the counter
     TIM2->CR1 |= 0x0001;  //CEN = 1 (Counter Enable) Starting CNT
     TIM2->EGR |= 0x0001;  // UG = 1 -> Generate update event
     TIM2->SR  = 0;    //Clear all flags

     // Enabling IRQ source for TIM2 in NVIC (position 28) interrupcion
     NVIC->ISER[0] |= (1 << 28);




 //TIM3 PARA EL BUZZER, medio segundo del pitido intermitente, CH1///////////////////////

     TIM3->CR1 = 0x0000;
     TIM3->CR2 = 0x0000;
     TIM3->SMCR = 0x0000;

     TIM3->CNT = 0;
     TIM3->PSC = 16000-1;
     TIM3->ARR = 0XFFFF; //para PWM, no tiene que estar en 0xFFFF
     TIM3->CCR1 = 1000;

     TIM3->DIER |= (1<<1);
     NVIC->ISER[0] |= (1<<29);

     TIM3->CCMR1 = 0x0000;
     TIM3->CCER = 0x0000;

     //Enabling the counter
     TIM3->CR1 |= 0x0001;  //CEN = 1 (Counter Enable) Starting CNT
     TIM3->EGR |= 0x0000;  // UG = 1 -> Generate update event
     TIM3->SR  = 0;    //Clear all flags


 //ADC config////////////////////////////////////////////////////////////////////////

     ADC1 -> CR2 &= ~(0x000000001); //ADC off, only on if ADON 1
     ADC1 -> CR1 = 0X000000020; //CR1, conversor apagado,  encendemos la interrupcion


     ADC1->CR2 = 0x00000400;   // EOCS = 1 (EOC to be activated after each conversion)
                             // DELS = 000 (no delay)
                             // CONT = 0 (single conversion)



     ADC1 -> SQR1 = 0X00000000; //un canal, SI SQR1 = 0 -> only one channel is going to be used
     ADC1 -> SQR5 = 0x00000004; //5 least significant bits of SQR5, shall state the channel to be converted, IN4

     ADC1->CR2 |= 0x00000001; //start the conversion

     //while((ADC1->SR & 0X0040) == 0); //esperar a q este disponible
     //ADC1->CR2 |= 0x40000000; //if ADONS = 1, start conversion (SWSTART = 1)

    NVIC->ISER[0] |= (1<<18); //interrupt, position 18 in NVIC, ADC1



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    //iniciar las variables con un estado "incial", para que empiece con un estado en concreto y vaya saltando de uno a otro

    buzzerNO_SOUND();


    state = EMITTING; //empiezo el trigger emitiendo
    pre_state = state;

    intermitente = NO_INTER; //empiezo el buzzer en no_inter
    pre_intermitente = intermitente;

    obstacle = OBSTACLE_YES; //empezamos diciendo que hay obstaculo
    pre_obstacle = obstacle;


    //iniciar la velocidad actual de cada rueda, a la max, determinada en cada parte por pwm o por el potenciometro con el adc value
    left_velocityAc = left_velocityMax;
    right_velocityAc = right_velocityMax;



  while (1)
  {

    //FOR TRIGGER, state de recv o emit,coge uno u otro y se mete en su parte del switch correspondiente

        if (pre_state != state) //si no son iguales
        {
          pre_state = state; //var para cambiar al contrario
          switch (state)
          {
          case EMITTING:
            trigger_ON();
            break;
          case RECEIVING:
            trigger_OFF();
            break;
          }
        }

    //FOR BUZZER INTERMITENTE

        //si los estados no son iguaes y el intermitente no esta en no_inter (no_inter es que este en buzzer inter), entonces el buzzer suena de forma continua
        if (pre_intermitente != intermitente && intermitente != NO_INTER)
        {
          pre_intermitente = intermitente; //para cambiar el estado de uno a otro
          if (intermitente == PARPADEO_OFF)
          {
            //esta al reves, deberia ser buzzer_SOUND(), pero como lo tenemos iniciado al reves arriba, en el no_sound, le damos =1, aqui tiene que ir el que suena
            //pq si el parpadeo_off -> es que el buzzer suena de forma continua, lo que tendria que sonar continuo que es dondele uno directamente al BSRR
            //Esta funcion es la que le da el uno
            buzzerNO_SOUND();
          }
          else //Este else se mete en que el buzzer empieza a pitar en parpadeo_on, asi que lo controlamos con timers y la funcion del buzzer = 0, para q no suene
          {
            //esta es la que le esta dando 0, para que no suene
            buzzerSOUND();
          }
        }


    //DYNAMICS WITH OBSTACLES and TURNS/////////////////////////////////////////////////////////////////////

        //The distante, for bigger than 20com its going to be 40, because the pulse goes and come, so 0,4m
        //we have to compare the pulse_duration with the t=0,4/speedOfSound;
        //dist for 20 = 0.4 / 0.00034 = 1176;
        //dist for 10 = 0.2 / 0.00034 = 589;


    if (pulse_duration < 589) //menos de 10
    {
      //menos de 10, buzzer continuo
      sound_off_inter();
      buzzerSOUND();

      if (pre_obstacle != obstacle)
      {
        pre_obstacle = obstacle; //para cambiar de uno a otro
        //switch para cambiar el movimiento del coche dependiendo de los obstaculos
        switch (obstacle)
        {
        case OBSTACLE_YES:
          set_direction(stop);
          break;
        case OBSTACLE_TURN:
          set_direction(turn_right);
          break;
        case OBSTACLE_NO:
          set_direction(forwards);
          break;
        }
      }

    }
    else
        {
          if (pulse_duration < 1176)  //entre 10 y 20
          {
            sound_on_inter();
            left_velocityAc = left_velocityMax/2;    //pwm, control this
            right_velocityAc = right_velocityMax/2;

            set_direction(forwards); //prueba para ver si gira

          }

          else
          {
            sound_off_inter();
            buzzerNO_SOUND();
            left_velocityAc = left_velocityMax;
            right_velocityAc = right_velocityMax;
            set_direction(forwards);

          }
        }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEG14_Pin SEG15_Pin SEG16_Pin SEG17_Pin
                           SEG18_Pin SEG19_Pin SEG20_Pin SEG21_Pin
                           SEG22_Pin SEG23_Pin */
  GPIO_InitStruct.Pin = SEG14_Pin|SEG15_Pin|SEG16_Pin|SEG17_Pin
                          |SEG18_Pin|SEG19_Pin|SEG20_Pin|SEG21_Pin
                          |SEG22_Pin|SEG23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG1_Pin SEG2_Pin COM0_Pin COM1_Pin
                           COM2_Pin SEG12_Pin */
  GPIO_InitStruct.Pin = SEG1_Pin|SEG2_Pin|COM0_Pin|COM1_Pin
                          |COM2_Pin|SEG12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG6_Pin SEG7_Pin SEG8_Pin SEG9_Pin
                           SEG10_Pin SEG11_Pin SEG3_Pin SEG4_Pin
                           SEG5_Pin */
  GPIO_InitStruct.Pin = SEG6_Pin|SEG7_Pin|SEG8_Pin|SEG9_Pin
                          |SEG10_Pin|SEG11_Pin|SEG3_Pin|SEG4_Pin
                          |SEG5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
