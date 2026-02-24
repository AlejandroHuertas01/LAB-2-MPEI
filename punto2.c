int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  int numeros[10]={	63,6,91,79,	102,109,125,7,127,111};
  int pul=0;
  int cont=0;
  int contAux=0;
  int unidades=0;
  int decenas=0;

  uint8_t sentido = 1;         // 1 = sube, 0 = baja
  uint8_t estadoAnterior = 1;  // porque pull-up

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //HAL_GPIO_TogglePin (GPIOB, LEDB_Pin);
	  //GPIOA->BSRR = numeros[i];
	  //GPIOA->ODR = (GPIOA->ODR & ~0x007F) | numeros[i];
	  HAL_Delay(10);

	  /* ----------- LECTURA DEL PULSADOR PC15 ----------- */

	  uint8_t estadoActual = (GPIOC->IDR & (1<<15)) ? 1 : 0;

	  // Detectar flanco de bajada (pulsación)
	  if(estadoAnterior == 1 && estadoActual == 0){
	      HAL_Delay(20);  // antirrebote simple

	      if((GPIOC->IDR & (1<<15)) == 0){
	          sentido = !sentido;  // cambiar sentido
	      }
	  }

	  estadoAnterior = estadoActual;

	  /* ----------- CONTADOR ----------- */

	  contAux++;

	  if(contAux == 6){

	      if(sentido)
	      {  // ASCENDENTE
	          unidades++;

	          if(unidades > 9){
	              unidades = 0;
	              decenas++;
	          }

	          if(decenas > 9){
	              decenas = 0;
	          }

	      }
	      else{        // DESCENDENTE

	          if(unidades == 0)
	          {
	              unidades = 9;

	              if(decenas == 0)
	                  decenas = 9;
	              else
	                  decenas--;
	          }
	          else{
	              unidades--;
	          }
	      }

	      contAux = 0;
	  }

	  /* -------- Multiplexación dinámica -------- */

	  switch(cont){

	      case 0:
	          GPIOA->ODR = (GPIOA->ODR & ~0x007F) | numeros[unidades];
	          GPIOB->ODR = (GPIOB->ODR & ~0x0003) | 0x0001;
	          break;

	      case 1:
	          GPIOA->ODR = (GPIOA->ODR & ~0x007F) | numeros[decenas];
	          GPIOB->ODR = (GPIOB->ODR & ~0x0003) | 0x0002;
	          break;
	  }

	  cont++;
	  if(cont > 1)
	      cont = 0;
  }
  /* USER CODE END 3 */
}