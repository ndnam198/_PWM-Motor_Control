#include "myDebug.h"

#if defined(configHAL_UART) /* configHAL_UART */
void vUARTSend(UART_HandleTypeDef huart, uint8_t *String)
{
#if defined(USE_DMA_TX) /* USE_DMA_TX */
	HAL_UART_Transmit_DMA(&huart, (uint8_t *)String, strlen((char *)String));
#else
	HAL_UART_Transmit(&huart, (uint8_t *)String, strlen((char *)String), 100);
#endif /* !USE_DMA_TX */
}
#endif					   /* !configHAL_UART */
#if defined(configLL_UART) /* configLL_UART */
void vUARTSend(USART_TypeDef *USARTx, uint8_t *String)
{
	uint32_t ulStringLen = 0;
	uint32_t i = 0;
	uint32_t ulBlockTime = 10000;
	ulStringLen = strlen((char *)String);

	while (ulStringLen)
	{
		LL_USART_TransmitData8(USARTx, (uint8_t)String[i]);
		i++;
		ulStringLen--;
		/* Check if transfer 1 byte completed */
		while (!LL_USART_IsActiveFlag_TC(USARTx))
		{
			if ((ulBlockTime--) == 0)
				break;
		}
	}
}
#endif /* !configLL_UART */

reset_cause_t resetCauseGet(void)
{
	reset_cause_t reset_cause;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
	{
		reset_cause = eRESET_CAUSE_LOW_POWER_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
	{
		reset_cause = eRESET_CAUSE_WINDOW_WATCHDOG_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
	{
		reset_cause = eRESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
	{
		reset_cause = eRESET_CAUSE_SOFTWARE_RESET; // This reset is induced by calling the ARM CMSIS `NVIC_SystemReset()` function!
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
	{
		reset_cause = eRESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
	{
		reset_cause = eRESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
	}
	// Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to ensure first that the reset cause is
	// NOT a POR/PDR reset. See note below.
	/* else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
     {
     reset_cause = eRESET_CAUSE_BROWNOUT_RESET;
     } */
	else
	{
		reset_cause = eRESET_CAUSE_UNKNOWN;
	}

	// Clear all the reset flags or else they will remain set during future resets until system power is fully removed.
	__HAL_RCC_CLEAR_RESET_FLAGS();

	return reset_cause;
}

const char *resetCauseGetName(reset_cause_t reset_cause)
{
	const char *reset_cause_name = "TBD";

	switch (reset_cause)
	{
	case eRESET_CAUSE_UNKNOWN:
		reset_cause_name = "UNKNOWN";
		break;
	case eRESET_CAUSE_LOW_POWER_RESET:
		reset_cause_name = "LOW_POWER_RESET";
		break;
	case eRESET_CAUSE_WINDOW_WATCHDOG_RESET:
		reset_cause_name = "WINDOW_WATCHDOG_RESET";
		break;
	case eRESET_CAUSE_INDEPENDENT_WATCHDOG_RESET:
		reset_cause_name = "INDEPENDENT_WATCHDOG_RESET";
		break;
	case eRESET_CAUSE_SOFTWARE_RESET:
		reset_cause_name = "SOFTWARE_RESET";
		break;
	case eRESET_CAUSE_POWER_ON_POWER_DOWN_RESET:
		reset_cause_name = "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)";
		break;
	case eRESET_CAUSE_EXTERNAL_RESET_PIN_RESET:
		reset_cause_name = "EXTERNAL_RESET_PIN_RESET";
		break;
	case eRESET_CAUSE_BROWNOUT_RESET:
		reset_cause_name = "BROWNOUT_RESET (BOR)";
		break;
	}
	return reset_cause_name;
}

void vIWDG_Init(IWDG_HandleTypeDef _hiwdg, uint32_t millis)
{
	uint32_t iwdg_timeout_millis = millis;

	/* Select INDEPENDENT_WATCHDOG */
	_hiwdg.Instance = IWDG;
	/* Use prescaler LSI/128 */
	_hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
	_hiwdg.Init.Reload = (int)(IWDG_RESOLUTION * ((float)iwdg_timeout_millis / PRESCALER_128_UPPER_LIMIT));

	if (HAL_IWDG_Init(&_hiwdg) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	else
	{
#if defined(PRINT_DEBUG)
		PRINTF("Watchdog Init with timeout = %ld\r\n", (uint32_t)millis);
#endif
	}
}

__weak void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
	{
		PRINTF("\r\nError file %s line %d", file, line);
	}
	/* USER CODE END Error_Handler_Debug */
}
