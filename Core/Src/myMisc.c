#include "myMisc.h"

uint32_t ucRandomNumber(uint32_t min, uint32_t max)
{
	return (min + rand() % (max + 1 - min));
}
