#include "stdint.h"



char  GetBusy(void);
void SetCS(char ON);
void SendCmd(uint8_t data);
uint16_t  GetResult(void);
char GetPENIRQ(void);


/**
 
void TOUCH_X_ActivateX(void);
void TOUCH_X_ActivateY(void);
void TOUCH_X_Disable(void);
int  TOUCH_X_MeasureX(void);
int  TOUCH_X_MeasureY(void);
*/