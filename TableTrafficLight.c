// ***** 0. Documentation Section *****
// TableTrafficLight.c for (Lab 10 edX), Lab 5 EE319K
// Runs on LM4F120/TM4C123
// Program written by: Raiyan Chowdhury, Timberlon Gray
// Date Created: 1/24/2015 
// Last Modified: 3/2/2016 
// Section 1-2pm     TA: Wooseok Lee
// Lab number: 5
// Hardware connections
// east/west red light connected to PA4
// east/west yellow light connected to PA3
// east/west green light connected to PA2
// north/south facing red light connected to PA7
// north/south facing yellow light connected to PA6
// north/south facing green light connected to PA5
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include <stdint.h>
#include "TExaS.h"
#include "tm4c123gh6pm.h"
#include "SysTick.h"

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

#define GW			0		//GoWest
#define YW			1		//YellowWest
#define SW			2		//StopWest
#define GS			3		//GoSouth
#define YS			4		//YellowSouth
#define SS			5		//StopSouth
#define YSW			6		//YellowSouthWalk
#define SSW			7		//StopSouthWalk
#define SWk			8		//SouthWalk
#define SFOn1		9		//SouthFlashOn1
#define SFOff1	10	//SouthFlashOff1
#define SFOn2		11
#define SFOff2	12
#define SFOn3		13
#define SFOff3	14
#define YWW			15	//YellowWestWalk
#define SWW			16	//StopWestWalk
#define WW			17	//WestWalk
#define WFOn1		18	//WestFlashOn1
#define WFOff1	19	//WestFlashOff1
#define WFOn2		20
#define WFOff2	21
#define WFOn3		22
#define WFOff3	23

struct state {
	uint8_t out;
	uint32_t wait;
	uint8_t next[8]; };

typedef struct state state_t;
	
const state_t FSM[24] = {
	{0x85, 200, {GW, GW, YW, YW, YWW, YWW, YWW, YWW}},
	{0x89, 100, {SW, SW, SW, SW, SW, SW, SW, SW}},
	{0x91, 50, {GS, GS, GS, GS, GS, GS, GS, GS}},
	{0x31, 200, {YS, YS, GS, YS, YSW, YSW, YSW, YSW}},
	{0x51, 100, {SS, SS, SS, SS, SS, SS, SS, SS}},
	{0x91, 50, {GW, GW, GW, GW, GW, GW, GW, GW}},
	{0x51, 100, {SSW, SSW, SSW, SSW, SSW, SSW, SSW, SSW}},
	{0x91, 50, {SWk, SWk, SWk, SWk, SWk, SWk, SWk, SWk}},
	{0x92, 200, {SFOn1, SFOn1, SFOn1, SFOn1, SFOn1, SFOn1, SFOn1, SFOn1}},
	{0x91, 20, {SFOff1, SFOff1, SFOff1, SFOff1, SFOff1, SFOff1, SFOff1, SFOff1}},
	{0x90, 20, {SFOn2, SFOn2, SFOn2, SFOn2, SFOn2, SFOn2, SFOn2, SFOn2}},
	{0x91, 20, {SFOff2, SFOff2, SFOff2, SFOff2, SFOff2, SFOff2, SFOff2, SFOff2}},
	{0x90, 20, {SFOn3, SFOn3, SFOn3, SFOn3, SFOn3, SFOn3, SFOn3, SFOn3}},
	{0x91, 20, {SFOff3, SFOff3, SFOff3, SFOff3, SFOff3, SFOff3, SFOff3, SFOff3}},
	{0x90, 20, {GW, GW, GS, GW, GW, GW, GS, GW}},
	{0x89, 100, {SWW, SWW, SWW, SWW, SWW, SWW, SWW, SWW}},
	{0x91, 50, {WW, WW, WW, WW, WW, WW, WW, WW}},
	{0x92, 200, {WFOn1, WFOn1, WFOn1, WFOn1, WFOn1, WFOn1, WFOn1, WFOn1}},
	{0x91, 20, {WFOff1, WFOff1, WFOff1, WFOff1, WFOff1, WFOff1, WFOff1, WFOff1}},
	{0x90, 20, {WFOn2, WFOn2, WFOn2, WFOn2, WFOn2, WFOn2, WFOn2, WFOn2}},
	{0x91, 20, {WFOff2, WFOff2, WFOff2, WFOff2, WFOff2, WFOff2, WFOff2, WFOff2}},
	{0x90, 20, {WFOn3, WFOn3, WFOn3, WFOn3, WFOn3, WFOn3, WFOn3, WFOn3}},
	{0x91, 20, {WFOff3, WFOff3, WFOff3, WFOff3, WFOff3, WFOff3, WFOff3, WFOff3}},
	{0x90, 20, {GW, GW, GS, GS, GW, GW, GS, GS}}
};
uint8_t CState; //current state
uint8_t Input;

// ***** 3. Subroutines Section *****

void PortAEF_Init(void){ volatile uint32_t delay;
	SYSCTL_RCGC2_R |= 0x01;
	SYSCTL_RCGC2_R |= 0x10;
	SYSCTL_RCGC2_R |= 0x20;
	delay = SYSCTL_RCGC2_R;
	
	GPIO_PORTA_AMSEL_R = 0x00;
	GPIO_PORTE_AMSEL_R = 0x00;
	GPIO_PORTF_AMSEL_R = 0x00;
	
	GPIO_PORTA_DIR_R |= 0xFC;
	GPIO_PORTE_DIR_R &= ~0x07;
	GPIO_PORTF_DIR_R |= 0x0A;
	
	GPIO_PORTA_AFSEL_R &= ~0xFC;
	GPIO_PORTE_AFSEL_R &= ~0x07;
	GPIO_PORTF_AFSEL_R &= ~0x0A;
	
	GPIO_PORTA_DEN_R |= 0xFC;
	GPIO_PORTE_DEN_R |= 0x07;
	GPIO_PORTF_DEN_R |= 0x0A;
}


int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PA765432); // activate grader and set system clock to 80 MHz
	SysTick_Init();
	PortAEF_Init();
	
  EnableInterrupts();
	CState = GW;
	uint8_t DWLED;	// don't walk LED
	uint8_t WLED;	// walk LED
  while(1){
		// Output based on CS
		GPIO_PORTA_DATA_R &= 0x03;
    GPIO_PORTA_DATA_R |= (FSM[CState].out & 0xFC);
		GPIO_PORTF_DATA_R &= ~0x0A;
		DWLED = (FSM[CState].out & 0x01) << 1;
		WLED = (FSM[CState].out & 0x02) << 2;
		WLED |= DWLED;
		GPIO_PORTF_DATA_R |= WLED;
		// Wait based on CS
		SysTick_Wait10ms(FSM[CState].wait);
		// Read input
		Input = (GPIO_PORTE_DATA_R & 0x07);
		// Change state based on CS & Input
		CState = FSM[CState].next[Input];
  }
}

