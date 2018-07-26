#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"


/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

prev_temp:	.long 0
temp_val: .long 0

	.global	counter
counter: .long 0

	.global stack
stack:
	.skip 100
	.global stackEnd
stackEnd:
	.long 0


	/* Code goes into .text section */
	.text
	.global entry
entry:
	move r3,stackEnd

	// Read every 10 timer cycles
	MOVE r1, counter //move address of counter global into r1
	ld r0,r1,0  // r0 = [r1+0]
	add r0,r0,1 // r0 += 1
	st r0,r1,0 // [r1+0] (counter) = r0
	jumpr waitNext,10,lt // halt if r0 < 10

    //reset counter
	MOVE r0, 0
	st r0,r1,0

	psr
	jump readBMP

	/* wake up after significant change
	move r1,pressure
	ld r0,r1,0
	move r1,prev_pressure
	ld r2,r1,0
	sub r0,r0,r2
	psr
	jump abs
	jumpr testTemp,6,lt
	*/
	//psr
	//jump wakeUp

testTemp:
    // SHIFT = 6
    // temp_msb<< (8-SHIFT) | temp_lsb >> SHIFT
	move r1,temp_msb
	ld r0,r1,0
	lsh r0,r0,8
	move r1,temp_lsb
	ld r1,r1,0
	//rsh r1,r1,6
	or r0,r1,r0

	//save incase we need it
	move r1, temp_val
	st r0,r1,0


	move r1,prev_temp
	ld r2,r1,0

	sub r0,r0,r2
	psr
	jump abs
	jumpr waitNext,30,lt

	//save new temp
	move r1, temp_val
	ld r0,r1,0

	move r1, prev_temp
	st r0,r1,0

wakeUp:
	/* Check if the system can be woken up */
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump wakeUp, eq

	/* Wake up the SoC, end program */
	/* safe new pressure and temp
	move r1,pressure
	ld r0,r1,0
	move r1,prev_pressure
	st r0,r1,0
	move r1,temp
	ld r0,r1,0
	move r1,prev_temp
	st r0,r1,0
	/* Wake up the SoC, end program */
	wake
	/* Stop the wakeup timer so it does not restart ULP */
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)

waitNext:
	halt

// Compute abs value of R0
abs:
	and r1,r0,0x8000
	jump noNegate,eq
	move r1,0
	sub r0,r1,r0
noNegate:
	ret
