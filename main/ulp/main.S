#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"


/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

prev_pres:	.long 0
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
	jumpr waitNext,5,lt // halt if r0 < 10

    //reset counter
	MOVE r0, 0
	st r0,r1,0

	psr
	jump readBMP

.global testTemp
testTemp:
	move r1,temp_msb
	ld r0,r1,0
	lsh r0,r0,8
	move r1,temp_lsb
	ld r1,r1,0
	or r0,r1,r0

	//save incase we need it
	move r1, temp_val
	st r0,r1,0

	move r1,prev_temp
	ld r2,r1,0

	sub r0,r0,r2
	psr
	jump abs
	// A threshold of 50 is equal to approx 0.25 C - adjust as needed
	jumpr testPres,50,lt

	//get new temp
	move r1, temp_val
	ld r0,r1,0

    //save in prev_temp register
	move r1, prev_temp
	st r0,r1,0
	jump wakeUp

testPres:
	move r1,pres_msb
	ld r0,r1,0
	lsh r0,r0,8
	move r1,pres_lsb
	ld r1,r1,0
	or r0,r1,r0

	//save incase we need it
	move r1, temp_val
	st r0,r1,0

	move r1,prev_pres
	ld r2,r1,0

	sub r0,r0,r2
	psr
	jump abs
	// A threshold of 50 is equal to approx 0.25 C - adjust as needed
	jumpr waitNext,50,lt

	//get new temp
	move r1, temp_val
	ld r0,r1,0

    //save in prev_temp register
	move r1, prev_pres
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
