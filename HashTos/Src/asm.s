	AREA MYCODE, CODE
	
		IMPORT mpsp
		IMPORT os_curr_task
		IMPORT os_next_task
		IMPORT handler
		IMPORT params
		IMPORT xPSR
		IMPORT thread_return
		
			ALIGN
_save_state FUNCTION
			EXPORT _save_state
			cpsid i					;interrupts off
			
			MRS r1, psp             ;get current value of PSP 
			STMDB r1!, {r4-r11}		;save registers r4 - r11 
			MSR psp,r1				;write the new sp to PSP 
			LDR r0, =os_curr_task	;get current task 
			LDR r2,[r0]				;load address of sp from task structure
			STR r1,[r2] 			;write new value of sp to address 
			
			cpsie i					; interrupts on
			bx lr
			ENDP
				
			ALIGN
_load_state FUNCTION
			EXPORT _load_state
			cpsid i					;interrupts off
			
			LDR r0, =os_next_task	;get next task
			LDR r2, [r0]			;load address of sp from task structure
			LDR r1, [r2]			;load sp from the address
			LDMFD r1!, {r4-r11}		;load registers r4-r11
			MSR psp, r1				;load the new sp into PSP 
			
			cpsie i					; interrupts on
			bx lr					; return to thread mode
			ENDP
				
					ALIGN
_load_initial_state FUNCTION
					EXPORT _load_initial_state
					cpsid i					;interrupts off
			
					LDR r2, =os_curr_task	;get current task
					LDR r0,[r2]				;load address of sp from task structure
					LDR r2,[r0]				;load the value of sp
					LDR r0 , [r2,#60]		;XPSR: Default value (0x01000000)
					MSR xpsr, r0			;set the value of xPSR
					LDR r1 , [r2,#56]		;PC: Point to the handler function
					LDR lr , [r2,#52]		;LR: Point to a function for thread return
					LDR r0 , [r2,#32]		;R0: Point to the function parameters
					cpsie i					;interrupts on
					bx r1					;jump to handler
					ENDP
		
		
		
		
					END
				