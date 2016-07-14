################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
source/F28M36x_Adc.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_Adc.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_Adc.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_CodeStartBranch.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_CodeStartBranch.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_CodeStartBranch.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_Comp.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_Comp.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_Comp.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_CpuTimers.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_CpuTimers.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_CpuTimers.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_DMA.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_DMA.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_DMA.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_DefaultIsr.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_DefaultIsr.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_DefaultIsr.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_EPwm.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_EPwm.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_EPwm.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_Gpio.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_Gpio.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_Gpio.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_Mcbsp.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_Mcbsp.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_Mcbsp.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_PieCtrl.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_PieCtrl.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_PieCtrl.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_PieVect.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_PieVect.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_PieVect.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_Sci.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_Sci.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_Sci.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_Spi.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_Spi.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_Spi.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_SysCtrl.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_SysCtrl.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_SysCtrl.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_TempSensorConv.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_TempSensorConv.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_TempSensorConv.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/F28M36x_usDelay.obj: D:/ARQ/Projects/C28/F28M36x_common/source/F28M36x_usDelay.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/ELP_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/SigGen" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TimeSlicer" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/DP_framework/TI_DCL" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/HRADC_board" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/IPC_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PS_modules" --include_path="D:/ARQ/Projects/C28/F28M36x_ELP_DRS/PWM_modules" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.10/include" --include_path="D:/ARQ/Projects/C28/F28M36x_common/include" --include_path="D:/ARQ/Projects/C28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="source/F28M36x_usDelay.pp" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


