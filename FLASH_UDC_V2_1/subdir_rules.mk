################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/gabriel/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --vcu_support=vcu0 -O2 --opt_for_speed=1 --include_path="/home/gabriel/lnls-elp/C28/elp_libs" --include_path="/home/gabriel/lnls-elp/C28/elp_libs/DP_framework" --include_path="/home/gabriel/lnls-elp/C28/elp_libs/DP_framework/ELP_DCL" --include_path="/home/gabriel/lnls-elp/C28/elp_libs/DP_framework/SigGen" --include_path="/home/gabriel/lnls-elp/C28/elp_libs/DP_framework/TimeSlicer" --include_path="/home/gabriel/lnls-elp/C28/elp_libs/DP_framework/TI_DCL" --include_path="/home/gabriel/lnls-elp/C28/elp_libs/HRADC_board" --include_path="/home/gabriel/lnls-elp/C28/elp_libs/IPC_modules" --include_path="/home/gabriel/lnls-elp/C28/elp_libs/PS_modules" --include_path="/home/gabriel/lnls-elp/C28/elp_libs/PWM_modules" --include_path="/home/gabriel/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.1.LTS/include" --include_path="/home/gabriel/lnls-elp/controlsuite/c28/F28M36x_common/include" --include_path="/home/gabriel/lnls-elp/controlsuite/c28/F28M36x_headers/include" -g --diag_warning=225 --display_error_number --issue_remarks --diag_wrap=off --preproc_with_compile --preproc_dependency="main.d" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


