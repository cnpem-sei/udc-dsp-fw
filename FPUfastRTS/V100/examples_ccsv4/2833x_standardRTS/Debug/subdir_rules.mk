################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
CodeStartBranch.obj: C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/CodeStartBranch.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.6/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.6/include" --include_path="/packages/ti/xdais" --include_path="C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/include" --quiet --diag_warning=225 --optimize_with_debug --large_memory_model --unified_memory --float_support=fpu32 -k --asm_listing --output_all_syms --preproc_with_compile --preproc_dependency="CodeStartBranch.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

fastRTS_sample.obj: C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.6/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.6/include" --include_path="/packages/ti/xdais" --include_path="C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/include" --quiet --diag_warning=225 --optimize_with_debug --large_memory_model --unified_memory --float_support=fpu32 -k --asm_listing --output_all_syms --preproc_with_compile --preproc_dependency="fastRTS_sample.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


