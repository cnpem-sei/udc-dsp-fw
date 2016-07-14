;***************************************************************
;* TMS320C2000 C/C++ Codegen                         PC v6.4.6 *
;* Date/Time created: Thu Feb 11 10:11:22 2016                 *
;***************************************************************
	.compiler_opts --float_support=fpu32 --hll_source=on --mem_model:code=flat --mem_model:data=large --object_format=coff --quiet --silicon_version=28 --symdebug:dwarf --symdebug:dwarf_version=3 
FP	.set	XAR2

$C$DW$CU	.dwtag  DW_TAG_compile_unit
	.dwattr $C$DW$CU, DW_AT_name("C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c")
	.dwattr $C$DW$CU, DW_AT_producer("TI TMS320C2000 C/C++ Codegen PC v6.4.6 Copyright (c) 1996-2015 Texas Instruments Incorporated")
	.dwattr $C$DW$CU, DW_AT_TI_version(0x01)
	.dwattr $C$DW$CU, DW_AT_comp_dir("C:\Users\ricieri.ohashi\Desktop\FAC_Imas\C28_new\FPUfastRTS\V100\examples_ccsv4\2833x_fastRTS\Debug")

$C$DW$1	.dwtag  DW_TAG_subprogram, DW_AT_name("cos")
	.dwattr $C$DW$1, DW_AT_TI_symbol_name("_cos")
	.dwattr $C$DW$1, DW_AT_type(*$C$DW$T$17)
	.dwattr $C$DW$1, DW_AT_declaration
	.dwattr $C$DW$1, DW_AT_external
$C$DW$2	.dwtag  DW_TAG_formal_parameter
	.dwattr $C$DW$2, DW_AT_type(*$C$DW$T$17)
	.dwendtag $C$DW$1


$C$DW$3	.dwtag  DW_TAG_subprogram, DW_AT_name("atan")
	.dwattr $C$DW$3, DW_AT_TI_symbol_name("_atan")
	.dwattr $C$DW$3, DW_AT_type(*$C$DW$T$17)
	.dwattr $C$DW$3, DW_AT_declaration
	.dwattr $C$DW$3, DW_AT_external
$C$DW$4	.dwtag  DW_TAG_formal_parameter
	.dwattr $C$DW$4, DW_AT_type(*$C$DW$T$17)
	.dwendtag $C$DW$3


$C$DW$5	.dwtag  DW_TAG_subprogram, DW_AT_name("sin")
	.dwattr $C$DW$5, DW_AT_TI_symbol_name("_sin")
	.dwattr $C$DW$5, DW_AT_type(*$C$DW$T$17)
	.dwattr $C$DW$5, DW_AT_declaration
	.dwattr $C$DW$5, DW_AT_external
$C$DW$6	.dwtag  DW_TAG_formal_parameter
	.dwattr $C$DW$6, DW_AT_type(*$C$DW$T$17)
	.dwendtag $C$DW$5


$C$DW$7	.dwtag  DW_TAG_subprogram, DW_AT_name("sqrt")
	.dwattr $C$DW$7, DW_AT_TI_symbol_name("_sqrt")
	.dwattr $C$DW$7, DW_AT_type(*$C$DW$T$17)
	.dwattr $C$DW$7, DW_AT_declaration
	.dwattr $C$DW$7, DW_AT_external
$C$DW$8	.dwtag  DW_TAG_formal_parameter
	.dwattr $C$DW$8, DW_AT_type(*$C$DW$T$17)
	.dwendtag $C$DW$7

	.global	_Step
_Step:	.usect	".ebss",20,1,1
$C$DW$9	.dwtag  DW_TAG_variable, DW_AT_name("Step")
	.dwattr $C$DW$9, DW_AT_TI_symbol_name("_Step")
	.dwattr $C$DW$9, DW_AT_location[DW_OP_addr _Step]
	.dwattr $C$DW$9, DW_AT_type(*$C$DW$T$22)
	.dwattr $C$DW$9, DW_AT_external
	.global	_Dlog
_Dlog:	.usect	".ebss",640,1,1
$C$DW$10	.dwtag  DW_TAG_variable, DW_AT_name("Dlog")
	.dwattr $C$DW$10, DW_AT_TI_symbol_name("_Dlog")
	.dwattr $C$DW$10, DW_AT_location[DW_OP_addr _Dlog]
	.dwattr $C$DW$10, DW_AT_type(*$C$DW$T$21)
	.dwattr $C$DW$10, DW_AT_external
;	C:\ti\ccsv6\tools\compiler\ti-cgt-c2000_6.4.6\bin\ac2000.exe -@C:\\Users\\RICIER~1.OHA\\AppData\\Local\\Temp\\3128012 
	.sect	".text"
	.clink
	.global	_main

$C$DW$11	.dwtag  DW_TAG_subprogram, DW_AT_name("main")
	.dwattr $C$DW$11, DW_AT_low_pc(_main)
	.dwattr $C$DW$11, DW_AT_high_pc(0x00)
	.dwattr $C$DW$11, DW_AT_TI_symbol_name("_main")
	.dwattr $C$DW$11, DW_AT_external
	.dwattr $C$DW$11, DW_AT_type(*$C$DW$T$10)
	.dwattr $C$DW$11, DW_AT_TI_begin_file("C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c")
	.dwattr $C$DW$11, DW_AT_TI_begin_line(0x41)
	.dwattr $C$DW$11, DW_AT_TI_begin_column(0x05)
	.dwattr $C$DW$11, DW_AT_TI_max_frame_size(-16)
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 66,column 1,is_stmt,address _main,isa 0

	.dwfde $C$DW$CIE, _main

;***************************************************************
;* FNAME: _main                         FR SIZE:  14           *
;*                                                             *
;* FUNCTION ENVIRONMENT                                        *
;*                                                             *
;* FUNCTION PROPERTIES                                         *
;*                            2 Parameter, 12 Auto,  0 SOE     *
;***************************************************************

_main:
	.dwcfi	cfa_offset, -2
	.dwcfi	save_reg_to_mem, 26, 0
        ADDB      SP,#14                ; [CPU_U] 
	.dwcfi	cfa_offset, -16
$C$DW$12	.dwtag  DW_TAG_variable, DW_AT_name("i")
	.dwattr $C$DW$12, DW_AT_TI_symbol_name("_i")
	.dwattr $C$DW$12, DW_AT_type(*$C$DW$T$11)
	.dwattr $C$DW$12, DW_AT_location[DW_OP_breg20 -3]
$C$DW$13	.dwtag  DW_TAG_variable, DW_AT_name("tempX")
	.dwattr $C$DW$13, DW_AT_TI_symbol_name("_tempX")
	.dwattr $C$DW$13, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$13, DW_AT_location[DW_OP_breg20 -6]
$C$DW$14	.dwtag  DW_TAG_variable, DW_AT_name("tempY")
	.dwattr $C$DW$14, DW_AT_TI_symbol_name("_tempY")
	.dwattr $C$DW$14, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$14, DW_AT_location[DW_OP_breg20 -8]
$C$DW$15	.dwtag  DW_TAG_variable, DW_AT_name("tempM")
	.dwattr $C$DW$15, DW_AT_TI_symbol_name("_tempM")
	.dwattr $C$DW$15, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$15, DW_AT_location[DW_OP_breg20 -10]
$C$DW$16	.dwtag  DW_TAG_variable, DW_AT_name("tempMmax")
	.dwattr $C$DW$16, DW_AT_TI_symbol_name("_tempMmax")
	.dwattr $C$DW$16, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$16, DW_AT_location[DW_OP_breg20 -12]
$C$DW$17	.dwtag  DW_TAG_variable, DW_AT_name("WatchdogWDCR")
	.dwattr $C$DW$17, DW_AT_TI_symbol_name("_WatchdogWDCR")
	.dwattr $C$DW$17, DW_AT_type(*$C$DW$T$26)
	.dwattr $C$DW$17, DW_AT_location[DW_OP_breg20 -14]
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 70,column 23,is_stmt,isa 0
        MOVL      XAR4,#28713           ; [CPU_U] |70| 
        MOVL      *-SP[14],XAR4         ; [CPU_] |70| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 73,column 5,is_stmt,isa 0
 EALLOW 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 74,column 5,is_stmt,isa 0
        MOVB      *+XAR4[0],#104,UNC    ; [CPU_] |74| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 75,column 5,is_stmt,isa 0
 EDIS 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 77,column 5,is_stmt,isa 0
        MOVIZ     R0H,#16032            ; [CPU_] |77| 
        MOVW      DP,#_Step             ; [CPU_U] 
        MOVXI     R0H,#55676            ; [CPU_] |77| 
        MOV32     @_Step,R0H            ; [CPU_] |77| 
        MOV32     *-SP[6],R0H           ; [CPU_] |77| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 80,column 5,is_stmt,isa 0
        MOVIZ     R0H,#16032            ; [CPU_] |80| 
        MOVXI     R0H,#55676            ; [CPU_] |80| 
        MOV32     @_Step+2,R0H          ; [CPU_] |80| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 81,column 5,is_stmt,isa 0
        ZERO      R0H                   ; [CPU_] |81| 
        MOV32     @_Step+4,R0H          ; [CPU_] |81| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 82,column 5,is_stmt,isa 0
        MOV32     @_Step+6,R0H          ; [CPU_] |82| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 83,column 5,is_stmt,isa 0
        MOVL      ACC,@_Step+4          ; [CPU_] |83| 
        MOVL      @_Step+8,ACC          ; [CPU_] |83| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 86,column 9,is_stmt,isa 0
        MOV       *-SP[3],#0            ; [CPU_] |86| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 86,column 14,is_stmt,isa 0
        MOV       AL,*-SP[3]            ; [CPU_] |86| 
        CMPB      AL,#64                ; [CPU_] |86| 
        BF        $C$L2,HIS             ; [CPU_] |86| 
        ; branchcc occurs ; [] |86| 
        CLRC      SXM                   ; [CPU_] 
        MOVB      XAR6,#0               ; [CPU_] |92| 
$C$L1:    
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 88,column 9,is_stmt,isa 0
        MOV       ACC,*-SP[3] << 1      ; [CPU_] |88| 
        MOVL      XAR4,#_Dlog           ; [CPU_U] |88| 
        ADDL      XAR4,ACC              ; [CPU_] |88| 
        MOV32     *+XAR4[0],R0H         ; [CPU_] |88| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 89,column 9,is_stmt,isa 0
        MOV       ACC,*-SP[3] << 1      ; [CPU_] |89| 
        MOVL      XAR4,#_Dlog+128       ; [CPU_U] |89| 
        ADDL      XAR4,ACC              ; [CPU_] |89| 
        MOV32     *+XAR4[0],R0H         ; [CPU_] |89| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 90,column 9,is_stmt,isa 0
        MOV       ACC,*-SP[3] << 1      ; [CPU_] |90| 
        MOVL      XAR4,#_Dlog+256       ; [CPU_U] |90| 
        ADDL      XAR4,ACC              ; [CPU_] |90| 
        MOV32     *+XAR4[0],R0H         ; [CPU_] |90| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 91,column 9,is_stmt,isa 0
        MOV       ACC,*-SP[3] << 1      ; [CPU_] |91| 
        MOVL      XAR4,#_Dlog+512       ; [CPU_U] |91| 
        ADDL      XAR4,ACC              ; [CPU_] |91| 
        MOV32     *+XAR4[0],R0H         ; [CPU_] |91| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 92,column 9,is_stmt,isa 0
        MOVL      XAR4,#_Dlog+384       ; [CPU_U] |92| 
        MOV       ACC,*-SP[3] << 1      ; [CPU_] |92| 
        ADDL      XAR4,ACC              ; [CPU_] |92| 
        MOVL      *+XAR4[0],XAR6        ; [CPU_] |92| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 86,column 33,is_stmt,isa 0
        INC       *-SP[3]               ; [CPU_] |86| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 86,column 14,is_stmt,isa 0
        MOV       AL,*-SP[3]            ; [CPU_] |86| 
        CMPB      AL,#64                ; [CPU_] |86| 
        BF        $C$L1,LO              ; [CPU_] |86| 
        ; branchcc occurs ; [] |86| 
$C$L2:    
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 95,column 5,is_stmt,isa 0
        MOVIZ     R0H,#16320            ; [CPU_] |95| 
        MOVW      DP,#_Step+12          ; [CPU_U] 
        MOV32     @_Step+12,R0H         ; [CPU_] |95| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 96,column 5,is_stmt,isa 0
        MOVIZ     R0H,#16128            ; [CPU_] |96| 
        MOV32     @_Step+16,R0H         ; [CPU_] |96| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 97,column 5,is_stmt,isa 0
        MOVIZ     R0H,#16416            ; [CPU_] |97| 
        MOV32     @_Step+14,R0H         ; [CPU_] |97| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 98,column 5,is_stmt,isa 0
        MOVIZ     R0H,#16256            ; [CPU_] |98| 
        MOV32     @_Step+18,R0H         ; [CPU_] |98| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 101,column 5,is_stmt,isa 0
        MOV32     R1H,@_Step+12         ; [CPU_] |101| 
        MOV32     R2H,@_Step+14         ; [CPU_] |101| 
        MOV32     R0H,@_Step+12         ; [CPU_] |101| 

        MPYF32    R0H,R1H,R0H           ; [CPU_] |101| 
||      MOV32     R3H,@_Step+14         ; [CPU_] |101| 

        MPYF32    R1H,R3H,R2H           ; [CPU_] |101| 
        SPM       #0                    ; [CPU_] 
        ADDF32    R0H,R0H,R1H           ; [CPU_] |101| 
$C$DW$18	.dwtag  DW_TAG_TI_branch
	.dwattr $C$DW$18, DW_AT_low_pc(0x00)
	.dwattr $C$DW$18, DW_AT_name("_sqrt")
	.dwattr $C$DW$18, DW_AT_TI_call
        LCR       #_sqrt                ; [CPU_] |101| 
        ; call occurs [#_sqrt] ; [] |101| 
        MOV32     *-SP[12],R0H          ; [CPU_] |101| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 103,column 9,is_stmt,isa 0
        MOV       *-SP[3],#0            ; [CPU_] |103| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 103,column 14,is_stmt,isa 0
        MOV       AL,*-SP[3]            ; [CPU_] |103| 
        CMPB      AL,#64                ; [CPU_] |103| 
        BF        $C$L6,HIS             ; [CPU_] |103| 
        ; branchcc occurs ; [] |103| 
$C$L3:    
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 108,column 9,is_stmt,isa 0
        MOVW      DP,#_Step+6           ; [CPU_U] 
        MOV32     R0H,@_Step+6          ; [CPU_] |108| 
$C$DW$19	.dwtag  DW_TAG_TI_branch
	.dwattr $C$DW$19, DW_AT_low_pc(0x00)
	.dwattr $C$DW$19, DW_AT_name("_sin")
	.dwattr $C$DW$19, DW_AT_TI_call
        LCR       #_sin                 ; [CPU_] |108| 
        ; call occurs [#_sin] ; [] |108| 
        MOVW      DP,#_Step+12          ; [CPU_U] 
        CLRC      SXM                   ; [CPU_] 
        MOVL      XAR4,#_Dlog           ; [CPU_U] |108| 
        MOV32     R1H,@_Step+12         ; [CPU_] |108| 
        MOV       ACC,*-SP[3] << 1      ; [CPU_] |108| 
        MPYF32    R0H,R1H,R0H           ; [CPU_] |108| 
        ADDL      XAR4,ACC              ; [CPU_] |108| 
        MOV32     *-SP[6],R0H           ; [CPU_] |108| 
        MOV32     *+XAR4[0],R0H         ; [CPU_] |108| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 109,column 9,is_stmt,isa 0
        MOV32     R0H,@_Step+8          ; [CPU_] |109| 
$C$DW$20	.dwtag  DW_TAG_TI_branch
	.dwattr $C$DW$20, DW_AT_low_pc(0x00)
	.dwattr $C$DW$20, DW_AT_name("_cos")
	.dwattr $C$DW$20, DW_AT_TI_call
        LCR       #_cos                 ; [CPU_] |109| 
        ; call occurs [#_cos] ; [] |109| 
        MOVW      DP,#_Step+14          ; [CPU_U] 
        CLRC      SXM                   ; [CPU_] 
        MOVL      XAR4,#_Dlog+128       ; [CPU_U] |109| 
        MOV32     R1H,@_Step+14         ; [CPU_] |109| 
        MOV       ACC,*-SP[3] << 1      ; [CPU_] |109| 
        MPYF32    R0H,R1H,R0H           ; [CPU_] |109| 
        ADDL      XAR4,ACC              ; [CPU_] |109| 
        MOV32     *-SP[8],R0H           ; [CPU_] |109| 
        MOV32     *+XAR4[0],R0H         ; [CPU_] |109| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 115,column 9,is_stmt,isa 0
        MOV32     R1H,*-SP[6]           ; [CPU_] |115| 
        MOV32     R0H,*-SP[6]           ; [CPU_] |115| 
        MPYF32    R0H,R1H,R0H           ; [CPU_] |115| 
        MOV32     R3H,*-SP[8]           ; [CPU_] |115| 
        MPYF32    R0H,R3H,R0H           ; [CPU_] |115| 
        MOV32     R1H,*-SP[8]           ; [CPU_] |115| 
        MPYF32    R0H,R1H,R0H           ; [CPU_] |115| 
$C$DW$21	.dwtag  DW_TAG_TI_branch
	.dwattr $C$DW$21, DW_AT_low_pc(0x00)
	.dwattr $C$DW$21, DW_AT_name("_sqrt")
	.dwattr $C$DW$21, DW_AT_TI_call
        LCR       #_sqrt                ; [CPU_] |115| 
        ; call occurs [#_sqrt] ; [] |115| 
        MOV32     *-SP[10],R0H          ; [CPU_] |115| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 116,column 9,is_stmt,isa 0
        MOV32     R1H,*-SP[12]          ; [CPU_] |116| 
$C$DW$22	.dwtag  DW_TAG_TI_branch
	.dwattr $C$DW$22, DW_AT_low_pc(0x00)
	.dwattr $C$DW$22, DW_AT_name("FS$$DIV")
	.dwattr $C$DW$22, DW_AT_TI_call
        LCR       #FS$$DIV              ; [CPU_] |116| 
        ; call occurs [#FS$$DIV] ; [] |116| 
        CLRC      SXM                   ; [CPU_] 
        MOVL      XAR4,#_Dlog+512       ; [CPU_U] |116| 
        MOV       ACC,*-SP[3] << 1      ; [CPU_] |116| 
        ADDL      XAR4,ACC              ; [CPU_] |116| 
        MOV32     *+XAR4[0],R0H         ; [CPU_] |116| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 120,column 9,is_stmt,isa 0
        MOVW      DP,#_Step             ; [CPU_U] 
        MOV32     R1H,@_Step            ; [CPU_] |120| 
        MOV32     R2H,@_Step+16         ; [CPU_] |120| 
        MPYF32    R1H,R2H,R1H           ; [CPU_] |120| 
        MOV32     R0H,@_Step+6          ; [CPU_] |120| 
        ADDF32    R0H,R0H,R1H           ; [CPU_] |120| 
        NOP       ; [CPU_] 
        MOV32     @_Step+6,R0H          ; [CPU_] |120| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 121,column 9,is_stmt,isa 0
        MOVIZ     R0H,#16585            ; [CPU_] |121| 
        MOVXI     R0H,#4059             ; [CPU_] |121| 
        MOV32     R1H,@_Step+6          ; [CPU_] |121| 
        CMPF32    R1H,R0H               ; [CPU_] |121| 
        MOVST0    ZF, NF                ; [CPU_] |121| 
        BF        $C$L4,LEQ             ; [CPU_] |121| 
        ; branchcc occurs ; [] |121| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 123,column 10,is_stmt,isa 0
        MOVIZ     R0H,#16585            ; [CPU_] |123| 
        MOVXI     R0H,#4059             ; [CPU_] |123| 
        SUBF32    R0H,R1H,R0H           ; [CPU_] |123| 
        NOP       ; [CPU_] 
        MOV32     @_Step+6,R0H          ; [CPU_] |123| 
$C$L4:    
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 126,column 9,is_stmt,isa 0
        MOV32     R0H,@_Step+2          ; [CPU_] |126| 
        MOV32     R1H,@_Step+18         ; [CPU_] |126| 
        MPYF32    R1H,R1H,R0H           ; [CPU_] |126| 
        MOV32     R3H,@_Step+8          ; [CPU_] |126| 
        ADDF32    R0H,R1H,R3H           ; [CPU_] |126| 
        NOP       ; [CPU_] 
        MOV32     @_Step+8,R0H          ; [CPU_] |126| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 127,column 9,is_stmt,isa 0
        MOVIZ     R0H,#16585            ; [CPU_] |127| 
        MOV32     R1H,@_Step+8          ; [CPU_] |127| 
        MOVXI     R0H,#4059             ; [CPU_] |127| 
        CMPF32    R1H,R0H               ; [CPU_] |127| 
        MOVST0    ZF, NF                ; [CPU_] |127| 
        BF        $C$L5,LEQ             ; [CPU_] |127| 
        ; branchcc occurs ; [] |127| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 129,column 10,is_stmt,isa 0
        MOVIZ     R0H,#16585            ; [CPU_] |129| 
        MOVXI     R0H,#4059             ; [CPU_] |129| 
        SUBF32    R0H,R1H,R0H           ; [CPU_] |129| 
        NOP       ; [CPU_] 
        MOV32     @_Step+8,R0H          ; [CPU_] |129| 
$C$L5:    
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 103,column 33,is_stmt,isa 0
        INC       *-SP[3]               ; [CPU_] |103| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 103,column 14,is_stmt,isa 0
        MOV       AL,*-SP[3]            ; [CPU_] |103| 
        CMPB      AL,#64                ; [CPU_] |103| 
        BF        $C$L3,LO              ; [CPU_] |103| 
        ; branchcc occurs ; [] |103| 
$C$L6:    
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 137,column 4,is_stmt,isa 0
        MOVIZ     R0H,#49440            ; [CPU_] |137| 
        MOVW      DP,#_Step+10          ; [CPU_U] 
        MOV32     @_Step+10,R0H         ; [CPU_] |137| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 138,column 8,is_stmt,isa 0
        MOV       *-SP[3],#0            ; [CPU_] |138| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 138,column 13,is_stmt,isa 0
        MOV       AL,*-SP[3]            ; [CPU_] |138| 
        CMPB      AL,#64                ; [CPU_] |138| 
        BF        $C$L8,HIS             ; [CPU_] |138| 
        ; branchcc occurs ; [] |138| 
$C$L7:    
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 140,column 9,is_stmt,isa 0
$C$DW$23	.dwtag  DW_TAG_TI_branch
	.dwattr $C$DW$23, DW_AT_low_pc(0x00)
	.dwattr $C$DW$23, DW_AT_name("_atan")
	.dwattr $C$DW$23, DW_AT_TI_call
        LCR       #_atan                ; [CPU_] |140| 
        ; call occurs [#_atan] ; [] |140| 
        CLRC      SXM                   ; [CPU_] 
        MOVL      XAR4,#_Dlog+256       ; [CPU_U] |140| 
        MOV       ACC,*-SP[3] << 1      ; [CPU_] |140| 
        ADDL      XAR4,ACC              ; [CPU_] |140| 
        MOV32     *+XAR4[0],R0H         ; [CPU_] |140| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 141,column 9,is_stmt,isa 0
        MOVW      DP,#_Step+10          ; [CPU_U] 
        MOV32     R0H,@_Step+10         ; [CPU_] |141| 
        ADDF32    R0H,R0H,#16032        ; [CPU_] |141| 
        NOP       ; [CPU_] 
        MOV32     @_Step+10,R0H         ; [CPU_] |141| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 138,column 32,is_stmt,isa 0
        INC       *-SP[3]               ; [CPU_] |138| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 138,column 13,is_stmt,isa 0
        MOV       AL,*-SP[3]            ; [CPU_] |138| 
        CMPB      AL,#64                ; [CPU_] |138| 
        BF        $C$L7,LO              ; [CPU_] |138| 
        ; branchcc occurs ; [] |138| 
$C$L8:    
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 145,column 5,is_stmt,isa 0
$C$DW$24	.dwtag  DW_TAG_TI_branch
	.dwattr $C$DW$24, DW_AT_low_pc(0x00)
	.dwattr $C$DW$24, DW_AT_name("_done")
	.dwattr $C$DW$24, DW_AT_TI_call
        LCR       #_done                ; [CPU_] |145| 
        ; call occurs [#_done] ; [] |145| 
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 146,column 1,is_stmt,isa 0
        MOVB      AL,#0                 ; [CPU_] |146| 
        SUBB      SP,#14                ; [CPU_U] 
	.dwcfi	cfa_offset, -2
$C$DW$25	.dwtag  DW_TAG_TI_branch
	.dwattr $C$DW$25, DW_AT_low_pc(0x00)
	.dwattr $C$DW$25, DW_AT_TI_return
        LRETR     ; [CPU_] 
        ; return occurs ; [] 
	.dwattr $C$DW$11, DW_AT_TI_end_file("C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c")
	.dwattr $C$DW$11, DW_AT_TI_end_line(0x92)
	.dwattr $C$DW$11, DW_AT_TI_end_column(0x01)
	.dwendentry
	.dwendtag $C$DW$11

	.sect	".text"
	.clink
	.global	_done

$C$DW$26	.dwtag  DW_TAG_subprogram, DW_AT_name("done")
	.dwattr $C$DW$26, DW_AT_low_pc(_done)
	.dwattr $C$DW$26, DW_AT_high_pc(0x00)
	.dwattr $C$DW$26, DW_AT_TI_symbol_name("_done")
	.dwattr $C$DW$26, DW_AT_external
	.dwattr $C$DW$26, DW_AT_TI_begin_file("C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c")
	.dwattr $C$DW$26, DW_AT_TI_begin_line(0x94)
	.dwattr $C$DW$26, DW_AT_TI_begin_column(0x06)
	.dwattr $C$DW$26, DW_AT_TI_max_frame_size(-2)
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 149,column 1,is_stmt,address _done,isa 0

	.dwfde $C$DW$CIE, _done

;***************************************************************
;* FNAME: _done                         FR SIZE:   0           *
;*                                                             *
;* FUNCTION ENVIRONMENT                                        *
;*                                                             *
;* FUNCTION PROPERTIES                                         *
;*                            0 Parameter,  0 Auto,  0 SOE     *
;***************************************************************

_done:
	.dwcfi	cfa_offset, -2
	.dwcfi	save_reg_to_mem, 26, 0
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 150,column 4,is_stmt,isa 0
   ESTOP0
	.dwpsn	file "C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c",line 151,column 1,is_stmt,isa 0
        SPM       #0                    ; [CPU_] 
$C$DW$27	.dwtag  DW_TAG_TI_branch
	.dwattr $C$DW$27, DW_AT_low_pc(0x00)
	.dwattr $C$DW$27, DW_AT_TI_return
        LRETR     ; [CPU_] 
        ; return occurs ; [] 
	.dwattr $C$DW$26, DW_AT_TI_end_file("C:/Users/ricieri.ohashi/Desktop/FAC_Imas/C28_new/FPUfastRTS/V100/examples_ccsv4/source/fastRTS_sample.c")
	.dwattr $C$DW$26, DW_AT_TI_end_line(0x97)
	.dwattr $C$DW$26, DW_AT_TI_end_column(0x01)
	.dwendentry
	.dwendtag $C$DW$26

;**************************************************************
;* UNDEFINED EXTERNAL REFERENCES                              *
;**************************************************************
	.global	_cos
	.global	_atan
	.global	_sin
	.global	_sqrt
	.global	FS$$DIV

;***************************************************************
;* TYPE INFORMATION                                            *
;***************************************************************

$C$DW$T$21	.dwtag  DW_TAG_structure_type
	.dwattr $C$DW$T$21, DW_AT_name("DATA_LOG_C")
	.dwattr $C$DW$T$21, DW_AT_byte_size(0x280)
$C$DW$28	.dwtag  DW_TAG_member
	.dwattr $C$DW$28, DW_AT_type(*$C$DW$T$19)
	.dwattr $C$DW$28, DW_AT_name("SINwaveform")
	.dwattr $C$DW$28, DW_AT_TI_symbol_name("_SINwaveform")
	.dwattr $C$DW$28, DW_AT_data_member_location[DW_OP_plus_uconst 0x0]
	.dwattr $C$DW$28, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$29	.dwtag  DW_TAG_member
	.dwattr $C$DW$29, DW_AT_type(*$C$DW$T$19)
	.dwattr $C$DW$29, DW_AT_name("COSwaveform")
	.dwattr $C$DW$29, DW_AT_TI_symbol_name("_COSwaveform")
	.dwattr $C$DW$29, DW_AT_data_member_location[DW_OP_plus_uconst 0x80]
	.dwattr $C$DW$29, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$30	.dwtag  DW_TAG_member
	.dwattr $C$DW$30, DW_AT_type(*$C$DW$T$19)
	.dwattr $C$DW$30, DW_AT_name("ATANwaveform")
	.dwattr $C$DW$30, DW_AT_TI_symbol_name("_ATANwaveform")
	.dwattr $C$DW$30, DW_AT_data_member_location[DW_OP_plus_uconst 0x100]
	.dwattr $C$DW$30, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$31	.dwtag  DW_TAG_member
	.dwattr $C$DW$31, DW_AT_type(*$C$DW$T$20)
	.dwattr $C$DW$31, DW_AT_name("Phase")
	.dwattr $C$DW$31, DW_AT_TI_symbol_name("_Phase")
	.dwattr $C$DW$31, DW_AT_data_member_location[DW_OP_plus_uconst 0x180]
	.dwattr $C$DW$31, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$32	.dwtag  DW_TAG_member
	.dwattr $C$DW$32, DW_AT_type(*$C$DW$T$19)
	.dwattr $C$DW$32, DW_AT_name("Mag")
	.dwattr $C$DW$32, DW_AT_TI_symbol_name("_Mag")
	.dwattr $C$DW$32, DW_AT_data_member_location[DW_OP_plus_uconst 0x200]
	.dwattr $C$DW$32, DW_AT_accessibility(DW_ACCESS_public)
	.dwendtag $C$DW$T$21


$C$DW$T$22	.dwtag  DW_TAG_structure_type
	.dwattr $C$DW$T$22, DW_AT_name("STEP")
	.dwattr $C$DW$T$22, DW_AT_byte_size(0x14)
$C$DW$33	.dwtag  DW_TAG_member
	.dwattr $C$DW$33, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$33, DW_AT_name("Xsize")
	.dwattr $C$DW$33, DW_AT_TI_symbol_name("_Xsize")
	.dwattr $C$DW$33, DW_AT_data_member_location[DW_OP_plus_uconst 0x0]
	.dwattr $C$DW$33, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$34	.dwtag  DW_TAG_member
	.dwattr $C$DW$34, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$34, DW_AT_name("Ysize")
	.dwattr $C$DW$34, DW_AT_TI_symbol_name("_Ysize")
	.dwattr $C$DW$34, DW_AT_data_member_location[DW_OP_plus_uconst 0x2]
	.dwattr $C$DW$34, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$35	.dwtag  DW_TAG_member
	.dwattr $C$DW$35, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$35, DW_AT_name("Yoffset")
	.dwattr $C$DW$35, DW_AT_TI_symbol_name("_Yoffset")
	.dwattr $C$DW$35, DW_AT_data_member_location[DW_OP_plus_uconst 0x4]
	.dwattr $C$DW$35, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$36	.dwtag  DW_TAG_member
	.dwattr $C$DW$36, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$36, DW_AT_name("X")
	.dwattr $C$DW$36, DW_AT_TI_symbol_name("_X")
	.dwattr $C$DW$36, DW_AT_data_member_location[DW_OP_plus_uconst 0x6]
	.dwattr $C$DW$36, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$37	.dwtag  DW_TAG_member
	.dwattr $C$DW$37, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$37, DW_AT_name("Y")
	.dwattr $C$DW$37, DW_AT_TI_symbol_name("_Y")
	.dwattr $C$DW$37, DW_AT_data_member_location[DW_OP_plus_uconst 0x8]
	.dwattr $C$DW$37, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$38	.dwtag  DW_TAG_member
	.dwattr $C$DW$38, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$38, DW_AT_name("Z")
	.dwattr $C$DW$38, DW_AT_TI_symbol_name("_Z")
	.dwattr $C$DW$38, DW_AT_data_member_location[DW_OP_plus_uconst 0xa]
	.dwattr $C$DW$38, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$39	.dwtag  DW_TAG_member
	.dwattr $C$DW$39, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$39, DW_AT_name("GainX")
	.dwattr $C$DW$39, DW_AT_TI_symbol_name("_GainX")
	.dwattr $C$DW$39, DW_AT_data_member_location[DW_OP_plus_uconst 0xc]
	.dwattr $C$DW$39, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$40	.dwtag  DW_TAG_member
	.dwattr $C$DW$40, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$40, DW_AT_name("GainY")
	.dwattr $C$DW$40, DW_AT_TI_symbol_name("_GainY")
	.dwattr $C$DW$40, DW_AT_data_member_location[DW_OP_plus_uconst 0xe]
	.dwattr $C$DW$40, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$41	.dwtag  DW_TAG_member
	.dwattr $C$DW$41, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$41, DW_AT_name("FreqX")
	.dwattr $C$DW$41, DW_AT_TI_symbol_name("_FreqX")
	.dwattr $C$DW$41, DW_AT_data_member_location[DW_OP_plus_uconst 0x10]
	.dwattr $C$DW$41, DW_AT_accessibility(DW_ACCESS_public)
$C$DW$42	.dwtag  DW_TAG_member
	.dwattr $C$DW$42, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$42, DW_AT_name("FreqY")
	.dwattr $C$DW$42, DW_AT_TI_symbol_name("_FreqY")
	.dwattr $C$DW$42, DW_AT_data_member_location[DW_OP_plus_uconst 0x12]
	.dwattr $C$DW$42, DW_AT_accessibility(DW_ACCESS_public)
	.dwendtag $C$DW$T$22

$C$DW$T$2	.dwtag  DW_TAG_unspecified_type
	.dwattr $C$DW$T$2, DW_AT_name("void")
$C$DW$T$4	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$4, DW_AT_encoding(DW_ATE_boolean)
	.dwattr $C$DW$T$4, DW_AT_name("bool")
	.dwattr $C$DW$T$4, DW_AT_byte_size(0x01)
$C$DW$T$5	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$5, DW_AT_encoding(DW_ATE_signed_char)
	.dwattr $C$DW$T$5, DW_AT_name("signed char")
	.dwattr $C$DW$T$5, DW_AT_byte_size(0x01)
$C$DW$T$6	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$6, DW_AT_encoding(DW_ATE_unsigned_char)
	.dwattr $C$DW$T$6, DW_AT_name("unsigned char")
	.dwattr $C$DW$T$6, DW_AT_byte_size(0x01)
$C$DW$T$7	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$7, DW_AT_encoding(DW_ATE_signed_char)
	.dwattr $C$DW$T$7, DW_AT_name("wchar_t")
	.dwattr $C$DW$T$7, DW_AT_byte_size(0x01)
$C$DW$T$8	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$8, DW_AT_encoding(DW_ATE_signed)
	.dwattr $C$DW$T$8, DW_AT_name("short")
	.dwattr $C$DW$T$8, DW_AT_byte_size(0x01)
$C$DW$T$9	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$9, DW_AT_encoding(DW_ATE_unsigned)
	.dwattr $C$DW$T$9, DW_AT_name("unsigned short")
	.dwattr $C$DW$T$9, DW_AT_byte_size(0x01)
$C$DW$T$10	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$10, DW_AT_encoding(DW_ATE_signed)
	.dwattr $C$DW$T$10, DW_AT_name("int")
	.dwattr $C$DW$T$10, DW_AT_byte_size(0x01)
$C$DW$T$26	.dwtag  DW_TAG_pointer_type
	.dwattr $C$DW$T$26, DW_AT_type(*$C$DW$T$10)
	.dwattr $C$DW$T$26, DW_AT_address_class(0x20)
$C$DW$T$11	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$11, DW_AT_encoding(DW_ATE_unsigned)
	.dwattr $C$DW$T$11, DW_AT_name("unsigned int")
	.dwattr $C$DW$T$11, DW_AT_byte_size(0x01)
$C$DW$T$12	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$12, DW_AT_encoding(DW_ATE_signed)
	.dwattr $C$DW$T$12, DW_AT_name("long")
	.dwattr $C$DW$T$12, DW_AT_byte_size(0x02)

$C$DW$T$20	.dwtag  DW_TAG_array_type
	.dwattr $C$DW$T$20, DW_AT_type(*$C$DW$T$12)
	.dwattr $C$DW$T$20, DW_AT_language(DW_LANG_C)
	.dwattr $C$DW$T$20, DW_AT_byte_size(0x80)
$C$DW$43	.dwtag  DW_TAG_subrange_type
	.dwattr $C$DW$43, DW_AT_upper_bound(0x3f)
	.dwendtag $C$DW$T$20

$C$DW$T$13	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$13, DW_AT_encoding(DW_ATE_unsigned)
	.dwattr $C$DW$T$13, DW_AT_name("unsigned long")
	.dwattr $C$DW$T$13, DW_AT_byte_size(0x02)
$C$DW$T$14	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$14, DW_AT_encoding(DW_ATE_signed)
	.dwattr $C$DW$T$14, DW_AT_name("long long")
	.dwattr $C$DW$T$14, DW_AT_byte_size(0x04)
$C$DW$T$15	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$15, DW_AT_encoding(DW_ATE_unsigned)
	.dwattr $C$DW$T$15, DW_AT_name("unsigned long long")
	.dwattr $C$DW$T$15, DW_AT_byte_size(0x04)
$C$DW$T$16	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$16, DW_AT_encoding(DW_ATE_float)
	.dwattr $C$DW$T$16, DW_AT_name("float")
	.dwattr $C$DW$T$16, DW_AT_byte_size(0x02)

$C$DW$T$19	.dwtag  DW_TAG_array_type
	.dwattr $C$DW$T$19, DW_AT_type(*$C$DW$T$16)
	.dwattr $C$DW$T$19, DW_AT_language(DW_LANG_C)
	.dwattr $C$DW$T$19, DW_AT_byte_size(0x80)
$C$DW$44	.dwtag  DW_TAG_subrange_type
	.dwattr $C$DW$44, DW_AT_upper_bound(0x3f)
	.dwendtag $C$DW$T$19

$C$DW$T$17	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$17, DW_AT_encoding(DW_ATE_float)
	.dwattr $C$DW$T$17, DW_AT_name("double")
	.dwattr $C$DW$T$17, DW_AT_byte_size(0x02)
$C$DW$T$18	.dwtag  DW_TAG_base_type
	.dwattr $C$DW$T$18, DW_AT_encoding(DW_ATE_float)
	.dwattr $C$DW$T$18, DW_AT_name("long double")
	.dwattr $C$DW$T$18, DW_AT_byte_size(0x04)
	.dwattr $C$DW$CU, DW_AT_language(DW_LANG_C)

;***************************************************************
;* DWARF CIE ENTRIES                                           *
;***************************************************************

$C$DW$CIE	.dwcie 26
	.dwcfi	cfa_register, 20
	.dwcfi	cfa_offset, 0
	.dwcfi	undefined, 0
	.dwcfi	undefined, 1
	.dwcfi	undefined, 2
	.dwcfi	undefined, 3
	.dwcfi	undefined, 20
	.dwcfi	undefined, 21
	.dwcfi	undefined, 22
	.dwcfi	undefined, 23
	.dwcfi	undefined, 24
	.dwcfi	undefined, 25
	.dwcfi	undefined, 26
	.dwcfi	same_value, 28
	.dwcfi	undefined, 29
	.dwcfi	undefined, 30
	.dwcfi	undefined, 31
	.dwcfi	undefined, 32
	.dwcfi	undefined, 33
	.dwcfi	undefined, 34
	.dwcfi	undefined, 35
	.dwcfi	undefined, 36
	.dwcfi	undefined, 37
	.dwcfi	undefined, 38
	.dwcfi	undefined, 75
	.dwcfi	undefined, 76
	.dwcfi	undefined, 77
	.dwcfi	undefined, 4
	.dwcfi	undefined, 5
	.dwcfi	same_value, 6
	.dwcfi	same_value, 7
	.dwcfi	same_value, 8
	.dwcfi	same_value, 9
	.dwcfi	same_value, 10
	.dwcfi	same_value, 11
	.dwcfi	undefined, 12
	.dwcfi	undefined, 13
	.dwcfi	undefined, 14
	.dwcfi	undefined, 15
	.dwcfi	undefined, 16
	.dwcfi	undefined, 17
	.dwcfi	undefined, 18
	.dwcfi	undefined, 19
	.dwcfi	undefined, 43
	.dwcfi	undefined, 44
	.dwcfi	undefined, 47
	.dwcfi	undefined, 48
	.dwcfi	undefined, 51
	.dwcfi	undefined, 52
	.dwcfi	undefined, 55
	.dwcfi	undefined, 56
	.dwcfi	same_value, 59
	.dwcfi	same_value, 60
	.dwcfi	same_value, 63
	.dwcfi	same_value, 64
	.dwcfi	same_value, 67
	.dwcfi	same_value, 68
	.dwcfi	same_value, 71
	.dwcfi	same_value, 72
	.dwcfi	undefined, 73
	.dwcfi	undefined, 74
	.dwcfi	undefined, 39
	.dwcfi	undefined, 40
	.dwcfi	undefined, 27
	.dwendentry

;***************************************************************
;* DWARF REGISTER MAP                                          *
;***************************************************************

$C$DW$45	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AL")
	.dwattr $C$DW$45, DW_AT_location[DW_OP_reg0]
$C$DW$46	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AH")
	.dwattr $C$DW$46, DW_AT_location[DW_OP_reg1]
$C$DW$47	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("PL")
	.dwattr $C$DW$47, DW_AT_location[DW_OP_reg2]
$C$DW$48	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("PH")
	.dwattr $C$DW$48, DW_AT_location[DW_OP_reg3]
$C$DW$49	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("SP")
	.dwattr $C$DW$49, DW_AT_location[DW_OP_reg20]
$C$DW$50	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("XT")
	.dwattr $C$DW$50, DW_AT_location[DW_OP_reg21]
$C$DW$51	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("T")
	.dwattr $C$DW$51, DW_AT_location[DW_OP_reg22]
$C$DW$52	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("ST0")
	.dwattr $C$DW$52, DW_AT_location[DW_OP_reg23]
$C$DW$53	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("ST1")
	.dwattr $C$DW$53, DW_AT_location[DW_OP_reg24]
$C$DW$54	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("PC")
	.dwattr $C$DW$54, DW_AT_location[DW_OP_reg25]
$C$DW$55	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("RPC")
	.dwattr $C$DW$55, DW_AT_location[DW_OP_reg26]
$C$DW$56	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("FP")
	.dwattr $C$DW$56, DW_AT_location[DW_OP_reg28]
$C$DW$57	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("DP")
	.dwattr $C$DW$57, DW_AT_location[DW_OP_reg29]
$C$DW$58	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("SXM")
	.dwattr $C$DW$58, DW_AT_location[DW_OP_reg30]
$C$DW$59	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("PM")
	.dwattr $C$DW$59, DW_AT_location[DW_OP_reg31]
$C$DW$60	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("OVM")
	.dwattr $C$DW$60, DW_AT_location[DW_OP_regx 0x20]
$C$DW$61	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("PAGE0")
	.dwattr $C$DW$61, DW_AT_location[DW_OP_regx 0x21]
$C$DW$62	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AMODE")
	.dwattr $C$DW$62, DW_AT_location[DW_OP_regx 0x22]
$C$DW$63	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("INTM")
	.dwattr $C$DW$63, DW_AT_location[DW_OP_regx 0x23]
$C$DW$64	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("IFR")
	.dwattr $C$DW$64, DW_AT_location[DW_OP_regx 0x24]
$C$DW$65	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("IER")
	.dwattr $C$DW$65, DW_AT_location[DW_OP_regx 0x25]
$C$DW$66	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("V")
	.dwattr $C$DW$66, DW_AT_location[DW_OP_regx 0x26]
$C$DW$67	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("PSEUDOH")
	.dwattr $C$DW$67, DW_AT_location[DW_OP_regx 0x4c]
$C$DW$68	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("VOL")
	.dwattr $C$DW$68, DW_AT_location[DW_OP_regx 0x4d]
$C$DW$69	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AR0")
	.dwattr $C$DW$69, DW_AT_location[DW_OP_reg4]
$C$DW$70	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("XAR0")
	.dwattr $C$DW$70, DW_AT_location[DW_OP_reg5]
$C$DW$71	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AR1")
	.dwattr $C$DW$71, DW_AT_location[DW_OP_reg6]
$C$DW$72	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("XAR1")
	.dwattr $C$DW$72, DW_AT_location[DW_OP_reg7]
$C$DW$73	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AR2")
	.dwattr $C$DW$73, DW_AT_location[DW_OP_reg8]
$C$DW$74	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("XAR2")
	.dwattr $C$DW$74, DW_AT_location[DW_OP_reg9]
$C$DW$75	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AR3")
	.dwattr $C$DW$75, DW_AT_location[DW_OP_reg10]
$C$DW$76	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("XAR3")
	.dwattr $C$DW$76, DW_AT_location[DW_OP_reg11]
$C$DW$77	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AR4")
	.dwattr $C$DW$77, DW_AT_location[DW_OP_reg12]
$C$DW$78	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("XAR4")
	.dwattr $C$DW$78, DW_AT_location[DW_OP_reg13]
$C$DW$79	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AR5")
	.dwattr $C$DW$79, DW_AT_location[DW_OP_reg14]
$C$DW$80	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("XAR5")
	.dwattr $C$DW$80, DW_AT_location[DW_OP_reg15]
$C$DW$81	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AR6")
	.dwattr $C$DW$81, DW_AT_location[DW_OP_reg16]
$C$DW$82	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("XAR6")
	.dwattr $C$DW$82, DW_AT_location[DW_OP_reg17]
$C$DW$83	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("AR7")
	.dwattr $C$DW$83, DW_AT_location[DW_OP_reg18]
$C$DW$84	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("XAR7")
	.dwattr $C$DW$84, DW_AT_location[DW_OP_reg19]
$C$DW$85	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R0HL")
	.dwattr $C$DW$85, DW_AT_location[DW_OP_regx 0x2b]
$C$DW$86	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R0H")
	.dwattr $C$DW$86, DW_AT_location[DW_OP_regx 0x2c]
$C$DW$87	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R1HL")
	.dwattr $C$DW$87, DW_AT_location[DW_OP_regx 0x2f]
$C$DW$88	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R1H")
	.dwattr $C$DW$88, DW_AT_location[DW_OP_regx 0x30]
$C$DW$89	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R2HL")
	.dwattr $C$DW$89, DW_AT_location[DW_OP_regx 0x33]
$C$DW$90	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R2H")
	.dwattr $C$DW$90, DW_AT_location[DW_OP_regx 0x34]
$C$DW$91	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R3HL")
	.dwattr $C$DW$91, DW_AT_location[DW_OP_regx 0x37]
$C$DW$92	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R3H")
	.dwattr $C$DW$92, DW_AT_location[DW_OP_regx 0x38]
$C$DW$93	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R4HL")
	.dwattr $C$DW$93, DW_AT_location[DW_OP_regx 0x3b]
$C$DW$94	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R4H")
	.dwattr $C$DW$94, DW_AT_location[DW_OP_regx 0x3c]
$C$DW$95	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R5HL")
	.dwattr $C$DW$95, DW_AT_location[DW_OP_regx 0x3f]
$C$DW$96	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R5H")
	.dwattr $C$DW$96, DW_AT_location[DW_OP_regx 0x40]
$C$DW$97	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R6HL")
	.dwattr $C$DW$97, DW_AT_location[DW_OP_regx 0x43]
$C$DW$98	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R6H")
	.dwattr $C$DW$98, DW_AT_location[DW_OP_regx 0x44]
$C$DW$99	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R7HL")
	.dwattr $C$DW$99, DW_AT_location[DW_OP_regx 0x47]
$C$DW$100	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("R7H")
	.dwattr $C$DW$100, DW_AT_location[DW_OP_regx 0x48]
$C$DW$101	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("RBL")
	.dwattr $C$DW$101, DW_AT_location[DW_OP_regx 0x49]
$C$DW$102	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("RB")
	.dwattr $C$DW$102, DW_AT_location[DW_OP_regx 0x4a]
$C$DW$103	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("STFL")
	.dwattr $C$DW$103, DW_AT_location[DW_OP_regx 0x27]
$C$DW$104	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("STF")
	.dwattr $C$DW$104, DW_AT_location[DW_OP_regx 0x28]
$C$DW$105	.dwtag  DW_TAG_TI_assign_register, DW_AT_name("FPUHAZ")
	.dwattr $C$DW$105, DW_AT_location[DW_OP_reg27]
	.dwendtag $C$DW$CU

