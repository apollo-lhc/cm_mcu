# gdb init script

### memfault
define hook-continue
   # Enable automatic halt for HardFault Exception using
   # by setting DEMCR->VC_HARDERR
   set *((uint32_t*)0xE000EDFC)|=0x0000400
end

define fault_regs
   set $cfsr=*(uint32_t*)0xE000ED28
   printf "CFSR: 0x%08x\n", $cfsr
   if $cfsr & 0x8000
       printf "BusFault Address Valid: 0x%8x\n", *(uint32_t)0xE000ED38
   end
end

 define unwind_stack
   # Set registers to their state prior to exception entry
   # to investigate origin of fault
   set $exc_frame = ($lr & 0x4) ? $psp : $msp
   set $stacked_xpsr = ((uint32_t *)$exc_frame)[7]
   set $exc_frame_len = 32 + ((($stacked_xpsr & (1 << 9)) ? 0x4 : 0x0) + (($lr & 0x10) ? 0 : 72)) 
   set $sp=($exc_frame + $exc_frame_len)
   set $lr=((uint32_t *)$exc_frame)[5]
   set $pc=((uint32_t *)$exc_frame)[6]
end

# mcu on eclipse
define armex
  printf "EXEC_RETURN (LR):\n",
  info registers $lr
    if (($lr & 0x4) == 0x4)
      printf "Uses MSP 0x%x return.\n", $msp
      set $armex_base = $msp
    else
      printf "Uses PSP 0x%x return.\n", $psp
      set $armex_base = $psp
    end
  
    printf "xPSR            0x%x\n", *($armex_base+28)
    printf "ReturnAddress   0x%x\n", *($armex_base+24)
    printf "LR (R14)        0x%x\n", *($armex_base+20)
    printf "R12             0x%x\n", *($armex_base+16)
    printf "R3              0x%x\n", *($armex_base+12)
    printf "R2              0x%x\n", *($armex_base+8)
    printf "R1              0x%x\n", *($armex_base+4)
    printf "R0              0x%x\n", *($armex_base)
    printf "Return instruction:\n"
    x/i *($armex_base+24)
    printf "LR instruction:\n"
    x/i *($armex_base+20)
end
  
document armex
ARMv7 Exception entry behavior.
xPSR, ReturnAddress, LR (R14), R12, R3, R2, R1, and R0
end