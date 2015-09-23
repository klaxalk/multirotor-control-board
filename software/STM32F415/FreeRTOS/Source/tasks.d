tasks.o tasks.d : ../../Source/tasks.c ../../Source/include/FreeRTOS.h \
  ../../Source/include/projdefs.h FreeRTOSConfig.h \
  ../../Source/include/portable.h \
  ../../Source/include/../portable/GCC/ATMega323/portmacro.h \
  ../../Source/include/mpu_wrappers.h ../../Source/include/task.h \
  ../../Source/include/list.h ../../Source/include/timers.h \
  ../../Source/include/task.h ../../Source/include/StackMacros.h
