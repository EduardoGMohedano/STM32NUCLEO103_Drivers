#Rules to compile for target STM32F103RB
#	drivers: This recipe will generate the object files of all Low level Drivers but will not link
#	all: 	This recipe will compile and link the respective given file and will generate a binary file




CC=arm-none-eabi-gcc
src=$(wildcard ./Src/*.c)
inc=-I './Inc/'

ARCHFLAGS=-mcpu=cortex-m3 -std=gnu11 --specs=nano.specs -mfloat-abi=soft -mthumb
CFLAGS=-Wall -g -O0 -std=c99
startup=startup_stm32f103rbtx
linker_file=STM32F103RBTX_FLASH.ld
target=./examples/$(output)

#append main file
src+=$(target).c

objs:=$(src:.c=.o)
%.o: %.c
	 $(CC) $< $(CFLAGS) $(inc) $(ARCHFLAGS) -c -o $@

.PHONY: all
all:	$(objs)	
	$(CC) $(startup).s $(CFLAGS) $(ARCHFLAGS) -c -o $(startup).o
	$(CC) $(objs) $(startup).o $(ARCHFLAGS) -T"./"$(linker_file) --specs=nosys.specs -o $(target).elf   


.PHONY: drivers
drivers: $(objs)
	@echo $(objs)
	@echo "Object files for all drivers were generated!!!"
	

.PHONY:	clean
clean:
	@echo "Removing object files and target binary!!!"
	@rm $(objs) $(startup).o ./examples/*.elf ./examples/*.o 

