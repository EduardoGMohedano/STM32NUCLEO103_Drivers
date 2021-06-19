#Rules to compile for target STM32F103RB
#	clean:  This recipe will remove all the object files of all Low level Drivers including binary and elf files
#	all: 	This recipe will compile and link the respective given file and will generate a binary file

CC=arm-none-eabi-gcc
SIZE=arm-none-eabi-size
CP=arm-none-eabi-objcopy
SF=st-flash
src=$(wildcard ./Src/*.c)
inc=-I './Inc/'

ARCHFLAGS=-mcpu=cortex-m3 --specs=nano.specs -mfloat-abi=soft -mthumb
CFLAGS=-ffunction-sections -fdata-sections -Wall -g -O0 -std=gnu11
startup=startup_stm32f103rbtx
linker_file=STM32F103RBTX_FLASH.ld
target=./examples/$(output)

#append main file
src+=$(target).c

objs:=$(src:.c=.o)
%.o: %.c
	@echo "Object files for all drivers were generated!!!"
	$(CC) $< $(CFLAGS) $(inc) $(ARCHFLAGS) -c -o $@

.PHONY: all
all:	$(objs)	
	$(CC) $(startup).s $(CFLAGS) $(ARCHFLAGS) -c -o $(startup).o
	$(CC) $(objs) $(startup).o $(ARCHFLAGS) -T"./"$(linker_file) --specs=nosys.specs -o $(target).elf   
	$(SIZE)	$(target).elf 
	$(CP) -O binary	$(target).elf $(target).bin 
	@mkdir ./Output 
	@mv ./examples/*.elf ./examples/*.bin ./Output

.PHONY: flash
flash:	
	@echo "Flashing on the target..."
	$(SF) write ./Output/*.bin 0x8000000	

.PHONY:	clean
clean:
	@echo "Removing object files and target binary!!!"
	@rm -f $(objs) $(startup).o ./examples/*.elf ./examples/*.o ./examples/*.bin 
	@rm -rf ./Output
