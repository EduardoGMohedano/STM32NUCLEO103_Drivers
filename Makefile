#Rules to compile for target STM32F103RB
#	drivers: This recipe will generate the object files of all Low level Drivers but will not link
#	all: 	This recipe will compile and link the respective given file and will generate a binary file




CC=arm-none-eabi-gcc
src=$(wildcard ./Src/*.c)
inc=$(wildcard ./Inc/*.h)

flags= -mcpu=cortex-m3 -std=gnu11 -O0 -Wall --specs=nano.specs --mfloat-abi=soft -mthumb

startup=startup_stm32f103rbtx.s
linker_file=STM32F103RBTX_FLASH.ld
target=output

objs:
	%.c:%.o


#echo $(src)

.PHONY: drivers
drivers:
	@echo "Generating object files for all drivers!!!"
	$(CC) $(src) -c -o $(objs)

.PHONY:	clean
clean:
	@echo "Removing object files and target binary"
	rm $(objs)

