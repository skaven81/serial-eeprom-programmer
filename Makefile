# vim: syntax=make ts=4 sts=4 sw=4 noexpandtab

msp430.hex: main.c
	msp430-gcc -Os -std=gnu99 -g -Wall --printf_support=minimal \
		-mmcu=msp430g2553 main.c -o msp430.hex

define RUN_COMMANDS
prog msp430.hex
run
exit
endef
export RUN_COMMANDS

run: msp430.hex
	 echo "$$RUN_COMMANDS" | mspdebug rf2500

debug: msp430.hex
	 mspdebug rf2500
