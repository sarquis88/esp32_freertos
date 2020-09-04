make:
	./scripts/build.sh

flash:
	./scripts/flash.sh

monitor:
	./scripts/monitor.sh

config:
	./scripts/menuconfig.sh

clean:
	rm -rf ./esp-idf/build/

erase:
	./scripts/erase_flash.sh

flashandtest:
	make flash
	make monitor
	
eraseandtest:
	make erase 
	make flash
	make monitor
