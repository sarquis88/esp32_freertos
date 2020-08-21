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
