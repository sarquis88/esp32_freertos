make:
	./scripts/build.sh

flash:
	./scripts/flash.sh

monitor:
	./scripts/monitor.sh

clean:
	rm -rf ./esp-idf/build/
