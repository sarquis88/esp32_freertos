make:
	./build.sh

flash:
	./flash.sh

monitor:
	./monitor.sh

clean:
	rm -rf ./esp-idf/build/
