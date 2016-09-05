TARGET = libcamcv.so
PREFIX = /usr/local

USERLAND_HOME = /opt/vc/userland
CFLAGS = -Wall -O3 -I$(USERLAND_HOME) -I$(USERLAND_HOME)/host_applications/linux/libs/bcm_host/include -fpic
LIBS = -L/usr/local/lib -lopencv_core -L/opt/vc/lib/ -L/usr/lib/ -lmmal -lmmal_core -lmmal_util -lbcm_host
LDFLAGS = $(LIBS) -shared

all: $(TARGET)

$(TARGET) : camcv.cpp camcv.hpp
	g++ $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm libcamcv

install:
	mkdir -p $(DESTDIR)$(PREFIX)/lib
	mkdir -p $(DESTDIR)$(PREFIX)/include/
	cp $(TARGET) $(DESTDIR)$(PREFIX)/lib/
	cp camcv.hpp $(DESTDIR)$(PREFIX)/include/
	chmod 0755 $(DESTDIR)$(PREFIX)/lib/$(TARGET)
	ldconfig

.PHONY: all clean install
