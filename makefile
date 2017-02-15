PREFIX = /usr

all:
	cython oak3d.py --embed --verbose
	gcc -O3 -I/usr/include/python3.6m -lpython3.6m oak3d.c -o oak3d-bin

install:
	mkdir -p $(DESTDIR)$(PREFIX)/bin
	cp -p oak3d-bin $(DESTDIR)$(PREFIX)/bin/oak3d

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/bin/oak3d

clean:
	rm oak3d.c
	rm oak3d-bin

