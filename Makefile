SUBDIR:=	bsp sys usr
SRCDIR:=	$(CURDIR)
export SRCDIR

_DEBUG_=1
export _DEBUG_

include $(SRCDIR)/mk/image.mk

#
# tonyg 20110306 - targets for floppy-disk images and booting
#
fd.img: all prexos
	make -C bsp/boot/x86/tools/bootsect
	mformat -i fd.img -a -C -f 1440 -B bsp/boot/x86/tools/bootsect/bootsect.bin ::
	mcopy -i fd.img prexos ::

curses: fd.img
	qemu -curses -fda fd.img

run: fd.img
	qemu -fda fd.img

debug: fd.img
	qemu -s -S -curses -fda fd.img
