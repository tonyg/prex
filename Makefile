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

hd.raw:
	qemu-img create $@ 20M

curses: fd.img hd.raw
	qemu -curses -fda fd.img -hda hd.raw

run: fd.img hd.raw
	qemu -fda fd.img -hda hd.raw

debug: fd.img
	qemu -s -S -fda fd.img -hda hd.raw
