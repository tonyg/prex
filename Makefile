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

image.iso: fd.img
	genisoimage -b fd.img -o image.iso fd.img

# We imagine that our 20M "disk" has 320 cylinders, 8 heads, and 16 sectors/cylinder.
# We partition it into four partitions, roughly 5 MB each, all typed as FAT16.
hd.raw:
	qemu-img create $@ 20M
	printf ",80,6\n,80,6\n,80,6\n,,6\n" | sfdisk -C 320 -H 8 -S 16 $@

curses: fd.img hd.raw
	qemu -curses -fda fd.img -hda hd.raw

run: fd.img hd.raw
	qemu -fda fd.img -hda hd.raw

debug: fd.img
	qemu -s -S -fda fd.img -hda hd.raw
