#include <driver.h>
#include <pci.h>
#include <sys/types.h>

#define CONFIG_ADDRESS	0xcf8
#define CONFIG_DATA	0xcfc

#define PCI_DEVICE_LIMIT	32 * 8

struct pci_device pci_devices[PCI_DEVICE_LIMIT];
size_t pci_device_count = 0;

static void addr_for(int bus, int dev, int fn, int offset, uint32_t *addr, uint32_t *dataport) {
  *addr = 0x80000000
    | ((bus & 0xff) << 16)
    | ((dev & 0x1f) << 11)
    | ((fn & 0x7) << 8)
    | (offset & 0xfc);
  *dataport = CONFIG_DATA + (offset & 0x03);
}

/*
 * Read a PCI configuration word. After http://wiki.osdev.org/PCI.
 *
 * Bus: 8 bits wide (0-255)
 * Device: 5 bits wide (0-31)
 * Function: 3 bits wide (0-7)
 * Offset: byte offset into the register, 0-255.
 * Width in bits between 0 and 32 inclusive.
 *
 * Everything is little-endian, remember, so offset 2 gets you the
 * upper halfword of a 32-bit value.
 */
static uint32_t read_pci_raw(int bus, int dev, int fn, int offset, int width) {
  uint32_t addr, dataport;

  addr_for(bus, dev, fn, offset, &addr, &dataport);

  bus_write_32(CONFIG_ADDRESS, addr);
  switch (width) {
    case 4:
      return bus_read_32(dataport);
    case 2:
      return bus_read_16(dataport);
    case 1:
      return bus_read_8(dataport);
    default:
      panic("read_pci_raw doesn't accept non 8/16/32 widths.");
      return 0xffffffff;
  }
}

uint32_t read_pci32(int bus, int dev, int fn, int offset) {
  return read_pci_raw(bus, dev, fn, offset, 4);
}

uint16_t read_pci16(int bus, int dev, int fn, int offset) {
  return (uint16_t) read_pci_raw(bus, dev, fn, offset, 2);
}

uint8_t read_pci8(int bus, int dev, int fn, int offset) {
  return (uint8_t) read_pci_raw(bus, dev, fn, offset, 1);
}

static void write_pci_raw(int bus, int dev, int fn, int offset, int width, int val) {
  uint32_t addr, dataport;

  addr_for(bus, dev, fn, offset, &addr, &dataport);

  bus_write_32(CONFIG_ADDRESS, addr);
  switch (width) {
    case 4:
      bus_write_32(dataport, val);
      break;
    case 2:
      bus_write_16(dataport, val);
      break;
    case 1:
      bus_write_8(dataport, val);
      break;
    default:
      panic("write_pci_raw doesn't accept non 8/16/32 widths.");
  }
}

void write_pci32(int bus, int dev, int fn, int offset, uint32_t val) {
  write_pci_raw(bus, dev, fn, offset, 4, val);
}

void write_pci16(int bus, int dev, int fn, int offset, uint16_t val) {
  write_pci_raw(bus, dev, fn, offset, 2, val);
}

void write_pci8(int bus, int dev, int fn, int offset, uint8_t val) {
  write_pci_raw(bus, dev, fn, offset, 1, val);
}

uint8_t read_pci_interrupt_line(int bus, int dev) {
  return read_pci8(bus, dev, 0, PCI_REGISTER_INTERRUPT_LINE);
}

void write_pci_interrupt_line(int bus, int dev, int irqno) {
  write_pci8(bus, dev, 0, PCI_REGISTER_INTERRUPT_LINE, irqno);
}

uint32_t read_pci_bar(int bus, int dev, int bar_number) {
  ASSERT(bar_number >= 0 && bar_number < N_PCI_BASE_ADDRESS_REGISTERS);
  return read_pci32(bus, dev, 0, PCI_REGISTER_BAR0 + bar_number * 4);
}

void write_pci_bar(int bus, int dev, int bar_number, uint32_t val) {
  ASSERT(bar_number >= 0 && bar_number < N_PCI_BASE_ADDRESS_REGISTERS);
  write_pci32(bus, dev, 0, PCI_REGISTER_BAR0 + bar_number * 4, val);
}

static void probe_pci(void) {
  int bus = 0; /* for now */
  int dev;
  int fn;

  for (dev = 0; dev < 32; dev++) {
    for (fn = 0; fn < 8; fn++) {
      int device_record_index = (dev << 3) | fn;
      struct pci_device *v = &pci_devices[device_record_index];
      v->bus = bus;
      v->slot = dev;
      v->function = fn;

      v->vendor_id = read_pci16(bus, dev, fn, PCI_REGISTER_VENDOR_ID);
      if (v->vendor_id != 0xffff) { /* all-ones is PCI's way of saying "what register?" */
	pci_device_count = device_record_index + 1;
	v->device_id = read_pci16(bus, dev, fn, PCI_REGISTER_DEVICE_ID);
	v->revision_id = read_pci8(bus, dev, fn, PCI_REGISTER_REVISION_ID);
	v->prog_if = read_pci8(bus, dev, fn, PCI_REGISTER_PROG_IF);
	v->subclass = read_pci8(bus, dev, fn, PCI_REGISTER_SUBCLASS);
	v->class_code = read_pci8(bus, dev, fn, PCI_REGISTER_CLASS_CODE);
	v->cache_line_size = read_pci8(bus, dev, fn, PCI_REGISTER_CACHE_LINE_SIZE);
	v->latency_timer = read_pci8(bus, dev, fn, PCI_REGISTER_LATENCY_TIMER);
	v->header_type = read_pci8(bus, dev, fn, PCI_REGISTER_HEADER_TYPE);
	v->bist = read_pci8(bus, dev, fn, PCI_REGISTER_BIST);

	if ((v->header_type & PCI_HEADER_TYPE_MASK) == PCI_HEADER_TYPE_GENERAL) {
	  v->subsystem_vendor_id = read_pci16(bus, dev, fn, PCI_REGISTER_SUBSYSTEM_VENDOR_ID);
	  v->subsystem_id = read_pci16(bus, dev, fn, PCI_REGISTER_SUBSYSTEM_ID);
	} else {
	  v->subsystem_vendor_id = 0xffff;
	  v->subsystem_id = 0xffff;
	}

	printf("PCI #%02x.%d htype=%02X %04X:%04X/%04X:%04X class=%02X:%02X rev=%02X progIF=%02X\n",
	       dev, fn,
	       v->header_type,
	       v->vendor_id, v->device_id,
	       v->subsystem_vendor_id, v->subsystem_id,
	       v->class_code, v->subclass,
	       v->revision_id, v->prog_if);
      }
    }
  }
}

static int pci_read(device_t dev, char *buf, size_t *nbyte, int blkno) {
  *nbyte = 0;
  return 0;
}

static int pci_write(device_t dev, char *buf, size_t *nbyte, int blkno) {
  return 0;
}

static int pci_init(struct driver *self) {
  device_create(self, "pci", D_CHR);
  probe_pci();
  return 0;
}

static struct devops pci_devops = {
	/* open */	no_open,
	/* close */	no_close,
	/* read */	pci_read,
	/* write */	pci_write,
	/* ioctl */	no_ioctl,
	/* devctl */	no_devctl,
};

struct driver pci_driver = {
	/* name */	"pci",
	/* devops */	&pci_devops,
	/* devsz */	0,
	/* flags */	0,
	/* probe */	NULL,
	/* init */	pci_init,
	/* shutdown */	NULL,
};
