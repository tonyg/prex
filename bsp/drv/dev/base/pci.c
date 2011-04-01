#include <driver.h>
#include <pci.h>
#include <sys/types.h>

#define CONFIG_ADDRESS	0xcf8
#define CONFIG_DATA	0xcfc

#define PCI_DEVICE_LIMIT	32

struct pci_device pci_devices[PCI_DEVICE_LIMIT];
size_t pci_device_count = 0;

static uint32_t addr_for(int bus, int dev, int fn, int offset) {
  return 0x80000000
    | ((bus & 0xff) << 16)
    | ((dev & 0x1f) << 11)
    | ((fn & 0x7) << 8)
    | (offset & 0xfc);
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
static uint32_t read_pci_raw(int bus, int dev, int fn, int offset, int width_bits) {
  uint32_t addr = addr_for(bus, dev, fn, offset);
  uint32_t result;

  bus_write_32(CONFIG_ADDRESS, addr);
  result = bus_read_32(CONFIG_DATA);
  result = result >> ((offset & 2) * 8);
  result = result & ~(- (1 << width_bits));
  return result;
}

uint32_t read_pci32(int bus, int dev, int fn, int offset) {
  return read_pci_raw(bus, dev, fn, offset, 32);
}

uint16_t read_pci16(int bus, int dev, int fn, int offset) {
  return (uint16_t) read_pci_raw(bus, dev, fn, offset, 16);
}

uint8_t read_pci8(int bus, int dev, int fn, int offset) {
  return (uint8_t) read_pci_raw(bus, dev, fn, offset, 8);
}

static void write_pci_raw(int bus, int dev, int fn, int offset, int width_bits, int val) {
  uint32_t addr = addr_for(bus, dev, fn, offset);

  /* TODO: figure out whether (and if so, how) to shift and mask val
     to write to the right subregister. Stupid x86 I/O space. */
  bus_write_32(CONFIG_ADDRESS, addr);
  switch (width_bits) {
    case 32:
      bus_write_32(CONFIG_DATA, val);
      break;
    case 16:
      bus_write_16(CONFIG_DATA, val);
      break;
    case 8:
      bus_write_8(CONFIG_DATA, val);
      break;
    default:
      panic("Dude write_pci_raw doesn't accept non 8/16/32 widths.\n");
  }
}

void write_pci32(int bus, int dev, int fn, int offset, uint32_t val) {
  write_pci_raw(bus, dev, fn, offset, 32, val);
}

void write_pci16(int bus, int dev, int fn, int offset, uint16_t val) {
  write_pci_raw(bus, dev, fn, offset, 16, val);
}

void write_pci8(int bus, int dev, int fn, int offset, uint8_t val) {
  write_pci_raw(bus, dev, fn, offset, 8, val);
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

  for (dev = 0; dev < 32; dev++) {
    struct pci_device *v = &pci_devices[dev];
    v->bus = bus;
    v->slot = dev;

    v->vendor_id = read_pci16(bus, dev, 0, 0);
    if (v->vendor_id != 0xffff) { /* all-ones is PCI's way of saying "what register?" */
      pci_device_count = dev + 1;
      v->device_id = read_pci16(bus, dev, 0, 2);
      v->revision_id = read_pci8(bus, dev, 0, 8);
      v->prog_if = read_pci8(bus, dev, 0, 9);
      v->subclass = read_pci8(bus, dev, 0, 10);
      v->class_code = read_pci8(bus, dev, 0, 11);
      v->cache_line_size = read_pci8(bus, dev, 0, 12);
      v->latency_timer = read_pci8(bus, dev, 0, 13);
      v->header_type = read_pci8(bus, dev, 0, 14);
      v->bist = read_pci8(bus, dev, 0, 15);

      printf("PCI #%02x %04X:%04X rev=%02X class=%02X:%02X:%02X\n",
	     dev,
	     v->vendor_id, v->device_id,
	     v->revision_id,
	     v->class_code, v->subclass, v->prog_if);
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
