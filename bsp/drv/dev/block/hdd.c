#include <driver.h>
#include <pci.h>

/* Much of the content of this driver was worked out using
   wiki.osdev.org, the FreeBSD source code and the grub2 source code
   as references, though no code was copied from those sources. The
   Intel datasheets and the ATA spec drafts were also really useful.

   http://www.intel.com/design/chipsets/datashts/29054901.pdf
   http://www.intel.com/assets/pdf/datasheet/290562.pdf
   http://www.t13.org/Documents/UploadedDocuments/project/d0948r4c-ATA-2.pdf
*/

#define HDC_IRQ		14	/* Yeah, there are more than 16 these days.
				   Does prex support anything more than 16? */
/* In any case, we should be spreading the load around, and not
 * sharing one IRQ for all the IDE controllers in the system. For now,
 * sharing the IRQ is fine.
 *
 * FIXME: OK, sharing *would* be fine, but see the comment in the
 * implementation of irq_attach that says that sharing isn't supported
 * by prex. */

typedef enum ata_port_register_t_ {
  ATA_REG_DATA = 0,
  ATA_REG_ERR = 1, /* osdev.org claims this is mostly for ATAPI? */
  ATA_REG_SECTOR_COUNT = 2,

  /* CHS addressing */
  ATA_REG_SECTOR_NUMBER = 3,
  ATA_REG_CYLINDER_LOW = 4,
  ATA_REG_CYLINDER_HIGH = 5,

  /* LBA addressing */
  ATA_REG_LBA_LOW = 3,
  ATA_REG_LBA_MID = 4,
  ATA_REG_LBA_HIGH = 5,

  ATA_REG_DISK_SELECT = 6, /* also contains head number for CHS addressing */
  ATA_REG_COMMAND_STATUS = 7
} ata_port_register_t;

#define ATA_LEGACY_PRIMARY_CONTROL_BASE		0x1f0
#define ATA_LEGACY_SECONDARY_CONTROL_BASE	0x170
/* In "legacy" mode, the control block base port number + this offset
   is the port number of the control/altstatus register. In PCI IDE
   mode, we look at the BARs instead: BAR1 points to a 4-byte space,
   within which offset 2 is the control/altstatus register. */
#define ATA_LEGACY_CONTROL_ALTERNATE_STATUS_OFFSET 0x206

typedef enum ata_status_flag_t_ {
  ATA_STATUS_FLAG_ERROR = 0x01,
  ATA_STATUS_FLAG_DRQ = 0x08,
  ATA_STATUS_FLAG_BUSY = 0x80
} ata_status_flag_t;

#define DEBUG_HDD 1

#if DEBUG_HDD
#define DPRINTF(a)	printf a
#else
#define DPRINTF(a)
#endif

#define BUFFER_LENGTH 65536 /* FIXME: proper caching please */

struct ata_channel {
  int base_port;
  int control_port;
  int dma_port;
};

struct ata_disk {
  int valid;
  int channel;
  int slave; /* 0 => master, 1 => slave */
  uint8_t identification_space[512];

  /* These fields are extracted from identification_space: */
  uint8_t serial_number[10];
  uint8_t firmware_revision[8];
  uint8_t model[40];
  int lba_supported;
  int dma_supported;
  uint32_t sector_capacity;
  uint32_t addressable_sector_count;
};

struct hdd_softc {
  device_t dev;
  char devname[4]; /* TODO: 4 is a magic number */
  struct pci_device *pci_dev;
  int isopen; /* FIXME: do we care? */
  struct irp irp;
  irq_t irq;
  struct ata_channel channel[2];
  struct ata_disk disk[4];
  uint8_t *buffer;
};

static int hdc_isr(void *arg) {
  struct hdd_softc *sc = arg;
  DPRINTF(("hdc_isr\n"));
}

static void hdc_ist(void *arg) {
  struct hdd_softc *sc = arg;
  DPRINTF(("hdc_ist\n"));
}

static void ata_write(struct hdd_softc *sc, int channelnum, int reg, uint8_t val) {
  bus_write_8(sc->channel[channelnum].base_port + reg, val);
}

static uint8_t ata_read(struct hdd_softc *sc, int channelnum, int reg) {
  return bus_read_8(sc->channel[channelnum].base_port + reg);
}

static void write_control(struct hdd_softc *sc, int channelnum, uint8_t val) {
  bus_write_8(sc->channel[channelnum].control_port, val);
}

static uint8_t read_altstatus(struct hdd_softc *sc, int channelnum) {
  return bus_read_8(sc->channel[channelnum].control_port);
}

/* A 400ns delay, used to wait for the device to start processing a
 * sent command and assert busy. */
static void ata_delay400(struct hdd_softc *sc, int channelnum) {
  read_altstatus(sc, channelnum);
  read_altstatus(sc, channelnum);
  read_altstatus(sc, channelnum);
  read_altstatus(sc, channelnum);
}

static void ata_wait(struct hdd_softc *sc, int channelnum) {
  unsigned int i;

  ata_delay400(sc, channelnum);

  for (i = 0; i < 0x80000000; i++) {
    if (!(read_altstatus(sc, channelnum) & ATA_STATUS_FLAG_BUSY)) {
      return;
    }
  }

  printf("ata_wait: busy never went away!!\n");
  /* TODO: reset device here, maybe? We'd have to retry or abort
     in-progress operations. */
}

static void ata_pio_read(struct hdd_softc *sc, int channelnum, uint8_t *buffer, size_t count) {
  ASSERT((count & 3) == 0); /* multiple of 4 bytes. */
  while (count > 0) {
    uint32_t v = bus_read_32(sc->channel[channelnum].base_port + ATA_REG_DATA);
    buffer[0] = v & 0xff;
    buffer[1] = (v >> 8) & 0xff;
    buffer[2] = (v >> 16) & 0xff;
    buffer[3] = (v >> 24) & 0xff;
    buffer += 4;
    count -= 4;
  }
}

static void fixup_string_endianness(uint8_t *p, size_t size) {
  while (size > 0) {
    uint8_t tmp = p[1];
    p[1] = p[0];
    p[0] = tmp;
    p += 2;
    size -= 2;
  }
}

static void setup_disk(struct hdd_softc *sc, int disknum) {
  struct ata_disk *disk = &sc->disk[disknum];

  disk->valid = 0; /* to be determined properly below */
  disk->channel = disknum >> 1;
  disk->slave = disknum & 1;

  /* Send IDENTIFY command (0xEC). */

  ata_write(sc, disk->channel, ATA_REG_DISK_SELECT, 0xA0 | (disk->slave << 4));
  ata_wait(sc, disk->channel);

  ata_write(sc, disk->channel, ATA_REG_SECTOR_COUNT, 0);
  ata_write(sc, disk->channel, ATA_REG_LBA_LOW, 0);
  ata_write(sc, disk->channel, ATA_REG_LBA_MID, 0);
  ata_write(sc, disk->channel, ATA_REG_LBA_HIGH, 0);

  ata_write(sc, disk->channel, ATA_REG_COMMAND_STATUS, 0xEC);
  ata_delay400(sc, disk->channel);

  if (ata_read(sc, disk->channel, ATA_REG_COMMAND_STATUS) == 0) {
    printf("Disk %d absent (wouldn't accept command).\n", disknum);
    return;
  }

  ata_wait(sc, disk->channel);
  if (read_altstatus(sc, disk->channel) & ATA_STATUS_FLAG_ERROR) {
    printf("Disk %d absent (wouldn't identify).\n", disknum);
    return;
  }

  /* ATAPI devices return special values in LBA_MID and LBA_HIGH. We
     don't check those here. (TODO) */

  ata_pio_read(sc, disk->channel, disk->identification_space, sizeof(disk->identification_space));

  memcpy(disk->serial_number, &disk->identification_space[20], sizeof(disk->serial_number));
  memcpy(disk->firmware_revision, &disk->identification_space[46], sizeof(disk->firmware_revision));
  memcpy(disk->model, &disk->identification_space[54], sizeof(disk->model));
  disk->lba_supported = ((disk->identification_space[100] & 2) != 0);
  disk->dma_supported = ((disk->identification_space[100] & 1) != 0);
  memcpy(&disk->sector_capacity, &disk->identification_space[114], sizeof(disk->sector_capacity));
  memcpy(&disk->addressable_sector_count,
	 &disk->identification_space[120],
	 sizeof(disk->addressable_sector_count));

  fixup_string_endianness(disk->serial_number, sizeof(disk->serial_number));
  fixup_string_endianness(disk->firmware_revision, sizeof(disk->firmware_revision));
  fixup_string_endianness(disk->model, sizeof(disk->model));

  printf("Disk %d:\n", disknum);
  printf(" - serial %.*s\n", sizeof(disk->serial_number), disk->serial_number);
  printf(" - firmware %.*s\n", sizeof(disk->firmware_revision), disk->firmware_revision);
  printf(" - model %.*s\n", sizeof(disk->model), disk->model);
  printf(" - sector count %d (0x%x)\n",
	 disk->addressable_sector_count,
	 disk->addressable_sector_count);

  disk->valid = 1;

  /* TODO: register the device for the disk. */
  /* hd0d0p0 ...? */
}

static void setup_device(struct driver *self, struct pci_device *v) {
  static char which_device = '0';
  char devname_tmp[4];
  int primary_native;
  int secondary_native;

  struct hdd_softc *sc;
  struct irp *irp;
  device_t dev;

  /* According to the "PCI IDE Controller Specification Revision 1.0",
     which I retrieved from
     http://suif.stanford.edu/~csapuntz/specs/pciide.ps, the prog_if
     value contains bits describing whether the IDE controller is in
     PCI native or compatibility mode:

     76543210
     |   ||||
     |   |||\--	0 => primary channel in compatibility mode, 1 => native
     |   ||\---	0 => primary channel can't switch modes, 1 => it can
     |   |\----	0 => secondary channel in compatibility mode, 1 => native
     |   \-----	0 => secondary channel can't switch modes, 1 => it can
     \---------	0 => can't bus-master-dma, 1 => can

     (That last one, bit 7 (0x80), is implied by the PIIX3
     documentation for the embedded IDE controller. I don't have a
     better source than that.)

     Therefore, looking at bits 0 and 2 will tell us which programming
     interface to use for the IDE controller we have in the system.
  */

  primary_native = ((v->prog_if & 0x01) != 0);
  secondary_native = ((v->prog_if & 0x04) != 0);

  {
    char *n = &devname_tmp[0];
    n[0] = 'h';
    n[1] = 'd';
    n[2] = which_device++; /* barrrrrrrrrrrrrrf */
    n[3] = '\0';
    /* Why is there a vsprintf but no vsnprintf or snprintf? */
    dev = device_create(self, n, D_BLK | D_PROT);
  }

  printf("device %d.%d.%d = %s\n", v->bus, v->slot, v->function, devname_tmp);

  sc = device_private(dev);
  sc->dev = dev;
  memcpy(&sc->devname[0], &devname_tmp[0], sizeof(devname_tmp));
  sc->pci_dev = v;
  sc->isopen = 0;

  irp = &sc->irp;
  irp->cmd = IO_NONE;
  event_init(&irp->iocomp, &sc->devname[0]);

  /* TODO: if we're operating in compatibility/legacy mode, we are
     behaving like an old school IDE adapter, which wants to use IRQ14
     for the primary and IRQ15 for the secondary controller. We
     currently only take one IRQ, so secondary controllers won't
     work. */

  /* TODO: claiming an IRQ more than once causes, um, issues, so don't do that. Ever. */
  sc->irq = irq_attach(HDC_IRQ, IPL_BLOCK, 0, hdc_isr, hdc_ist, sc);

  if (primary_native || secondary_native) {
    /* Tell the controller which IRQ to use, if we're in native mode. */
    write_pci_interrupt_line(v, HDC_IRQ);
  }

  sc->buffer = ptokv(page_alloc(BUFFER_LENGTH));

  /* TODO: It is unclear whether, in native mode, the BARs contain
     port numbers directly, or whether they should be masked with
     ~0x03. The low two bits might be used as flags?? */

  if (primary_native) {
    sc->channel[0].base_port = read_pci_bar(v, 0);
    sc->channel[0].control_port = read_pci_bar(v, 1) + 2;
  } else {
    sc->channel[0].base_port = ATA_LEGACY_PRIMARY_CONTROL_BASE;
    sc->channel[0].control_port =
      ATA_LEGACY_PRIMARY_CONTROL_BASE + ATA_LEGACY_CONTROL_ALTERNATE_STATUS_OFFSET;
  }

  if (secondary_native) {
    sc->channel[1].base_port = read_pci_bar(v, 2);
    sc->channel[1].control_port = read_pci_bar(v, 3) + 2;
  } else {
    sc->channel[1].base_port = ATA_LEGACY_SECONDARY_CONTROL_BASE;
    sc->channel[1].control_port =
      ATA_LEGACY_SECONDARY_CONTROL_BASE + ATA_LEGACY_CONTROL_ALTERNATE_STATUS_OFFSET;
  }

  /* BAR4 points to a 16-byte block of I/O port space, the low 8 bytes
     of which are for the primary and the high 8 bytes for the
     secondary controller. */
  sc->channel[0].dma_port = read_pci_bar(v, 4);
  sc->channel[1].dma_port = sc->channel[0].dma_port + 8;

  printf(" - pri 0x%04x/0x%04x/0x%04x, sec 0x%04x/0x%04x/0x%04x\n",
	 sc->channel[0].base_port, sc->channel[0].control_port, sc->channel[0].dma_port,
	 sc->channel[1].base_port, sc->channel[1].control_port, sc->channel[1].dma_port);

  /* Disable interrupts from the two channels. */
  write_control(sc, 0, 2);
  write_control(sc, 1, 2);

  {
    int disk;
    for (disk = 0; disk < 4; disk++) {
      setup_disk(sc, disk);
    }
  }

  /* Reenable interrupts from the two channels. */
  write_control(sc, 0, 0);
  write_control(sc, 1, 0);
}

static int hdd_init(struct driver *self) {
  int i;

  for (i = 0; i < pci_device_count; i++) {
    struct pci_device *v = &pci_devices[i];
    if (v->class_code == PCI_CLASS_STORAGE &&
	v->subclass == 1 /* IDE */)
    {
      setup_device(self, v);
    }
  }

  return 0;
}

static int hdd_open(device_t dev, int mode) {
  struct hdd_softc *sc = device_private(dev);

  if (sc->isopen > 0) {
    return EBUSY;
  }
  /* Is this a race? fdd.c does the same thing. */
  sc->isopen++;
  sc->irp.cmd = IO_NONE;
  return 0;
}

static int hdd_close(device_t dev) {
  struct hdd_softc *sc = device_private(dev);

  if (sc->isopen != 1) {
    return EINVAL;
  }
  /* Is this a race? fdd.c does the same thing. */
  sc->isopen--;
  sc->irp.cmd = IO_NONE;
  /* TODO: reset the controller perhaps? Or shut it down? The fdd
     driver switches off the drive motor here. */
  return 0;
}

static int hdd_read(device_t dev, char *buf, size_t *nbyte, int blnko) {
  struct hdd_softc *sc = device_private(dev);
  return EINVAL;
}

static int hdd_write(device_t dev, char *buf, size_t *nbyte, int blkno) {
  struct hdd_softc *sc = device_private(dev);
  return EINVAL;
}

static struct devops hdd_devops = {
	/* open */	hdd_open,
	/* close */	hdd_close,
	/* read */	hdd_read,
	/* write */	hdd_write,
	/* ioctl */	no_ioctl,
	/* devctl */	no_devctl,
};

struct driver hdd_driver = {
	/* name */	"hdd",
	/* devsops */	&hdd_devops,
	/* devsz */	sizeof(struct hdd_softc),
	/* flags */	0,
	/* probe */	NULL,
	/* init */	hdd_init,
	/* shutdown */	NULL,
};
