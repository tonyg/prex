#include <sys/param.h>
#include <driver.h>
#include <pci.h>

typedef unsigned long long uint64_t; /* Hmm. */

/* Much of the content of this driver was worked out using
   wiki.osdev.org, the FreeBSD source code and the grub2 source code
   as references, though no code was copied from those sources. The
   Intel datasheets and the ATA spec drafts were also really useful.

   http://www.intel.com/design/chipsets/datashts/29054901.pdf
   http://www.intel.com/assets/pdf/datasheet/290562.pdf
   http://www.t13.org/Documents/UploadedDocuments/project/d0948r4c-ATA-2.pdf
   http://www.t13.org/Documents/UploadedDocuments/docs2007/D1699r4a-ATA8-ACS.pdf
   http://www.t13.org/documents/UploadedDocuments/docs2006/D1700r3-ATA8-AAM.pdf
   http://suif.stanford.edu/~csapuntz/specs/pciide.ps
   http://suif.stanford.edu/~csapuntz/specs/idems100.ps
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

#define SECTOR_SIZE	512

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
  ATA_STATUS_FLAG_DEVICE_FAILURE = 0x20,
  ATA_STATUS_FLAG_BUSY = 0x80
} ata_status_flag_t;

#define DEBUG_HDD 1

#if DEBUG_HDD
#define DPRINTF(a)	printf a
#else
#define DPRINTF(a)
#endif

#define BUFFER_LENGTH 65536 /* FIXME: proper caching please */
#define BUFFER_LENGTH_IN_SECTORS (BUFFER_LENGTH / SECTOR_SIZE)

struct ata_channel {
  int base_port;
  int control_port;
  int dma_port;
};

struct ata_disk {
  int valid;
  struct ata_controller *controller;
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
  uint64_t addressable_sector_count;

  char devname[MAXDEVNAME]; /* "hdXdX\0" */
  device_t dev; /* the PREX device */
};

struct ata_controller {
  char devname[MAXDEVNAME]; /* "hdX\0" */
  struct pci_device *pci_dev;
  int isopen; /* FIXME: do we care? */
  struct irp irp;
  struct ata_disk *active_disk; /* disk using the irp right now */
  irq_t irq;
  struct ata_channel channel[2];
  struct ata_disk disk[4];
  uint8_t *buffer;
};

static void ata_write(struct ata_controller *c, int channelnum, int reg, uint8_t val) {
  bus_write_8(c->channel[channelnum].base_port + reg, val);
}

static uint8_t ata_read(struct ata_controller *c, int channelnum, int reg) {
  return bus_read_8(c->channel[channelnum].base_port + reg);
}

static void write_control(struct ata_controller *c, int channelnum, uint8_t val) {
  bus_write_8(c->channel[channelnum].control_port, val);
}

static uint8_t read_altstatus(struct ata_controller *c, int channelnum) {
  return bus_read_8(c->channel[channelnum].control_port);
}

/* A 400ns delay, used to wait for the device to start processing a
 * sent command and assert busy. */
static void ata_delay400(struct ata_controller *c, int channelnum) {
  read_altstatus(c, channelnum);
  read_altstatus(c, channelnum);
  read_altstatus(c, channelnum);
  read_altstatus(c, channelnum);
}

static void ata_wait(struct ata_controller *c, int channelnum) {
  unsigned int i;

  ata_delay400(c, channelnum);

  for (i = 0; i < 0x80000000; i++) {
    if (!(read_altstatus(c, channelnum) & ATA_STATUS_FLAG_BUSY)) {
      return;
    }
  }

  printf("ata_wait: busy never went away!!\n");
  /* TODO: reset device here, maybe? We'd have to retry or abort
     in-progress operations. */
}

static void ata_pio_read(struct ata_controller *c,
			 int channelnum,
			 uint8_t *buffer,
			 size_t count)
{
  ASSERT((count & 3) == 0); /* multiple of 4 bytes. */
  while (count > 0) {
    uint32_t v = bus_read_32(c->channel[channelnum].base_port + ATA_REG_DATA);
    buffer[0] = v & 0xff;
    buffer[1] = (v >> 8) & 0xff;
    buffer[2] = (v >> 16) & 0xff;
    buffer[3] = (v >> 24) & 0xff;
    buffer += 4;
    count -= 4;
  }
}

/* interrupt service routine. Lowest-level responder to an interrupt -
   try to avoid doing "real work" here */
static int hdc_isr(void *arg) {
  struct ata_controller *c = arg;
  struct ata_disk *disk = c->active_disk;
  uint8_t status = read_altstatus(c, disk->channel);
  if (status & (ATA_STATUS_FLAG_DRQ | ATA_STATUS_FLAG_DEVICE_FAILURE | ATA_STATUS_FLAG_ERROR)) {
    return INT_CONTINUE;
  } else {
    return 0;
  }
}

/* interrupt service thread. The main workhorse for communicating with
   the device. */
static void hdc_ist(void *arg) {
  struct ata_controller *c = arg;
  struct ata_disk *disk = c->active_disk;
  struct irp *irp = &c->irp;
  uint8_t status = ata_read(c, disk->channel, ATA_REG_COMMAND_STATUS);

  c->active_disk = NULL;

  if (status & (ATA_STATUS_FLAG_ERROR | ATA_STATUS_FLAG_DEVICE_FAILURE)) {
    irp->error = 0x80000000 | (status << 16) | ata_read(c, disk->channel, ATA_REG_ERR);
    sched_wakeup(&irp->iocomp);
    return;
  }

  irp->error = 0;
  switch (irp->cmd) {
    case IO_READ:
      ata_pio_read(c, disk->channel, irp->buf, irp->blksz * SECTOR_SIZE);
      break;
    case IO_WRITE:
      panic("hdd_ist IO_WRITE not implemented"); /* TODO */
      /* TODO: add flush-to-disk ioctl? */
      break;
    default:
      panic("hdd_ist invalid irp->cmd");
      break;
  }
  sched_wakeup(&irp->iocomp);
}

static void hdd_setup_io(struct ata_disk *disk,
			 int cmd,
			 uint64_t lba,
			 size_t sector_count)
{
  struct ata_controller *c = disk->controller;
  uint8_t final_cmd;

  c->active_disk = disk;

  switch (cmd) {
    case IO_READ:
      /* Send READ SECTORS EXT command. */
      ata_write(c, disk->channel, ATA_REG_DISK_SELECT, 0x40 | (disk->slave << 4));
      final_cmd = 0x24;
      break;
    case IO_WRITE:
      panic("hdd_setup_io IO_WRITE not implemented"); /* TODO */
      final_cmd = 0; /* TODO */
      break;
    default:
      panic("hdd_setup_io invalid cmd");
      return;
  }

  ata_write(c, disk->channel, ATA_REG_SECTOR_COUNT, (sector_count >> 8) & 0xff);
  ata_write(c, disk->channel, ATA_REG_LBA_LOW, (lba >> 24) & 0xff);
  ata_write(c, disk->channel, ATA_REG_LBA_MID, (lba >> 32) & 0xff);
  ata_write(c, disk->channel, ATA_REG_LBA_HIGH, (lba >> 40) & 0xff);
  ata_write(c, disk->channel, ATA_REG_SECTOR_COUNT, sector_count & 0xff);
  ata_write(c, disk->channel, ATA_REG_LBA_LOW, lba & 0xff);
  ata_write(c, disk->channel, ATA_REG_LBA_MID, (lba >> 8) & 0xff);
  ata_write(c, disk->channel, ATA_REG_LBA_HIGH, (lba >> 16) & 0xff);
  ata_write(c, disk->channel, ATA_REG_COMMAND_STATUS, final_cmd);

  /* We'll get an interrupt sometime, if interrupts aren't disabled;
     otherwise, we'll need to check the status register by polling. */
}

/* Useful only from within the kernel, while we're probing and setting
   everything up. */
static int read_during_setup(struct ata_disk *disk, uint64_t lba, uint8_t *buf, size_t count) {
  struct ata_controller *c = disk->controller;
  int status;
  hdd_setup_io(disk, IO_READ, lba, count);
  ata_wait(c, disk->channel);
  status = ata_read(c, disk->channel, ATA_REG_COMMAND_STATUS);
  if (status & (ATA_STATUS_FLAG_ERROR | ATA_STATUS_FLAG_DEVICE_FAILURE)) {
    printf("Couldn't read_during_setup %s (lba %d, count %d): 0x%02x, 0x%02x\n",
	   disk->devname,
	   lba, count,
	   status, ata_read(c, disk->channel, ATA_REG_ERR));
    return EIO;
  }

  ata_pio_read(c, disk->channel, buf, count * SECTOR_SIZE);
  return 0;
}

static void setup_partitions(struct driver *self, struct ata_disk *disk) {
  uint8_t *sector0 = kmem_alloc(SECTOR_SIZE);

  if (read_during_setup(disk, 0, sector0, 1)) {
    kmem_free(sector0);
    return;
  }

  if (0xaa55 == (* (uint16_t *) (&sector0[SECTOR_SIZE - 2]))) {
    int partition;
    /* Valid DOS disklabel? */
    for (partition = 0; partition < 4; partition++) {
      struct {
	uint8_t flags;
	uint8_t start_chs[3];
	uint8_t system_id;
	uint8_t end_chs[3];
	uint32_t start_lba;
	uint32_t sector_count;
      } *p = (void *) (&sector0[0x1be + (partition * 16)]);
      printf(" - partition %sp%c, type 0x%02x, 0x%08x size 0x%08x\n",
	     disk->devname,
	     '0' + partition,
	     p->system_id,
	     p->start_lba,
	     p->sector_count);
    }
  }

  kmem_free(sector0);
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

static void setup_disk(struct driver *self, struct ata_controller *c, int disknum) {
  struct ata_disk *disk = &c->disk[disknum];

  disk->valid = 0; /* to be determined properly below */
  disk->controller = c;
  disk->channel = disknum >> 1;
  disk->slave = disknum & 1;

  /* Send IDENTIFY command (0xEC). */

  ata_write(c, disk->channel, ATA_REG_DISK_SELECT, 0xA0 | (disk->slave << 4));
  ata_delay400(c, disk->channel);

  ata_write(c, disk->channel, ATA_REG_SECTOR_COUNT, 0);
  ata_write(c, disk->channel, ATA_REG_LBA_LOW, 0);
  ata_write(c, disk->channel, ATA_REG_LBA_MID, 0);
  ata_write(c, disk->channel, ATA_REG_LBA_HIGH, 0);

  ata_write(c, disk->channel, ATA_REG_COMMAND_STATUS, 0xEC);
  ata_delay400(c, disk->channel);

  if (ata_read(c, disk->channel, ATA_REG_COMMAND_STATUS) == 0) {
    printf("Disk %d absent (wouldn't accept command).\n", disknum);
    return;
  }

  ata_wait(c, disk->channel);
  if (read_altstatus(c, disk->channel) & ATA_STATUS_FLAG_ERROR) {
    printf("Disk %d absent (wouldn't identify).\n", disknum);
    return;
  }

  /* ATAPI devices return special values in LBA_MID and LBA_HIGH. We
     don't check those here. (TODO) */

  ata_pio_read(c, disk->channel, disk->identification_space, sizeof(disk->identification_space));

  memcpy(disk->serial_number, &disk->identification_space[20], sizeof(disk->serial_number));
  memcpy(disk->firmware_revision, &disk->identification_space[46], sizeof(disk->firmware_revision));
  memcpy(disk->model, &disk->identification_space[54], sizeof(disk->model));
  disk->lba_supported = ((disk->identification_space[100] & 2) != 0);
  disk->dma_supported = ((disk->identification_space[100] & 1) != 0);
  memcpy(&disk->sector_capacity, &disk->identification_space[114], sizeof(disk->sector_capacity));

  {
    uint32_t lba28_count;
    memcpy(&lba28_count, &disk->identification_space[120], sizeof(lba28_count));
    if (lba28_count == 0x0fffffff) {
      uint64_t lba48_count;
      /* More than 28 bits' worth of sectors - read the lba48 area */
      memcpy(&lba48_count, &disk->identification_space[200], sizeof(lba48_count));
      disk->addressable_sector_count = lba48_count;
    } else {
      disk->addressable_sector_count = lba28_count;
    }
  }

  fixup_string_endianness(disk->serial_number, sizeof(disk->serial_number));
  fixup_string_endianness(disk->firmware_revision, sizeof(disk->firmware_revision));
  fixup_string_endianness(disk->model, sizeof(disk->model));

  printf("Disk %d:\n", disknum);
  printf(" - serial %.*s\n", sizeof(disk->serial_number), disk->serial_number);
  printf(" - firmware %.*s\n", sizeof(disk->firmware_revision), disk->firmware_revision);
  printf(" - model %.*s\n", sizeof(disk->model), disk->model);
  printf(" - sector count %d (0x%08x%08x)\n",
	 (uint32_t) disk->addressable_sector_count,
	 (uint32_t) (disk->addressable_sector_count >> 32),
	 (uint32_t) disk->addressable_sector_count);

  disk->valid = 1;

  /* TODO: register the device for the disk. */
  memcpy(disk->devname, c->devname, 3);
  disk->devname[3] = 'd';
  disk->devname[4] = '0' + disknum;
  disk->devname[5] = '\0';
  disk->dev = device_create(self, disk->devname, D_BLK | D_PROT);
  *((struct ata_disk **) device_private(disk->dev)) = disk;

  setup_partitions(self, disk);
}

static void setup_controller(struct driver *self, struct pci_device *v) {
  static char which_device = '0';
  char devname_tmp[MAXDEVNAME];
  int primary_native;
  int secondary_native;

  struct ata_controller *c;
  struct irp *irp;

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
  }

  printf("device %d.%d.%d = %s\n", v->bus, v->slot, v->function, devname_tmp);

  c = kmem_alloc(sizeof(struct ata_controller));
  memcpy(&c->devname[0], &devname_tmp[0], sizeof(c->devname));
  c->pci_dev = v;
  c->isopen = 0;

  irp = &c->irp;
  irp->cmd = IO_NONE;
  event_init(&irp->iocomp, &c->devname[0]);

  /* TODO: if we're operating in compatibility/legacy mode, we are
     behaving like an old school IDE adapter, which wants to use IRQ14
     for the primary and IRQ15 for the secondary controller. We
     currently only take one IRQ, so secondary controllers won't
     work. */

  /* TODO: claiming an IRQ more than once causes, um, issues, so don't do that. Ever. */
  c->irq = irq_attach(HDC_IRQ, IPL_BLOCK, 0, hdc_isr, hdc_ist, c);

  if (primary_native || secondary_native) {
    /* Tell the controller which IRQ to use, if we're in native mode. */
    write_pci_interrupt_line(v, HDC_IRQ);
  }

  c->buffer = ptokv(page_alloc(BUFFER_LENGTH));

  /* TODO: It is unclear whether, in native mode, the BARs contain
     port numbers directly, or whether they should be masked with
     ~0x03. The low two bits might be used as flags?? */

  if (primary_native) {
    c->channel[0].base_port = read_pci_bar(v, 0);
    c->channel[0].control_port = read_pci_bar(v, 1) + 2;
  } else {
    c->channel[0].base_port = ATA_LEGACY_PRIMARY_CONTROL_BASE;
    c->channel[0].control_port =
      ATA_LEGACY_PRIMARY_CONTROL_BASE + ATA_LEGACY_CONTROL_ALTERNATE_STATUS_OFFSET;
  }

  if (secondary_native) {
    c->channel[1].base_port = read_pci_bar(v, 2);
    c->channel[1].control_port = read_pci_bar(v, 3) + 2;
  } else {
    c->channel[1].base_port = ATA_LEGACY_SECONDARY_CONTROL_BASE;
    c->channel[1].control_port =
      ATA_LEGACY_SECONDARY_CONTROL_BASE + ATA_LEGACY_CONTROL_ALTERNATE_STATUS_OFFSET;
  }

  /* BAR4 points to a 16-byte block of I/O port space, the low 8 bytes
     of which are for the primary and the high 8 bytes for the
     secondary controller. */
  c->channel[0].dma_port = read_pci_bar(v, 4);
  c->channel[1].dma_port = c->channel[0].dma_port + 8;

  printf(" - pri 0x%04x/0x%04x/0x%04x, sec 0x%04x/0x%04x/0x%04x\n",
	 c->channel[0].base_port, c->channel[0].control_port, c->channel[0].dma_port,
	 c->channel[1].base_port, c->channel[1].control_port, c->channel[1].dma_port);

  /* Disable interrupts from the two channels. */
  write_control(c, 0, 2);
  write_control(c, 1, 2);

  {
    int disk;
    for (disk = 0; disk < 4; disk++) {
      setup_disk(self, c, disk);
    }
  }

  /* Reenable interrupts from the two channels. */
  write_control(c, 0, 0);
  write_control(c, 1, 0);
}

static int hdd_init(struct driver *self) {
  int i;

  for (i = 0; i < pci_device_count; i++) {
    struct pci_device *v = &pci_devices[i];
    if (v->class_code == PCI_CLASS_STORAGE &&
	v->subclass == 1 /* IDE */)
    {
      setup_controller(self, v);
    }
  }

  return 0;
}

static struct ata_disk *get_disk(device_t dev) {
  return * (struct ata_disk **) device_private(dev);
}

static int hdd_open(device_t dev, int mode) {
  struct ata_disk *disk = get_disk(dev);

  if (disk->controller->isopen > 0) {
    return EBUSY;
  }
  /* Is this a race? fdd.c does the same thing. */
  disk->controller->isopen++;
  disk->controller->irp.cmd = IO_NONE;
  return 0;
}

static int hdd_close(device_t dev) {
  struct ata_disk *disk = get_disk(dev);

  if (disk->controller->isopen != 1) {
    return EINVAL;
  }
  /* Is this a race? fdd.c does the same thing. */
  disk->controller->isopen--;
  disk->controller->irp.cmd = IO_NONE;
  /* TODO: reset the controller perhaps? Or shut it down? The fdd
     driver switches off the drive motor here. */
  return 0;
}

static int hdd_rw(struct ata_disk *disk, struct irp *irp, int cmd,
		  uint8_t *buf, size_t block_count, int blkno)
{
  int err;

  irp->cmd = cmd;
  irp->ntries = 0;
  irp->error = 0;
  irp->blkno = blkno;
  irp->blksz = block_count;
  irp->buf = buf;

  sched_lock();

  hdd_setup_io(disk, irp->cmd, irp->blkno, irp->blksz); /* TODO: 64 bit irp->blkno? */

  if (sched_sleep(&irp->iocomp) == SLP_INTR) {
    err = EINTR;
  } else {
    err = irp->error;
  }
  sched_unlock();

  return err;
}

static int hdd_read(device_t dev, char *buf, size_t *nbyte, int blkno) {
  struct ata_disk *disk = get_disk(dev);
  uint8_t *kbuf;
  size_t sector_count = *nbyte / SECTOR_SIZE;
  size_t transferred_total = 0;

  if ((blkno < 0) || (blkno + sector_count >= disk->addressable_sector_count))
    return EIO;

  kbuf = kmem_map(buf, *nbyte);
  if (kbuf == NULL)
    return EFAULT;
  /* TODO: could it be possible that noncontiguous physical pages are
     backing this portion of virtual address space? The code here (and
     in fdd.c, which it is based on) assumes not, I think... */

  while (sector_count > 0) {
    size_t transfer_sector_count =
      (sector_count > BUFFER_LENGTH_IN_SECTORS) ? BUFFER_LENGTH_IN_SECTORS : sector_count;
    size_t transfer_byte_count = SECTOR_SIZE * transfer_sector_count;
    int err;

    err = hdd_rw(disk, &disk->controller->irp, IO_READ,
		 disk->controller->buffer, transfer_sector_count, blkno);
    if (err) {
      printf("hdd_read error: %d\n", err);
      *nbyte = transferred_total;
      return EIO;
    }

    memcpy(kbuf, disk->controller->buffer, transfer_byte_count);

    transferred_total += transfer_byte_count;
    kbuf += transfer_byte_count;
    blkno += transfer_sector_count;
    sector_count -= transfer_sector_count;
  }

  *nbyte = transferred_total;
  return 0;
}

static int hdd_write(device_t dev, char *buf, size_t *nbyte, int blkno) {
  struct ata_disk *disk = get_disk(dev);
  return EINVAL;
  /*
  uint8_t *kbuf;
  size_t sector_count = *nbyte / SECTOR_SIZE;
  size_t transferred_total = 0;

  if ((blkno < 0) || (blkno + sector_count >= disk->addressable_sector_count))
    return EIO;

  kbuf = kmem_map(buf, *nbyte);
  if (kbuf == NULL)
    return EFAULT;

  while (sector_count > 0) {
    size_t transfer_sector_count =
      (sector_count > BUFFER_LENGTH_IN_SECTORS) ? BUFFER_LENGTH_IN_SECTORS : sector_count;
    size_t transfer_byte_count = SECTOR_SIZE * transfer_sector_count;
    int err;

    memcpy(disk->controller->buffer, kbuf, transfer_byte_count);

    err = hdd_rw(disk, &disk->controller->irp, IO_WRITE,
		 disk->controller->buffer, transfer_sector_count, blkno);
    if (err) {
      printf("hdd_write error: %d\n", err);
      *nbyte = transferred_total;
      return EIO;
    }

    transferred_total += transfer_byte_count;
    kbuf += transfer_byte_count;
    blkno += transfer_sector_count;
    sector_count -= transfer_sector_count;
  }

  *nbyte = transferred_total;
  return 0;
  */
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
	/* devsz */	sizeof(struct ata_disk *),
	/* flags */	0,
	/* probe */	NULL,
	/* init */	hdd_init,
	/* shutdown */	NULL,
};
