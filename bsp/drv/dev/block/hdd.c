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

#define HDC_PRIMARY_IRQ		14	/* Yeah, there are more than 16 these days.
					   Does prex support anything more than 16? */
#define HDC_SECONDARY_IRQ	15
/* In any case, we should be spreading the load around, and not
 * sharing one IRQ for all the IDE controllers in the system. For now,
 * sharing the IRQ is fine.
 *
 * FIXME: OK, sharing *would* be fine, but see the comment in the
 * implementation of irq_attach that says that sharing isn't supported
 * by prex. */

#define SECTOR_SIZE	512

/* These are the offsets to various IDE/ATA registers, relative to
 * ata_channel.base_port in I/O port space. */
typedef enum ata_port_register_t_ {
  ATA_REG_DATA = 0,
  ATA_REG_ERR = 1, /* osdev.org claims this is mostly for ATAPI? */
  ATA_REG_SECTOR_COUNT = 2,

  /* CHS addressing, which this driver doesn't use or support */
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

/* Some controllers operate in "PCI native" mode, where they specify
   the I/O ports they want us to use. Others operate in
   "compatibility" (a.k.a. legacy) mode, where we just have to know
   ahead of time that the following two ports are used to communicate
   with the controller. */
#define ATA_LEGACY_PRIMARY_CONTROL_BASE		0x1f0
#define ATA_LEGACY_SECONDARY_CONTROL_BASE	0x170
/* In "legacy" mode, the control block base port number + the
   following offset is the port number of the control/altstatus
   register. In PCI native IDE mode, we look at the BARs instead: BAR1
   points to a 4-byte space, within which offset 2 is the
   control/altstatus register. */
#define ATA_LEGACY_CONTROL_ALTERNATE_STATUS_OFFSET 0x206

/* These flags appear in the contents of ATA_REG_COMMAND_STATUS. These
   aren't the only flags that exist, there are others (see the various
   sources of information linked above), but these are the only ones
   this driver cares about for now. */
typedef enum ata_status_flag_t_ {
  ATA_STATUS_FLAG_ERROR = 0x01,
  ATA_STATUS_FLAG_DRQ = 0x08,
  ATA_STATUS_FLAG_DRIVE_SEEK_COMPLETED = 0x10, /* also "needs service" bit?? */
  ATA_STATUS_FLAG_DEVICE_FAILURE = 0x20, /* also DMA-ready bit?? */
  ATA_STATUS_FLAG_DEVICE_READY = 0x40, /* set when spun up and no error? */
  ATA_STATUS_FLAG_BUSY = 0x80
} ata_status_flag_t;

/* These flags appear in the contents of ATA_REG_ERR. */
typedef enum ata_error_flag_t_ {
  ATA_ERROR_FLAG_ILLEGAL_LENGTH = 0x01,
  ATA_ERROR_FLAG_NO_MEDIA = 0x02,
  ATA_ERROR_FLAG_ABORT = 0x04,
  ATA_ERROR_FLAG_MEDIA_CHANGE_REQUEST = 0x08,
  ATA_ERROR_FLAG_ID_NOT_FOUND = 0x10,
  ATA_ERROR_FLAG_MEDIA_CHANGED = 0x20,
  ATA_ERROR_FLAG_UNCORRECTABLE_DATA = 0x40,
  ATA_ERROR_FLAG_BAD_BLOCK = 0x80 /* or, according to FreeBSD, UDMA CRC error */
} ata_error_flag_t;

#define DEBUG_HDD 1

#if DEBUG_HDD
#define DPRINTF(a)	printf a
#else
#define DPRINTF(a)
#endif

/* At present, we limit individual transfers to a maximum of
   BUFFER_LENGTH bytes. TODO: once we switch to DMA, this will
   probably become obsolete. */
#define BUFFER_LENGTH 65536
#define BUFFER_LENGTH_IN_SECTORS (BUFFER_LENGTH / SECTOR_SIZE)

/**
 * Represents the kernel's handle on some device object exposed
 * through this driver. */
struct ata_device_handle {
  enum {
    ATA_DEVICE_WHOLEDISK,
    ATA_DEVICE_PARTITION
  } kind;
  union {
    struct ata_disk *wholedisk;
    struct ata_partition *partition;
  } pointer;
};

/**
 * Represents a single partition on an ata_disk. */
struct ata_partition {
  struct list link; /* link to other partitions within this disk */
  struct ata_disk *disk; /* the disk this partition is part of */

  uint8_t system_id; /* the partition type, from the partition table */
  uint32_t start_lba; /* base block address of partition on the disk */
  uint32_t sector_count; /* total number of SECTORS within the partition */

  /* The name of this device as it is known to the kernel. The file
     system's "/dev" node for this device is named using this. */
  char devname[MAXDEVNAME]; /* "hdXdXpXX\0" */
  device_t dev; /* the PREX kernel's device handle for this device */
};

/**
 * Represents a detected ATA disk/device attached to a channel of a
 * controller. */
struct ata_disk {
  struct list link; /* link to other disks on this controller */
  struct ata_controller *controller; /* the controller for this device */

  int channel; /* 0 => primary, 1 => secondary */
  int slave; /* 0 => master, 1 => slave */

  /* Copy of the ATA identification space buffer sent back in response
     to the ATA IDENTIFY command for this device. Some of the fields
     in this buffer are extracted into fields below. */
  uint8_t identification_space[512];

  /* These fields are extracted from identification_space: */
  uint8_t serial_number[10];
  uint8_t firmware_revision[8];
  uint8_t model[40];
  int lba_supported;
  int dma_supported;
  uint32_t sector_capacity;
  uint64_t addressable_sector_count;
  int logical_sectors_per_physical_sector;
  int bytes_per_logical_sector;

  /* The name of this device as it is known to the kernel. The file
     system's "/dev" node for this device is named using this. */
  char devname[MAXDEVNAME]; /* "hdXdX\0" */
  device_t dev; /* the PREX kernel's device handle for this device */

  struct list partition_list; /* all detected partitions on the disk */
};

/**
 * Represents a single channel within an IDE controller. IDE
 * controllers have *two* channels: primary and secondary. On each
 * channel, there can be up to two disks/devices. Each channel is
 * accessed via a different region of I/O port space. */
struct ata_channel {
  int base_port;
  int control_port;
  int dma_port;
};

/**
 * A single I/O request in progress. */
struct hdd_request {
  enum {
    REQ_INVALID = 0,
    REQ_NOT_STARTED,
    REQ_WAITING_FOR_DEVICE,
    REQ_COMPLETE /* TODO: do we really need this? */
  } state;
  struct ata_disk *disk;
  struct irp irp;
  struct queue link; /* link in chain of outstanding I/O requests */
};

/**
 * Represents a single IDE controller. */
struct ata_controller {
  char devname[MAXDEVNAME]; /* "hdX\0"; used for debugging etc. */
  struct pci_device *pci_dev; /* the PCI config for this device */
  struct queue request_queue; /* queue of hdd_requests. lock (splhigh) before using this */
  int disk_active; /* whether any request is outstanding. lock (splhigh) before using this */
  irq_t irq; /* we registered an IRQ with the kernel; this is the handle we were given */
  irq_t irq_secondary; /* we may have registered an IRQ for the secondary channel too */
  timer_t tmr; /* timeout timer id */
  struct ata_channel channel[2]; /* the two channels within the controller */
  struct list disk_list; /* all disks attached to this controller */
};

#if 0 /* we don't need this yet */
/* Append two strings, making sure not to read or write outside each
   string's allocated area. (I wish C had a unit test framework.) */
static char *strcat_limited(char *dest, size_t dest_max, const char *src, size_t src_max) {
  size_t dest_len = strnlen(dest, dest_max);
  size_t remaining_space = dest_max - dest_len;
  size_t i;
  for (i = 0 ; (i < remaining_space) && (i < src_max) && (src[i] != '\0') ; i++) {
    dest[dest_len + i] = src[i];
  }
  if (i < remaining_space) {
    dest[dest_len + i] = '\0';
  }
  return dest;
}
#endif

static struct ata_device_handle *get_handle(device_t dev) {
  return (struct ata_device_handle *) device_private(dev);
}

/* Writes to an ATA control register. */
static void ata_write(struct ata_controller *c, int channelnum, int reg, uint8_t val) {
  bus_write_8(c->channel[channelnum].base_port + reg, val);
}

/* Reads from an ATA control register. */
static uint8_t ata_read(struct ata_controller *c, int channelnum, int reg) {
  return bus_read_8(c->channel[channelnum].base_port + reg);
}

/* Writes to the special control/altstatus register. */
static void write_control(struct ata_controller *c, int channelnum, uint8_t val) {
  bus_write_8(c->channel[channelnum].control_port, val);
}

/* Reads from the special control/altstatus register. */
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

/* Poll until the BUSY flag clears. */
static void ata_wait(struct ata_controller *c, int channelnum) {
  unsigned int i;

  /* TODO: find out whether one I/O port read is roughly one microsecond. */
  for (i = 0; i < 1000000 /* 1 second? */ ; i++) {
    if (!(read_altstatus(c, channelnum) & ATA_STATUS_FLAG_BUSY)) {
      return;
    }
  }

  printf("ata_wait: busy never went away!!\n");
  /* TODO: reset device here, maybe? We'd have to retry or abort
     in-progress operations. */
}

/* Programmed I/O (PIO) read a buffer's worth of data from the controller. */
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

/* Programmed I/O (PIO) write a buffer's worth of data to the controller. */
static void ata_pio_write(struct ata_controller *c,
			  int channelnum,
			  uint8_t *buffer,
			  size_t count)
{
  ASSERT((count & 3) == 0); /* multiple of 4 bytes. */
  while (count > 0) {
    uint32_t v =
      buffer[0]
      | (buffer[1] << 8)
      | (buffer[2] << 16)
      | (buffer[3] << 24);
    bus_write_32(c->channel[channelnum].base_port + ATA_REG_DATA, v);
    buffer += 4;
    count -= 4;
  }
}

static struct hdd_request *first_pending_request(struct ata_controller *c) {
  queue_t q = queue_first(&c->request_queue);
  return queue_entry(q, struct hdd_request, link);
}

/* Sends an I/O command to the disk, including the address of the
   block concerned, using LBA48 mode. Usable for setting up either
   interrupt-based or polling-based transfers. */
static void hdd_setup_io(struct ata_disk *disk,
			 int cmd,
			 uint64_t lba,
			 size_t sector_count)
{
  struct ata_controller *c = disk->controller;
  uint8_t final_cmd;

  switch (cmd) {
    case IO_READ:
      /* Send READ SECTORS EXT command. */
      ata_write(c, disk->channel, ATA_REG_DISK_SELECT, 0x40 | (disk->slave << 4));
      final_cmd = 0x24;
      break;
    case IO_WRITE:
      /* Send WRITE SECTORS EXT command. */
      ata_write(c, disk->channel, ATA_REG_DISK_SELECT, 0x40 | (disk->slave << 4));
      final_cmd = 0x34;
      break;
    default:
      panic("hdd_setup_io invalid cmd");
      return;
  }

  /* DPRINTF(("%08x Send cmd %d lba %d\n", timer_ticks(), cmd, (int) lba)); */

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

  /* PIO writes should happen here. The IRQ will signal that the drive
     has accepted a sector's worth, not that it wants a sector's
     worth.

     According to http://wiki.osdev.org/ATA_PIO_Mode#28_bit_PIO, after
     transferring a sector's worth of data, one should "loop back to
     waiting for the next IRQ (or poll again -- see next note) for
     each successive sector." */
}

static void hdc_ist(void *arg); /* prototype, because referenced in timeout_handler. */

static void timeout_handler(void *arg) {
  DPRINTF(("%08x hdd timeout!\n", timer_ticks()));
  hdc_ist(arg);
}

/* CALL ONLY WITH splhigh! */
static void maybe_send_next_request(struct ata_controller *c) {
  if (!c->disk_active && !queue_empty(&c->request_queue)) {
    struct hdd_request *req = first_pending_request(c);
    struct irp *irp = &req->irp;

    ASSERT(req->state == REQ_NOT_STARTED);

    hdd_setup_io(req->disk, irp->cmd, irp->blkno, irp->blksz); /* TODO: 64 bit irp->blkno? */
    if (irp->cmd == IO_WRITE) {
      ata_wait(c, req->disk->channel);
      ata_pio_write(c, req->disk->channel, irp->buf, SECTOR_SIZE);
      /* Adjust buf and blksz in the interrupt handler. */
    }
    req->state = REQ_WAITING_FOR_DEVICE;
    c->disk_active = 1;

    /* We call the ist handler directly on timeout (!) */
    timer_callout(&c->tmr, 1000, timeout_handler, c);
  }
}

/* CALL ONLY WITH splhigh! */
static void complete_request(struct hdd_request *req) {
  sched_wakeup(&req->irp.iocomp);
  queue_remove(&req->link);

  req->disk->controller->disk_active = 0; /* because we've just finished a request. */
  /* Omitting the previous line would cause maybe_send_next_request to
     happily ignore the remaining elements in the queue. */
  maybe_send_next_request(req->disk->controller);

  req->state = REQ_COMPLETE; /* won't last long, we're about to free it */
  kmem_free(req);
}

/* interrupt service routine. Lowest-level responder to an interrupt -
   try to avoid doing "real work" here */
static int hdc_isr(void *arg) {
  /* There's nothing useful we can do in this context. Delay it all
     for the hdc_ist. We won't miss any interrupts, because the
     kernel's irq.c maintains istreq, a counter of outstanding
     interrupts, for each IRQ. */
  struct ata_controller *c = arg;
  timer_stop(&c->tmr);
  return INT_CONTINUE;
}

static int irp_error(struct ata_controller *c, int channel, uint8_t status) {
  return 0x80000000 | (status << 16) | ata_read(c, channel, ATA_REG_ERR);
}

/* Wait a while for DRQ to become set. If we see ERROR, give up. Read
   altstatus to avoid clearing any pending interrupt. Returns 0 for
   success, nonzero error code otherwise. */
static int wait_for_drq(struct ata_controller *c, int channel) {
  uint8_t status = 0;
  int i;

  for (i = 0; i < 50; i++) {
    status = read_altstatus(c, channel);
    if (status & (ATA_STATUS_FLAG_ERROR | ATA_STATUS_FLAG_DEVICE_FAILURE)) {
      return irp_error(c, channel, status);
    }
    if (status & ATA_STATUS_FLAG_DRQ) {
      return 0;
    }
  }

  /* DRQ never got set. */
  return 0xC0000000 | (status << 16);
}

/* interrupt service thread. The main workhorse for communicating with
   the device. */
static void hdc_ist(void *arg) {
  struct ata_controller *c = arg;
  int s = splhigh();

  /* Don't return from this function without calling splx(s). */

  if (!c->disk_active) {
    /* Nothing's happening, in theory! Read the real status registers
       to permit subsequent interrupts to fire. */
#if DEBUG_HDD
    uint8_t status0 = ata_read(c, 0, ATA_REG_COMMAND_STATUS);
    uint8_t status1 = ata_read(c, 1, ATA_REG_COMMAND_STATUS);
    if ((status0 & (ATA_STATUS_FLAG_DRQ | ATA_STATUS_FLAG_ERROR)) ||
	(status1 & (ATA_STATUS_FLAG_DRQ | ATA_STATUS_FLAG_ERROR)))
      {
	DPRINTF(("%08x Spurious disk interrupt (0x%02x, 0x%02x)\n",
		 timer_ticks(), status0, status1));
      }
#else
    ata_read(c, 0, ATA_REG_COMMAND_STATUS);
    ata_read(c, 1, ATA_REG_COMMAND_STATUS);
#endif
  } else if (queue_empty(&c->request_queue)) {
    panic("Active without a request in the queue!");
  } else {
    /* Here, we know we're supposed to be running, and we also know
       what we're supposed to be doing. */
    struct hdd_request *req = first_pending_request(c);
    struct ata_disk *disk = req->disk;
    struct irp *irp = &req->irp;
    uint8_t status;

    ASSERT(req->state == REQ_WAITING_FOR_DEVICE);

    /* DPRINTF(("%08x ist cmd %d\n", timer_ticks(), irp->cmd)); */

    /* Wait for BUSY to clear, if it's set. */
    ata_wait(c, disk->channel);

    /* Now, properly read-and-clear the status flags. */
    status = ata_read(c, disk->channel, ATA_REG_COMMAND_STATUS);
    /* DPRINTF(("Initial status %02x\n", status)); */

    /* If successful, do the transfer. Otherwise complete the request
       without doing anything more. */
    if (status & (ATA_STATUS_FLAG_ERROR | ATA_STATUS_FLAG_DEVICE_FAILURE)) {
      irp->error = irp_error(c, disk->channel, status);
      complete_request(req);
    } else {
      switch (irp->cmd) {
	case IO_READ:
	  /* Wait for DRQ, and check for errors. */
	  irp->error = wait_for_drq(c, disk->channel);
	  if (irp->error) {
	    complete_request(req);
	  } else {
	    ata_pio_read(c, disk->channel, irp->buf, SECTOR_SIZE);
	    irp->buf = ((char *) irp->buf) + SECTOR_SIZE;
	    irp->blksz--;
	  }
	  break;

	case IO_WRITE:
	  irp->buf = ((char *) irp->buf) + SECTOR_SIZE;
	  irp->blksz--;
	  if (irp->blksz > 0) {
	    irp->error = wait_for_drq(c, disk->channel);
	    if (irp->error) {
	      complete_request(req);
	    } else {
	      ata_pio_write(c, disk->channel, irp->buf, SECTOR_SIZE);
	    }
	  }
	  /* TODO: add flush-to-disk ioctl? */
	  break;

	default:
	  panic("hdd_ist invalid irp->cmd");
      }

      /* Multiple sector transfers are supposed to send an interrupt
	 FOR EACH SECTOR, so we should keep the entry in the queue
	 until it's completely finished with. */
      if (irp->blksz == 0) {
	complete_request(req);
      }
    }
  }

  splx(s);
}

/* Useful only from within the kernel, while we're probing and setting
   everything up. */
static int read_during_setup(struct ata_disk *disk, uint64_t lba, uint8_t *buf, size_t count) {
  struct ata_controller *c = disk->controller;
  int status;
  hdd_setup_io(disk, IO_READ, lba, count);
  ata_delay400(c, disk->channel);
  ata_wait(c, disk->channel);
  status = ata_read(c, disk->channel, ATA_REG_COMMAND_STATUS);
  if (status & (ATA_STATUS_FLAG_ERROR | ATA_STATUS_FLAG_DEVICE_FAILURE)) {
    printf("Couldn't read_during_setup %s (lba %u, count %u): 0x%02x, 0x%02x\n",
	   disk->devname,
	   (unsigned int) lba, count,
	   status, ata_read(c, disk->channel, ATA_REG_ERR));
    return EIO;
  }

  ata_pio_read(c, disk->channel, buf, count * SECTOR_SIZE);
  return 0;
}

/* Read a disk's partition table. */
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
      struct ata_partition *part = NULL;

      if ((p->start_lba == 0) || (p->sector_count == 0) || (p->system_id == 0)) {
	/* No allocated partition in this slot. */
	continue;
      }

      part = kmem_alloc(sizeof(struct ata_partition));

      list_insert(list_last(&disk->partition_list), &part->link);
      part->disk = disk;
      part->system_id = p->system_id;
      part->start_lba = p->start_lba;
      part->sector_count = p->sector_count;
      /* TODO: sanity-check sector_count, to make sure it doesn't
	 reach past the addressable_sector_count known to the whole
	 disk. */

      strlcpy(part->devname, disk->devname, MAXDEVNAME);
      {
	char *p = part->devname + strnlen(part->devname, MAXDEVNAME);
	p[0] = 'p';
	p[1] = '0' + (partition / 10);
	p[2] = '0' + (partition % 10);
	p[3] = '\0';
      }
      part->dev = device_create(self, part->devname, D_BLK | D_PROT);
      get_handle(part->dev)->kind = ATA_DEVICE_PARTITION;
      get_handle(part->dev)->pointer.partition = part;

      printf(" - partition %s, type 0x%02x, 0x%08x size 0x%08x\n",
	     part->devname,
	     part->system_id,
	     part->start_lba,
	     part->sector_count);
    }
  }

  /* TODO: loop back around and add any partitions found in the table
     in an extended partition. */

  kmem_free(sector0);
}

/* Byteswap a "string" of 16-bit words. See comments near callers. */
static void fixup_string_endianness(uint8_t *p, size_t size) {
  while (size > 0) {
    uint8_t tmp = p[1];
    p[1] = p[0];
    p[0] = tmp;
    p += 2;
    size -= 2;
  }
}

static int setup_disk(struct driver *self, struct ata_controller *c, int disknum) {
  struct ata_disk *disk = kmem_alloc(sizeof(struct ata_disk));

  /* disk->link will be initialised once we insert into our controller's disk_list. */
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
    goto cancel_setup;
  }

  ata_delay400(c, disk->channel);
  ata_wait(c, disk->channel);
  if (read_altstatus(c, disk->channel) & ATA_STATUS_FLAG_ERROR) {
    printf("Disk %d absent (wouldn't identify).\n", disknum);
    goto cancel_setup;
  }

  /* ATAPI devices return special values in LBA_MID and LBA_HIGH. We
     don't check those here. (TODO) */

  ata_pio_read(c, disk->channel, disk->identification_space, sizeof(disk->identification_space));

  memcpy(disk->serial_number, &disk->identification_space[20], sizeof(disk->serial_number));
  memcpy(disk->firmware_revision, &disk->identification_space[46], sizeof(disk->firmware_revision));
  memcpy(disk->model, &disk->identification_space[54], sizeof(disk->model));
  disk->lba_supported = ((disk->identification_space[99] & 2) != 0);
  disk->dma_supported = ((disk->identification_space[99] & 1) != 0);
  memcpy(&disk->sector_capacity, &disk->identification_space[114], sizeof(disk->sector_capacity));
  {
    uint16_t w;
    memcpy(&w, &disk->identification_space[212], sizeof(w));
    if (w & 0x2000) {
      disk->logical_sectors_per_physical_sector = 1 << (w & 0x000f);
    } else {
      disk->logical_sectors_per_physical_sector = 1;
    }
    if (w & 0x1000) {
      uint16_t w2;
      memcpy(&w2, &disk->identification_space[234], sizeof(w2));
      disk->bytes_per_logical_sector = w2 * 2; /* the value in the block is in 16-bit words */
    } else {
      disk->bytes_per_logical_sector = 512;
    }
  }

  if (!disk->lba_supported) {
    printf("Disk %d doesn't support LBA.\n", disknum);
    goto cancel_setup;
  }

  if (!disk->dma_supported) {
    printf("Disk %d doesn't support DMA.\n", disknum);
    goto cancel_setup;
  }

  /* Decide how many sectors this physical disk supports. If the
     lba28_count is the maximum possible, the convention is that the
     lba48_count is valid and should be used. */
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

  /* Weirdly, the ASCII strings in the identification_space are
     byte-swapped, because it was originally defined as a region of
     16-bit words (!) */
  fixup_string_endianness(disk->serial_number, sizeof(disk->serial_number));
  fixup_string_endianness(disk->firmware_revision, sizeof(disk->firmware_revision));
  fixup_string_endianness(disk->model, sizeof(disk->model));

  /* At this point, the disk has identified itself, and it looks
     more-or-less like the kind of thing we might be able to use. Add
     it to the list in our controller. */
  list_insert(list_last(&c->disk_list), &disk->link);

  memcpy(disk->devname, c->devname, 3);
  disk->devname[3] = 'd';
  disk->devname[4] = '0' + disknum;
  disk->devname[5] = '\0';
  disk->dev = device_create(self, disk->devname, D_BLK | D_PROT);
  get_handle(disk->dev)->kind = ATA_DEVICE_WHOLEDISK;
  get_handle(disk->dev)->pointer.wholedisk = disk;

  list_init(&disk->partition_list);

  printf("Disk %d/%s:\n", disknum, disk->devname);
  printf(" - serial %.*s\n", sizeof(disk->serial_number), disk->serial_number);
  printf(" - firmware %.*s\n", sizeof(disk->firmware_revision), disk->firmware_revision);
  printf(" - model %.*s\n", sizeof(disk->model), disk->model);
  printf(" - sector count %d (0x%08x%08x)\n",
	 (uint32_t) disk->addressable_sector_count,
	 (uint32_t) (disk->addressable_sector_count >> 32),
	 (uint32_t) disk->addressable_sector_count);
  printf(" - %d log/phys, %d bytes/logical sector\n",
	 disk->logical_sectors_per_physical_sector,
	 disk->bytes_per_logical_sector);

  setup_partitions(self, disk);
  return 0;

 cancel_setup:
  kmem_free(disk);
  return -1;
}

static void setup_controller(struct driver *self, struct pci_device *v) {
  static char which_device = '0';
  char devname_tmp[MAXDEVNAME];
  int primary_native;
  int secondary_native;

  struct ata_controller *c;

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
  queue_init(&c->request_queue);
  c->disk_active = 0;

  /* TODO: if we're operating in compatibility/legacy mode, we are
     behaving like an old school IDE adapter, which wants to use IRQ14
     for the primary and IRQ15 for the secondary controller. We
     currently only take one IRQ, so secondary controllers won't
     work. */

  /* TODO: claiming an IRQ more than once causes, um, issues, so don't do that. Ever. */
  c->irq = irq_attach(HDC_PRIMARY_IRQ, IPL_BLOCK, 0, hdc_isr, hdc_ist, c);
  if (secondary_native) {
    c->irq_secondary = 0;
  } else {
    c->irq_secondary = irq_attach(HDC_SECONDARY_IRQ, IPL_BLOCK, 0, hdc_isr, hdc_ist, c);
  }

  if (primary_native || secondary_native) {
    /* Tell the controller which IRQ to use, if we're in native mode. */
    /* It's an arbitrary choice between primary and secondary. */
    write_pci_interrupt_line(v, HDC_PRIMARY_IRQ);
  }

  list_init(&c->disk_list); /* no disks yet; will scan in a moment */

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

static int hdd_open(device_t dev, int mode) {
  /* There's nothing needs doing here. The device tree is static after
     the probe, and we no longer use locking at the device level
     (locking a request queue, instead). This applies to close etc as
     well, at least until we get asynchronous requests in Prex. */
  return 0;
}

static int hdd_close(device_t dev) {
  /* See hdd_open's comment. */
  return 0;
}

static int hdd_rw(struct ata_disk *disk, int cmd, uint8_t *buf, size_t block_count, int blkno)
{
  struct hdd_request *req = kmem_alloc(sizeof(struct hdd_request));
  int err;

  req->state = REQ_NOT_STARTED;
  req->disk = disk;
  req->irp.cmd = cmd;
  req->irp.ntries = 0;
  req->irp.error = 0;
  req->irp.blkno = blkno;
  req->irp.blksz = block_count;
  req->irp.buf = buf;
  event_init(&req->irp.iocomp, "hdd_rw");

  sched_lock();

  {
    int s = splhigh();
    enqueue(&disk->controller->request_queue, &req->link);
    maybe_send_next_request(disk->controller);
    splx(s);
  }

  if (sched_sleep(&req->irp.iocomp) == SLP_INTR) {
    err = EINTR;
  } else {
    err = req->irp.error;
  }
  sched_unlock();

  return err;
}

static void adjust_blkno(device_t dev,
			 struct ata_disk **disk_p,
			 int *blkno_p,
			 size_t *limit_p)
{
  struct ata_device_handle *handle = get_handle(dev);
  switch (handle->kind) {
    case ATA_DEVICE_WHOLEDISK:
      *disk_p = handle->pointer.wholedisk;
      /* No adjustment to blkno required. */
      *limit_p = handle->pointer.wholedisk->addressable_sector_count;
      break;

    case ATA_DEVICE_PARTITION:
      *disk_p = handle->pointer.partition->disk;
      *blkno_p += handle->pointer.partition->start_lba;
      *limit_p = handle->pointer.partition->start_lba + handle->pointer.partition->sector_count;
      break;

    default:
      panic("Unknown ata_device_handle kind");
  }
}

static int hdd_read(device_t dev, char *buf, size_t *nbyte, int blkno) {
  struct ata_disk *disk = NULL;
  uint8_t *kbuf;
  size_t sector_count = *nbyte / SECTOR_SIZE;
  size_t transferred_total = 0;
  size_t sector_limit = 0; /* number of first invalid sector */

  /* DPRINTF(("R Pre adjustment: %08x count %d (%d bytes)\n", blkno, sector_count, *nbyte)); */
  adjust_blkno(dev, &disk, &blkno, &sector_limit);
  /* DPRINTF(("R Post adjustment: %08x limit %08x\n", blkno, sector_limit)); */
  if ((blkno < 0) || (blkno + sector_count > sector_limit))
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

    err = hdd_rw(disk, IO_READ, kbuf, transfer_sector_count, blkno);
    if (err) {
      printf("hdd_read error: 0x%08x\n", err);
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
}

static int hdd_write(device_t dev, char *buf, size_t *nbyte, int blkno) {
  struct ata_disk *disk = NULL;
  uint8_t *kbuf;
  size_t sector_count = *nbyte / SECTOR_SIZE;
  size_t transferred_total = 0;
  size_t sector_limit = 0; /* number of first invalid sector */

  /* DPRINTF(("W Pre adjustment: %08x count %d (%d bytes)\n", blkno, sector_count, *nbyte)); */
  adjust_blkno(dev, &disk, &blkno, &sector_limit);
  /* DPRINTF(("W Post adjustment: %08x limit %08x\n", blkno, sector_limit)); */
  if ((blkno < 0) || (blkno + sector_count > sector_limit))
    return EIO;

  kbuf = kmem_map(buf, *nbyte);
  if (kbuf == NULL)
    return EFAULT;
  /* TODO: same kmem_map caveat as for hdd_read */

  /* printf("total of %d sectors to write starting at %d\n", sector_count, blkno); */
  while (sector_count > 0) {
    size_t transfer_sector_count =
      (sector_count > BUFFER_LENGTH_IN_SECTORS) ? BUFFER_LENGTH_IN_SECTORS : sector_count;
    size_t transfer_byte_count = SECTOR_SIZE * transfer_sector_count;
    int err;

    /* printf("about to write %d sectors at blkno %d\n", transfer_sector_count, blkno); */
    err = hdd_rw(disk, IO_WRITE, kbuf, transfer_sector_count, blkno);
    if (err) {
      printf("hdd_write error: 0x%08x\n", err);
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
	/* devsz */	sizeof(struct ata_device_handle),
	/* flags */	0,
	/* probe */	NULL,
	/* init */	hdd_init,
	/* shutdown */	NULL,
};
