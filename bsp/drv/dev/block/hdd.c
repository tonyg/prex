#include <driver.h>
#include <pci.h>

#define HDC_IRQ		14	/* Yeah, there are more than 16 these days.
				   Does prex support anything more than 16? */
/* In any case, we should be spreading the load around, and not
 * sharing one IRQ for all the IDE controllers in the system. For now,
 * sharing the IRQ is fine.
 *
 * FIXME: OK, sharing *would* be fine, but see the comment in the
 * implementation of irq_attach that says that sharing isn't supported
 * by prex. */

#define DEBUG_HDD 1

#if DEBUG_HDD
#define DPRINTF(a)	printf a
#else
#define DPRINTF(a)
#endif

#define BUFFER_LENGTH 4096 /* FIXME: proper caching please */

struct hdd_softc {
  device_t dev;
  char devname[4]; /* TODO: 4 is a magic number */
  struct pci_device *pci_dev;
  int isopen; /* FIXME: do we care? */
  struct irp irp;
  irq_t irq;
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

static void setup_device(struct driver *self, struct pci_device *v) {
  static char which_device = '0';
  char devname_tmp[4];

  struct hdd_softc *sc;
  struct irp *irp;
  device_t dev;

  /* Try setting the IRQ line to something stupid: 0xfe. If it takes
     it, it's an IRQ-configurable device. */
  write_pci_interrupt_line(v, 0xfe);
  if (read_pci_interrupt_line(v) != 0xfe) {
    /* Nope. Legacy horror. Punt? */
    printf("Not trying to cope with legacy hardware at the moment\n");
    return;
  }

  /* At this point, we know our device likes to be told which IRQ to
     use. */

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
  {
    int i;
    for (i = 0; i < N_PCI_BASE_ADDRESS_REGISTERS; i++) {
      printf(" - bar %d = 0x%08X\n", i, read_pci_bar(v, i));
    }
  }

  sc = device_private(dev);
  sc->dev = dev;
  memcpy(&sc->devname[0], &devname_tmp[0], sizeof(devname_tmp));
  sc->pci_dev = v;
  sc->isopen = 0;

  irp = &sc->irp;
  irp->cmd = IO_NONE;
  event_init(&irp->iocomp, &sc->devname[0]);

  /* TODO: claiming an IRQ more than once causes, um, issues, so don't do that. Ever. */
  sc->irq = irq_attach(HDC_IRQ, IPL_BLOCK, 0, hdc_isr, hdc_ist, sc);

  /* Tell the controller which IRQ to use, for real this time. */
  write_pci_interrupt_line(v, HDC_IRQ);

  sc->buffer = page_alloc(BUFFER_LENGTH);
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
