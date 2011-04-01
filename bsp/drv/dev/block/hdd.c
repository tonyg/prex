#include <driver.h>

#define DEBUG_HDD 1

#if DEBUG_HDD
#define DPRINTF(a)	printf a
#else
#define DPRINTF(a)
#endif

struct hdd_softc {
  int dummy;
};

static int hdd_open(device_t dev, int mode) {
  return EINVAL;
}

static int hdd_close(device_t dev) {
  return EINVAL;
}

static int hdd_read(device_t dev, char *buf, size_t *nbyte, int blnko) {
  return EINVAL;
}

static int hdd_write(device_t dev, char *buf, size_t *nbyte, int blkno) {
  return EINVAL;
}

static int hdd_probe(struct driver *self) {
  /* 0 means alive. */
  return 0;
}

static int hdd_init(struct driver *self) {
  DPRINTF(("HELLO WORLD\n"));
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
	/* devsz */	sizeof(struct hdd_softc),
	/* flags */	0,
	/* probe */	hdd_probe,
	/* init */	hdd_init,
	/* shutdown */	NULL,
};
