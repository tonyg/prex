/* Ultra-simple partitioning tool. Just enough to get things started. */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <sys/prex.h>
#include <sys/types.h>

struct partition {
  uint8_t flags;
  uint8_t start_chs[3];
  uint8_t system_id;
  uint8_t end_chs[3];
  uint32_t start_lba;
  uint32_t sector_count;
};

static void usage(void) {
  fprintf(stderr, "usage: wparttab { -r | -w } <devicename>\n");
  fprintf(stderr, "If -r is given, prints output suitable for feeding back to its input;\n");
  fprintf(stderr, "if -w is given, expects stdin to contain partition entries of the form:\n");
  fprintf(stderr, "<partnum>,<typecode>,<startlba>,<sectorcount>\n");
  exit(1);
}

static void feedback(unsigned char *sector) {
  int i;
  for (i = 0; i < 4; i++) {
    struct partition part;
    memcpy(&part, &sector[0x1be + (i * 16)], sizeof(part));
    if (part.start_lba && part.sector_count && part.system_id) {
      printf("%u,%u,%u,%u\n",
	     i,
	     part.system_id,
	     part.start_lba,
	     part.sector_count);
    }
  }
}

static int
valid_blkno (device_t dev, int blkno)
{
  static char buf[512];
  size_t size = sizeof(buf);
  int error = device_read(dev, buf, &size, blkno);
  return !error;
}

int main(int argc, char *argv[]) {
  char *devname;
  char mode = '\0';
  static unsigned char sector[512];

  if (argc < 3) {
    usage();
  }

  if (argv[1][0] == '-' && argv[1][2] == '\0') {
    mode = argv[1][1];
  } else {
    usage();
  }

  devname = argv[2];

  switch (mode) {
    default:
      usage();

    case 'r': {
      {
	device_t dev;
	size_t size;

	errno = device_open(devname, 0, &dev);
	if (errno) {
	  perror("open");
	  exit(1);
	}

	{
	  off_t high, low;
	  low = 0;
	  for (high = 1; valid_blkno(dev, high); high *= 2)
	    low = high;
	  while (low < high - 1) {
	    off_t mid = (low + high) / 2;
	    if (valid_blkno(dev, mid))
	      low = mid;
	    else
	      high = mid;
	  }
	  ++low;
	  fprintf(stderr, "Total sector count = %u\n", low);
	}

	size = sizeof(sector);
	errno = device_read(dev, sector, &size, 0);
	if (errno) {
	  perror("read");
	  exit(1);
	}

	errno = device_close(dev);
	if (errno) {
	  perror("close");
	  exit(1);
	}
      }

      feedback(sector);

      break;
    }

    case 'w': {
      memset(sector, 0, sizeof(sector));
      sector[510] = 0x55;
      sector[511] = 0xaa;

      while (1) {
	char buf[1024];
	char *t;
	int partnum;
	struct partition part;

	if (fgets(buf, sizeof(buf), stdin) == NULL) {
	  break;
	}

	memset(&part, 0, sizeof(part));

	if ((t = strtok(buf, ",")) == NULL) continue;
	partnum = strtoul(t, NULL, 0);
	if ((t = strtok(NULL, ",")) == NULL) continue;
	part.system_id = strtoul(t, NULL, 0);
	if ((t = strtok(NULL, ",")) == NULL) continue;
	part.start_lba = strtoul(t, NULL, 0);
	if ((t = strtok(NULL, ",")) == NULL) continue;
	part.sector_count = strtoul(t, NULL, 0);

	printf("Partition %u will start at %u for %u sectors, type %u\n",
	       partnum, part.start_lba, part.sector_count, part.system_id);
	memcpy(&sector[0x1be + (partnum * 16)], &part, sizeof(part));
      }

      feedback(sector);

      {
	device_t dev;
	size_t size;

	errno = device_open(devname, 0, &dev);
	if (errno) {
	  perror("open");
	  exit(1);
	}

	size = sizeof(sector);
	errno = device_write(dev, sector, &size, 0);
	if (errno) {
	  perror("write");
	  exit(1);
	}

	errno = device_close(dev);
	if (errno) {
	  perror("close");
	  exit(1);
	}
      }
      break;
    }
  }

  return 0;
}
