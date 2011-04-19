#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>

#define LARGE_BUFFER	(1024*1024)
#define SMALL_BUFFER	1

static void usage(void) {
  fprintf(stderr, "usage: fsperf -n <filename> [ -w <bytecount> | -r | -t | -l ]\n");
  fprintf(stderr, "  -n <filename>   Specifies filename on which to write\n");
  fprintf(stderr, "  -w <bytecount>  Tests write throughput/latency (default; 1MB)\n");
  fprintf(stderr, "  -r              Tests read throughput/latency\n");
  fprintf(stderr, "  -t              Selects throughput mode (default)\n");
  fprintf(stderr, "  -l              Selects latency mode\n");
  exit(1);
}

static double now(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (tv.tv_usec / 1000000.0);
}

static void do_write(char *filename, int throughputmode, size_t bytecount) {
  size_t buflen = throughputmode ? LARGE_BUFFER : SMALL_BUFFER;
  char *buffer = malloc(buflen);
  size_t count_written = 0;
  size_t iocount = 0;
  int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC);
  double starttime, stoptime;

  if (fd == -1) {
    perror("open");
    exit(1);
  }

  memset(buffer, 0x5a, buflen);

  fflush(NULL);
  starttime = now();
  while (count_written < bytecount) {
    size_t remaining = bytecount - count_written;
    size_t xferlen = (remaining > buflen) ? buflen : remaining;
    int result;
    if (throughputmode) {
      fprintf(stderr, "count_written = %u...\n", count_written);
    }
    result = write(fd, buffer, xferlen);
    if (result == 0) {
      fprintf(stderr, "no data transferred\n");
      exit(1);
    }
    if (result == -1) {
      perror("write");
      exit(1);
    }
    count_written += result;
    iocount++;
  }
  stoptime = now();
  fprintf(stderr, "count_written = %u.\n", count_written);

  close(fd);

  if (throughputmode) {
    printf("Wrote %u bytes in %u milliseconds = %u bytes/second\n",
	   bytecount,
	   (unsigned) ((stoptime - starttime) * 1000),
	   (unsigned) (bytecount / (stoptime - starttime)));
  } else {
    printf("Wrote %u bytes in %u I/Os taking %u milliseconds = %u microseconds/IO\n",
	   bytecount,
	   iocount,
	   (unsigned) ((stoptime - starttime) * 1000),
	   (unsigned) ((stoptime - starttime) * 1000000 / iocount));
  }
}

static void do_read(char *filename, int throughputmode) {
  size_t buflen = throughputmode ? LARGE_BUFFER : SMALL_BUFFER;
  char *buffer = malloc(buflen);
  size_t count_read = 0;
  size_t iocount = 0;
  int fd = open(filename, O_RDONLY);
  double starttime, stoptime;

  if (fd == -1) {
    perror("open");
    exit(1);
  }

  fflush(NULL);
  starttime = now();
  while (1) {
    int result;
    if (throughputmode) {
      fprintf(stderr, "count_read = %u...\n", count_read);
    }
    result = read(fd, buffer, buflen);
    if (result == 0) {
      break;
    }
    if (result == -1) {
      perror("fread");
      exit(1);
    }
    count_read += result;
    iocount++;
  }
  stoptime = now();
  fprintf(stderr, "count_read = %u.\n", count_read);

  close(fd);

  if (throughputmode) {
    printf("Read %u bytes in %u milliseconds = %u bytes/second\n",
	   count_read,
	   (unsigned) ((stoptime - starttime) * 1000),
	   (unsigned) (count_read / (stoptime - starttime)));
  } else {
    printf("Read %u bytes in %u I/Os taking %u milliseconds = %u microseconds/IO\n",
	   count_read,
	   iocount,
	   (unsigned) ((stoptime - starttime) * 1000),
	   (unsigned) ((stoptime - starttime) * 1000000 / iocount));
  }
}

int main(int argc, char *argv[]) {
  int ch;
  char *filename = NULL;
  size_t bytecount = 1024*1024;
  int writemode = 1;
  int throughputmode = 1;

  while ((ch = getopt(argc, argv, "n:w:rtl")) != -1) {
    switch (ch) {
      case 'n': {
	filename = strdup(optarg);
	break;
      }
      case 'w': {
	bytecount = strtoul(optarg, NULL, 0);
	writemode = 1;
	break;
      }
      case 'r': {
	writemode = 0;
	break;
      }
      case 't': {
	throughputmode = 1;
	break;
      }
      case 'l': {
	throughputmode = 0;
	break;
      }
      default:
      case '?':
	usage();
    }
  }

  if (filename == NULL) {
    usage();
  }

  printf("FSperf\n");
  printf("filename %s\n", filename);
  printf("%s mode measuring %s\n",
	 writemode ? "write" : "read",
	 throughputmode ? "throughput" : "latency");
  if (writemode) {
    printf("bytecount %u\n", (unsigned int) bytecount);
  }

  if (writemode) {
    do_write(filename, throughputmode, bytecount);
  } else {
    do_read(filename, throughputmode);
  }

  return 0;
}
