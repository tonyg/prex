#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>

#define ICHECK(x)	if ((x) == -1) { perror(#x); exit(1); }

#define MAPPED_FILE_PATH "/tmp/bigmappedfile"
#define PAGE_LEN 4096
#define N_PAGES 256
#define MAPPED_FILE_LENGTH (PAGE_LEN * N_PAGES)

static void setup_file(void) {
  int fd;
  int n;
  static char buffer[MAPPED_FILE_LENGTH];

  memset(buffer, 0xff, MAPPED_FILE_LENGTH);
  ICHECK(fd = open(MAPPED_FILE_PATH, O_CREAT | O_WRONLY));
  n = write(fd, buffer, sizeof(buffer));
  if (n != sizeof(buffer)) {
    perror("writing");
    exit(1);
  }
  ICHECK(close(fd));
}

static void time_access(int nrepeats, unsigned char *addr, char const *description, int forwards) {
  int repeat;
  int page;
  unsigned long sum = 0;
  struct timeval start, stop;
  unsigned long total_us = 0;

  if (forwards) {
    gettimeofday(&start, NULL);
    for (repeat = 0; repeat < nrepeats; repeat++) {
      for (page = 0; page < N_PAGES; page++) {
	sum += addr[page * PAGE_LEN];
      }
    }
    gettimeofday(&stop, NULL);
  } else {
    gettimeofday(&start, NULL);
    for (repeat = 0; repeat < nrepeats; repeat++) {
      for (page = N_PAGES - 1; page >= 0; page--) {
	sum += addr[page * PAGE_LEN];
      }
    }
    gettimeofday(&stop, NULL);
  }

  total_us = (stop.tv_usec - start.tv_usec) + 1000000 * (stop.tv_sec - start.tv_sec);
  printf("%10s - %lu microseconds per touched page (%s)\n",
	 description,
	 total_us / (nrepeats * N_PAGES),
	 forwards ? "forwards" : "backwards");
  if (sum != nrepeats * N_PAGES * 0xff) {
    printf("Weird: sum is %lu\n", sum);
  }
}

static void try_mmap(int forwards) {
  int fd;
  unsigned char *addr;

  ICHECK(fd = open(MAPPED_FILE_PATH, O_RDONLY));
  addr = mmap(NULL, MAPPED_FILE_LENGTH, PROT_READ, MAP_PRIVATE, fd, 0);
  if (addr == MAP_FAILED) {
    perror("mmap");
    exit(1);
  }
  ICHECK(close(fd));
  printf("mmap returned %p\n", addr);
  time_access(1, addr, "unmapped", forwards);
  time_access(100, addr, "mapped", forwards);
}

int main(int argc, char *argv[])
{
  printf("mmap() timer\n");
  setup_file();
  try_mmap(1);
  try_mmap(0);
  return 0;
}
