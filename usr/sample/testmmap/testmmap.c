#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#define ICHECK(x)	if ((x) == -1) { perror(#x); exit(1); }

static void segv_handler(int signo, siginfo_t *si, void *context) {
  printf("Eeek %d %p %d %p\n", signo, si->si_addr, si->si_code, context);
  exit(9);
}

#define MAPPED_FILE_PATH "/tmp/mappedfile"

static char hexchar(int n) {
  return ("0123456789abcdef")[n % 16];
}

static void setup_file(void) {
  int fd;
  int n;
  static char buffer[16384];

  for (n = 0; n < sizeof(buffer); n++) {
    buffer[n] = hexchar(n);
  }

  ICHECK(fd = open(MAPPED_FILE_PATH, O_CREAT | O_WRONLY));
  n = write(fd, buffer, sizeof(buffer));
  if (n != sizeof(buffer)) {
    perror("writing");
    exit(1);
  }
  ICHECK(close(fd));
}

static void dumpbyte(char *addr, int offset) {
  printf("addr[%d] (should be '%c') == '%c'\n", offset, hexchar(offset), addr[offset]);
  fflush(NULL);
}

static void try_mmap(void) {
  int fd;
  char *addr;

  ICHECK(fd = open(MAPPED_FILE_PATH, O_RDONLY));
  addr = mmap(NULL, 4096, PROT_READ, MAP_PRIVATE, fd, 0);
  if (addr == MAP_FAILED) {
    perror("mmap");
    exit(1);
  }
  printf("mmap returned %p\n", addr);
  dumpbyte(addr, 0);
  dumpbyte(addr, 15);
  dumpbyte(addr, 8000);
  dumpbyte(addr, 40000);
}

int main(int argc, char *argv[])
{
  struct sigaction sa;

  printf("Fault tester\n");

  sa.sa_sigaction = segv_handler;
  sa.sa_mask = 0;
  sa.sa_flags = SA_SIGINFO;
  sigaction(SIGSEGV, &sa, NULL);

  setup_file();
  try_mmap();

  printf("Causing fault...\n");
  printf("%d\n", *(int *) 0x34567800);
  return 0;
}
