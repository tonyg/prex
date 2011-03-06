#include <stdio.h>
#include <signal.h>

static void segv_handler(int signo, siginfo_t *si, void *context) {
  printf("Eeek %d %p %d %p\n", signo, si->si_addr, si->si_code, context);
}

int main(int argc, char *argv[])
{
  struct sigaction sa;

  printf("Fault tester\n");

  sa.sa_sigaction = segv_handler;
  sa.sa_mask = 0;
  sa.sa_flags = SA_SIGINFO;
  sigaction(SIGSEGV, &sa, NULL);

  printf("Causing fault...\n");
  printf("%d\n", *(int *) 0x34567800);
  return 0;
}
