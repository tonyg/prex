#ifndef _SYS_MMAN_H_
#define _SYS_MMAN_H_

/* PROT_READ, PROT_WRITE, and PROT_EXEC are the same as in prex.h, used with vm_attribute(). */
#include <sys/prex.h>
#define PROT_NONE	0

#define MAP_SHARED	0x1
#define MAP_PRIVATE	0x2
#define MAP_FIXED	0x10
#define MAP_FILE	0

#define MAP_FAILED	((void *) -1)

/**
 * Limited implementation of mmap().
 * We only support prot==PROT_READ and flags==MAP_PRIVATE|MAP_FILE for
 * now.  We don't transmute a SIGSEGV into SIGBUS when the user
 * attempts to read past the end of the file. Basically, many of the
 * subtleties of mmap() remain unimplemented. */
extern void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);

#endif
