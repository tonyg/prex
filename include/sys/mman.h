#ifndef _SYS_MMAN_H_
#define _SYS_MMAN_H_

#define PROT_READ	0x1
#define PROT_WRITE	0x2
#define PROT_EXEC	0x4
#define PROT_NONE	0

#define MAP_SHARED	0x1
#define MAP_PRIVATE	0x2
#define MAP_FIXED	0x10
#define MAP_FILE	0

#define MAP_FAILED	((void *) -1)

/**
 * Limited implementation of mmap().
 * We only support prot==PROT_READ and flags==MAP_PRIVATE|MAP_FILE for now.
 */
extern void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);

#endif
