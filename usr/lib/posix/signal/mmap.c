#include <sys/prex.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>

#define NMAPPINGS 16
#define PAGE_SIZE 4096 /* TODO: get this from somewhere authoritative */

typedef struct mapping_t_ {
  int fd;		/* open file descriptor to mapped file */
  off_t curroffset;	/* current offset within fd; used to avoid seeks */
  off_t baseoffset;	/* base offset within fd; must be page aligned */
  uint8_t *baseaddr;	/* base address of mapping; must be page aligned */
  uint8_t *limitaddr;	/* limit address of mapping; must be page aligned */
} mapping_t;

#ifdef _REENTRANT
static volatile mutex_t __mmap_lock;
#define MMAP_LOCK()	mutex_lock(&__mmap_lock)
#define MMAP_UNLOCK()	mutex_unlock(&__mmap_lock)
#else
#define MMAP_LOCK()
#define MMAP_UNLOCK()
#endif

/* Guarded by mmap_lock, when _REENTRANT: */
static mapping_t __mappings[NMAPPINGS];
static int __mapping_count = 0;
static uint8_t *__mapping_next = (uint8_t *) 0x60000000; /* TODO: better place?? */
/* end guarded. */

void
__mmap_init(void)
{
#ifdef _REENTRANT
  mutex_init(&__mmap_lock);
#endif
}

static void *
round_to_page(void *addr)
{
  uint32_t x = (uint32_t) addr;
  x = (x / PAGE_SIZE) * PAGE_SIZE;
  return (void *) x;
}

static void
__mmap_map_page(mapping_t *mapping, void *pagev) {
  uint8_t *page = (uint8_t *) pagev;
  off_t required_offset = page - mapping->baseaddr;

  /* printf("Mapping %p from fd %d at offset %x\n", page, mapping->fd, required_offset); */

  if (required_offset != mapping->curroffset) {
    mapping->curroffset = lseek(mapping->fd, required_offset, SEEK_SET);
    if (mapping->curroffset != required_offset) {
      perror("__mmap_map_page lseek");
      exit(-1);
    }
  }

  {
    void *alloc_addr = page;
    errno = vm_allocate(task_self(), &alloc_addr, PAGE_SIZE, 0);
    if (errno != 0) {
      perror("__mmap_map_page vm_allocate");
      exit(-1);
    }
    if (alloc_addr != page) {
      fprintf(stderr, "__mmap_map_page: vm_allocate returned %p, we needed %p\n",
	      alloc_addr,
	      page);
      exit(-1);
    }
  }

  {
    int n = read(mapping->fd, page, PAGE_SIZE);
    if (n < 0) {
      perror("__mmap_map_page read");
      exit(-1);
    }
    if (n != PAGE_SIZE) {
      fprintf(stderr, "__mmap_map_page: read %d instead of %d bytes\n",
	      n,
	      PAGE_SIZE);
      exit(-1);
    }
  }

  /* We only implement PROT_READ for now. */
  errno = vm_attribute(task_self(), page, PROT_READ);
  if (errno != 0) {
    perror("__mmap_map_page vm_attribute");
    exit(-1);
  }
}

int
__mmap_handle_segv(void *faultaddr, uint32_t faultflags)
{
  int i = 0;
  mapping_t *mapping = NULL;

  if ((faultflags & PAGE_FAULT_ACCESS_VIOLATION) != 0) {
    /* An access violation -> page already mapped! Not our problem. */
    return 0;
  }

  MMAP_LOCK();
  for (i = 0; i < __mapping_count; i++) {
    mapping = &__mappings[i];
    if (((uint8_t *) faultaddr >= mapping->baseaddr) &&
	((uint8_t *) faultaddr < mapping->limitaddr))
      {
	/* printf("Fault at %p\n", faultaddr); */
	__mmap_map_page(mapping, round_to_page(faultaddr));
	return 1;
      }
  }
  MMAP_UNLOCK();

  return 0;
}

void *
mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset)
{
  int newfd = -1;
  mapping_t *mapping = NULL;
  uint8_t *baseaddr = NULL;

  if (prot != PROT_READ) {
    errno = EINVAL;
    return MAP_FAILED;
  }

  if (flags != (MAP_PRIVATE | MAP_FILE)) {
    errno = EINVAL;
    return MAP_FAILED;
  }

  if (length == 0 || (length % PAGE_SIZE)) {
    errno = EINVAL;
    return MAP_FAILED;
  }

  if (offset % PAGE_SIZE) {
    errno = EINVAL;
    return MAP_FAILED;
  }

  newfd = dup(fd);
  if (newfd == -1) {
    return MAP_FAILED;
  }

  /* Experimental seek to make sure it works on the fd given, and to
     ensure we're at a known offset. */
  if (lseek(newfd, offset, SEEK_SET) == -1) {
    close(newfd);
    return MAP_FAILED;
  }

  MMAP_LOCK();
  baseaddr = __mapping_next;
  __mapping_next += length;

  if (__mapping_count >= NMAPPINGS) {
    MMAP_UNLOCK();
    errno = ENOMEM;
    return MAP_FAILED;
  }
  mapping = &__mappings[__mapping_count];
  __mapping_count++;

  mapping->fd = newfd;
  mapping->curroffset = offset;
  mapping->baseoffset = offset;
  mapping->baseaddr = baseaddr;
  mapping->limitaddr = baseaddr + length;
  MMAP_UNLOCK();

  return baseaddr;
}
