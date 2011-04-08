#include <sys/prex.h>
#include <sys/posix.h>
#include <ipc/fs.h>

#include <limits.h>
#include <string.h>
#include <errno.h>

/*
 * Task-specific interface to vfs_findroot. Doesn't implement the full
 * generality of the function: just enough to get mount(8) to work.
 *
 * path - input - the path to find the root for
 * m_path - output - a PATH_MAX-length buffer
 * m_dev - output - the device that was mounted
 * m_flags - output - mount flags
 *
 * Give NULL for any of m_path, m_dev or m_flags if you don't want
 * those particular outputs returned to you.
 *
 * Returns 0 on success, or -1 on failure and sets errno.
 */
int vfs_simple_findroot(char const *path, char *m_path, dev_t *m_dev, int *m_flags) {
  struct findroot_msg m;

  m.hdr.code = FS_FINDROOT;
  strlcpy(m.path, path, PATH_MAX);

  if (__posix_call(__fs_obj, &m, sizeof(m), 1) != 0) {
    return -1;
  }

  if (m_path != NULL) {
    strlcpy(m_path, m.path, PATH_MAX);
  }
  if (m_dev != NULL) {
    *m_dev = m.dev;
  }
  if (m_flags != NULL) {
    *m_flags = m.flags;
  }
  return 0;
}
