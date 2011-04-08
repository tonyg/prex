#ifndef __FSTAB_H_
#define __FSTAB_H_

#include <sys/param.h>
#include <sys/mount.h>
#include <stdio.h>

#define MAX_FS_TYPE 3
#define MAX_FS_SPECIAL (MAXDEVNAME+5) /* add 5 for /dev/ */
#define MAXOPTBUF 1024 /* rw,auto,noexec etc.. */

#define FSTAB_RO "ro"
#define FSTAB_RW "rw"
#define FSTAB_RQ "rq" /* XXX: Quotas not in Prex */

typedef struct _fstab {
  char fs_type[MAX_FS_TYPE]; /* ro,rw,rq */
  char fs_vfstype[MAX_VFS_TYPE];
  char fs_mntopts[MAXOPTBUF];
  char fs_file[MAXPATHLEN];
  char fs_special[MAX_FS_SPECIAL];
} fstab, *pfstab;

typedef struct _fstab_traversal {
  FILE *fp;
  pfstab data;
} fstab_traversal, *pfstab_traversal;

int openfstab(pfstab_traversal trav);
void closefstab(pfstab_traversal trav);
int getfsent(pfstab_traversal);
int getfsfile(char *, pfstab);
int getfsspec(char *, pfstab);


#endif
