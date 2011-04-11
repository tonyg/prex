#include <sys/param.h>
#include <sys/mount.h>
#include <sys/prex.h>

#include <fstab.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int      verbose;

static int      hasopt(const char *, const char *);
static void     usage(void);
static void     mountall(int, char *, const char **);
static void      mountupdate(int, const char *, char *, char **);
static void      mounttrycanonical(int, const char *, char *, char **);
static int mount_on_name(dev_t, char *);

int
main(int argc, char *argv[])
{
        const char **vfslist;

        int all, ch, init_flags, rval;
        static char options[MAXOPTBUF];
        static char canonical_path_buf[MAXPATHLEN];
        char *canonical_path;

        /* started as "mount" */
        all = init_flags = 0;
        vfslist = NULL;
        printf("Here\n");
        while ((ch = getopt(argc, argv, "a:rwt:uv")) != -1) {
          printf("WTF %d %c\n", argc, ch);
                switch (ch) {
                case 'a':
                        all = 1;
                        break;
                case 'r':
                        init_flags |= MNT_RDONLY;
                        break;
                case 'u':
                        init_flags |= MNT_UPDATE;
                        break;
                case 'v':
                        verbose++;
                        break;
                case 'w':
                        init_flags &= ~MNT_RDONLY;
                        break;
                case '?':
                default:
                        usage();
                        /* NOTREACHED */
                }
        }
        argc -= optind;
        argv += optind;

#define BADTYPE(type)                                                   \
        (strcmp(type, FSTAB_RO) &&                                      \
            strcmp(type, FSTAB_RW) && strcmp(type, FSTAB_RQ))

        rval = 0;
        switch (argc) {
        case 0:
                if (all)
                  mountall(init_flags, options, vfslist);
                break;
                /* NOTREACHED */
        case 1:
                if (vfslist != NULL) {
                        usage();
                        /* NOTREACHED */
                }

                /*
                 * Create a canonical version of the device or mount path
                 * passed to us.  It's ok for this to fail.  It's also ok
                 * for the result to be exactly the same as the original.
                 */
                canonical_path = realpath(*argv, canonical_path_buf);
                if (canonical_path == NULL) {
                  printf("unknown special file or file system %s.", *argv);
                  exit(1);
                }

                if (init_flags & MNT_UPDATE) {
                  mountupdate(init_flags, (const char *)options, canonical_path, (char **)argv);
                } else {
                  mounttrycanonical(init_flags, (const char *)options, canonical_path, (char **)argv);
                }
                break;
        default:
                usage();
                /* NOTREACHED */
        }

        exit(rval);
        /* NOTREACHED */
}

/* Read the fstab and mount all */
void mountall(int init_flags, char *options, const char **vfslist)
{
  int rval = 0;
  fstab_traversal trav;
  fstab fs;
  char *mntfromname;
  char m_path[PATH_MAX];
  
  trav.data = &fs;

  openfstab(&trav);
  while (getfsent(&trav)) {
    if (!strcmp(fs.fs_type, FSTAB_RO)) {
      init_flags |= MNT_RDONLY;
    } else if (!strcmp(fs.fs_type, FSTAB_RW)) {
      /* XXX: Not supported by Prex */
    } else if (!strcmp(fs.fs_type, FSTAB_RQ)) {
      /* XXX: Not supported by Prex */
    } else {
      continue; 
    }
    if (hasopt(fs.fs_mntopts, "noauto"))
      continue;
    if (strcmp(fs.fs_special, "from_mount") == 0) {
      if (vfs_simple_findroot(fs.fs_file, m_path, NULL, NULL) != 0) {
        printf("unknown file system %s.", fs.fs_file);
        exit(1);
      }
      mntfromname = m_path;
    } else {
      mntfromname = fs.fs_special;
    }
    mount(mntfromname, fs.fs_file, fs.fs_type, init_flags, 0);
    rval = 1;
  }
  closefstab(&trav);
  exit(rval);
}

/* mountupdate:
 * Try looking up the canonical path first,
 * then try exactly what the user entered.
 */
void
mountupdate(int init_flags, const char *options, char *canonical_path, char **argv)
{
  fstab fs;
  char special[MAX_FS_SPECIAL];
  char m_path[PATH_MAX];
  dev_t m_dev;
  char *mntfromname = NULL;

  if ((vfs_simple_findroot(canonical_path, m_path, &m_dev, NULL) != 0) &&
      (vfs_simple_findroot(*argv, m_path, &m_dev, NULL) != 0))
    {
      printf("unknown special file or file system %s.", *argv);
      exit(1);
    }
  mntfromname = m_path;
  
  mount_on_name(m_dev, special);
  if (getfsfile(special, &fs)) {
    if (strcmp(fs.fs_special, "from_mount") != 0)
      mntfromname = fs.fs_special;
    /* ignore the fstab file options.  */
    fs.fs_mntopts[0] = '\0';
  }

  mount(mntfromname, fs.fs_file, fs.fs_type, init_flags, NULL);
}

int mount_on_name(dev_t dev, char *result) {
  struct devinfo itr;
  itr.cookie = 0;
  while (sys_info(INFO_DEVICE, &itr) == 0) {
    /* For some braindead reason, Kousuke uses dev_t for m_dev
     * instead of device_t. There is an explicit cast in vfs_mount.c */
    if (itr.id == (device_t)(dev)) {
      snprintf(result, MAX_FS_SPECIAL, "/dev/%s", itr.name);
      return 1; /* found */
    }
    itr.cookie++;
  }
  return 0; /* not found */
}

void
mounttrycanonical(int init_flags, const char *options, char *canonical_path, char **argv)
{
  fstab fs;
  char *mntfromname;
  char m_path[PATH_MAX];

  /*
   * Try looking up the canonical path first,
   * then try exactly what the user entered.
   */
  if (!getfsfile(canonical_path, &fs) &&
      !getfsspec(canonical_path, &fs))
    {
      if (!getfsfile(*argv, &fs) &&
          !getfsspec(*argv, &fs))
        {
          printf("%s: unknown special file or file system.", *argv);
          exit(1);
        }
    }
  if (!strcmp(fs.fs_type, FSTAB_RO)) {
    init_flags |= MNT_RDONLY;
  } else if (!strcmp(fs.fs_type, FSTAB_RW)) {
    /* XXX: Not supported by Prex */
  } else if (!strcmp(fs.fs_type, FSTAB_RQ)) {
    /* XXX: Not supported by Prex */
  } else {
    printf("%s has unknown file system type.", *argv);
    exit(1);
  }
  
  if (strcmp(fs.fs_special, "from_mount") == 0) {
    if (vfs_simple_findroot(canonical_path, m_path, NULL, NULL) != 0)
      {
        printf("unknown special file or file system %s.", *argv);
        exit(1);
      }
    mntfromname = m_path;
  } else {
    mntfromname = fs.fs_special;
  }
  mount(mntfromname, /* fstypename */
        fs.fs_vfstype,
        fs.fs_file, /* mntonname */
        init_flags, NULL);
}


/***********************************************************************
 * Fringe functions for nice human interaction                         *
 **********************************************************************/
int
hasopt(const char *mntopts, const char *option)
{
        int negative, found;
        char *opt, *optbuf;

        if (option[0] == 'n' && option[1] == 'o') {
                negative = 1;
                option += 2;
        } else
                negative = 0;
        optbuf = strdup(mntopts);
        found = 0;
        for (opt = optbuf; (opt = strtok(opt, ",")) != NULL; opt = NULL) {
                if (opt[0] == 'n' && opt[1] == 'o') {
                        if (!strcasecmp(opt + 2, option))
                                found = negative;
                } else if (!strcasecmp(opt, option))
                        found = !negative;
        }
        free(optbuf);
        return (found);
}

static void
usage(void)
{

        (void)fprintf(stderr,
            "usage: mount %s\n       mount %s\n       mount %s\n",
            "[-aruvw] [-t type]",
            "[-ruvw] special | node",
            "[-ruvw] [-o options] [-t type] special node");
        exit(1);
        /* NOTREACHED */
}
