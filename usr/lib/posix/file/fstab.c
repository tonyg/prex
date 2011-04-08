#include <fstab.h>
#include <string.h>

#define FSTAB_LOCATION "/etc/fstab"

int openfstab(pfstab_traversal trav)
{
  FILE *fp = fopen(FSTAB_LOCATION, "r");
  if (fp == NULL) /* error */
    return -1;
  trav->fp = fp;
  trav->data = NULL;
  return 0;
}

void closefstab(pfstab_traversal trav)
{
  if (trav->fp != NULL) {
    fclose(trav->fp);
    trav->fp = NULL;
  }
}

int getfsent(pfstab_traversal trav)
{
  const char* seps = " \t\n";
  const char* nodev = "";
  char *spec = NULL, *type = NULL;
  char *file, *vfstype, *p, *options, line[1024];

  if (trav->fp == NULL)
    return 0;
  /* Skip non-tokens or comments */
  while(spec == NULL || *spec == '#') {
    if ((p = fgets(line, sizeof(line), trav->fp)) == NULL)
      return 0; /* Done */
    spec = strtok(p, seps); /* tokenize by whitespace */
  }
  file = strtok(NULL, seps);
  vfstype = strtok(NULL, seps);
  options = strtok(NULL, seps);
  if (!strcmp(spec, "none"))
    spec = nodev;


  if (options != NULL) { /* read_only default */
    type = strtok(options, ",");
    while(strcmp(type, FSTAB_RO) && strcmp(type, FSTAB_RW) && strcmp(type, FSTAB_RQ)) {
      type = strtok(NULL, ",");
    }
  }
  /* No options or type not found in options */
  if (type == NULL) {
    type = FSTAB_RO;
  }

  strlcpy(trav->data->fs_file, file, MAXPATHLEN);
  strlcpy(trav->data->fs_special, spec, MAX_FS_SPECIAL);
  strlcpy(trav->data->fs_mntopts, options, MAXOPTBUF);
  strlcpy(trav->data->fs_type, type, 3);
  strlcpy(trav->data->fs_vfstype, type, MAX_VFS_TYPE);

  return 1;
}

/* Search fsstab for the entry mounted on "path" */
int getfsfile(char *path, pfstab result)
{
  fstab_traversal trav;
  int found = 0;

  trav.data = result;

  openfstab(&trav);
  while(getfsent(&trav)) {
    if (!strcmp(trav.data->fs_file, path)) {
      found = 1;
      break; /* result has data */
    }
  }
  closefstab(&trav);
  return found;
}

/* Search fsstab for the entry mounting special file "path" */
int getfsspec(char *path, pfstab result)
{
  fstab_traversal trav;
  int found = 0;

  trav.data = result;

  openfstab(&trav);
  while(getfsent(&trav)) {
    if (!strcmp(trav.data->fs_special, path)) {
      found = 1;
      break; /* result has data */
    }
  }
  closefstab(&trav);
  return found;
}
