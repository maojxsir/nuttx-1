#include <sys/stat.h>
#include <sys/statfs.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stm32_ccm.h>
#include <ficl/ficl.h>

void *ficlMalloc(size_t size)
{
#ifdef HAVE_CCM_HEAP
  void * p = NULL;
  p = ccm_malloc(size);
  printf("ficlMalloc %x : %x\n",p,size);
  return p;
#else
  return malloc(size);
#endif
}

void *ficlRealloc(void *p, size_t size)
{
#ifdef HAVE_CCM_HEAP
  return ccm_realloc(p,size);
#else
  return realloc(p, size);
#endif
}

void ficlFree(void *p)
{
#ifdef HAVE_CCM_HEAP
  ccm_free(p);
#else
  free(p);
#endif
}

void  ficlCallbackDefaultTextOut(ficlCallback *callback, char *message)
{
  FICL_IGNORE(callback);
  if (message != NULL)
      fputs(message, stdout);
  else
      fflush(stdout);
  return;
}

int ficlFileStatus(char *filename, int *status)
{
    struct stat statbuf;
    if (stat(filename, &statbuf) == 0)
    {
        *status = statbuf.st_mode;
        return 0;
    }
    *status = ENOENT;
    return -1;
}

long ficlFileSize(ficlFile *ff)
{
    struct stat statbuf;
    if (ff == NULL)
        return -1;
  
    statbuf.st_size = -1;
    if (stat(ff->filename, &statbuf) != 0)
        return -1;
  
    return statbuf.st_size;
}

void ficlSystemCompilePlatform(ficlSystem *system)
{
    return;
}


