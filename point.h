/*--------------------------------------------------------------*/
/* point.h --							*/
/*								*/
/* Memory mapped point allocation (header file)			*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, April 2017, based on code from Magic */
/*--------------------------------------------------------------*/

#ifdef HAVE_SYS_MMAN_H
#include <sys/mman.h>
#include <unistd.h>

/* Page size is 4KB so we mmap a segment equal to 64 pages */
#define POINT_STORE_BLOCK_SIZE (4 * 1024 * 64)

extern POINT PointStoreFreeList;
extern POINT PointStoreFreeList_end;

#endif /* HAVE_SYS_MMAN_H */

extern POINT allocPOINT();
extern void freePOINT(POINT gp);
