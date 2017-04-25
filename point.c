/*--------------------------------------------------------------*/
/* point.c --							*/
/*								*/
/* Memory mapped point allocation				*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, April 2017, based on code from Magic	*/
/*--------------------------------------------------------------*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "qrouter.h"
#include "point.h"

#ifdef HAVE_SYS_MMAN_H

POINT POINTStoreFreeList = NULL;
POINT POINTStoreFreeList_end = NULL;

/* The memory mapped POINT Allocation scheme */

static void *_block_begin = NULL;
static void *_current_ptr = NULL;
static void *_block_end = NULL;

/* MMAP the point store */
static signed char
mmapPOINTStore()
{
    int prot = PROT_READ | PROT_WRITE;
    int flags = MAP_ANON | MAP_PRIVATE;
    u_long map_len = POINT_STORE_BLOCK_SIZE; 	

    _block_begin = mmap(NULL, map_len, prot, flags, -1, 0);
    if (_block_begin == MAP_FAILED)
    {
	fprintf(stderr, "mmapPOINTStore: Unable to mmap ANON SEGMENT\n");
	exit(1);
    }
    _block_end = (void *) ((unsigned long) _block_begin + map_len);
    _current_ptr = _block_begin;
    return 0;
}

POINT
allocPOINT()
{
    POINT _return_point = NULL;

    if (!_block_begin && !_block_end) mmapPOINTStore();

    /* Check if we can get the point from the 
     * Free list
     */

    if (POINTStoreFreeList) {
	_return_point = POINTStoreFreeList;
	POINTStoreFreeList = (POINT)POINTStoreFreeList->next;
	return _return_point;
    }

    /* Get it from the mmap */

    if (((unsigned long)_current_ptr + sizeof(struct point_)) >
		(unsigned long)_block_end)
	 mmapPOINTStore();

    _current_ptr  = (void *)((unsigned long)_current_ptr
		+ sizeof(struct point_));
	
    if ((unsigned long)_current_ptr > (unsigned long) _block_end) {
	fprintf(stderr,
		"allocPOINT(): internal assertion failure.");
	exit(1);
    }
    return (POINT)((unsigned long)_current_ptr - sizeof(struct point_));
}

void
freePOINT(POINT gp)
{
    if (!POINTStoreFreeList_end || !POINTStoreFreeList) {
	POINTStoreFreeList_end = gp;
	gp->next = (POINT)NULL;
	POINTStoreFreeList = POINTStoreFreeList_end;
    }
    else {
	POINTStoreFreeList_end->next = gp;
	POINTStoreFreeList_end = gp;
	POINTStoreFreeList_end->next = (POINT)NULL;
    }
}

#else

POINT
allocPOINT()
{
    POINT newpoint;

    newpoint = (POINT)malloc((unsigned)(sizeof(struct point_)));
    return (newpoint);
}

void
freePOINT(POINT gp)
{
    free((char *)gp);
}

#endif /* !HAVE_SYS_MMAN_H */

