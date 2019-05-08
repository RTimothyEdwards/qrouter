/*--------------------------------------------------------------*/
/* qrouter entry point for non-Tcl compile version		*/
/*--------------------------------------------------------------*/

#include <stdio.h>

#include "qrouter.h"

/*--------------------------------------------------------------*/
/* Procedure main() performs the basic route steps without any	*/
/* interaction or scripting, writes a DEF file as output, and	*/
/* exits.							*/
/*								*/
/* Precedure mimics the "standard_route" script (up to date as	*/
/* of November 25, 2015)					*/
/*--------------------------------------------------------------*/

int
main(int argc, char *argv[])
{
    int result;
 
    result = runqrouter(argc, argv);
    if (result != 0) return result;

    read_def(NULL);
    maskMode = MASK_AUTO;
    dofirststage(0, -1);
    maskMode = MASK_NONE;
    result = dosecondstage(0, FALSE, FALSE, (u_int)100);
    if (result < 5)
	dosecondstage(0, FALSE, FALSE, (u_int)100);
    write_def(NULL);
    write_delays((delayfilename == NULL) ? "stdout" : delayfilename);
    return 0;
}

/*--------------------------------------------------------------*/
/* Define graphics routines as empty subroutines.  May want to	*/
/* provide a simple X11 graphics environment outside of Tcl/Tk	*/
/*--------------------------------------------------------------*/

void
highlight_source(void) {
}

void
highlight_dest(void) {
}

void
highlight_starts(POINT glist) {
}

void
highlight_mask(void) {
}

void
highlight(int x, int y) {
}

void
draw_net(NET net, u_char single, int *lastlayer) {
}

void
draw_layout(void) {
}

int
recalc_spacing(void) {
   return 0;
}
