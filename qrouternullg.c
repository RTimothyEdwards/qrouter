/*----------------------------------------------------------------------*/
/* qrouternullg.c							*/
/*----------------------------------------------------------------------*/

#include <stdio.h>

#include <tcl.h>

/*----------------------------------------------------------------------*/
/* Application initiation.  This is exactly like the AppInit routine	*/
/* for "tclsh", minus the cruft, but with "tcl_rcFileName" set to	*/
/* "qrouter.tcl" instead of "~/.tclshrc".				*/
/*----------------------------------------------------------------------*/

int
qrouter_AppInit(interp)
    Tcl_Interp *interp;
{
    if (Tcl_Init(interp) == TCL_ERROR) {
	return TCL_ERROR;
    }
    Tcl_StaticPackage(interp, "Tcl", Tcl_Init, Tcl_Init);

    /* This is where we replace the home ".wishrc" file with	*/
    /* qrouter's startup script.				*/

    Tcl_SetVar(interp, "tcl_rcFileName", QROUTER_PATH "/qrouter.tcl",
		TCL_GLOBAL_ONLY);

    /* Additional variable can be used to tell if qrouter is in non-	*/
    /* graphics mode.							*/
    Tcl_SetVar(interp, "no_graphics_mode", "true", TCL_GLOBAL_ONLY);

    return TCL_OK;
}

/*----------------------------------------------------------------------*/
/* The main procedure;  replacement for "tclsh".			*/
/*----------------------------------------------------------------------*/

int
main(argc, argv)
   int argc;
   char **argv;
{
    Tcl_Main(argc, argv, qrouter_AppInit);
    return 0;
}

/*----------------------------------------------------------------------*/
