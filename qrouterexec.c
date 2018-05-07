/*----------------------------------------------------------------------*/
/* qrouterexec.c							*/
/*----------------------------------------------------------------------*/

#include <stdio.h>

#include <tk.h>
#include <tcl.h>

/*----------------------------------------------------------------------*/
/* Application initiation.  This is exactly like the AppInit routine	*/
/* for "wish", minus the cruft, but with "tcl_rcFileName" set to	*/
/* "qrouter.tcl" instead of "~/.wishrc".				*/
/*----------------------------------------------------------------------*/

int
qrouter_AppInit(interp)
    Tcl_Interp *interp;
{
    if (Tcl_Init(interp) == TCL_ERROR) {
	return TCL_ERROR;
    }

    // Ignore Tk_Init return code---maybe can attempt to run in
    // a non-graphics mode.

    if (Tk_Init(interp) == TCL_ERROR) {
	return TCL_ERROR;
    }
    Tcl_StaticPackage(interp, "Tk", Tk_Init, Tk_SafeInit);

    /* This is where we replace the home ".wishrc" file with	*/
    /* qrouter's startup script.				*/

    Tcl_SetVar(interp, "tcl_rcFileName", QROUTER_PATH "/qrouter.tcl",
		TCL_GLOBAL_ONLY);
    return TCL_OK;
}

/*----------------------------------------------------------------------*/
/* The main procedure;  replacement for "wish".				*/
/*----------------------------------------------------------------------*/

int
main(argc, argv)
   int argc;
   char **argv;
{
    Tk_Main(argc, argv, qrouter_AppInit);
    return 0;
}

/*----------------------------------------------------------------------*/
