#
# qrouter Makefile
#

# Main compiler arguments
CFLAGS += -g
CPPFLAGS =  -m64 -fPIC
DEFS = -DPACKAGE_NAME=\"\" -DPACKAGE_TARNAME=\"\" -DPACKAGE_VERSION=\"\" -DPACKAGE_STRING=\"\" -DPACKAGE_BUGREPORT=\"\" -DPACKAGE_URL=\"\" -DSTDC_HEADERS=1 -DHAVE_SETENV=1 -DHAVE_PUTENV=1 -DHAVE_VA_COPY=1 -DHAVE___VA_COPY=1 -DHAVE_LIBXT=1 -DHAVE_SYS_TYPES_H=1 -DHAVE_SYS_STAT_H=1 -DHAVE_STDLIB_H=1 -DHAVE_STRING_H=1 -DHAVE_MEMORY_H=1 -DHAVE_STRINGS_H=1 -DHAVE_INTTYPES_H=1 -DHAVE_STDINT_H=1 -DHAVE_UNISTD_H=1 -DHAVE_SYS_MMAN_H=1 -DTCL_QROUTER=1 -DLINUX=1 -DSYSV=1 -DVERSION=\"1.4\" -DREVISION=\"7\"
STUB_DEFS =  -DUSE_TCL_STUBS -DUSE_TK_STUBS
LIBS = -lXt 
LDFLAGS += 
LDDL_FLAGS = -shared -Wl,-soname,$@ -Wl,--version-script=symbol.map
LD_RUN_PATH = 
SHLIB_CFLAGS = -fPIC
LIB_SPECS_NOSTUB =  -L/usr/lib64 -ltk8.6 -L/usr/lib64 -ltcl8.6
LIB_SPECS =  -L/usr/lib64 -ltkstub8.6 -L/usr/lib64 -ltclstub8.6
INC_SPECS = 
TCL_LIB_DIR = /usr/lib64
TK_LIB_DIR = /usr/lib64
EXTRA_LIB_SPECS = -ldl
INSTALL = /usr/bin/install -c
SHDLIB_EXT = .so
EXEEXT = 
X_LIBS = 
X_EXTRA_LIBS = 
X_PRE_LIBS =  -lSM -lICE
QROUTER_LIB_DIR = share/qrouter
WISH_EXE = /usr/bin/wish
VERSION = 1.4
REVISION = 7
prefix = /usr/local

INSTALL_TARGET := install-tcl
ALL_TARGET := tcl

SOURCES = qrouter.c point.c maze.c mask.c node.c output.c qconfig.c lef.c def.c
OBJECTS := $(patsubst %.c,%.o,$(SOURCES))

SOURCES2 = graphics.c tclqrouter.c tkSimple.c delays.c antenna.c
OBJECTS2 := $(patsubst %.c,%.o,$(SOURCES2))

SOURCES3 = qrouterexec.c
OBJECTS3 := $(patsubst %.c,%.o,$(SOURCES3))

SOURCES4 = qrouternullg.c
OBJECTS4 := $(patsubst %.c,%.o,$(SOURCES4))

SOURCES5 = main.c
OBJECTS5 := $(patsubst %.c,%.o,$(SOURCES5))

BININSTALL = ${prefix}/bin
LIBINSTALL = ${prefix}/${QROUTER_LIB_DIR}
EXTRA_DEFS = -DQROUTER_PATH=\"${LIBINSTALL}\"

all: $(ALL_TARGET)

install: $(INSTALL_TARGET)

nointerp: qrouter$(EXEEXT)

tcl: qrouter.sh qrouter.tcl qrouter$(SHDLIB_EXT) qrouterexec$(EXEEXT) \
	qrouternullg$(EXEEXT)

qrouter.tcl: qrouter.tcl.in
	sed -e '/LIBDIR/s#LIBDIR#${LIBINSTALL}#' \
	    -e '/VERSION/s#VERSION#${VERSION}#' \
	    -e '/REVISION/s#REVISION#${REVISION}#' \
		qrouter.tcl.in > $@

qrouter.sh: qrouter.sh.in
	sed -e '/WISH_EXE/s#WISH_EXE#${WISH_EXE}#' \
	    -e '/LIBDIR/s#LIBDIR#${LIBINSTALL}#' \
		qrouter.sh.in > $@
	chmod 0755 $@

qrouter$(EXEEXT): $(OBJECTS) $(OBJECTS5)
	$(CC) $(LDFLAGS) $(OBJECTS) $(OBJECTS5) -o $@ $(LIBS) -lm

qrouter$(SHDLIB_EXT): $(OBJECTS) $(OBJECTS2)
	$(RM) qrouter$(SHDLIB_EXT)
	$(CC) ${CFLAGS} ${STUB_DEFS} ${SHLIB_CFLAGS} -o $@ \
		${LDDL_FLAGS} $(OBJECTS) $(OBJECTS2) \
		${LDFLAGS} -lc ${LIBS} ${X_PRE_LIBS} -lX11 ${X_LIBS} \
		${X_EXTRA_LIBS} ${LIB_SPECS} ${EXTRA_LIB_SPECS} -lm

qrouterexec$(EXEEXT): $(OBJECTS3)
	$(RM) qrouterexec$(EXEEXT)
	$(CC) ${CFLAGS} ${CPPFLAGS} ${DEFS} ${EXTRA_DEFS} \
		${SOURCES3} ${INC_SPECS} -o $@  ${LIB_SPECS_NOSTUB} \
		${LD_RUN_PATH} ${LDFLAGS} ${X_PRE_LIBS} -lX11 ${X_LIBS} \
		${X_EXTRA_LIBS} ${LIBS} ${EXTRA_LIB_SPECS} -lm

qrouternullg$(EXEEXT): $(OBJECTS4)
	$(RM) qrouternullg$(EXEEXT)
	$(CC) ${CFLAGS} ${CPPFLAGS} ${DEFS} ${EXTRA_DEFS} \
		${SOURCES4} ${INC_SPECS} -o $@  ${LIB_SPECS_NOSTUB} \
		${LD_RUN_PATH} ${LDFLAGS} ${LIBS} ${EXTRA_LIB_SPECS} -lm

install-nointerp:
	@echo "Installing qrouter"
	$(INSTALL) -d $(DESTDIR)${BININSTALL}
	$(INSTALL) qrouter $(DESTDIR)${BININSTALL}

install-tcl: qrouter.sh qrouter.tcl qrouter$(SHDLIB_EXT) \
		qrouterexec$(EXEEXT) qrouternullg$(EXEEXT)
	@echo "Installing qrouter"
	$(INSTALL) -d $(DESTDIR)${BININSTALL}
	$(INSTALL) -d $(DESTDIR)${LIBINSTALL}
	$(INSTALL) qrouter.sh $(DESTDIR)${BININSTALL}/qrouter
	$(INSTALL) qrouter$(SHDLIB_EXT) $(DESTDIR)${LIBINSTALL}
	$(INSTALL) qrouterexec$(EXEEXT) $(DESTDIR)${LIBINSTALL}
	$(INSTALL) qrouternullg$(EXEEXT) $(DESTDIR)${LIBINSTALL}
	$(INSTALL) console.tcl $(DESTDIR)${LIBINSTALL}
	$(INSTALL) tkcon.tcl $(DESTDIR)${LIBINSTALL}
	$(INSTALL) qrouter.tcl $(DESTDIR)${LIBINSTALL}

uninstall:
	$(RM) $(DESTDIR)${BININSTALL}/qrouter

clean:
	$(RM) $(OBJECTS)
	$(RM) $(OBJECTS2)
	$(RM) $(OBJECTS3)
	$(RM) $(OBJECTS4)
	$(RM) $(OBJECTS5)
	$(RM) qrouterexec$(EXEEXT)
	$(RM) qrouternullg$(EXEEXT)
	$(RM) qrouter$(EXEEXT)
	$(RM) qrouter$(SHDLIB_EXT)
	$(RM) qrouter.tcl
	$(RM) qrouter.sh

veryclean:
	$(RM) $(OBJECTS)
	$(RM) $(OBJECTS2)
	$(RM) $(OBJECTS3)
	$(RM) $(OBJECTS4)
	$(RM) $(OBJECTS5)
	$(RM) qrouterexec$(EXEEXT)
	$(RM) qrouternullg$(EXEEXT)
	$(RM) qrouter$(EXEEXT)
	$(RM) qrouter$(SHDLIB_EXT)
	$(RM) qrouter.tcl
	$(RM) qrouter.sh

distclean:
	$(RM) $(OBJECTS)
	$(RM) $(OBJECTS2)
	$(RM) $(OBJECTS3)
	$(RM) $(OBJECTS4)
	$(RM) $(OBJECTS5)
	$(RM) qrouterexec$(EXEEXT)
	$(RM) qrouternullg$(EXEEXT)
	$(RM) qrouter$(EXEEXT)
	$(RM) qrouter$(SHDLIB_EXT)
	$(RM) qrouter.tcl
	$(RM) qrouter.sh

.c.o:
	$(CC) $(CFLAGS) $(CPPFLAGS) $(SHLIB_CFLAGS) $(DEFS) $(STUB_DEFS) \
		$(EXTRA_DEFS) $(INC_SPECS) -c $< -o $@
