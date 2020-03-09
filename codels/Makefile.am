lib_LTLIBRARIES = libuavvs_codels.la

libuavvs_codels_la_SOURCES  =	uavvs_c_types.h
libuavvs_codels_la_SOURCES +=	uavvs_codels.cc
libuavvs_codels_la_SOURCES +=	uavvs_main_codels.cc

libuavvs_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libuavvs_codels_la_LIBADD   =	$(requires_LIBS)
libuavvs_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libuavvs_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libuavvs_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	uavvs_c_types.h
CLEANFILES=	${BUILT_SOURCES}

uavvs_c_types.h: ${top_srcdir}/uavvs.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/uavvs_c_types.h.d
