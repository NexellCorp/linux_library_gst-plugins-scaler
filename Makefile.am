ACLOCAL_AMFLAGS = -I m4 ${ACLOCAL_FLAGS}

AM_CFLAGS = \
	$(WARN_CFLAGS) \
	$(GLIB_CFLAGS) \
	-I${includedir} \
	-I./ \
	-I${EXTRAPATH_INCLUDE} \
	-I${EXTRAPATH}/lib/glib-2.0/include \
	-I${EXTRAPATH_INCLUDE}/glib-2.0 \
	-I${EXTRAPATH_INCLUDE}/gstreamer-1.0 \
	-I${libdir}/gstreamer-${GST_API_VERSION}/include

libgstnxscaler_la_LTLIBRARIES = libgstnxscaler.la
libgstnxscaler_ladir = ${libdir}/${GST_PLUGIN_PATH}
libgstnxscaler_la_LDFLAGS = -L${libdir} -L${EXTRAPATH_LIB}
libgstnxscaler_la_LIBADD = -lgstvideo-1.0 -lgstallocators-1.0 -lgstbase-1.0 -lgmodule-2.0 -lgobject-2.0 -lglib-2.0
libgstnxscaler_la_LIBADD += -lnx_scaler -lnx_drm_allocator -lnxgstmeta
libgstnxscaler_la_SOURCES = \
	gstnxscaler.c

gstnxscalerincludedir = ${includedir}
gstnxscalerinclude_HEADERS =
