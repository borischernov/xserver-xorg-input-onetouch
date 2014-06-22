AM_CFLAGS = $(XORG_CFLAGS)

bin_PROGRAMS = onetouch_control onetouch_to_ascii
onetouch_control_SOURCES = onetouch_control.c
onetouch_control_LDFLAGS = -lX11 -lXi
onetouch_to_ascii_SOURCES = onetouch_to_ascii.c

@DRIVER_NAME@_drv_la_LTLIBRARIES = @DRIVER_NAME@_drv.la
@DRIVER_NAME@_drv_la_LDFLAGS = -module -avoid-version
@DRIVER_NAME@_drv_ladir = @inputdir@

@DRIVER_NAME@_drv_la_SOURCES = calib-math.c calib-math.h xf86OneTouch.c

