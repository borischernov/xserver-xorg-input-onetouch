/*
 * Copyright 2014 by Boris Chernov <boris@imode.lv>
 * Based on onetouch driver by Alessandro Rubini <rubini@linux.it>
 * Copyright 2000-2003 by Alessandro Rubini <rubini@linux.it>
 *      somehow based on xf86Summa.c:
 * Copyright 1996 by Steven Lang <tiger@tyger.org>
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that
 * copyright notice and this permission notice appear in supporting
 * documentation, and that the name of the authors not be used in advertising
 * or publicity pertaining to distribution of the software without specific,
 * written prior permission.  The authors make no representations about the
 * suitability of this software for any purpose.  It is provided "as is"
 * without express or implied warranty.
 *
 * THE AUTHORS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL STEVEN LANG BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTIONS, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <xorgVersion.h>

#include <misc.h>
#include <xf86.h>
#include <xf86_OSproc.h>
#include <xf86Xinput.h>
#include <exevents.h>

#include <xf86Module.h>

#include <fcntl.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>

//#include <keysym.h>
//#include <mipointer.h>

#include "calib-math.h"

#undef sleep
#define sleep(t) xf86WaitForInput(-1, 1000 * (t))
#define wait_for_fd(fd) xf86WaitForInput((fd), 1000)
#define tcflush(fd, n) xf86FlushInput((fd))
#undef read
#define read(a,b,c) xf86ReadSerial((a),(b),(c))
#undef write
#define write(a,b,c) xf86WriteSerial((a),(char*)(b),(c))
#undef close
#define close(a) xf86CloseSerial((a))
#define XCONFIG_PROBED "(==)"
#define XCONFIG_GIVEN "(**)"
#define xf86Verbose 1
#undef PRIVATE
#define PRIVATE(x) XI_PRIVATE(x)

/* 
 * Be sure to set vmin appropriately for your device's protocol. You want to
 * read a full packet before returning
 */
static const char *default_options[] =
{
	"BaudRate", "9600",
	"StopBits", "1",
	"DataBits", "8",
	"Parity", "None",
	"Vmin", "1",
	"Vtime", "10",
	"FlowControl", "None",
	NULL
};

static InputDriverPtr gnzDrv;

#if defined(__QNX__) || defined(__QNXNTO__)
#define POSIX_TTY
#endif

#include "onetouch.h" /* Definitions for the led meanings */

/*
** Debugging macros
*/
#ifdef DBG
#undef DBG
#endif
#ifdef DEBUG
#undef DEBUG
#endif

static int      debug_level = 0;
#define DEBUG	1
#if DEBUG
#define 	DBG(lvl, f) 	{if ((lvl) <= debug_level) f;}
#else
#define 	DBG(lvl, f)
#endif

/*
** Device records
*/
#define ONETOUCH_MAXPHYSCOORD 1023
#define ONETOUCH_MAXCOORD     (64*1024-1) /* oversampled, synthetic value */
#define FLAG_PRE_TAPPING	1 /* this is motion, despite being pen-down */
#define FLAG_WAS_UP		2 /* last event was a pen-up event */

#define ONETOUCH_DEFAULT_TAPPING_DELAY 0 /* off */
#define ONETOUCH_DEFAULT_JITTER_DELAY 50 /* milli Seconds */

/*
 * Prototype for a callback to handle delayed release event.
 * This is handy for integrating out the on/off jitter that
 * the Onetouch device tends to produce during a user drag (Chris Howe)
 */
static CARD32 touchButtonTimer(OsTimerPtr timer, CARD32 now, pointer arg);

 
typedef struct 
{
    char	*onetDevice;	/* device file name */
    int		flags;		/* various flags */
    int		onetBaud;	/* 9600 or 19200 */
    int		onetDlen;	/* data length (3 or 11) */
    int		onetAvgX;	/* previous X position */
    int		onetAvgY;	/* previous Y position */
    int		onetSmooth;	/* how smooth the motion is */
    int		onetPrevButton;	/* previous button state */
    int		onetBytes;	/* number of bytes read */
    int         onetButton;     /* 0 (default), 1, 2, -1, -2 for "once" */
    unsigned char onetData[16];	/* data read on the device */
    double      onetCalib[6];    /* Calibration parameters */
    char	*onetConfig;     /* filename for configuration */
    int         onetJitterDelay;  /* time to delay before issuing a buttonup */
    int         onetTappingDelay;  /* longer hops are not considered ticks */
    int		onetDisableDrag;   /* disable drag */
    long        onetUpSec;         /* when did pen-up happen */
    long        onetUpUsec;        /* when did pen-up happen */
    LedCtrl    *onetLeds;
    OsTimerPtr  timer;
} OnetouchDeviceRec, *OnetouchDevicePtr;

#define ONETOUCH_SERIAL_DLEN 11
#define ONETOUCH_PS2_DLEN     3

/*
** Configuration data
*/
#define ONETOUCH_SECTION_NAME    "Onetouch"
#define ONETOUCH_DEFAULT_CFGFILE "/etc/onetouch/onetouch.calib"


/*
** Contants and macro
*/
#define BUFFER_SIZE	120	  /* size of reception buffer */
#define XI_NAME 	"ONETOUCH" /* X device name for the touch screen */
#define MSGID           "xf86Onetouch: "

#define SYSCALL(call) while(((call) == -1) && (errno == EINTR))

/*
** Replacement for missing xf86getsecs
*/
static void
xf86getsecs(long *secs, long *usecs)
{
	struct timeval t;

	gettimeofday(&t, NULL);
	*secs = t.tv_sec;
	*usecs = t.tv_usec;
}

/*
** xf86OnetouchReadInput
** Reads from the touch screen, uses calibration data
** and posts any new events to the server.
*/
static void
xf86OnetouchReadInput(InputInfoPtr local)
{
    OnetouchDevicePtr	priv = (OnetouchDevicePtr) local->private;
    unsigned char *	pkt = priv->onetData;
    int			len, loop;
    int			x, y, button;
    double *		calib = priv->onetCalib;
    DeviceIntPtr	device;
    unsigned char	buffer[BUFFER_SIZE];
    static 		int oldbytes; /* bytes left over on the buffer */
    static		unsigned char up_packet[] = "\xff\xfe\xfe";
    #define		up_size 3
    long                sec, usec;
  
    DBG(7, ErrorF("xf86OnetouchReadInput BEGIN device=%s fd=%d (bytes %i)\n",
       priv->onetDevice, local->fd, priv->onetBytes));

    oldbytes = priv->onetBytes;
    priv->onetBytes = 0;
    memcpy(buffer, pkt, oldbytes);

    SYSCALL(len = read(local->fd, buffer+oldbytes, sizeof(buffer)-oldbytes));

    if (len <= 0) {
	ErrorF("error reading Onetouch touch screen device");
	return;
    }
    DBG(7, ErrorF("OldBytes: %d, NewBytes: %d\n", oldbytes, len));
    len += oldbytes;

    /*
     * Serial protocol:
     *    0xff, <lowx>, <highx>, <lowy>, <highy>  (5b)
     *    0xff, 0xfe, 0xfe (3b) == release
     */
    for (loop = 0; loop < len; ) {
	/* resync, in case we lost something */
	if (buffer[loop] != 0xff) {
	    loop++;
	    if (loop == len) {
		oldbytes = 0;
		return;
	    }
	}
	/* check if not enough data for a packet */
	if ( len-loop < 3
	     || (strncmp(buffer+loop, up_packet, up_size) && len-loop < 5)) {
	    DBG(6, ErrorF("short (%i) %02x %02x %02x\n", len-loop,
			  buffer[loop], buffer[loop+1],buffer[loop+2]));
	    memcpy(priv->onetData, buffer+loop, len-loop);
	    priv->onetBytes = len-loop;
	    return;
	}
	/* handle the 3-byte packet */
	if (len-loop >= 3 && !strncmp(buffer+loop, up_packet, up_size)) {
	    /* If there is further data, ignore this up event */
	    if (len-loop > 4 && buffer[loop+3] == 0xff) {
		loop += 3;
		DBG(6, ErrorF("ignore up"));
		continue;
	    }
	    /*
	     * The user has let go of the touchpad, but we dont know 
	     * if this is a bounce or a real button up event. Set up
	     * a timer to fire off the button up event unless something
	     * else happens (Chris Howe)
	     */
            DBG(6, ErrorF("process up"));
	    xf86getsecs(&priv->onetUpSec, &priv->onetUpUsec); /* tap info */
	    if (priv->onetJitterDelay) {
		priv->timer = TimerSet( priv->timer, 0,
					priv->onetJitterDelay  /* delay */,
					touchButtonTimer, local);
	    } else {
		touchButtonTimer(NULL, NULL, local);
	    }
	    loop += 3;
	    continue;
	}
	/*
	 * everything from here onwards is about the 5-byte packet
	 */
	x = buffer[loop+1] + (buffer[loop+2]<<8);
	y = buffer[loop+3] + (buffer[loop+4]<<8);
	DBG(6, ErrorF("BeforeCalib: %5i, %5i\n", x, y));

	/* calibration leads to values that span 16 bits like before */
	x = (int)(calib[0]*(double)x + calib[1]*(double)y + calib[2]);
	y = (int)(calib[3]*(double)x + calib[4]*(double)y + calib[5]);
	button = 1;

	/* smooth it down, unless first touch */
	if (!(priv->flags & FLAG_WAS_UP)) {
	    x = (priv->onetAvgX * priv->onetSmooth + x)/(priv->onetSmooth+1);
	    y = (priv->onetAvgY * priv->onetSmooth + y)/(priv->onetSmooth+1);
	}

	/* FIXME: this isn't coordinated with debouncing */
	if (!button) priv->flags |= FLAG_WAS_UP;
	else priv->flags &= ~FLAG_WAS_UP;

	DBG(6, ErrorF("Cooked: %5i, %5i (%i)\n", x, y, !!button));
	
	/* Now send events */
	device = local->dev;
	if ( (priv->onetAvgX != x) || (priv->onetAvgY != y) ) {
	    if (priv->onetDisableDrag) {
	    /* Send button up event before any motion event to prevent drag */
            	int b = priv->onetButton;
            	if (b < 0) b = -b;
	    	xf86PostButtonEvent(device, 1 /* absolute */, b+1 /* button */,
                      0 /* isdown */, 0, 2, x, y);
	    }
	    xf86PostMotionEvent(device, 1 /* absolute */, 0, 2, x, y);
	}

	/*
	 * If we were in an up state and now we are in a down state
	 * consider sending a button down event.
	 */
	if (!priv->onetPrevButton) {
	    if( priv->timer ) {
		/*
		 * Uh-oh. We have detected a bounce. Cancel the timer
		 * and do not send a button event.
		 */
		TimerFree( priv->timer );
		priv->timer = NULL;
	    } else {
		int deltams = 0;
		/* No timer: real touchdown. Is this pre-tap? */
		xf86getsecs(&sec, &usec);
		deltams = (sec - priv->onetUpSec) * 1000
		    + (usec - priv->onetUpUsec) / 1000;
		if (!priv->onetTappingDelay 
		    || (deltams < priv->onetTappingDelay)) {
		    int b = priv->onetButton;
		    if (b < 0) b = -b;
		    xf86PostButtonEvent(device, 1 /* absolute */, 
					b+1 /* button, 0 is b1 etc */,
					1 /* isdown */, 0, 2, x, y);
		    DBG(1, ErrorF("Post button %i\n", b+1));
		} else {
		    priv->flags |= FLAG_PRE_TAPPING;
		    DBG(1, ErrorF("Pre-tapping\n"));
		}
	    }
	}
	/* remember data */
	priv->onetPrevButton = button;
	priv->onetAvgX = x;
	priv->onetAvgY = y;
	loop += 5;
    }
    priv->onetBytes = len - loop;
    memcpy(priv->onetData, buffer+loop, len-loop);
    return;
}


/* send a button event after a delay */
static CARD32
touchButtonTimer(OsTimerPtr timer, CARD32 now, pointer arg)
{
    InputInfoPtr local = (InputInfoPtr) arg;
    OnetouchDevicePtr priv = (OnetouchDevicePtr)(local->private);
    int sigstate;
    int b = priv->onetButton;

    if (b < 0) b = -b;
    sigstate = xf86BlockSIGIO();
    xf86PostButtonEvent(local->dev, 1 /* absolute */, b+1 /* button */,
		      0 /* isdown */, 0, 2, 
		      priv->onetAvgX, priv->onetAvgY);
    xf86UnblockSIGIO(sigstate);
    if (priv->onetButton < 0) { /* "once" done, reset it */
	priv->onetLeds->led_values = 0;
	priv->onetButton = 0;
    }
    priv->onetPrevButton = 0;

    priv->flags &= ~FLAG_PRE_TAPPING;
    DBG(1, ErrorF("Post delayed button-up %i\n", b+1));
    priv->timer = NULL;

    return(0);
}


/*
** xf86OnetouchControlProc
** This is called for each device control that is defined at init time.
** It currently isn't use, but I plan to make tapping, smoothness and
** on/off available as integer device controls.
*/
static void
xf86OnetouchControlProc(DeviceIntPtr	device, PtrCtrl *ctrl)
{
    DBG(2, ErrorF("xf86OnetouchControlProc\n"));
}

/* Read the configuration file or revert to default (identity) cfg */
static int xf86OnetouchReadCalib(OnetouchDevicePtr priv, int activate)
{
    int i, err = 1;
    static int numbers[10];
    FILE *f;
    double precision; /* unused by now */

    f = fopen(priv->onetConfig, "r");
    if (f) {
	char s[80];
	fgets(s, 80, f); /* discard the comment */
	for (i=0; i<10; i++)
	    err = fscanf(f, "%i", numbers+i);
	err = (err == 1) ? 0 : 1; /* 1 means ok, 0 or -1 means error */
	fclose(f);
	}
    if (err)
	ErrorF(MSGID "Calibration data absent or invalid, using defaults\n");
    if (!err) {
	/* ok, we now have the 10 raw values, convert them */
	err = official_5p_calibration( 1<<16, 1<<16, /* 64k range */
				      numbers, priv->onetCalib, &precision);
    }
    if (err || !activate) {
	/* LED_UNCALIBRATE passes through here, to avoid duplication */
	memset(priv->onetCalib, 0, 6 * sizeof(double));
	priv->onetCalib[0] = 1.0; /* X = 1*x + 0*y + 0 */
	priv->onetCalib[4] = 1.0; /* Y = 0*x + 1*y + 0 */
	return 0;
    }
    ErrorF(MSGID "Calibration data valid\n");
    ErrorF(MSGID "Calibration: X = %f x + %f y + %f\n",
	   priv->onetCalib[0], priv->onetCalib[1], priv->onetCalib[2]);
    ErrorF(MSGID "Calibration: Y = %f x + %f y + %f\n",
	   priv->onetCalib[3], priv->onetCalib[4], priv->onetCalib[5]);
    return 0;
}

/*
 * This feedback function is used to get commands from a client application
 */
static void 
xf86OnetouchLeds(DeviceIntPtr dev, LedCtrl *ctrl)
{
    InputInfoPtr	local = (InputInfoPtr)dev->public.devicePrivate;
    OnetouchDevicePtr	priv = (OnetouchDevicePtr)local->private;
    int cmd;

    if (!priv->onetLeds) {
	/* fist time */
	priv->onetLeds = ctrl;
	ctrl->led_mask = ~0;
    }

    DBG(9, ErrorF(MSGID "ledfeedback  %x %x\n",
		  ctrl->led_values, ctrl->led_mask));
    cmd = ctrl->led_values & ctrl->led_mask;

    if (cmd & OTLED_UNCALIBRATE) {
	/* remove calibration, used before making a new calibration */
	xf86OnetouchReadCalib(priv, 0);
    }
    if (cmd & OTLED_RECALIBRATE) {
	/* xf86OnetouchProc( (DeviceIntPtr)local, DEVICE_ON); */
	if (local->fd < 0) return;
	tcflush(local->fd, TCIFLUSH); /* flush pending input */
	/* double-enable is fine, so we can do it always */
	AddEnabledDevice(local->fd);
	xf86OnetouchReadCalib(priv, 1);
	/* ptr->public.on = TRUE; */
    }
    if (cmd & OTLED_OFF) {
	if (local->fd >= 0)
	    RemoveEnabledDevice(local->fd);
    }


    if (cmd & OTLED_BUTTON1) {
	priv->onetButton = 0;
    }
    if (cmd & OTLED_BUTTON2) {
	priv->onetButton = 1;
    }
    if (cmd & OTLED_BUTTON3) {
	priv->onetButton = 2;
    }
    if (cmd & OTLED_BUTTON2ONCE) {
	priv->onetButton = -1;
    }
    if (cmd & OTLED_BUTTON3ONCE) {
	priv->onetButton = -2;
    }
}

/*
 ***************************************************************************
 * xf86OnetouchOpen
 * Open and initialize the panel, as well as probe for any needed data.
 ***************************************************************************
 */
#define WAIT(t)                                                 \
    err = xf86WaitForInput(-1, ((t) * 1000));                   \
    if (err == -1) {                                            \
       ErrorF("Onetouch select error : %s\n", strerror(errno));   \
       return !Success;                                        \
    }

static Bool
xf86OnetouchOpen(InputInfoPtr local)
{

    OnetouchDevicePtr	priv = (OnetouchDevicePtr)local->private;

    DBG(1, ErrorF("opening %s (calibration file is \"%s\")\n",
		  priv->onetDevice, priv->onetConfig));

    xf86OnetouchReadCalib(priv, 1);

    priv->onetDlen = ONETOUCH_SERIAL_DLEN;

    local->fd = xf86OpenSerial(local->options);

    if (local->fd == -1) {
	ErrorF(priv->onetDevice);
	return !Success;
    }
    DBG(2, ErrorF("%s opened as fd %d\n", priv->onetDevice, local->fd));

    DBG(1, ErrorF("initializing Onetouch touch screen\n"));

    /* Hmm... close it, so it doens't say anything before we're ready */
    /* FIXME */

    /* Clear any pending input */
    tcflush(local->fd, TCIFLUSH);
  
    if (xf86Verbose)
	ErrorF("%s Onetouch touch screen\n", XCONFIG_PROBED);
    return Success;
}

/*
** xf86OnetouchOpenDevice
** Opens and initializes the device driver stuff
*/
static int
xf86OnetouchOpenDevice(DeviceIntPtr ptr)
{
    InputInfoPtr	local = (InputInfoPtr)ptr->public.devicePrivate;
    Atom 		axis_labels[2] = { 0, 0 };

    if (xf86OnetouchOpen(local) != Success) {
	if (local->fd >= 0) {
	    SYSCALL(close(local->fd));
	}
	local->fd = -1;
    }

    /* Initialize the axes */
    InitValuatorAxisStruct(ptr, 0, /* X */
			   axis_labels[0],
			   0, ONETOUCH_MAXCOORD, /* min, max val */
			   500000, /* resolution */
			   0, 500000, /* min, max_res */
			   Absolute); /* mode */
    InitValuatorAxisStruct(ptr, 1, /* Y */
			   axis_labels[1],
			   0, ONETOUCH_MAXCOORD, /* min, max val */
			   500000, /* resolution */
			   0, 500000, /* min, max_res */
			   Absolute); /* mode */

    /* Register led feedback */
    if (InitLedFeedbackClassDeviceStruct(ptr, xf86OnetouchLeds) == FALSE)
        DBG(0, ErrorF("Onetouch: can't initialize the feedback structure\n"));

    return (local->fd != -1);
}

/* ################################################# */


/*
** xf86OnetouchProc
** Handle requests to do stuff to the driver.
** In order to allow for calibration, the function is called (with DEVICE_OFF
** and DEVICE_ON commands) also by xf86OnetouchChangeControl.
*/
static int
xf86OnetouchProc(DeviceIntPtr ptr, int what)
{
    CARD8		map[25]; /* 25? */
    int			nbaxes;
    int			nbbuttons;
    int			loop;
    InputInfoPtr	local = (InputInfoPtr)ptr->public.devicePrivate;
    OnetouchDevicePtr	priv = (OnetouchDevicePtr)(local->private);
    Atom                button_labels[1] = { 0 };
    Atom                axis_labels[2] = { 0, 0 };

    DBG(2, ErrorF("BEGIN xf86OnetouchProc dev=0x%x priv=0x%x what=%d\n", ptr, priv, what));

    switch (what) {
	case DEVICE_INIT:
	    DBG(1, ErrorF("xf86OnetouchProc ptr=0x%x what=INIT\n", ptr));

	    nbaxes = 2;			/* X, Y */
	    nbbuttons = 3; /* three buttons, thanks to an helper app */

	    for(loop=1; loop<=nbbuttons; loop++) map[loop] = loop;

	    if (InitButtonClassDeviceStruct(ptr,
					    nbbuttons,
					    button_labels,
					    map) == FALSE) {
		ErrorF("unable to allocate Button class device\n");
		return !Success;
	    }

	    if (InitFocusClassDeviceStruct(ptr) == FALSE) {
		ErrorF("unable to init Focus class device\n");
		return !Success;
	    }

	    /*
	     * Use default (?) feedback stuff. 
	     * I'll use device feedback controls to change parameters at
	     * run time.
	     */
	    if (InitPtrFeedbackClassDeviceStruct(ptr,
		   xf86OnetouchControlProc) == FALSE) {
		ErrorF("unable to init ptr feedback\n");
		return !Success;
	    }

	    if (InitProximityClassDeviceStruct(ptr) == FALSE) {
		ErrorF("unable to init proximity class device\n"); 
		return !Success;
	    }

	    if (InitValuatorClassDeviceStruct(ptr,
		   nbaxes,
		   axis_labels,
		   GetMotionHistorySize(),
		   Absolute)
		   == FALSE) {
		ErrorF("unable to allocate Valuator class device\n"); 
		return !Success;
	    }
	    /* allocate the motion history buffer if needed */
	    xf86MotionHistoryAllocate(local);

	    /* open the device to gather informations */
	    xf86OnetouchOpenDevice(ptr);
	    break;

	case DEVICE_ON:
	    DBG(1, ErrorF("xf86OnetouchProc ptr=0x%x what=ON\n", ptr));

	    if ((local->fd < 0) && (!xf86OnetouchOpenDevice(ptr))) {
		return !Success;
	    }
	    tcflush(local->fd, TCIFLUSH); /* flush pending input */
	    AddEnabledDevice(local->fd);
	    ptr->public.on = TRUE;
	    break;

	case DEVICE_OFF:
	    DBG(1, ErrorF("xf86OnetouchProc  ptr=0x%x what=OFF\n", ptr));
	    if (local->fd >= 0)
		RemoveEnabledDevice(local->fd);
	    ptr->public.on = FALSE;
	    break;

	case DEVICE_CLOSE:
	    DBG(1, ErrorF("xf86OnetouchProc  ptr=0x%x what=CLOSE\n", ptr));
	    SYSCALL(close(local->fd));
	    local->fd = -1;
	    break;

	default:
	    ErrorF("unsupported mode=%d\n", what);
	    return !Success;
	    break;
    }
    DBG(2, ErrorF("END   xf86OnetouchProc Success what=%d dev=0x%x priv=0x%x\n",
	   what, ptr, priv));
    return Success;
}

/*
** xf86OnetouchSwitchMode
** Switches the mode. Only absolute is allowed.
*/
static int
xf86OnetouchSwitchMode(ClientPtr client, DeviceIntPtr dev, int mode)
{
    DBG(3, ErrorF("xf86OnetouchSwitchMode dev=0x%x mode=%d\n", dev, mode));

    switch(mode) {
	case Absolute:
	    break;

	default:
	    DBG(1, ErrorF("xf86OnetouchSwitchMode dev=0x%x invalid mode=%d\n",
		   dev, mode));
	    return BadMatch;
    }
    return Success;
}


/*
** xf86OnetouchAllocate
** Allocates the device structures for the Onetouch Touch Screen.
*/
static int
xf86OnetouchAllocate(InputInfoPtr local)
{
    OnetouchDevicePtr	priv = (OnetouchDevicePtr)malloc(sizeof(OnetouchDeviceRec));

    local->name = XI_NAME;
    local->type_name = "Onetouch Touch Screen";
    local->flags = 0; /*XI86_NO_OPEN_ON_INIT;*/
    local->device_control = xf86OnetouchProc;
    local->read_input = xf86OnetouchReadInput;
    local->switch_mode = xf86OnetouchSwitchMode;
    local->control_proc = NULL;
    local->fd = -1;
    local->dev = NULL;
    local->private = priv;

    priv->onetDevice = "";         /* device file name */
    priv->onetConfig = ONETOUCH_DEFAULT_CFGFILE;
    priv->onetDlen = 0;            /* unknown */
    priv->onetBaud = 9600;
    priv->onetSmooth = 9;          /* quite smooth */
    priv->onetAvgX = -1;           /* previous (avg) X position */
    priv->onetAvgY = -1;           /* previous (avg) Y position */
    priv->onetPrevButton = 0;      /* previous buttons state */
    priv->flags = FLAG_WAS_UP;    /* first event is button-down */
    priv->onetBytes = 0;           /* number of bytes read */
    priv->onetButton = 0;
    priv->onetTappingDelay = ONETOUCH_DEFAULT_TAPPING_DELAY;
    xf86getsecs(&priv->onetUpSec, &priv->onetUpUsec);
    priv->timer = NULL;

    return Success;
}


/*
 * xf86OnetUninit --
 *
 * called when the driver is unloaded.
 */
static void
xf86OnetUninit(InputDriverPtr    drv,
              InputInfoPtr    local,
              int flags)
{
    OnetouchDevicePtr      priv = (OnetouchDevicePtr) local->private;

    DBG(1, ErrorF("xf86OnetUninit\n"));

    xf86OnetouchProc(local->dev, DEVICE_OFF);

    free(priv);
    xf86DeleteInput(local, 0);
}

/*
 * xf86OnetInit --
 *
 * called when the module subsection is found in XF86Config
 */
static int
xf86OnetInit(InputDriverPtr      drv,
	     InputInfoPtr	 pInfo,
	     int		 flags)
{   
   OnetouchDevicePtr priv = NULL;
   int		     rc;

   gnzDrv = drv; /* used by xf86OnetouchAllocate() */

   //fakeLocal = (InputInfoPtr) xcalloc(1, sizeof(InputInfo));
   //if (!fakeLocal) return NULL;

   //fakeLocal->conf_idev = dev;

   /* Force default serial port options to exist because the serial init
    * phasis is based on those values.
    */
   //xf86CollectInputOptions(fakeLocal, default_options, NULL);

   rc  = xf86OnetouchAllocate(pInfo);
   if (rc != Success) {
      xf86Msg(X_ERROR, "Can't allocate touchscreen data\n");
      goto SetupProc_fail;
   }

   priv = (OnetouchDevicePtr)(pInfo->private);
   pInfo->name = xf86SetStrOption(pInfo->options, "DeviceName", XI_TOUCHSCREEN);

   /* Device name is mandatory */
   priv->onetDevice = xf86FindOptionValue(pInfo->options, "Device");
   if (!priv->onetDevice) {
       xf86Msg(X_ERROR, "%s: `Device' not specified\n",
	       pInfo->name);
       goto SetupProc_fail;
   }

   xf86ProcessCommonOptions(pInfo, pInfo->options);
   priv->onetSmooth = xf86SetIntOption(pInfo->options, "Smoothness", 9);
   priv->onetJitterDelay = xf86SetIntOption(pInfo->options, "JitterDelay",
					   ONETOUCH_DEFAULT_JITTER_DELAY );
   priv->onetDisableDrag = xf86SetIntOption(pInfo->options, "DisableDrag", 0);
   priv->onetTappingDelay = xf86SetIntOption(pInfo->options, "TappingDelay",
					    ONETOUCH_DEFAULT_TAPPING_DELAY );
   debug_level = xf86SetIntOption(pInfo->options, "DebugLevel", 0);

   /* FIXME: configuration file */


   return Success;

   SetupProc_fail:
   if (priv)
      free(priv);
   return BadValue;
}

#ifdef XFree86LOADER
static
#endif
InputDriverRec ONETOUCH = {
    1,                          /* driver version */
    "onetouch",                    /* driver name */
    NULL,                       /* identify */
    xf86OnetInit,                /* pre-init */
    xf86OnetUninit,              /* un-init */
    NULL,                       /* module */
    0                           /* ref count */
};

/*
 ***************************************************************************
 *
 * Dynamic loading functions
 *
 ***************************************************************************
 */
#ifdef XFree86LOADER
/*
 * xf86OnetUnplug --
 *
 * called when the module subsection is found in XF86Config
 */
static void
xf86OnetUnplug(pointer   p)
{
    DBG(1, ErrorF("xf86OnetUnplug\n"));
}

/*
 * xf86OnetPlug --
 *
 * called when the module subsection is found in XF86Config
 */
static pointer
xf86OnetPlug(pointer     module,
            pointer     options,
            int         *errmaj,
            int         *errmin)
{
    DBG(1, ErrorF("xf86OnetPlug\n"));
	
    xf86AddInputDriver(&ONETOUCH, module, 0);

    return module;
}

static XF86ModuleVersionInfo xf86OnetVersionRec =
{
    "onetouch",
    "Alessandro Rubini and Automata S.p.A. (Cannon group)",
    MODINFOSTRING1,
    MODINFOSTRING2,
    XORG_VERSION_CURRENT,
    1, 2, 0,  /* VERSION */
    ABI_CLASS_XINPUT,
    ABI_XINPUT_VERSION,
    MOD_CLASS_XINPUT,
    {0, 0, 0, 0}		/* signature, to be patched into the file by */
				/* a tool */
};

XF86ModuleData onetouchModuleData = {&xf86OnetVersionRec,
				  xf86OnetPlug,
				  xf86OnetUnplug};

#endif /* XFree86LOADER */

/* end of xf86Onetouch.c */
