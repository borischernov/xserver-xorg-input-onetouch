/*
 * Trivial control program to get information and turn the device off/onn
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <X11/Xlib.h>
#include <X11/Xproto.h>
#include <X11/extensions/XI.h>
#include <X11/extensions/XInput.h>

#include "onetouch.h"

char *prgname;

static int devinfo(Display *disp, int arg)
{
    XExtensionVersion *ext;
    XDeviceInfo *info;
    int infolen, i;

    ext = XGetExtensionVersion(disp, INAME);
    printf("%s: %s", INAME, ext->present ? "Present" : "Absent");
    if (ext->present)
	printf(" (version %i.%i)", ext->major_version, ext->minor_version);
    printf("\n");
    XFree(ext);

    info = XListInputDevices(disp, &infolen);
    for (i=0; i<infolen; i++) {
	printf("Device %s (id %i): %i classes, use is %s\n",
	       info[i].name, (int)(info[i].id),
	       info[i].num_classes,
	       info[i].use == IsXKeyboard ? "keyboard" :
	       (info[i].use == IsXPointer ? "pointer" :
		(info[i].use == IsXExtensionDevice ? "extension" :
		 "unknown")));
    }
    XFreeDeviceList(info);
    return 0;
}

static int finddevice(Display *disp)
{
    XDeviceInfo *info;
    int infolen, i;

    info = XListInputDevices(disp, &infolen);
    for (i=0; i<infolen; i++) {
//	if (info[i].use != IsXExtensionDevice)
//	    continue;
	if (strstr(info[i].name, "touch"))
	    break;
	if (strstr(info[i].name, "Touch"))
	    break;
    }
    if (i == infolen)
	i = -1 /* not found */;
    else
	i = info[i].id;
    /* XFreeDeviceList(info); */
    return i;
}



/* another helper */
XLedFeedbackControl *findleds(Display *disp, int id, XDevice *dev,
			      XFeedbackState **stateptrptr)
{
    XFeedbackState *list, *ptr;
    int nfb, i;
    
    list = XGetFeedbackControl(disp, dev, &nfb);
    if (!list) return NULL;

    for (ptr = list, i = 0; i<nfb; i++) {
	if (ptr->class == LedFeedbackClass) {
	    /* let's assume there's only one */
	    *stateptrptr = list;
	    return (XLedFeedbackControl *)ptr;
	}
	ptr = (XFeedbackState *)((unsigned long)ptr + ptr->length);
    }
    return NULL;
}

/* get led status */
static int getleds(Display *disp, int arg)
{
    XDevice *dev;
    int id = finddevice(disp);
    XFeedbackState *list;
    XLedFeedbackControl *leds;

    if (id < 0) {
	fprintf(stderr, "%s: can't find a touch screen\n", prgname);
	return 1;
    }
    dev = XOpenDevice(disp, id);

    leds = findleds(disp, id, dev, &list);
    if (!leds) {
	fprintf(stderr, "%s: can't find feedback or can't find leds\n",
		prgname);
	return 1;
    }
    printf("leds: 0x%04x (mask 0x%04x)\n", leds->led_values, leds->led_mask);

    XFreeFeedbackList(list);
    return 0;
}

/* light the led and return */
static int doled(Display *disp, int arg)
{
    XDevice *dev;
    int id = finddevice(disp);
    XFeedbackState *list;
    XLedFeedbackControl *leds;

    if (id < 0) {
	fprintf(stderr, "%s: can't find a touch screen\n", prgname);
	return 1;
    }
    dev = XOpenDevice(disp, id);

    leds = findleds(disp, id, dev, &list);
    if (!leds) {
	fprintf(stderr, "%s: can't find feedback or can't find leds\n",
		prgname);
	return 1;
    }

    leds->led_values = arg;
    leds->led_mask = ~0;

    XChangeFeedbackControl(disp, dev, DvLed,
			   (XFeedbackControl *)leds);

    XFreeFeedbackList(list);
    return 0;
}

/* light the led and wait for it to get cleared */
int doledwait(Display *disp, int arg)
{
    XDevice *dev;
    int retval, id = finddevice(disp);
    XFeedbackState *list;
    XLedFeedbackControl *leds;

    if (id < 0) {
	fprintf(stderr, "%s: can't find a touch screen\n", prgname);
	return 1;
    }
    dev = XOpenDevice(disp, id);

    retval = doled(disp, arg);
    if (retval) return retval;

    while (1) {
	usleep(200 * 1000);

	leds = findleds(disp, id, dev, &list);
	if (!leds) {
	    fprintf(stderr, "%s: can't find feedback or can't find leds\n",
		    prgname);
	    return 1;
	}
	if ( !(leds->led_values & leds->led_mask & arg))	
	    break;

	XFreeFeedbackList(list);
    }
    return 0;
}


struct commands {
    char *name;
    int (*f)(Display *disp, int arg);
    int arg;
} commands[] = {
    {"devinfo",   devinfo,     0},
    {"getleds",   getleds,     0},
    {"on",        doled,       OTLED_RECALIBRATE},
    {"off",       doled,       OTLED_OFF},
    {"raw",       doled,       OTLED_UNCALIBRATE},
    {"b1",        doled,       OTLED_BUTTON1},
    {"b2",        doled,       OTLED_BUTTON2},
    {"b3",        doled,       OTLED_BUTTON3},
    {"b2once",    doledwait,   OTLED_BUTTON2ONCE},
    {"b3once",    doledwait,   OTLED_BUTTON3ONCE},
    {NULL,}
};

int main (int argc, char **argv)
{
    Display *disp;
    struct commands *cmd;
    int retval = 0;

    if (argc != 2) {
	fprintf(stderr, "%s: Use \"%s <cmd>\"\n", argv[0], argv[0]);
	fprintf(stderr, "   cmds:");
	for (cmd = commands; cmd->name; cmd++)
	    fprintf(stderr, " %s%c", cmd->name, cmd[1].name ? ',' : '.');
	fprintf(stderr, "\n");
	exit(1);
    }
    prgname = argv[0];

    /* Open display, once for all */
    disp = XOpenDisplay(NULL);
    if (!disp) {
	fprintf(stderr, "%s: can't open display\n", argv[0]);
	exit(1);
    }

    for (cmd = commands; cmd->name; cmd++)
	if (!strcmp(argv[1], cmd->name)) {
	    retval =  cmd->f(disp, cmd->arg);
	    break;
	}
    XCloseDisplay(disp);
    if (!cmd->name)
	return main(1, argv); /* recycle the error message */
    return retval;
}




