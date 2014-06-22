/*
 * onetouch_to_ascii.c: a simple protocol decoder.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <limits.h>

#include <sys/types.h>
#include <sys/ioctl.h>

/* these are kernel-specific, but I need them */
#include <linux/types.h>
#include <asm/byteorder.h> /* to convert little-endian stuff */


#define OPEN_FLAGS		(O_RDWR  | O_NOCTTY | O_NONBLOCK)

#define TTY_IFLAG		(IGNBRK  |IGNPAR)
#define TTY_OFLAG		0
#define TTY_CFLAG		(CS8 | CREAD | CLOCAL)
#define TTY_LFLAG	        0
#define TTY_SPEED		B9600

static int configure(int fd)
{
    struct termios newTermIo;

    memset(&newTermIo, 0, sizeof(struct termios));
    
    newTermIo.c_iflag = TTY_IFLAG;
    newTermIo.c_oflag = TTY_OFLAG;
    newTermIo.c_cflag = TTY_CFLAG;
    newTermIo.c_lflag = TTY_LFLAG;
    newTermIo.c_cc[VMIN] = 1;
    newTermIo.c_cc[VTIME] = 0;

    if (cfsetispeed(&newTermIo, TTY_SPEED) < 0)
	return -1;
    if (tcsetattr(fd, TCSANOW, &newTermIo) < 0)
	return -1;
    tcflush(fd, TCIOFLUSH);
    fcntl(fd, F_SETFL, 0); /* clear ndelay */
    return 0;
}

__u16 buffer[3];
unsigned char *packet = (unsigned char *)buffer + 1; /* to align data */

int main(int argc, char **argv)
{
    int fd, i, j, x, y;
    int binary = 0;

    if (argc > 1 && !strcmp(argv[1],"-b")) {
	binary++; argv[1] = argv[2]; argc--;
    }

    if (argc != 2) {
	fprintf(stderr, "%s: Use \"%s [-b] <dev>\"\n", argv[0], argv[0]);
	exit(1);
    }
    if( (fd=open(argv[1], OPEN_FLAGS)) == -1){
	fprintf(stderr, "%s: %s: %s\n", argv[0], argv[1], strerror(errno));
	exit(1);
    }
    configure(fd);

    while (1) {
	i = read(fd, packet, 5);
	if (binary) {
	    printf("%i:",i);
	    for (j = 0; j < i; j++)
		printf(" %02x", packet[j]);
	    putchar('\n');
	    fflush(stdout);
	    continue;
	}
	/*
	 * not binary: try to decode meaningful ascii
	 */
	if (i &&packet[0] != 0xff) {
	    /* resync... */
	    for (j = 1; j < i; j++)
		if (packet[j] == 0xff) {
		    memcpy(packet, packet+j, 5-j);
		    i -= j;
		    break;
		}
	}
	/* complete a 3-byte packet */
	while (i < 3) i += read(fd, packet+i, 3-i);
	/* handle the 3-byte packet */
	if (i == 3 && !strncmp(packet, "\xff\xfe\xfe", 3)) {
	    printf("up\n");
	    fflush(stdout);
	    continue;
	}
	/* complete a 5-byte packet */
	while (i < 5) i += read(fd, packet+i, 5-i);
	/* print it */
	x = __le16_to_cpu(buffer[1]);
	y = __le16_to_cpu(buffer[2]);
	printf("%5i %5i (%04x, %04x)\n", x, y, x, y);
	fflush(stdout);
    }
    exit(0);
}



