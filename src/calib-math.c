/*
 * Copyright (c) 2003 Alessandro Rubini <rubini@linux.it>
 * Copyright (c) 2003 Federica Gaia <federica@gnudd.com>
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "calib-math.h"

struct tp { /* test point */
    int xe, ye; /* expected */
    int xm, ym; /* measured */
};

struct sol {
    double v[6];
};


/*
 * To calibrate we solve a linear system like this:
 *
 * X1 = Ax1 + By1 + C
 * X2 = Ax2 + By2 + C
 * X3 = Ax3 + By3 + C
 * Y1 = Dx1 + Ey1 + F
 * Y2 = Dx2 + Ey2 + F
 * Y3 = Dx3 + Ey3 + F
 * 
 * which can be represented as:
 * 
 *  x1  y1   1   0   0   0       A        X1
 *  x2  y2   1   0   0   0       B        X2
 *  x3  y3   1   0   0   0       C        X3
 *   0   0   0  x1  y1   1   x   D   =    Y1
 *   0   0   0  x2  y2   1       E        Y2
 *   0   0   0  x3  y3   1       F        Y3
 * 
 * This means, splitting:
 * 
 *  x1  y1   1     A        X1
 *  x2  y2   1  x  B   =    X2
 *  x3  y3   1     C        X3
 * 
 *  x1  y1   1     D        Y1
 *  x2  y2   1  x  E   =    Y2
 *  x3  y3   1     F        Y3
 * 
 * that means
 * 
 *        A
 * [M] x {B} = {X}
 *        C
 * 
 *        D
 * [M] x {E} = {Y}
 *        F
 * 
 * So:
 * 
 * A = det [MX1] / det [M]  
 * B = det [MX2] / det [M] 
 * C = det [MX3] / det [M]
 * 
 * D = det [MY1] / det [M]  
 * E = det [MY2] / det [M] 
 * F = det [MY3] / det [M]
 * 
 */

static double det(int a11, int a12, int a13,
		  int a21, int a22, int a23,
		  int a31, int a32, int a33)
{
    /* in the most brutal way, but make calculations as double ... */
    return (double)
	+ (double)a11 * (double)a22 * (double)a33
	+ (double)a12 * (double)a23 * (double)a31
	+ (double)a13 * (double)a21 * (double)a32
	- (double)a13 * (double)a22 * (double)a31
	- (double)a11 * (double)a23 * (double)a32
	- (double)a12 * (double)a21 * (double)a33;
}

int solve(struct sol *s, struct tp *p1, struct tp *p2, struct tp *p3)
{
    double detm;

    detm = det(p1->xm, p1->ym, 1,
	       p2->xm, p2->ym, 1,
	       p3->xm, p3->ym, 1);

    if (detm == 0.0)
	return -1;

    /* substitute the "xe" column into "xm, ym, 1" lines */
    s->v[0] = det(p1->xe, p1->ym, 1,
		  p2->xe, p2->ym, 1,
		  p3->xe, p3->ym, 1) / detm;

    s->v[1] = det(p1->xm, p1->xe, 1,
		  p2->xm, p2->xe, 1,
		  p3->xm, p3->xe, 1) / detm;

    s->v[2] = det(p1->xm, p1->ym, p1->xe,
		  p2->xm, p2->ym, p2->xe,
		  p3->xm, p3->ym, p3->xe) / detm;
	       
    /* substitute the "ye" column into "xm, ym, 1" lines */
    s->v[3] = det(p1->ye, p1->ym, 1,
		  p2->ye, p2->ym, 1,
		  p3->ye, p3->ym, 1) / detm;

    s->v[4] = det(p1->xm, p1->ye, 1,
		  p2->xm, p2->ye, 1,
		  p3->xm, p3->ye, 1) / detm;

    s->v[5] = det(p1->xm, p1->ym, p1->ye,
		  p2->xm, p2->ym, p2->ye,
		  p3->xm, p3->ym, p3->ye) / detm;

#if 0
    printf("%5i %5i (%5i %5i)\n", p1->xm, p1->ym, p1->xe, p1->ye);
    printf("%5i %5i (%5i %5i)\n", p2->xm, p2->ym, p2->xe, p2->ye);
    printf("%5i %5i (%5i %5i)\n", p3->xm, p3->ym, p3->xe, p3->ye);
    printf("%f %f %f    %f %f %f\n", s->v[0], s->v[1], s->v[2], s->v[3], s->v[4], s->v[5]);
#endif
    return 0;
}

/*
 * solve for an arbitrary number of test points, returning sigma, too.
 * The return value is the number of valid triplets
 */
static int solven(struct sol *s, struct sol *sigma, int ntp, struct tp *tp)
{
    struct sol tmpsol; int i;
    int c1, c2, c3; /* cursors */
    int retval = 0;

    if (ntp < 3)
	return 0;

    memset(s, 0, sizeof(*s));
    memset(sigma, 0, sizeof(*sigma));

    for (c1 = 0; c1 < ntp-2; c1++)
	for (c2 = c1+1; c2 < ntp-1; c2++)
	    for (c3 = c2+1; c3 < ntp; c3++) {
		/* run this triplet */
		i = solve(&tmpsol, tp+c1, tp+c2, tp+c3);
		if (i < 0) continue;
		for (i=0; i<6; i++) {
		    s->v[i] += tmpsol.v[i];
		    sigma->v[i] += tmpsol.v[i] * tmpsol.v[i];
		}
		retval++;
	    }
    if (retval == 0)
	return 0;
    for (i=0; i<6; i++) {
	/* average */
	s->v[i] /= (double)retval;

	/*
	 * sigma^2 = sum((i-avg)^2)/(n-1)
	 * sigma^2 * (n-1) = sum(i^2) + sum(avg^2) - 2 * avg * sum(i)
	 */
	sigma->v[i] = sigma->v[i] /* sum of squares */
	    + retval * s->v[i] * s->v[i]   /* sum of square of avg */
	    - 2 * s->v[i] * (retval * s->v[i]);
	sigma->v[i] = sqrt(sigma->v[i] / retval);
    }
    return retval;
}

/*
 * Expected test points, based in a 1000x1000 square
 * NOTE: These must be the same points as in the calibration program
 */

int tpe[5][2] = { /* expected */
    {125, 875},
    {875, 125},
    {125, 125},
    {875, 875},
    {500, 300}
};


/* this function is not used by main(), but is used by the XFree module */
int official_5p_calibration(int wid, int hei, /* screen size */
			    int *numbers, double *solutions, double *precision)
{
    int i;
    struct sol avg;
    struct sol sigma;
    struct tp tp[5];

    for (i=0; i<5; i++) {
	tp[i].xe = tpe[i][0]*wid/1000; tp[i].xm = numbers[2*i+0];
	tp[i].ye = tpe[i][1]*hei/1000; tp[i].ym = numbers[2*i+1];
    }
    i = solven(&avg, &sigma, 4, tp);
    if (i<1) return -1;

#if 0 /* to be completed */
    /* turn the sigma into error percent, get worst case */
    *precision = ....;

#endif

    for (i=0; i<6; i++) {
	solutions[i] = avg.v[i];
    }
    return 0;
}

/* You can define MAIN to compile this as standalone */
#ifdef MAIN
int main (int argc, char **argv)
{
    struct tp tp[5]; /* test points */
    struct sol avg;
    struct sol sigma;
    int i, tpc = 0; /* tp count */

    if (argc < 7 || argc > 11 || (argc%2)!=1) {
	fprintf(stderr, "Wrong number of arguments\n");
	exit(1);
    }
    argv++, argc--;
    while (argc > 1) {
	tp[tpc].xe = tpe[tpc][0];
	tp[tpc].ye = tpe[tpc][1];
	tp[tpc].xm = atoi(argv[0]);
	tp[tpc].ym = atoi(argv[1]);
	printf("%3i %3i    %3i %3i\n", tp[tpc].xe, tp[tpc].ye,
	       tp[tpc].xm, tp[tpc].ym);
	tpc++;
	argv += 2;
	argc -= 2;
    }
    i = solven(&avg, &sigma, tpc, tp);
    printf("solved for %i triplets\n", i);
    for (i=0; i<6; i++) 
	printf("%c: %f (%f)\n", "ABCDEF"[i], avg.v[i], sigma.v[i]);

    exit(0);
}
#endif /* MAIN */

#ifdef MAIN2
int main (int argc, char **argv)
{
    int numbers[10];
    double solutions[6];
    double precision; /* unused by now */
    int i;

    if (argc != 11) exit(1);
    for (i=0; i<10; i++)
	numbers[i] = atoi(argv[i+1]);

    i = official_5p_calibration(1<<16, 1<<16, numbers, solutions, &precision);

    if (i) {fprintf(stderr, "calibration invalid\n"); exit(1);}

    printf("Calibration data valid\n");
    printf("Calibration: X = %f x + %f y + %f\n",
	   solutions[0], solutions[1], solutions[2]);
    printf("Calibration: Y = %f x + %f y + %f\n",
	   solutions[3], solutions[4], solutions[5]);

    exit(0);
}
#endif
