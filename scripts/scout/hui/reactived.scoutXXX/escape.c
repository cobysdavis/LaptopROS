# include <stdio.h>
# include <sys/fcntl.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <netinet/in.h>
# include <sys/mman.h>
# include <sys/time.h>
# include <unistd.h>
# include <math.h>
# include "Nclient.h"

# include "scout.h"

# include "escape.h"

/* # define DEBUG */
# define ESCAPED 0.20 /* minimum distance (in m) we will permit */
# define NSONARS 16   /* number of sonars on the machine        */

/* determine if we are blocked between sensor1 and sensor2 */
static int Blocked(float *sonars, long bumper, int sensor1, int sensor2,
				   float *closest)
{
	int i, max, result = 0;
	
	/* order sensors so we can start with sensor1 and end with sensor2 */
	if(sensor2 >= sensor1)
		max = sensor2;
	else
		max = sensor2 + NSONARS;
	
	/* loop through the sensor range, determine shortist distance */
	for(i=sensor1;i <= max; i++){
		if(sonars[i % NSONARS] < ESCAPED){
			if(!result)
				*closest = sonars[i % NSONARS];
			else if (sonars[i % NSONARS] < *closest )
				*closest = sonars[i % NSONARS];
			result = 1;
		}
	}
	
	/* if the bumper hits anywhere, it hits everywhere */
	if(bumper){
		*closest = ESCAPED;
		result = 1;
	}
	return result;
}

/* escape from close obstacles */
int escapeBehaviour(float *sonars, long bumper, float *escapeT, float *escapeD)
{
	int blockfore, blockaft, blockleft, blockright, result;
	float closest;
	int i;
	
	/* build a bitmask based on if we are blocked in each of four quadrants */
	blockfore = Blocked(sonars, bumper, 13, 3, &closest) << 3;
	blockaft = Blocked(sonars, bumper, 6, 10, &closest) << 2;
	blockleft = Blocked(sonars, bumper, 2, 6, &closest) << 1;
	blockright = Blocked(sonars, bumper, 10, 13, &closest);
	
	/* by default, do nothing */
	*escapeT = 0.0;
	*escapeD = 0.0;
	result = 0;
	
	/* based on the quadrants, decide what to do */
	switch(blockfore|blockaft|blockleft|blockright){
		case 0: /* clear sailing */
# ifdef DEBUG
			(void) fprintf(stderr,"escape: clear sailing\n");
# endif    
			break;
		case 1: /* blockright */
		case 13:/* blockfore|blockaft|blockright */
		case 9: /* blockfore|blockright */
			/* turn left 45 deg. and check again */
# ifdef DEBUG
			(void) fprintf(stderr,"escape: Turning left to avoid obstacle.\n");
# endif
			*escapeT = 90.0;
			result = 1;
			break;
		case 2: /* blockleft */
		case 12:/* blockfore|blockaft */
		case 14:/* blockfore|blockaft|blockleft */
		case 10:/* blockfore|blockleft */
# ifdef DEBUG
			(void) fprintf(stderr, "escape: Turning right to avoid obstacle.\n");
# endif
			*escapeT = -90.0;
			result = 1;
			break;
		case 3: /* blockright|blockleft */
# ifdef DEBUG
			(void) fprintf(stderr,"escape: Moving forward to avoid obstacle (3).\n");
# endif
			*escapeD = 0.10;
			result = 1;
			break;
		case 4: /* blockaft */
# ifdef DEBUG
			(void) fprintf(stderr,"escape: Moving forward to avoid obstacle (4).\n");
# endif
			*escapeD = 0.10;
			result = 1;
			break;
		case 5: /* blockaft|blockright */
# ifdef DEBUG
			(void) fprintf(stderr,"escape: Moving forward to avoid obstacle (5).\n");
# endif
			*escapeD = 0.10;
			result = 1;
			break;
		case 6: /* blockaft|blockleft */
# ifdef DEBUG
			(void) fprintf(stderr,"escape: Moving forward to avoid obstacle (6).\n");
# endif    
			*escapeD = 0.10;
			result = 1;
			break;
		case 7: /* blockaft|blockleft|blockright */
# ifdef DEBUG
			(void) fprintf(stderr,"escape: Moving forward to avoid obstacle (7).\n");
# endif
			*escapeD = 0.10;
			result = 1;
			break;
		case 8: /* blockfore */
		case 11:/* blockfore|blockleft|blockright */
# ifdef DEBUG
			(void) fprintf(stderr,"escape: Moving backward to avoid obstacle.\n");
# endif
			*escapeD = -0.10;
			result = 1;
			break;
		case 15:/* blockfore|blockaft|blockleft|blockright */
# ifdef DEBUG
			(void) fprintf(stderr,"escape: Completely blocked in.\n");
# endif
			result = 1;
			break;
		default:/* can't happen */
			(void) fprintf(stderr,"escape: invalid state\n");
			rob_abort("sounds/dontthink.au");
			/*NOTREACHED*/
	}
	return(result);
}
