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

# include "seek.h"

/* Target Seeking Behaviour. Sector elimination method */
int  seekBehaviour(float *sonars, long bumper, float *seekT, float *seekD)
{
  float theta, dist, t;

# ifdef DEBUG
  printf("seekBehaviour: state (%f,%f,%f) goal (%f,%f,%f)\n",
	  robot.x,robot.y,robot.theta,goal.x,goal.y,goal.theta);
# endif

  /* work around fabs(0,0) problem on this box */
  if((fabs(goal.y-robot.y) < 0.01) && (fabs(goal.x-robot.x) < 0.01))
    theta = 0.0f;
  else
    theta  = atan2(goal.y - robot.y, goal.x - robot.x) * 180.0 / M_PI;
  dist = sqrt((goal.y - robot.y) * (goal.y - robot.y)+
              (goal.x - robot.x) * (goal.x - robot.x));

  /* close to goal. correct orientation */
  if(dist < 0.10){
    t = goal.theta - robot.theta;
    while(t > 180.0)
      t = t - 360.0;
    while(t < -180.0)
      t = t + 360.0;
    if(fabs(t) < 20.0) /* was 10 degrees */
      return(0); /* pointing right there */
    *seekT = t;
    *seekD = 0.0;
    return(1);
  }

  /* pointing at goal? */
  t = theta - robot.theta;
  /* go the short way around */
  if(t > 180.0)
    t = t - 360.0;
  if(t < -180.0)
      t = t + 360.0;
  if(fabs(t) > 10.0){ /* 10 degrees slop */
    *seekT = t;
    *seekD = 0.0f;
    return(1);
  } 

  /* move there */
  *seekT = 0.0f;
  *seekD = (dist > 0.5)? 0.5: dist; /* lets go a bit of the way */
  return(1);
}
