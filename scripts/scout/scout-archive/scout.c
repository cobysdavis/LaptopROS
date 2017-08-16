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
# include "video.h"
# include "escape.h"

/*
 * scout.c - The reactived system for the super scout. 
 *
 * Michael Jenkin, 1999.
 */

/* scout definitons from Ndirect.c */
#define ROTATION_CONSTANT    0.118597  /* inches/degree (known to 100 ppm) */

#define RIGHT(trans, steer)  (trans + (int)((float)steer*ROTATION_CONSTANT))
#define LEFT(trans, steer)   (trans - (int)((float)steer*ROTATION_CONSTANT))

#define scout_vm(trans, steer)   vm(RIGHT(trans, steer), LEFT(trans, steer), 0)
#define scout_pr(trans, steer)   pr(RIGHT(trans, steer), LEFT(trans, steer), 0)
#define scout_ws(secs)           ws((unsigned char)1, (unsigned char)1, \
                                    (unsigned char)0, (unsigned char)secs)

# define IN_PER_METER 39.37

/* position control loop nb: we have about a 0.30 sec cycle */
#define MAXPV        0.20 /* m/sec */
#define MAXPERR      0.02 /* m */

/* parameters for the orientation control loop */
#define MAXTV        20.0 /* deg/sec */
#define MAXTERR      5.0 /* deg */     

/* parameter for the exponential weighting function */
# define LAMBDA      0.1

#define DEBUG 


static int robotup = 0;        /* is the robot up (used to cycle hardware) */
static int mfd;                /* file descriptor for video system         */

/* define states (from reactcom.h) */
#define RCT_MOVING           0
#define RCT_GOAL             1
#define RCT_BLOCKED          2
/*
 * primitive operations for the superscout
 *
 */

/* set hardware pose information */
static void setHardwarePose(float x, float y, float theta)
{
  fprintf(stderr,"setHardwarePose %f %f %f\n", x, y, theta);

  /* set their (x,y) position */
  dp((int)(IN_PER_METER * 10.0 * x), (int)(IN_PER_METER * 10.0 * y));

  /* set their theta value */
  if((theta < -180.0)||(theta > 180.0))
     fprintf(stderr,"setHardwarePose - bad theta\n");
  else if(theta < 0)
    theta = 360  + theta;
  da((int)(theta * 10), 0);
}

/* get hardware pose information */
static void getHardwarePose(float *x, float *y, float *theta)
{
  /* map from their (x,y) to ours */
  *x = State[STATE_CONF_X] / (10.0 * IN_PER_METER);
  *y = State[STATE_CONF_Y] / (10.0 * IN_PER_METER);
  *theta = State[STATE_CONF_STEER] / 10.0;

  /* map from their theta to ours */
  if((*theta < 0)||(*theta > 360.0))
    fprintf(stderr,"gerHardwarePose - bad theta\n");
  else if(*theta > 180.0)
    *theta = *theta - 360;

# ifdef DEBUG
  fprintf(stderr,"getHardware %f %f %f (%d %d %d)\n",*x,*y,*theta,
	  State[STATE_CONF_X], State[STATE_CONF_Y], State[STATE_CONF_STEER]);
# endif
}

/* turn the sonars on */
static void enableSonars(void)
{
  int sn_order[16] = {0, 2, 15, 1, 14, 3, 13, 4, 12, 5, 11, 6, 10, 7, 9, 8}; 
  
  /* turn on the sonars in the specified order */
  conf_sn(15, sn_order);
  
  conf_tm(5); 
}

/* turn the sonars off */
static void disableSonars(void)
{
  int sn_order[16] = {255, 2, 15, 1, 14, 3, 13, 4, 12, 5, 11, 6, 10, 7, 9, 8}; 
  
  /* turn on the sonars in the specified order */
  conf_sn(15, sn_order);
  
  conf_tm(5); 
}

/* read the sonars */
static void readSonars(float *state)
{
  int i;

  for(i=STATE_SONAR_0;i<=STATE_SONAR_15;i++)
    state[i-STATE_SONAR_0] = State[i] / IN_PER_METER; /* in meters */
}


/* say something */
void say(char *path)
{
  FILE *audio, *fd;
  char buf[1024];
  int count;

  if((audio = fopen("/dev/audio", "w")) == NULL){
    (void) fprintf(stderr,"unable to open audio output stream\n");
    return;
  }

  if((fd = fopen(path, "r")) == NULL){
    (void) fprintf(stderr,"audio file %s not found\n", path);
    (void) fclose(audio);
    return;
  }

  while((count = fread((void *)buf, 1, 1024, fd)) > 0)
    (void) fwrite((void *)buf, 1, count, audio);
  
  (void) fclose(audio);
  (void) fclose(fd);
}
  


void initializeHardware(void)
{
  if(robotup){
    (void) fprintf(stderr,"reactived: trying to reinitialize robot\n");
    return;
  }

  /* initialize hardware */
  if(connect_robot(1, MODEL_SCOUT2, "/dev/ttyUSB0", 38400) == FALSE){
    (void) fprintf(stderr,"reactived: unable to connect to robot hardware\n");
    say("sounds/dontthink.au");
    exit(1);
  }
  enableSonars();

  /* put the robot in a sensible state */
  robot.state = RCT_MOVING;
  robot.x = goal.x = 0.0;
  robot.y = goal.y = 0.0;
  robot.theta = goal.theta = 0.0;
  setHardwarePose(robot.x, robot.y, robot.theta);

  /* initialize the video grabber */
  if((mfd = videoInit()) < 0){
    rob_abort("sounds/dontthink.au");
    /*NOTREACHED*/
  }

  /* and mark us as up */
  robotup = 1;
  say("sounds/wakeup.au");
}

/* cleanly shutdown the hardware */
void shutdownHardware(void)
{
  if(robotup){
    scout_vm(0,0); /* stop moving */
    disableSonars();
    disconnect_robot(1);
    videoQuit(mfd);
    robotup = 0;
    say("sounds/shutdown.au");
  }
}

/*
 * Deal with a time slice. Critical here is the maintenance of the 
 * cycleTime, how often do we actually get called.
 */

void robotTimeSlice(void)
{
  float sonars[16];
  int i;
  float escapeT, escapeD, seekT, seekD, finalT, finalD;
  int escape, seek;
  char buf[80];

  /* cycle time between slices */
  static int firstSlice = 1;
  static long last_s, last_us;
  static float slice;

  struct timeval tp;
  float dt, bestv;

  /* we are in innert piece of clay */
  if(!robotup)
    return;

  /* get current time */
  if(gettimeofday(&tp, (void *)NULL) < 0){
    (void) fprintf(stderr,"scout: gettimeofday failed\n");
    rob_abort("sounds/dontthink.au");
    /*NOTREACHED*/
  }

  /* compute cycle time */
  if(firstSlice){ /* first time through */
    firstSlice = 0;
    slice = 0.25; /* approximately correct ? */
  } else {
    dt = (float)((tp.tv_sec - last_s) + (tp.tv_usec - last_us) / 1000000.0);
    slice = dt * LAMBDA + (1-LAMBDA) * slice;
  }
  last_s = tp.tv_sec;
  last_us = tp.tv_usec;

  /* update our model from the robot hardware */
  gs();

  /* copy sonar data into local datatype */
  readSonars(sonars);

  /* update our state from the hardware */
  getHardwarePose(&(robot.x), &(robot.y), &(robot.theta));

  /* let the two experts decide what to do... */
  escape = escapeBehaviour(sonars, State[STATE_BUMPER], &escapeT, &escapeD);
  seek = seekBehaviour(sonars, State[STATE_BUMPER], &seekT, &seekD);

  /* and combine their outputs (absolute priority) */
  finalT = 0.0;
  finalD = 0.0;
  if(seek){
    finalT = seekT;
    finalD = seekD;
  }
  if(escape){
    finalT = escapeT;
    finalD = escapeD;
  }
  
  /* move the robot based on finalT or finalD */
  if(fabs(finalT) > MAXTERR){ /* rotate by finalT */
    if(fabs(finalT) > 190.0){
      fprintf(stderr,"Sanity check. Trying to rotate by %f\n",finalT);
      rob_abort("sounds/dontthink.au");
      /*NOTREACHED*/
    }

    /* compute best rotational velocity */
    bestv = finalT / slice;
    if(bestv > MAXTV) bestv = MAXTV;
    if(bestv < -MAXTV) bestv = -MAXTV;
# ifdef DEBUG
    printf("Rotation: state (%f,%f,%f) by %f vel %f for %f sec\n"
	   ,robot.x,robot.y,robot.theta, finalT, bestv, slice);
# endif
    scout_vm(0, (int)(10 * bestv));
    robot.state = RCT_MOVING;
  } else if(fabs(finalD) > MAXPERR){ /* translate by finalD */
    if(fabs(finalD) > 1.0){
      fprintf(stderr,"Sanity check. Trying to move by %f\n",finalD);
      rob_abort("sounds/dontthink.au");
      /*NOTREACHED*/
    }

    /* compute best translational velocity */
    bestv = finalD / slice;
    if(bestv > MAXPV) bestv = MAXPV;
    if(bestv < -MAXPV) bestv = -MAXPV;
# ifdef DEBUG
    printf("Translation: state (%f,%f,%f) by %f vel %f for %f sec\n"
	   ,robot.x,robot.y,robot.theta, finalD, bestv, slice);
# endif
    scout_vm((int)(10 * IN_PER_METER * bestv), 0);
    robot.state = RCT_MOVING;
  } else { /* we are at the goal (or close enough) */
    scout_vm(0,0); /* not moving */
    robot.state = RCT_GOAL;
  }
}


/* glue from comm */
void setPose(float x, float y, float theta)
{
# ifdef DEBUG
  fprintf(stderr,"setting pose to (%f,%f,%f)\n",x,y,theta);
# endif
  gs();
  setHardwarePose(x, y, theta);
  getHardwarePose(&(robot.x), &(robot.y), &(robot.theta));
}

void setGoal(float x, float y, float theta)
{
# ifdef DEBUG
  fprintf(stderr,"setting goal to (%f,%f,%f)\n",x,y,theta);
# endif
  goal.x = x;
  goal.y = y;
  goal.theta = theta;
}

void getPose(float *x, float *y, float *theta)
{
# ifdef DEBUG
  fprintf(stderr,"getting robot pose (%f,%f,%f)\n",
	  robot.x,robot.y,robot.theta);
# endif
  gs();
  getHardwarePose(&(robot.x), &(robot.y), &(robot.theta));
  *x = robot.x;
  *y = robot.y;
  *theta = robot.theta;
}

void snapPicture(float *x, float *y, float *theta)
{
  int dcmd;

# ifdef DEBUG
  fprintf(stderr,"getting picture at (%f,%f,%f)\n",
          robot.x, robot.y, robot.theta);
# endif
  gs();
  getHardwarePose(&(robot.x), &(robot.y), &(robot.theta));
  *x = robot.x;
  *y = robot.y;
  *theta = robot.theta;
  videoTakePic(mfd);
}

void robotGetVideoParam(int *bright, int *contrast, int *colour, int *hue)
{
  videoGetParam(mfd, bright, contrast, colour, hue);
}

void robotSetVideoParam(int bright, int contrast, int colour, int hue)
{
  videoSetParam(mfd, bright, contrast, colour, hue);
}


void getState(int *s)
{
# ifdef DEBUG
  fprintf(stderr,"getting robot state %d\n", robot.state);
# endif
  *s = robot.state;
}  

void rob_abort(char *s)
{
  scout_vm(0, 0); /* stop the robot */
  say(s);
  exit(1);
}
