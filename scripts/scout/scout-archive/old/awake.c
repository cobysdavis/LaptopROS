#include <stdio.h>
#include <stdlib.h>

#include "Nclient.h"

#define ROTATION_CONSTANT    0.118597  /* inches/degree (known to 100 ppm) */

#define RIGHT(trans, steer)  (trans + (int)((float)steer*ROTATION_CONSTANT))
#define LEFT(trans, steer)   (trans - (int)((float)steer*ROTATION_CONSTANT))

#define scout_vm(trans, steer)   vm(RIGHT(trans, steer), LEFT(trans, steer), 0)
#define scout_pr(trans, steer)   pr(RIGHT(trans, steer), LEFT(trans, steer), 0)
#define scout_ws(secs)           ws((unsigned char)1, (unsigned char)1, \
                                    (unsigned char)0, (unsigned char)secs)


void moveForward(float inches)
{
  scout_pr((int)(10 * inches), 0);
  scout_ws(255);
}

void rotate(float degrees)
{
  scout_pr(0, (int)(10 * degrees));
  scout_ws(255);
}

void enableSonars(void)
{
  int sn_order[16] = {0, 2, 15, 1, 14, 3, 13, 4, 12, 5, 11, 6, 10, 7, 9, 8}; 
  
  /* turn on the sonars in the specified order */
  conf_sn(15, sn_order);
  
  conf_tm(5); 
}

void readSonars(long *state)
{
  int i;

  for(i=STATE_SONAR_0;i<=STATE_SONAR_15;i++)
    state[i-STATE_SONAR_0] = State[i];
}

/* monitor the active sonar as specified in parameter "order" */
/* returns the sonar which detects the closest object and is less
   than min_dist */
short shortest_front_sonar(long *state, short min_dist)
{
  short i, shortest, shortest_sonar;
  
  shortest = 255;
  shortest_sonar = 0;
  
  i = 17;
  do {
    if (state[i] < shortest) {
      shortest = (short)state[i];
      shortest_sonar = i-16;
    }
    printf("%d: %d ", i-17, state[i]);
    i++;
    if (i == 20)
      i = 31;
  } while (i < 33);
  
  if (shortest > min_dist)
    shortest_sonar = 0;
  
  printf("\n");
  return(shortest_sonar);
}

void wander(short track_dist)
{
  short del_angle;
  short s_v = 0;
  short vmax = 100;
  short shortest_front;
  static short cwturncounter;
  static short ccwturncounter;
  static short lastdir;

  shortest_front = shortest_front_sonar(State, track_dist);
  
  if (shortest_front == 0) {
    scout_vm(vmax, 0);
    ccwturncounter =cwturncounter = lastdir = 0;
    return;
  }

  if (shortest_front < 5) {
    /* Check for getting stuck */
    if ((cwturncounter > 5) && (ccwturncounter > 5)) {
      shortest_front = (shortest_front + 3);
    } else
      shortest_front = -(shortest_front + 3);
    if (lastdir == 0)
      cwturncounter += 1;
    lastdir = 1;
  }
  if (shortest_front > 12) {
    shortest_front = shortest_front - 12;
    if (lastdir == 1)
      ccwturncounter += 1;
    lastdir = 0;
  }
  del_angle = shortest_front*225;
  s_v = del_angle/4;
  scout_vm(0, s_v);
}

/*
 * main
 */
main(void)
{
  short c, i;
  long sonars[16];
    
  /* set up */
  
  /* connect to the robot */
  printf("Connecting....\n");
  connect_robot(1, MODEL_SCOUT2, "/dev/ttyS0", 38400);
  moveForward(12.0);
  rotate(90.0);
  enableSonars();
  while(1){
    readSonars(sonars);
    for(i=0;i<16;i++)
      printf("%3d ",sonars[i]);
    printf("\n");
  }
  return(0);
}


