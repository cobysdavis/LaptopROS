#ifndef REACTCOMM_H
#define REACTCOMM_H

/*
 * Update:  Jan. 30, 1995
 *
 * packet modified to include magic constant as its header.
 * This is there to ensure that garbage is not treated like
 * a packet.  It might also be helpful in the future, if
 * we need to try resyncing.
 *
 * MR
 */

/*
 * environ.h defines types: int16, int32, uint16, uint32, and real
 * and defines "REORDER_BYTES" based on the current architecture.
 *
 */
#include "environ.h"
#include "dstruct.h" /* for sonar map type */
#include "image.h" /* for image type */

/* define magic number constant */
#define RCT_MAGIC_NUMBER 0x12345678

/* define message types */
/* don't define message # 0 - an undefined message */
#define RCT_UNDEFINED_MESSAGE    0        /* indicates an error */

/* util block */
#define RCT_STARTUP              4        /* startup robot */
#define RCT_SHUTDOWN             5        /* shutdown robot */

/* state block */
#define RCT_SET_POSE             6        /* set (x,y,t) of robot */
#define RCT_SET_GOAL             7        /* set (gx,gy,gt) of robot */
#define RCT_QUERY_POSE          10        /* return (x,y,t) of robot */
#define RCT_QUERY_STATE         12        /* what is the current status? */

#define RCT_RETRIEVE_IMAGE      19        /* pass image over link */
#define RCT_SNAP_IMAGE          20        /* digitize an image */


#define RCT_RETRIEVE_IMAGE_FLYCAP      275 //19        /* pass image over link */
#define RCT_SNAP_IMAGE_FLYCAP       276 //20        /* digitize an image */



/* motor block */
#define RCT_HALT_MOTOR          21        /* stop all motors */
#define RCT_STEER_VEL           22        /* set steering velocity */
#define RCT_TRANS_VEL           23        /* set trans. velocity */
#define RCT_TURRET_VEL          24        /* set turret velocity */
#define RCT_LOCK_QUERY          25        /* is turret locked to steer? */
#define RCT_LOCK_TURRET         26        /* (un)lock turret to steering */

/* sensor block */
#define RCT_GET_SONAR           28        /* return sonar */

/* yet more commands */
# define RCT_PLAY_SOUND         49

/* define the two response types */
#define ACK                  1
#define NACK                 0

/* define how long a hostname can be */
#define HOSTNAME_LENGTH     40

/* define states */
#define RCT_MOVING           0
#define RCT_GOAL             1
#define RCT_BLOCKED          2

typedef struct rmsg_struct {
  int32 magic_number; /* header value */
  int16 message;
  int16 response;
  union {
    struct {
      real pose_x, pose_y, pose_t;
      real goal_x, goal_y, goal_t;
      real d_x, d_y, d_t;
      real radius;
      int32 behaviour;
      int32 state;
    } state;
    struct {
      int32 width, height, depth; /* size of image */
    } image;
    struct {
      unsigned char fname[48];
    } sound;
    struct {
      unsigned char filler[48];
    } filler;
  } data;
} ReactMsg;


#endif /* REACTCOMM_H */
