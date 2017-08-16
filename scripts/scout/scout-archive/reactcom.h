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
#define RCT_POWER_OFF            1        /* turn power off */
#define RCT_REQUEST_TELEMETRY    2        /* send telemetry to socket */
#define RCT_CANCEL_TELEMETRY     3        /* turn telemetry off */
#define RCT_STARTUP              4        /* startup robot */
#define RCT_SHUTDOWN             5        /* shutdown robot */

/* state block */
#define RCT_SET_POSE             6        /* set (x,y,t) of robot */
#define RCT_SET_GOAL             7        /* set (gx,gy,gt) of robot */
#define RCT_CORRECT_POSE         8        /* send pose correction */
#define RCT_SET_SAFETY_REGION    9        /* set min. obstacle distance */
#define RCT_QUERY_POSE          10        /* return (x,y,t) of robot */
#define RCT_SET_BEHAVIOUR       11        /* set guidance behaviour */
#define RCT_QUERY_STATE         12        /* what is the current status? */

/* ldmark block */
#define RCT_ADD_LANDMARK        13        /* add landmark id (x,y,z) */
#define RCT_REMOVE_LANDMARK     14        /* remove a landmark */

/* these commands are not associated with any block */
#define RCT_CLEAR_MAP           15        /* clear sonar map */
#define RCT_QUERY_MAP           16        /* query sonar map */

/* image block */
#define RCT_ACQUIRE_IMAGE       17        /* take picture at (x,y,z) */
/*
 * note:  RCT_IMAGE_STATUS is now defined as RCT_GET_IMAGE_STATUS
 *        but has the same function.  This is done to match the
 *        addition of RCT_SET_IMAGE_STATUS below. 
 *        RCT_IMAGE_STATUS is still defined to make life easier
 */
#define RCT_IMAGE_STATUS        18        /* status of image buffer */
#define RCT_GET_IMAGE_STATUS    18        /* -- same -- */
#define RCT_RETRIEVE_IMAGE      19        /* pass image over link */
#define RCT_SNAP_IMAGE          20        /* digitize an image */

/* motor block */
#define RCT_HALT_MOTOR          21        /* stop all motors */
#define RCT_STEER_VEL           22        /* set steering velocity */
#define RCT_TRANS_VEL           23        /* set trans. velocity */
#define RCT_TURRET_VEL          24        /* set turret velocity */
#define RCT_LOCK_QUERY          25        /* is turret locked to steer? */
#define RCT_LOCK_TURRET         26        /* (un)lock turret to steering */

/* sensor block */
#define RCT_PING_SONAR          27        /* measure sonar */
#define RCT_GET_SONAR           28        /* return sonar */
#define RCT_CLEAR_SONAR         29        /* clear sonar values */
#define RCT_SONAR_SIZE          30        /* how many sonars? */
#define RCT_PING_INFRARED       31        /* measure INFRARED */
#define RCT_GET_INFRARED        32        /* return INFRARED */
#define RCT_CLEAR_INFRARED      33        /* clear INFRARED values */
#define RCT_INFRARED_SIZE       34        /* how many INFRAREDs? */
#define RCT_BUMPER_SIZE         35        /* bumper count */
#define RCT_GET_BUMPER          36        /* get bumper status */

/* ptu block */
#define RCT_SET_PANTILT         37        /* set pan-tilt registers */
#define RCT_GET_PANTILT         38        /* get pan-tilt registers */
#define RCT_GET_PT_MIN          39        /* get min values */
#define RCT_GET_PT_MAX          40        /* get max values */
#define RCT_PTU_CALIBRATE       41        /* calibrate PTU */

/* new image functions */
#define RCT_SET_IMAGE_STATUS    42        /* set various options for dig. */

/* options that can be set for an image are in the "imageopt" structure */

#define RCT_ACQUIRE_LDMARK      43        /* snap picture of landmark i */
#define RCT_START_IMBLOCK       44        /* send first packet of image */
#define RCT_NEXT_IMBLOCK        45        /* send next packet of image */
#define RCT_TRACK_TARGET        46        /* set landmark i for tracking */

/* new commands added to play with V4L */
# define RCT_SET_IMAGE_PARAM    47
# define RCT_GET_IMAGE_PARAM    48
/*
 * Various constants used in communication 
 */

#define RCT_NO_LANDMARK        (-1)   /* assumes all landmarks are >= 0 */

#define RCT_IMBLOCK              1    /* an imblock follows this message */
#define RCT_NO_IMBLOCK           0    /* no imblock follows this message */

/*
 * Image status values
 */
#define RCT_ISTAT_IDLE           0
#define RCT_ISTAT_BUSY           1
#define RCT_ISTAT_FAIL           2

typedef struct rct_sensor {
    real distance;
    real angle;
} ReactSensor;

typedef struct rct_bumper {
    int32 status;
    real angle;
} ReactBumper;

/* define the two response types */
#define ACK                  1
#define NACK                 0

/* define how long a hostname can be */
#define HOSTNAME_LENGTH     40

/* define states */
#define RCT_MOVING           0
#define RCT_GOAL             1
#define RCT_BLOCKED          2

typedef struct im_opt {
  int32 width, height, depth; /* size of image */
  int32 packet_size;
  real des_x, des_y, des_z;
  real cur_x, cur_y, cur_z;
  int32 status;
} ImageOptions;

typedef struct rmsg_struct {
    int32 magic_number; /* header value */
    int16 message;
    int16 response;
    union {
	struct {
	    /* ipadress, socket */
	    char host[HOSTNAME_LENGTH];
	    int16  port;
	} util;
	struct {
	    real pose_x, pose_y, pose_t;
	    real goal_x, goal_y, goal_t;
	    real d_x, d_y, d_t;
	    real radius;
	    int32 behaviour;
 	    int32 state;
	} state;
	struct {
	    int32 id;
	    real x,y,z;
	} ldmark;
	ImageOptions image;
	struct {
	    int32 width, height, depth; /* size and depth of image */
	    int32 packet_size;
	} imageopt;
	struct {
	    int32 status; /* status of xmit */
	    int32 packet_size; /* size of this packet */
	} imblock;
	struct {
	    real steer_accel, steer_vel;
	    real turret_accel, turret_vel;
	    real trans_accel, trans_vel;
	    int32 lock_flag;
	} motor;
	struct {
	    int32 size; 
	} sensor;
	struct {
	    real pan,tilt;
	} ptu;
        struct { /* added Mar 02 to support video parameters */
	  int32 bright;
	  int32 colour;
	  int32 contrast;
	  int32 hue;
	} imparam;
    } data;
} ReactMsg;

/* 
 * We've blended the real-time (non-blocking) and blocking
 * communications into the same call.
 *
 */
#define COMM_BLOCKING    0
#define COMM_NONBLOCKING 1

#define COMM_UNINIT    -1
#define COMM_IDLE       0
#define COMM_BUSY       1
#define COMM_ERROR      2  /* usually means socket has closed */

int rctInitialize( void ); /* does any necessary initialization */

int rctCreateSocket( char *host, int *port, int blockFlag );
int rctAcceptSocket( int sockFd, int blockFlag );
/* returns fd */
int rctOpenSocket( char *host, int port, int blockFlag );
int rctCloseSocket( int fd );
int rctSendMessage( int fd, ReactMsg * ); /* if comm is busy, send */
int rctRecMessage( int fd, ReactMsg * );
int rctSendBlock( int fd, void *, int32 );
int rctRecBlock( int fd, void *, int32 * );
int rctQueryComm( int fd );


/* functional interface - this is for the planner only and will
   probably use blocking I/O to get the job done. */
int rctPowerOff( void );
int rctRequestTelemetry( char *host, int port );
int rctCancelTelemetry( void );
int rctStartup( void );
int rctShutdown( void );

int rctSetPose( float x, float y, float t );
int rctSetGoal( float x, float y, float t );
int rctCorrectPose( float x, float y, float t,
		    float dx, float dy, float dt );
int rctSetSafetyRegion( float r );
int rctQueryPose( float *x, float *y, float *t );
int rctSetBehaviour( int );
int rctQueryState( int * );

int rctAddLandmark( int id, float x, float y, float z );
int rctRemoveLandmark( int );

int rctClearMap( void );
int rctQueryMap( SonarMap * );

int rctAcquireImage( float x, float y, float z );
int rctImageStatus( ImageOptions * );
int rctSnapImage( void );
int rctRetrieveImage( ImageBlock * );
int rctRetrieveIMBlocks( ImageBlock * );

int rctHaltMotor( float tr_ac, float st_ac, float tu_ac );
int rctSteerVel( float tr_ac, float st_ac, float tu_ac, float v );
int rctTurretVel( float tr_ac, float st_ac, float tu_ac, float v );
int rctTransVel( float tr_ac, float st_ac, float tu_ac, float v );
int rctLockQuery( int *lock );
int rctLockTurret( int lock );

int rctPingSonar( void );
int rctGetSonar( ReactSensor * );
int rctClearSonar( void );
int rctSonarSize( int * );
int rctPingInfrared( void );
int rctGetInfrared( ReactSensor * );
int rctClearInfrared( void );
int rctInfraredSize( int * );
int rctBumperSize( int * );
int rctGetBumper( ReactBumper * );

int rctSetPanTilt( float, float );
int rctGetPanTilt( float *, float * );
int rctGetPanTiltMin( float *, float * );
int rctGetPanTiltMax( float *, float * );
int rctPtuCalibrate( void );

int rctGetImageStatus( ImageOptions * );
int rctSetImageStatus(int width, int height, int depth,
		      int packet_size);
int rctAcquireLandmark(int index);
int rctStartIMBlock( void *buffer, int32 *size, int *result );
int rctNextIMBlock( void *buffer, int32 *size,  int *result );
int rctTrackTarget(int index);

# ifdef REORDER_BYTES
void reorderReactMsg( ReactMsg * );
void reorderInt32( int32 * );
void reorderInt16( int16 * );
void reorderReal( real * );
# endif /* REORDER_BYTES */

#endif /* REACTCOMM_H */
