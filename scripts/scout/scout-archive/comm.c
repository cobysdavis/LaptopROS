# include <stdio.h>
# include <sys/time.h>
# include <sys/types.h>
# include <unistd.h>
# include <math.h>

# define UNIX
# include "reactcom.h"

# include "scout.h"
# include "video.h"

/* forward definitions */
static void processMsg(int, ReactMsg *);
static void reorderCommon(ReactMsg *);
static void reorderState(ReactMsg *);
static void reorderVideoOptions(ReactMsg *);
static void reorderVideoParams(ReactMsg *); 

void react(int msgsock) 
{
  fd_set rfds;
  struct timeval tv;
  int retval, len;
  ReactMsg msg;

  while(1){

    /* check for (and process) input on the file descriptor */
    FD_ZERO(&rfds);
    FD_SET(msgsock, &rfds);
    tv.tv_sec = tv.tv_usec = 0;
    if(retval = select(msgsock+1, &rfds, NULL, NULL, &tv)){
      if(retval < 0){
	perror("select (read)");
	return;
      }
      if((len = read(msgsock, &msg, sizeof(msg))) != sizeof(msg)){
 	fprintf(stderr,"read %d not %d bytes\n", len, sizeof(msg));
	perror("read (buffer too short)");
	shutdownHardware();
	return;
      }
  fprintf(stderr,"Prior to swap\n");
  fprintf(stderr,"Got magic number 0x%lx\n", msg.magic_number);
  fprintf(stderr,"Got message 0x%x\n",msg.message);
  fprintf(stderr,"Got response 0x%x\n", msg.response);
  fprintf(stderr,"Prior to swap\n");
      if(msg.magic_number == RCT_MAGIC_NUMBER)
	(void) fprintf(stderr, "reactived: swapping ok bytes!\n");
      reorderCommon(&msg);
      processMsg(msgsock, &msg);
    }

    /* do a time-slice for the robot controller */
    robotTimeSlice();
  }
}

static void processMsg(int msgsock, ReactMsg *msg)
{
  ReactMsg resp;
  float x, y, t;
  int s;
  int bright, colour, contrast, hue;

  /* fill return packet with (error) default */
  resp.magic_number = RCT_MAGIC_NUMBER;
  resp.response = NACK;

  fprintf(stderr,"Got magic number 0x%lx\n", msg->magic_number);
  fprintf(stderr,"Got message 0x%x\n",msg->message);
  fprintf(stderr,"Got response 0x%x\n", msg->response);

  /* process the message */
  if(msg->magic_number != RCT_MAGIC_NUMBER)
    fprintf(stderr,"reactived: Bad magic number 0x%x\n",msg->magic_number);
  fprintf(stderr,"Got message 0x%x\n", msg->message);
  switch(msg->message){
  case RCT_UNDEFINED_MESSAGE:
    fprintf(stderr,"reactived: undefined message received!\n");
    break;
  case RCT_POWER_OFF:
  case RCT_REQUEST_TELEMETRY:
  case RCT_CANCEL_TELEMETRY:
    break;
  case RCT_STARTUP: /* startup message */
    initializeHardware();
    fprintf(stderr,"reactived: received powerup message\n");
    resp.response = ACK;
    break;
  case RCT_SHUTDOWN: /* a clean shutdown, so do so */
    shutdownHardware();
    fprintf(stderr,"reactived: received shutdown message\n");
    resp.response = ACK;
    reorderCommon(&resp);
    if(write(msgsock, (void *)&resp, sizeof(ReactMsg)) != sizeof(ReactMsg)){
      perror("write");
      exit(1);
    }
    exit(0);
    break;
  case RCT_SET_POSE: /* set the robot's pose */
    reorderState(msg);
    setPose(msg->data.state.pose_x,
	    msg->data.state.pose_y,
	    msg->data.state.pose_t);
    setGoal(msg->data.state.pose_x,
	    msg->data.state.pose_y,
	    msg->data.state.pose_t);
    resp.response = ACK;
    break;
  case RCT_SET_GOAL: /* set the robot's goal location */
    reorderState(msg);
    setGoal(msg->data.state.goal_x,
	    msg->data.state.goal_y,
	    msg->data.state.goal_t);
    resp.response = ACK;
    break;
  case RCT_CORRECT_POSE:
  case RCT_SET_SAFETY_REGION:
    break;
  case RCT_QUERY_POSE: /* query the current pose of the robot */
    getPose(&x, &y, &t);
    resp.data.state.pose_x = x;
    resp.data.state.pose_y = y;
    resp.data.state.pose_t = t;
    reorderState(&resp);
    resp.response = ACK;
    break;
  case RCT_SET_BEHAVIOUR:
    break;
  case RCT_QUERY_STATE:
    getState(&s);
    resp.data.state.state = s;
    reorderState(&resp);
    resp.response = ACK;
    break;
  case RCT_ADD_LANDMARK:
  case RCT_REMOVE_LANDMARK:


  case RCT_CLEAR_MAP:
  case RCT_QUERY_MAP:

  case RCT_ACQUIRE_IMAGE:
  case RCT_IMAGE_STATUS:
   break;
  case RCT_RETRIEVE_IMAGE: 
    reorderVideoOptions(msg);
    s = msg->data.image.width;
    printf("Getting packet  %d\n",s);
    resp.response = ACK;
    reorderCommon(&resp); /* send packet end */
    if(write(msgsock, (void *)&resp, sizeof(ReactMsg)) != sizeof(ReactMsg)){
      perror("write");
      exit(1);
    }
    printf("writing video buffer %d\n",s);
    if(write(msgsock, videoOffset(s), VID_BLOCK_LEN) !=VID_BLOCK_LEN){
      perror("image row write");
      exit(1);
    }
    printf("wrote video buffer %d successfully (%d bytes)\n",s,VID_BLOCK_LEN);
    return; /* NB: Do *not* fall throgh to the bottom */
  case RCT_SNAP_IMAGE:
    snapPicture(&x, &y, &t);
    resp.data.state.pose_x = x;
    resp.data.state.pose_y = y;
    resp.data.state.pose_t = t;
    reorderState(&resp);
    resp.response = ACK;
    break;

  case RCT_HALT_MOTOR:
  case RCT_STEER_VEL:
  case RCT_TRANS_VEL:
  case RCT_TURRET_VEL:
  case RCT_LOCK_QUERY:
  case RCT_LOCK_TURRET:

  case RCT_PING_SONAR:
  case RCT_GET_SONAR:
  case RCT_CLEAR_SONAR:
  case RCT_SONAR_SIZE:
  case RCT_PING_INFRARED:
  case RCT_GET_INFRARED:
  case RCT_CLEAR_INFRARED:
  case RCT_INFRARED_SIZE:
  case RCT_BUMPER_SIZE:
  case RCT_GET_BUMPER:

  case RCT_SET_PANTILT:
  case RCT_GET_PANTILT:
  case RCT_GET_PT_MIN:
  case RCT_GET_PT_MAX:
  case RCT_PTU_CALIBRATE:

  case RCT_SET_IMAGE_STATUS:
  case RCT_ACQUIRE_LDMARK:
  case RCT_START_IMBLOCK:
  case RCT_NEXT_IMBLOCK:
  case RCT_TRACK_TARGET:
    break;
  case RCT_SET_IMAGE_PARAM:
    reorderVideoParams(msg);
    bright = msg->data.imparam.bright;
    contrast =  msg->data.imparam.contrast;
    colour = msg->data.imparam.colour;
    hue = msg->data.imparam.hue;
    robotSetVideoParam(bright, contrast, colour, hue);
    resp.response = ACK;
    break;
  case RCT_GET_IMAGE_PARAM:
    robotGetVideoParam(&bright, &contrast, &colour, &hue);
    resp.data.imparam.bright = bright;
    resp.data.imparam.contrast = contrast;
    resp.data.imparam.colour = colour;
    resp.data.imparam.hue = hue;
    reorderVideoParams(&resp);
    resp.response = ACK;
    printf("GET_PARAM %d %d %d %d\n",bright,contrast,colour,hue);
  }
  reorderCommon(&resp);
  if(write(msgsock, (void *)&resp, sizeof(ReactMsg)) != sizeof(ReactMsg)){
    perror("write");
    exit(1);
  }
}

/* do intel->sun hardware conversions */
static void reorderInt32(uint32 *l)
{
  uint32 k;
  k = (((*l)&0xff000000)>>24) | (((*l)&0x00ff0000)>>8)|
      (((*l)&0x0000ff00)<<8)  | (((*l)&0x000000ff)<<24);
  *l = k;
}

static void reorderInt16(uint16 *s)
{
  uint k;
  k = (((*s)&0xff00)>>8) | (((*s)&0xff)<<8);
  *s = k;
}

/* reorder video options fields */
static void reorderVideoParams(ReactMsg *p)
{
  reorderInt32((uint32 *) &(p->data.imparam.bright));
  reorderInt32((uint32 *) &(p->data.imparam.colour));
  reorderInt32((uint32 *) &(p->data.imparam.contrast));
  reorderInt32((uint32 *) &(p->data.imparam.hue));
}

/* reorder some filed of video */
static void reorderVideoOptions(ReactMsg *p)
{
  reorderInt32((uint32 *) &(p->data.image.width));
}

/* reorder common fields of message */
static void reorderCommon(ReactMsg *p)
{
  reorderInt32((uint32 *) &(p->magic_number));
  reorderInt16((uint16 *) &(p->message));
  reorderInt16((uint16 *) &(p->response));
}

/* reorder state block */
static void reorderState(ReactMsg *p)
{
  reorderInt32((uint32 *) &(p->data.state.pose_x));
  reorderInt32((uint32 *) &(p->data.state.pose_y));
  reorderInt32((uint32 *) &(p->data.state.pose_t));
  reorderInt32((uint32 *) &(p->data.state.goal_x));
  reorderInt32((uint32 *) &(p->data.state.goal_y));
  reorderInt32((uint32 *) &(p->data.state.goal_t));
  reorderInt32((uint32 *) &(p->data.state.d_x));
  reorderInt32((uint32 *) &(p->data.state.d_y));
  reorderInt32((uint32 *) &(p->data.state.d_t));
  reorderInt32((uint32 *) &(p->data.state.radius));
  reorderInt32((uint32 *) &(p->data.state.behaviour));
  reorderInt32((uint32 *) &(p->data.state.state));
}

    


