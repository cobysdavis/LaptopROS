# include <stdio.h>
# include <sys/time.h>
# include <sys/types.h>
# include <unistd.h>
# include <math.h>

# define UNIX
# include "reactcom.h"

# include "scout.h"
# include "video.h"

//# include "Nclient.h"

/* forward definitions */
static void processMsg(int, ReactMsg *);
static void reorderCommon(ReactMsg *);
static void reorderState(ReactMsg *);
static void reorderVideoOptions(ReactMsg *);
static void reorderInt32(unsigned long *);

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
            //  shutdownHardware();   // changed by Hui. Don't shutdown so client can close without closing everything
                return;               // such a shutdown can be done now with RCT_SHUTDOWN command.
            }
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
  char sbuf[128];

  printf("pcrocessing -----------------------------------processing-------------------\n");
    
  /* fill return packet with (error) default */
  resp.magic_number = RCT_MAGIC_NUMBER;
  resp.response = NACK;
    
  /* process the message */
  if(msg->magic_number != RCT_MAGIC_NUMBER)
    fprintf(stderr,"reactived: Bad magic number 0x%x\n",msg->magic_number);
  switch(msg->message){
  case RCT_UNDEFINED_MESSAGE:
    fprintf(stderr,"reactived: undefined message received!\n");
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
    printf("setGOAL  GOAL -----------\n");
    reorderState(msg);
    setGoal(msg->data.state.goal_x,
	    msg->data.state.goal_y,
	    msg->data.state.goal_t);
    resp.response = ACK;
    break;
  case RCT_QUERY_POSE: /* query the current pose of the robot */
    printf("queryPOSE  POSE -----------\n");
    getPose(&x, &y, &t);
    resp.data.state.pose_x = x;
    resp.data.state.pose_y = y;
    resp.data.state.pose_t = t;
    reorderState(&resp);
    resp.response = ACK;
    break;
  case RCT_QUERY_STATE: /* query the current state of the robot */
    getState(&s);
    resp.data.state.state = s;
    reorderState(&resp);
    resp.response = ACK;
    break;
# ifdef DO_VIDEO
  case RCT_RETRIEVE_IMAGE:  /* retrieve block 's' from the current image */
    printf("retrieve IMAGE -----------\n");

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
  case RCT_SNAP_IMAGE: /* take a picture */
    printf("snapIMAGE  IMAGE -----------\n");
    
    snapPicture(&x, &y, &t);
 
    resp.data.state.pose_x = x;
    resp.data.state.pose_y = y;
    resp.data.state.pose_t = t;
    resp.response = ACK;
    reorderState(&resp);
    break;

  /* added 2 cases */
  case RCT_RETRIEVE_IMAGE_FLYCAP:  /* retrieve block 's' from the current image */
    printf("retrieve IMAGE --flycap---------\n");

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
 // if(write(msgsock, videoOffset(s), VID_BLOCK_LEN) !=VID_BLOCK_LEN){
    if(write(msgsock, videoOffsetFlyCap(s), VID_BLOCK_LEN) !=VID_BLOCK_LEN){
   //if(write(msgsock, &ptr, VID_BLOCK_LEN) !=VID_BLOCK_LEN){
      perror("image row write");
      exit(1);
    }
    printf("wrote video buffer %d successfully (%d bytes)\n",s,VID_BLOCK_LEN);
    return; /* NB: Do *not* fall throgh to the bottom */
  case RCT_SNAP_IMAGE_FLYCAP: /* take a picture */
    printf("snapIMAGE  IMAGE ---flycap--------\n");
    
    //snapPicture(&x, &y, &t);
    snapPictureFlyCap(&x, &y, &t);   // newly added function

    resp.data.state.pose_x = x;
    resp.data.state.pose_y = y;
    resp.data.state.pose_t = t;
    resp.response = ACK;
    reorderState(&resp);
    break;
# endif
  case RCT_GET_SONAR: /* get the current sonar data */
    resp.data.state.pose_x = x;
    resp.data.state.pose_y = y;
    resp.data.state.pose_t = t;
    resp.response = ACK;
    reorderState(&resp); /* reorder state data */
    reorderCommon(&resp); /* send packet end */
    if(write(msgsock, (void *)&resp, sizeof(ReactMsg)) != sizeof(ReactMsg)){
      perror("write");
      exit(1);
    }

    /* prepare data for the data packet to follow the header */
    for(s = 0; s < 16; s++)
      reorderInt32((unsigned long *) &sonars[s]);

    if(write(msgsock, (void *)sonars, (sizeof(float)*16)) != (sizeof(float)*16)){
      perror("write sonars");
      exit(1);
    }
    fprintf(stderr,"sonar packet sent..awaiting next message\n");
    return; /* NB: Do *not* fall throgh to the bottom */
  case RCT_PLAY_SOUND:
    resp.response = ACK;
    printf("got a play sound command\n");

    (void) sprintf(sbuf, "./sounds/aplay sounds/%s &", msg->data.sound.fname);
    system(sbuf);
    fprintf(stderr,"executing %s\n",sbuf);
    break;
  }

  reorderCommon(&resp);
  if(write(msgsock, (void *)&resp, sizeof(ReactMsg)) != sizeof(ReactMsg)){
    perror("write");
    exit(1);
  }
}

/* do intel->sun hardware conversions */
static void reorderInt32(unsigned long *l)
{
    unsigned long k;
    k = (((*l)&0xff000000)>>24) | (((*l)&0x00ff0000)>>8)|
        (((*l)&0x0000ff00)<<8)  | (((*l)&0x000000ff)<<24);
    *l = k;
}

static void reorderInt16(unsigned short *s)
{
    unsigned short k;
    k = (((*s)&0xff00)>>8) | (((*s)&0xff)<<8);
    *s = k;
}

/* reorder some filed of video */
static void reorderVideoOptions(ReactMsg *p)
{
    reorderInt32((unsigned long *) &(p->data.image.width));
}

/* reorder common fields of message */
static void reorderCommon(ReactMsg *p)
{
    reorderInt32((unsigned long *) &(p->magic_number));
    reorderInt16((unsigned short *) &(p->message));
    reorderInt16((unsigned short *) &(p->response));
}

/* reorder state block */
static void reorderState(ReactMsg *p)
{
    reorderInt32((unsigned long *) &(p->data.state.pose_x));
    reorderInt32((unsigned long *) &(p->data.state.pose_y));
    reorderInt32((unsigned long *) &(p->data.state.pose_t));
    reorderInt32((unsigned long *) &(p->data.state.goal_x));
    reorderInt32((unsigned long *) &(p->data.state.goal_y));
    reorderInt32((unsigned long *) &(p->data.state.goal_t));
    reorderInt32((unsigned long *) &(p->data.state.d_x));
    reorderInt32((unsigned long *) &(p->data.state.d_y));
    reorderInt32((unsigned long *) &(p->data.state.d_t));
    reorderInt32((unsigned long *) &(p->data.state.radius));
    reorderInt32((unsigned long *) &(p->data.state.behaviour));
    reorderInt32((unsigned long *) &(p->data.state.state));
}




