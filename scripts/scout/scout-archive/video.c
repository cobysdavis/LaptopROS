# include <unistd.h>
# include <sys/types.h>
# include <sys/stat.h>
# include <fcntl.h>
# include <stdio.h>
# include <sys/ioctl.h>
# include <stdlib.h>

#include <linux/types.h>
#ifdef VIDEO
#include <linux/videodev.h>
#endif


# include "video.h"


# define DEV "/dev/video0"

static unsigned char vbuf[VIDEO_WIDTH * VIDEO_HEIGHT * 3];

/*
 * Initialize the video device. Return the file descriptor number or -1
 * on error.
 */
int videoInit()
{
# ifdef VIDEO
  int fd;
  struct video_capability cap;
  struct video_channel channel;
  struct video_window win;
  struct video_picture vpic;

  if((fd = open(DEV,O_RDONLY)) < 0){
    perror("Open of video card failed");
    return(-1);
  }
     
  if (ioctl(fd, VIDIOCGCAP, &cap) < 0) {
    perror("Unable to VIDEOCGAP the video device: not a video4linux device?");
    close(fd);
    return(-1);
  }
  printf("Video name %31s nchannels %d naudio %d max (%d,%d) min (%d,%d)\n",
	 cap.name,cap.channels,cap.audios,cap.maxwidth,cap.maxheight,
	 cap.minwidth,cap.minheight);
  if((VID_TYPE_CAPTURE & cap.type) != VID_TYPE_CAPTURE) {
    fprintf(stderr,"Video device does not capture. Cap is %d\n",cap.type);
    fprintf(stderr,"VID_TYPE_CAPTURE %d\n", VID_TYPE_CAPTURE);
    close(fd);
    return(-1); 
  }

  if (ioctl(fd, VIDIOCGWIN, &win) < 0) {
    perror("Unable to VIDIOCGWIN the video devce: camera not connected?");
    close(fd);
    return(-1);
  }

  printf("Video card is currently set to (%d,%d)\n",win.width,win.height);

  /* now set to our preferred size */
  win.width = VIDEO_WIDTH;
  win.height = VIDEO_HEIGHT;

  if (ioctl(fd, VIDIOCSWIN, &win) < 0) {
    perror("Unable to VIDIOCSWIN the video devce: camera not connected?");
    close(fd);
    return(-1);
  }

  if (ioctl(fd, VIDIOCGWIN, &win) < 0) {
    perror("Unable to VIDIOCGWIN the video devce: camera not connected?");
    close(fd);
    return(-1);
  }
  printf("Video card was reset to (%d,%d)\n",win.width,win.height);

  printf("Video card is currently set to (%d,%d)\n",win.width,win.height);
  if((win.width != VIDEO_WIDTH)||(win.height != VIDEO_HEIGHT)){
    fprintf(stderr,"video width %d height %d\n",win.width,win.height);
    close(fd);
    return(-1);
  }

  /* set the format that we want from the video system (8bits of r, g, b) */
  vpic.depth = 24;
  vpic.palette = VIDEO_PALETTE_RGB24;
  if(ioctl(fd, VIDIOCSPICT, &vpic) < 0) {
    perror("unable to set desired video picture format\n");
    close(fd);
    return(-1);
  }
  
  return(fd);
# else
  return(99);
# endif
}
  

/*
 * Shut down the video system cleanly.
 */
void videoQuit(int mfd)
{
# ifdef VIDEO
  if(mfd >= 0)
    close(mfd);
# endif
}

/*
 * take a picture
 */
int videoTakePic(int mfd)
{
# ifdef VIDEO
  return read(mfd, vbuf, VIDEO_WIDTH * VIDEO_HEIGHT * 3);
# else
  return 0;
# endif
}

/*
 * Return the offset to vbuf on which a specific block can be found.
 */
void *videoOffset(int blockid)
{
# ifdef VIDEO
  return((void *)&vbuf[blockid*VID_BLOCK_LEN]);
# else
  return 0;
# endif
}

/*
 * Adjust brightness. This little hack was stolen from vgrabber.c
 */
void videoAdjustBrightness(int mfd)
{
# ifdef VIDEO
  struct video_picture vpic;
  int i, count, offset;

  do {
    if(ioctl(mfd, VIDIOCGPICT, &vpic) < 0) {
      fprintf(stderr,"Unable to access mfd (adjustBrightness)\n");
      close(mfd);
      exit(1);
    }
    fprintf(stderr,"Video Adjust: current brightness %d\n",vpic.brightness);
    fprintf(stderr,"Video Adjust: current contrast %d\n",vpic.contrast);
    fprintf(stderr,"Video Adjust: current colour %d\n",vpic.colour);
    fprintf(stderr,"Video Adjust: current hue %d\n",vpic.hue);
    videoTakePic(mfd);

    count = 0;
    for(i=0;i<VIDEO_WIDTH*VIDEO_HEIGHT*3;i++)
      count += (unsigned char)vbuf[i];
    offset = 128 - count / (VIDEO_WIDTH*VIDEO_HEIGHT*3);
    fprintf(stderr,"Video adjust: offset is %d\n",offset);
    vpic.brightness += (offset << 8);

    if(ioctl(mfd, VIDIOCSPICT, &vpic) < 0) {
       fprintf(stderr,"Unable to access mfd (adjustBrightness)\n");
      close(mfd);
      exit(1);
    }
  } while((offset < -10)||(offset > 10));
# endif
}

void videoGetParam(int mfd, int *bright, int *contr, int *colour, int *hue)
{
# ifdef VIDEO
  struct video_picture vpic;
  if(ioctl(mfd, VIDIOCGPICT, &vpic) < 0) {
    fprintf(stderr,"Unable to access mfd (adjustBrightness)\n");
    close(mfd);
    exit(1);
  }
  *bright = vpic.brightness;
  *contr = vpic.contrast;
  *colour = vpic.colour;
  *hue = vpic.hue;
  printf("Bright %d\n", vpic.brightness);
  printf("Contrast %d\n", vpic.contrast);
  printf("Colour %d\n", vpic.colour);
  printf("Hue %d\n", vpic.hue);
# endif
}

void videoSetParam(int mfd, int bright, int contr, int colour, int hue)
{
# ifdef VIDEO
  struct video_picture vpic;
  if(ioctl(mfd, VIDIOCGPICT, &vpic) < 0) {
    fprintf(stderr,"Unable to access mfd (adjustBrightness)\n");
    close(mfd);
    exit(1);
  }
 vpic.brightness = bright; 
  vpic.contrast = contr;
  vpic.colour = colour;
  vpic.hue = hue;
  if(ioctl(mfd, VIDIOCSPICT, &vpic) < 0) {
    fprintf(stderr,"Unable to access mfd (adjustBrightness)\n");
    close(mfd);
    exit(1);
  }
# endif
}
