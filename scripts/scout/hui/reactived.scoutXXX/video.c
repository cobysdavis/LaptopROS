# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include <fcntl.h>
# include <errno.h>
# include <sys/stat.h>
# include <sys/types.h>
# include <sys/mman.h>
# include <sys/ioctl.h>
# include <linux/videodev2.h>

 # ifdef DEBUGGING
 # include <jpeglib.h>
 # endif

# define CLEAR(x) memset (&(x), 0, sizeof (x))
# define NBUFFERS 4

# include "video.h"
# define VIDEO_DEV "/dev/video0"

struct buffer {
        void *                  start;
        size_t                  length;
};
static struct buffer *buffers = NULL;

static unsigned char vidbuf[VIDEO_WIDTH*VIDEO_HEIGHT*3];

/**
  Convert from YUV422 format to RGB888. Formulae are described on http://en.wikipedia.org/wiki/YUV

  \param width width of image
  \param height height of image
  \param src source
  \param dst destination
*/
static void YUV422toRGB888(int width, int height, unsigned char *src, unsigned char *dst)
{
  int line, column;
  unsigned char *py, *pu, *pv;
  unsigned char *tmp = dst;

  /* In this format each four bytes is two pixels. Each four bytes is two Y's, a Cb and a Cr. 
     Each Y goes to one of the pixels, and the Cb and Cr belong to both pixels. */
  py = src;
  pu = src + 1;
  pv = src + 3;

  #define CLIP(x) ( (x)>=0xFF ? 0xFF : ( (x) <= 0x00 ? 0x00 : (x) ) )

  for (line = 0; line < height; ++line) {
    for (column = 0; column < width; ++column) {
      *tmp++ = CLIP((double)*py + 1.402*((double)*pv-128.0));
      *tmp++ = CLIP((double)*py - 0.344*((double)*pu-128.0) - 0.714*((double)*pv-128.0));      
      *tmp++ = CLIP((double)*py + 1.772*((double)*pu-128.0));

      // increase py every time
      py += 2;
      // increase pu,pv every second time
      if ((column & 1)==1) {
        pu += 4;
        pv += 4;
      }
    }
  }
}

/*
 * Initialize the video. This is mostly stolen from capture.c a
 * sample v4l2 capture program. Sorry. Its a little conservative, but
 * there is nothing wrong with that.
 *
 * returns -1 on error, the fd on success
 */
int videoInit() {
  int fd;
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  struct v4l2_requestbuffers req;
  enum v4l2_buf_type type;
  struct stat st; 
  unsigned int n_buffers;
  int i;

  if(stat (VIDEO_DEV, &st) < 0) {
    fprintf (stderr, "Cannot identify '%s': %d", VIDEO_DEV, errno);
    return -1;
  }

  if (!S_ISCHR (st.st_mode)) {
    fprintf (stderr, "%s is no device\n", VIDEO_DEV);
    return -1;
  }

  if((fd = open (VIDEO_DEV, O_RDWR /* required */ | O_NONBLOCK, 0)) < 0) {
    fprintf (stderr, "Cannot open '%s': %d\n", VIDEO_DEV, errno);
    return -1;
  }

  if(ioctl (fd, VIDIOC_QUERYCAP, &cap) < 0) {
    fprintf (stderr, "Unable to query %s as a V4L2 device\n", VIDEO_DEV);
    (void) close(fd);
    return -1;
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf (stderr, "%s is no video capture device\n", VIDEO_DEV);
    (void) close(fd);
    return -1;
  }

  /* only use memory mapped video modes */
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    fprintf (stderr, "%s does not support streaming i/o\n", VIDEO_DEV);
    (void) close(fd);
    return -1;
  }

  /* set cropping to default (ignore if we can't) */
  CLEAR (cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(!ioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */
    (void) ioctl (fd, VIDIOC_S_CROP, &crop);
  }

  /* format as 640x480 */
  CLEAR (fmt);
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = VIDEO_WIDTH; 
  fmt.fmt.pix.height      = VIDEO_HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
  if(ioctl (fd, VIDIOC_S_FMT, &fmt) < 0) {
    fprintf (stderr, "%s unable to set video format\nn", VIDEO_DEV);
    (void) close(fd);
    return -1;
  }

  /* initialize memory mapping */
  CLEAR (req);
  req.count               = NBUFFERS;
  req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory              = V4L2_MEMORY_MMAP;
  if(ioctl (fd, VIDIOC_REQBUFS, &req) < 0) {
    fprintf(stderr,"Failure in memory mapping %s\n", VIDEO_DEV);
    (void) close(fd);
    return -1;
  }

  if (req.count < 2) {
    fprintf (stderr, "Insufficient memory (memory map)\n");
    (void) close(fd);
    return -1;
  }
  
  if((buffers = calloc (NBUFFERS, sizeof (*buffers))) == NULL) {
    fprintf (stderr, "Out of memory\n");
    (void) close(fd);
    return -1;
  }


  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = n_buffers;

    if (ioctl (fd, VIDIOC_QUERYBUF, &buf) < 0) {
      fprintf (stderr, "Unable to query mmap buffer status\n");
      (void) close(fd);
      return -1;
    }

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start =
      mmap (NULL /* start anywhere */,
	    buf.length,
	    PROT_READ | PROT_WRITE /* required */,
	    MAP_SHARED /* recommended */,
	    fd, buf.m.offset);

    if (MAP_FAILED == buffers[n_buffers].start) {
      fprintf (stderr, "Memory mapping failed\n");
      (void) close(fd);
      return -1;
    }
  }
  

  /* start capturing */
  for (i = 0; i < n_buffers; ++i) {
    struct v4l2_buffer buf;

    CLEAR (buf);
    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = i;

    if (ioctl (fd, VIDIOC_QBUF, &buf) < 0) {
      fprintf(stderr,"Queing buffers failed\n");
      (void) close(fd);
      return -1;
    }
  }
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if(ioctl (fd, VIDIOC_STREAMON, &type) < 0) {
    fprintf(stderr,"Starting capture failed\n");
    (void) close(fd);
    return -1;
  }
  return fd;
}


 void videoQuit(int fd) {
   enum v4l2_buf_type type;
   type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   if(ioctl (fd, VIDIOC_STREAMOFF, &type) < 0) {
     fprintf(stderr,"Unable to stop capturing\n");
   }
   (void) close(fd);
 }

int videoTakePic(int fd) { 
  struct v4l2_buffer buf;
  fd_set fds;
  struct timeval tv;
  int r;
  unsigned int i;


  for (;;) {
    FD_ZERO (&fds);
    FD_SET (fd, &fds);
    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    fprintf(stderr, "Waiting for camera\n");
    if(select (fd + 1, &fds, NULL, NULL, &tv) < 0) {
      if(errno == EINTR)
        continue;
    }
    break;

  }
  fprintf(stderr,"camera is ready to read\n");


  CLEAR (buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  /* find the most recent buffer */
  while(1) {
    if(ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
      if(errno == EAGAIN)
        break;
      else
        return -1;
    }
    fprintf(stderr,"discarding buf index %d\n", buf.index);
    if(ioctl (fd, VIDIOC_QBUF, &buf) < 0) {
      fprintf(stderr,"Unable to re-queue buffer\n");
      return -1;
    }
  }

  /* now get the next buffer */
  while(1) {
    if(ioctl (fd, VIDIOC_DQBUF, &buf) < 0) {
      if(errno != EAGAIN)
	return -1;
    } else
      break;
  }

  fprintf(stderr,"buf index %d\n", buf.index);
  fprintf(stderr,"Buffer starts at 0x%x\n", (unsigned int) buffers[buf.index].start);
  fprintf(stderr,"Buffer has size %d\n", buffers[buf.index].length);

  YUV422toRGB888(VIDEO_WIDTH, VIDEO_HEIGHT, buffers[buf.index].start, vidbuf);

  if(ioctl (fd, VIDIOC_QBUF, &buf) < 0) {
    fprintf(stderr,"Unable to re-queue buffer\n");
    return -1;
  }
  return 0;
}

void *videoOffset(int blockid) 
{ 
  return((void *)&vidbuf[blockid*VID_BLOCK_LEN]);
}

 # ifdef DEBUGGING

/**
  Write image to jpeg file.

  \param img image to write
*/
static void jpegWrite(unsigned char *jpegFilename, unsigned char* img)
{
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
	
  JSAMPROW row_pointer[1];
  FILE *outfile = fopen( jpegFilename, "w" );

  // try to open file for saving
  if (!outfile) {
    fprintf(stderr,"Unable to write output file %s\n", jpegFilename);
    return;
  }

  // create jpeg data
  cinfo.err = jpeg_std_error( &jerr );
  jpeg_create_compress(&cinfo);
  jpeg_stdio_dest(&cinfo, outfile);

  // set image parameters
  cinfo.image_width = VIDEO_WIDTH;	
  cinfo.image_height = VIDEO_HEIGHT;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;

  // set jpeg compression parameters to default
  jpeg_set_defaults(&cinfo);
  // and then adjust quality setting
  jpeg_set_quality(&cinfo, 70, TRUE);

  // start compress 
  jpeg_start_compress(&cinfo, TRUE);

  // feed data
  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer[0] = &img[cinfo.next_scanline * cinfo.image_width *  cinfo.input_components];
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }

  // finish compression
  jpeg_finish_compress(&cinfo);

  // destroy jpeg data
  jpeg_destroy_compress(&cinfo);

  // close output file
  fclose(outfile);
}

  main()
{
  FILE *f;
  char buf[100];
  int i;
  int fd = videoInit();
  for(i=0;i<10;i++) {
    videoTakePic(fd);
    sprintf(buf,"file%d.jpg",i);
    jpegWrite(buf, vidbuf); 
  }
  videoQuit(fd);
}
  # endif
