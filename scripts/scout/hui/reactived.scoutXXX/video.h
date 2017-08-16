# define VIDEO_WIDTH  640
# define VIDEO_HEIGHT 480
# define VIDEO_BLOCK  48
# define VID_BLOCK_LEN (VIDEO_WIDTH*3*VIDEO_BLOCK)

/* function prototypes */
int videoInit();
void videoQuit(int);
int videoTakePic(int);
void *videoOffset(int);
