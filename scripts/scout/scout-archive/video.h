# define VIDEO_WIDTH  160
# define VIDEO_HEIGHT 120
# define VIDEO_BLOCK  16
# define VID_BLOCK_LEN (VIDEO_WIDTH*3*VIDEO_BLOCK)

/* function prototypes */
int videoInit();
void videoQuit(int);
int videoTakePic(int);
void *videoOffset(int);
void videoAdjustBrightness(int);
void videoSetParam(int, int, int, int, int);
void videoGetParam(int, int *, int *, int *, int *);
