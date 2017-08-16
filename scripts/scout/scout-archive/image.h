#ifndef IMAGE_H
#define IMAGE_H

/* define image packet information */


/* define image types - assuming greyscale only for now */
#define IMAGE_GREY		0
#define IMAGE_GREY_RLE		1

typedef struct img_struct {
	int type; /* RLE doesn't make sense here */
	int width, height;
	unsigned char *data;
} Image;

typedef struct iblock_struct {
	long size;
	void *data;
} ImageBlock;

void BuildImage( Image *, int /* type */,
		 int /* width */, int /* height */,
		 char * /* data */ );
void KillImage( Image * );
int BuildBlockFromImage( Image *, ImageBlock *, int );
int BuildImageFromBlock( Image *, ImageBlock *, int, int );

#define Pixel(i,x,y) ((i)->data[(y)*(i)->width+(x)])

/* image block macros */
#define IBlockSize(i) ((i)->size)
#define IBlockType(i) ((int)*((char *)((i)->data)))
#define IBlockData(i) (&(((char *)((i)->data))[1]))

int EncodeRLE( unsigned char *, long, unsigned char *, long * );
int MeasureRLE( unsigned char *, long, long * );
int DecodeRLE( unsigned char *, unsigned char *, long );

#endif /* IMAGE_H */
