# include <stdio.h>
# include "video.h"

unsigned char buf[VIDEO_WIDTH*VIDEO_HEIGHT*3];
unsigned char name[20];
main()
{
  int vfd, b, j, k, i, len;
  int contr, colour, hue, bright;
  unsigned char *ptr;
  FILE *fd;

  printf("vtest\n");
  if((vfd = videoInit()) < 0){
    printf("Unable to initialize video system\n");
    exit(1);
  }

  for(i=0;i<100;i++){
    videoGetParam(vfd, &bright, &contr, &colour, &hue);
    printf("bright    %d:",bright); scanf("%d",&bright);
    printf("contrast  %d:",contr); scanf("%d",&contr);
    printf("colour    %d:",colour); scanf("%d",&colour);
    printf("hue       %d:",hue); scanf("%d",&hue);
    printf("Taking a picture\n");
    videoSetParam(vfd, bright, contr, colour, hue);
    if((len = videoTakePic(vfd)) !=VIDEO_WIDTH*VIDEO_HEIGHT*3)
      printf("Short read ");
    printf("read %d bytes\n", len);
    printf("Done\n");
    for(b=0,j=0;j<VIDEO_WIDTH*VIDEO_HEIGHT*3;j+=VID_BLOCK_LEN,b++){
      ptr = videoOffset(b);
      for(k=0;k<VID_BLOCK_LEN;k++)
	buf[j+k] = *ptr++;
    }
    printf("We have our data\n");

    sprintf(name,"pic.ppm",i);
    if((fd = fopen(name,"w")) == NULL)
      printf("Unable to creat output file %s\n",name);
    else {
      fprintf(fd, "P6 %d %d 255\n", VIDEO_WIDTH, VIDEO_HEIGHT);
      fwrite(buf, VIDEO_WIDTH*VIDEO_HEIGHT*3, 1, fd);
      fclose(fd);
    }
    printf("written\n");
  }
      
    

  videoQuit(vfd);
}



