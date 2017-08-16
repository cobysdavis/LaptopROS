/* include file for scout.c */

void initializeHardware(void);
void shutdownHardware(void);
void robotTimeSlice(void);
void setPose(float, float, float);
void setGoal(float, float, float);
void getPose(float *, float *, float *);
void snapPicture(float *, float *, float *);
void getState(int *);
void say(char *);
void rob_abort(char *);
void robotGetVideoParam(int *, int *, int *, int *);
void robotsetVideoParam(int, int, int, int);

struct point {
  float x, y, theta;
  int state;
} robot, goal;
