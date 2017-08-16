#ifndef DSTRUCT_H
#define DSTRUCT_H

/* something ridiculous */
#define SONAR_MAP_SIZE 30
#define MAX_IMPORTANCE 1.0e10

#include "environ.h"

typedef struct smap_elem {
    float x,y; /* location */
    float importance; /* rating (higher is better) */
} SonarMapElem;

typedef struct smap_struct {
    int32 size;
    SonarMapElem data[SONAR_MAP_SIZE];
} SonarMap;

#endif /* DSTRUCT_H */
