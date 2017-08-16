/*
 * This file handles machine dependent stuff, like word sizes,
 * byte-ordering, various important header files.
 *
 * Only one architecture out of the list should be defined 
 *
 */

#ifndef ENVIRON_H
#define ENVIRON_H

#ifdef MSDOS
typedef short int16;
typedef long  int32;
typedef unsigned short uint16;
typedef unsigned long  uint32;
typedef float real;
#ifndef REORDER_BYTES
#define REORDER_BYTES
#endif /* REORDER_BYTES */
#endif /* MSDOS */

#ifdef UNIX
typedef short int16;
typedef long  int32;
typedef unsigned short uint16;
typedef unsigned long  uint32;
typedef float real;
#endif /* UNIX */

#endif /* ENVIRON_H */
