#ifndef COMMON_H

//~ Defines and prototypes
#define WAIT_FOR_BIT_SET(r, bit) while(((r) & (bit)) == 0)
#define WAIT_FOR_BIT_CLR(r, bit) while(((r) & (bit)) == (bit))
#define ArrayCount(arr) (sizeof(arr)/sizeof(arr[0]))
#define COMMON_H
#endif
