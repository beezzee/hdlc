#ifndef _utils_h
#define _utils_h

/**
   Read two consecutive bytes in little-endian format and return as uint16_t.
*/
#define uint16_from_little_endian(p) (*((uint16_t *) (p)))

#define uint16_to_little_endian(p,u)  (p)[0] = (0xFF & (u)) ; (p)[1]= ((u)>>8)

#endif //_utils_h
