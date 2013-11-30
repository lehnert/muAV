/*
**  SNIPMATH.H - Header file for SNIPPETS math functions and macros
*/

#ifndef ISQRT_H
#define ISQRT__H


/*
**  File: ISQRT.C
*/

struct int_sqrt {
      unsigned sqrt,
               frac;
};

void usqrt(unsigned long x, struct int_sqrt *q);


#endif /* SNIPMATH__H */
