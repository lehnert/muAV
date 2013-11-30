
#ifndef __INV_USTORE_DELEGATE_IO_H__
#define __INV_USTORE_DELEGATE_IO_H__


#include "mltypes.h"

/* Public APIs: to be used by load store delegates */
inv_error_t inv_ustore_byte(unsigned char b);
inv_error_t inv_ustore_mem(const void *b, int length);

inv_error_t inv_uload_byte(unsigned char *b);
inv_error_t inv_uload_mem(void *b, int length);


#endif // __INV_USTORE_DELEGATE_IO_H__

