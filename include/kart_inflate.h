#ifndef KART_INFLATE_H
#define KART_INFLATE_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Raw DEFLATE decompression (RFC 1951), with no zlib or gzip wrapper.

   The track scenes are the only large payload the demos embed, so this keeps
   them compressed in the executable without pulling in an external library.

   Writes at most destination_size bytes and returns the number written, or 0
   if the stream is malformed, truncated, or would overrun the destination. */
size_t kart_inflate_raw(
    void *destination,
    size_t destination_size,
    const void *source,
    size_t source_size);

#ifdef __cplusplus
}
#endif

#endif
