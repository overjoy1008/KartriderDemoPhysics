#ifndef KART_TRACK_TEXTURE_H
#define KART_TRACK_TEXTURE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The texture table scripts/pack_track_textures.py builds.

   A KTRK mesh already names the texture track.1s gave its material and carries
   the asset's own UVs per vertex; this is the missing half, the pixels, taken
   from the extracted theme archives. Names collide between themes, so a lookup
   is keyed by "<theme>/<mesh texture name>" - the same order the asset
   resolution in scripts/map_track_textures.ps1 uses.

   Images are stored small (the packer caps them, 64x64 by default) in RGB565
   with a one-bit coverage mask, which is what a software rasterizer can sample
   without a per-pixel multiply and what the cutout art needs. */

#define KART_TRACK_TEXTURE_VERSION 2u
#define KART_TRACK_TEXTURE_KEY_BYTES 112u

/* Bit 0: the source had an alpha channel, so `mask` is meaningful. */
#define KART_TRACK_TEXTURE_MASKED 1u
/* Bit 1: `alpha` holds the source's own 8-bit coverage. Only the kart images
   carry it: 0x00417160 composites them over a solid colour when the kart is
   built, which needs the real alpha rather than a cutout. */
#define KART_TRACK_TEXTURE_ALPHA8 2u
/* Bit 2: blend source alpha while rasterizing. The original skid-mark TGA
   needs this; kart skins keep their existing precomposited treatment. */
#define KART_TRACK_TEXTURE_BLEND_ALPHA8 4u

typedef struct KartTrackTextureImage {
    uint32_t width;
    uint32_t height;
    uint32_t flags;
    uint16_t *texels;
    /* One bit per texel, row major, bit (i & 7) of byte (i >> 3). */
    uint8_t *mask;
    /* One byte per texel, or NULL unless KART_TRACK_TEXTURE_ALPHA8 is set. */
    uint8_t *alpha;
} KartTrackTextureImage;

typedef struct KartTrackTextureEntry {
    char key[KART_TRACK_TEXTURE_KEY_BYTES];
    uint32_t image;
} KartTrackTextureEntry;

typedef struct KartTrackTextureTable {
    KartTrackTextureEntry *entries;
    uint32_t entry_count;
    KartTrackTextureImage *images;
    uint32_t image_count;
} KartTrackTextureTable;

bool kart_track_texture_load_memory(
    KartTrackTextureTable *table,
    const void *data,
    size_t size);

/* The KTXZ container: "KTXZ", the uint32 little-endian payload size, then the
   payload as a raw DEFLATE stream. Same shape as the scenes' KTKZ. */
bool kart_track_texture_load_compressed(
    KartTrackTextureTable *table,
    const void *data,
    size_t size);

void kart_track_texture_free(KartTrackTextureTable *table);

/* Returns NULL when the theme/name pair is not in the table, which is what
   happens for the handful of names the demo's archives never shipped. The
   entries are sorted by key, so this is a binary search. */
const KartTrackTextureImage *kart_track_texture_find(
    const KartTrackTextureTable *table,
    const char *theme,
    const char *name);

#endif
