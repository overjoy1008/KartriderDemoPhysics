#ifndef KART_TRACK_TEXTURE_RESOURCES_H
#define KART_TRACK_TEXTURE_RESOURCES_H

/* One shared table for all 13 tracks: the same texture name appears in several
   of them, so keying by theme and packing once keeps the resource to a single
   payload instead of thirteen overlapping ones. */
#define IDR_TRACK_TEXTURES 260

#endif
