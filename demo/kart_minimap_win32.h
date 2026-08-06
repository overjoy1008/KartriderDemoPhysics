#ifndef KART_MINIMAP_WIN32_H
#define KART_MINIMAP_WIN32_H

#include <windows.h>

#include "kart_demo_data.h"

#include <stdbool.h>

#define KART_DEMO_MINIMAP_CAPACITY 32

typedef struct KartDemoMinimap {
    HBITMAP bitmap;
    unsigned int width;
    unsigned int height;
    RECT content_bounds;
} KartDemoMinimap;

typedef struct KartDemoMinimapSet {
    KartDemoMinimap entries[KART_DEMO_MINIMAP_CAPACITY];
} KartDemoMinimapSet;

void kart_demo_minimap_set_load(HINSTANCE instance, KartDemoMinimapSet *set);
void kart_demo_minimap_set_free(KartDemoMinimapSet *set);
const KartDemoMinimap *kart_demo_minimap_for_track(
    const KartDemoMinimapSet *set,
    const KartDemoTrackSpec *track);
void kart_demo_draw_minimap_bitmap(
    HDC dc,
    RECT destination,
    const KartDemoMinimap *minimap);
POINT kart_demo_minimap_kart_point(
    RECT destination,
    const KartDemoMinimap *minimap,
    const KartDemoTrackSpec *track,
    KartVec3 position);
/* Same mapping for a track with no original minimap: places a world position
   inside a plain bounds rectangle using the axis flips below. */
POINT kart_demo_minimap_bounds_point(
    RECT destination,
    const KartDemoTrackSpec *track,
    KartVec3 position);
KartVec3 kart_demo_minimap_direction(
    const KartDemoTrackSpec *track,
    KartVec3 world_direction);

#endif
