#ifndef KART_MINIMAP_WIN32_H
#define KART_MINIMAP_WIN32_H

#include <windows.h>

#include "kart_demo_data.h"

#include <stdbool.h>

#define KART_DEMO_MINIMAP_CAPACITY 32
#define KART_DEMO_MINIMAP_MARKER_SCALE 0.50f

typedef struct KartDemoMinimap {
    HBITMAP bitmap;
    unsigned int width;
    unsigned int height;
    RECT content_bounds;
    KartQuat camera_orientation;
    unsigned int camera_time_ms;
    bool camera_initialized;
} KartDemoMinimap;

typedef struct KartDemoMinimapSet {
    KartDemoMinimap entries[KART_DEMO_MINIMAP_CAPACITY];
} KartDemoMinimapSet;

void kart_demo_minimap_set_load(HINSTANCE instance, KartDemoMinimapSet *set);
void kart_demo_minimap_set_free(KartDemoMinimapSet *set);
KartDemoMinimap *kart_demo_minimap_for_track(
    KartDemoMinimapSet *set,
    const KartDemoTrackSpec *track);
void kart_demo_draw_minimap_bitmap(
    HDC dc,
    RECT destination,
    const KartDemoMinimap *minimap);
void kart_demo_draw_original_minimap_camera(
    HDC dc,
    RECT destination,
    KartDemoMinimap *minimap,
    const KartDemoTrackSpec *track,
    KartVec3 world_position,
    KartQuat world_orientation,
    unsigned int frame_time_ms,
    COLORREF marker_colour);
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
