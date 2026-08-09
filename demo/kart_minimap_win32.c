#define COBJMACROS
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <objbase.h>
#include <wincodec.h>

#include "kart_minimap_win32.h"
#include "kart_minimap_resources.h"
#include "kart_camera.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const int MINIMAP_RESOURCE_IDS[] = {
    0, /* flat_test: synthetic, no original minimap */
    IDR_MINIMAP_DESERT_I01,
    IDR_MINIMAP_DESERT_I02,
    IDR_MINIMAP_DESERT_R01,
    IDR_MINIMAP_FOREST_I01,
    IDR_MINIMAP_FOREST_I02,
    IDR_MINIMAP_FOREST_R02,
    IDR_MINIMAP_ICE_I01,
    IDR_MINIMAP_ICE_I02,
    IDR_MINIMAP_ICE_R01,
    IDR_MINIMAP_VILLAGE_I01,
    IDR_MINIMAP_VILLAGE_I02,
    IDR_MINIMAP_VILLAGE_R01,
    IDR_MINIMAP_VILLAGE_R03,
};

/* Serialized the::ToMinimap values from each track.1s, in TRACKS[] order.
   The zero entry belongs to flat_test, which is not an original demo track. */
typedef struct OriginalMinimapMapping {
    float origin_x;
    float origin_y;
    float scale;
    unsigned int width;
    unsigned int height;
} OriginalMinimapMapping;

static const OriginalMinimapMapping ORIGINAL_MINIMAP_MAPPINGS[] = {
    {0.0f, 0.0f, 0.0f, 0, 0},
    {529.7672f,   598.147949f, 0.3f,         256, 256},
    {390.977051f, 595.9359f,   0.3f,         256, 256},
    {614.902832f, 461.68692f,  0.3f,         256, 256},
    {468.612122f, 408.32428f,  0.3f,         256, 256},
    {481.403046f, 618.0281f,   0.3f,         256, 256},
    {571.3153f,   723.649048f, 0.211053431f, 256, 256},
    {562.5881f,   751.542236f, 0.3f,         256, 256},
    {566.9641f,   487.197235f, 0.3f,         256, 256},
    {863.445557f, 897.8673f,   0.160318315f, 256, 256},
    {326.083923f, 484.866364f, 0.3f,         256, 256},
    {259.2857f,   356.454651f, 0.3f,         256, 256},
    {631.49884f,  744.7253f,   0.144852176f, 256, 256},
    {573.462f,    769.314758f, 0.1878337f,   256, 256},
};

_Static_assert(
    sizeof(MINIMAP_RESOURCE_IDS) / sizeof(MINIMAP_RESOURCE_IDS[0]) ==
        KART_DEMO_TRACK_COUNT,
    "one minimap resource id per track, in TRACKS[] order; use 0 for a track "
    "with no original minimap");

_Static_assert(
    sizeof(ORIGINAL_MINIMAP_MAPPINGS) / sizeof(ORIGINAL_MINIMAP_MAPPINGS[0]) ==
        KART_DEMO_TRACK_COUNT,
    "one original ToMinimap mapping per track, in TRACKS[] order");

static const OriginalMinimapMapping *original_mapping_for_track(
    const KartDemoTrackSpec *track)
{
    unsigned int i;
    if (track == NULL) return NULL;
    for (i = 0; i < kart_demo_track_count(); ++i) {
        if (kart_demo_track_at(i) == track) {
            return ORIGINAL_MINIMAP_MAPPINGS[i].width != 0
                ? &ORIGINAL_MINIMAP_MAPPINGS[i]
                : NULL;
        }
    }
    return NULL;
}

static bool decode_png_resource(
    HINSTANCE instance,
    IWICImagingFactory *factory,
    int resource_id,
    KartDemoMinimap *minimap)
{
    HRSRC resource;
    HGLOBAL loaded;
    BYTE *data;
    DWORD data_size;
    IWICStream *stream = NULL;
    IWICBitmapDecoder *decoder = NULL;
    IWICBitmapFrameDecode *frame = NULL;
    IWICFormatConverter *converter = NULL;
    UINT width = 0;
    UINT height = 0;
    BITMAPINFO info;
    BYTE *pixels = NULL;
    HBITMAP bitmap = NULL;
    HRESULT result;
    bool success = false;
    UINT x;
    UINT y;
    LONG left;
    LONG top;
    LONG right;
    LONG bottom;

    if (resource_id == 0) return false;
    resource = FindResourceA(instance, MAKEINTRESOURCEA(resource_id), RT_RCDATA);
    loaded = resource != NULL ? LoadResource(instance, resource) : NULL;
    data_size = resource != NULL ? SizeofResource(instance, resource) : 0;
    data = loaded != NULL ? (BYTE *)LockResource(loaded) : NULL;
    if (data == NULL || data_size == 0) goto cleanup;

    result = IWICImagingFactory_CreateStream(factory, &stream);
    if (FAILED(result)) goto cleanup;
    result = IWICStream_InitializeFromMemory(stream, data, data_size);
    if (FAILED(result)) goto cleanup;
    result = IWICImagingFactory_CreateDecoderFromStream(
        factory, (IStream *)stream, NULL, WICDecodeMetadataCacheOnLoad, &decoder);
    if (FAILED(result)) goto cleanup;
    result = IWICBitmapDecoder_GetFrame(decoder, 0, &frame);
    if (FAILED(result)) goto cleanup;
    result = IWICImagingFactory_CreateFormatConverter(factory, &converter);
    if (FAILED(result)) goto cleanup;
    result = IWICFormatConverter_Initialize(
        converter,
        (IWICBitmapSource *)frame,
        &GUID_WICPixelFormat32bppPBGRA,
        WICBitmapDitherTypeNone,
        NULL,
        0.0,
        WICBitmapPaletteTypeCustom);
    if (FAILED(result)) goto cleanup;
    result = IWICBitmapSource_GetSize(
        (IWICBitmapSource *)converter, &width, &height);
    if (FAILED(result) || width == 0 || height == 0 ||
        width > 4096 || height > 4096) goto cleanup;

    memset(&info, 0, sizeof(info));
    info.bmiHeader.biSize = sizeof(info.bmiHeader);
    info.bmiHeader.biWidth = (LONG)width;
    info.bmiHeader.biHeight = -(LONG)height;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    info.bmiHeader.biCompression = BI_RGB;
    bitmap = CreateDIBSection(
        NULL, &info, DIB_RGB_COLORS, (void **)&pixels, NULL, 0);
    if (bitmap == NULL || pixels == NULL) goto cleanup;
    result = IWICBitmapSource_CopyPixels(
        (IWICBitmapSource *)converter,
        NULL,
        width * 4u,
        width * height * 4u,
        pixels);
    if (FAILED(result)) goto cleanup;

    left = (LONG)width;
    top = (LONG)height;
    right = 0;
    bottom = 0;
    for (y = 0; y < height; ++y) {
        for (x = 0; x < width; ++x) {
            const BYTE *pixel = pixels + ((size_t)y * width + x) * 4u;
            if (pixel[3] > 16 && (pixel[0] > 32 || pixel[1] > 32 || pixel[2] > 32)) {
                if ((LONG)x < left) left = (LONG)x;
                if ((LONG)y < top) top = (LONG)y;
                if ((LONG)x + 1 > right) right = (LONG)x + 1;
                if ((LONG)y + 1 > bottom) bottom = (LONG)y + 1;
            }
        }
    }
    if (right <= left || bottom <= top) {
        left = 0;
        top = 0;
        right = (LONG)width;
        bottom = (LONG)height;
    }
    minimap->bitmap = bitmap;
    minimap->width = width;
    minimap->height = height;
    minimap->content_bounds = (RECT){left, top, right, bottom};
    bitmap = NULL;
    success = true;

cleanup:
    if (bitmap != NULL) DeleteObject(bitmap);
    if (converter != NULL) IWICFormatConverter_Release(converter);
    if (frame != NULL) IWICBitmapFrameDecode_Release(frame);
    if (decoder != NULL) IWICBitmapDecoder_Release(decoder);
    if (stream != NULL) IWICStream_Release(stream);
    return success;
}

void kart_demo_minimap_set_load(HINSTANCE instance, KartDemoMinimapSet *set)
{
#if defined(KART_EMBED_MINIMAPS)
    IWICImagingFactory *factory = NULL;
    HRESULT initialized;
    HRESULT result;
    unsigned int i;
    if (set == NULL) return;
    memset(set, 0, sizeof(*set));
    initialized = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);
    result = CoCreateInstance(
        &CLSID_WICImagingFactory,
        NULL,
        CLSCTX_INPROC_SERVER,
        &IID_IWICImagingFactory,
        (void **)&factory);
    if (SUCCEEDED(result) && factory != NULL) {
        const unsigned int count = kart_demo_track_count();
        for (i = 0; i < count && i < KART_DEMO_MINIMAP_CAPACITY &&
                    i < sizeof(MINIMAP_RESOURCE_IDS) / sizeof(MINIMAP_RESOURCE_IDS[0]); ++i) {
            /* Id 0 marks a track with no original minimap, such as the flat
               test track; its panel falls back to the grid drawing. */
            if (MINIMAP_RESOURCE_IDS[i] == 0) continue;
            decode_png_resource(
                instance, factory, MINIMAP_RESOURCE_IDS[i], &set->entries[i]);
        }
        IWICImagingFactory_Release(factory);
    }
    if (initialized == S_OK || initialized == S_FALSE) CoUninitialize();
#else
    (void)instance;
    if (set != NULL) memset(set, 0, sizeof(*set));
#endif
}

void kart_demo_minimap_set_free(KartDemoMinimapSet *set)
{
    unsigned int i;
    if (set == NULL) return;
    for (i = 0; i < KART_DEMO_MINIMAP_CAPACITY; ++i) {
        if (set->entries[i].bitmap != NULL) DeleteObject(set->entries[i].bitmap);
    }
    memset(set, 0, sizeof(*set));
}

KartDemoMinimap *kart_demo_minimap_for_track(
    KartDemoMinimapSet *set,
    const KartDemoTrackSpec *track)
{
    unsigned int i;
    if (set == NULL || track == NULL) return NULL;
    for (i = 0; i < kart_demo_track_count() && i < KART_DEMO_MINIMAP_CAPACITY; ++i) {
        if (kart_demo_track_at(i) == track) {
            return set->entries[i].bitmap != NULL ? &set->entries[i] : NULL;
        }
    }
    return NULL;
}

static KartVec3 minimap_forward_from_quaternion(KartQuat q)
{
    return (KartVec3){
        -2.0f * (q.x * q.y - q.w * q.z),
        -(1.0f - 2.0f * (q.x * q.x + q.z * q.z)),
        -2.0f * (q.y * q.z + q.w * q.x),
    };
}

static KartVec3 minimap_vec_scale(KartVec3 v, float scale)
{
    return (KartVec3){v.x * scale, v.y * scale, v.z * scale};
}

static KartVec3 minimap_vec_cross(KartVec3 a, KartVec3 b)
{
    return (KartVec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

static KartVec3 minimap_vec_normalize(KartVec3 v)
{
    const float length = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    return length > 0.0f ? minimap_vec_scale(v, 1.0f / length) : v;
}

void kart_demo_draw_original_minimap_camera(
    HDC dc,
    RECT destination,
    KartDemoMinimap *minimap,
    const KartDemoTrackSpec *track,
    KartVec3 world_position,
    KartQuat world_orientation,
    unsigned int frame_time_ms,
    COLORREF marker_colour)
{
    const OriginalMinimapMapping *mapping = original_mapping_for_track(track);
    const int output_width = destination.right - destination.left;
    const int output_height = destination.bottom - destination.top;
    const float center_x = (track->minimum.x + track->maximum.x) * 0.5f;
    const float center_y = (track->minimum.y + track->maximum.y) * 0.5f;
    KartQuat target;
    KartVec3 forward;
    KartVec3 camera_y;
    KartVec3 camera_x;
    KartVec3 camera_z;
    KartVec3 camera_position;
    float map_x;
    float map_y;
    BITMAP source_bitmap;
    BITMAPINFO output_info;
    HDC output_dc;
    HBITMAP output_bitmap;
    HGDIOBJ old_output_bitmap;
    BYTE *dib_pixels;
    BYTE *output_pixels;
    POINT marker_points[3];
    int x;
    int y;

    if (dc == NULL || minimap == NULL || minimap->bitmap == NULL ||
        mapping == NULL || output_width <= 0 || output_height <= 0) return;
    if (GetObject(minimap->bitmap, sizeof(source_bitmap), &source_bitmap) == 0 ||
        source_bitmap.bmBits == NULL) return;

    /* Asset->simulator is S=diag(-1,1,1). R_asset=S*R_world*S. */
    target = (KartQuat){world_orientation.w, world_orientation.x,
                        -world_orientation.y, -world_orientation.z};
    if (!minimap->camera_initialized) {
        minimap->camera_orientation = target;
        minimap->camera_time_ms = frame_time_ms;
        minimap->camera_initialized = true;
    } else {
        const unsigned int elapsed = frame_time_ms - minimap->camera_time_ms;
        float t = (float)elapsed / 1500.0f;
        const float dot = minimap->camera_orientation.w * target.w +
                          minimap->camera_orientation.x * target.x +
                          minimap->camera_orientation.y * target.y +
                          minimap->camera_orientation.z * target.z;
        if (dot < 0.0f) {
            minimap->camera_orientation.w = -minimap->camera_orientation.w;
            minimap->camera_orientation.x = -minimap->camera_orientation.x;
            minimap->camera_orientation.y = -minimap->camera_orientation.y;
            minimap->camera_orientation.z = -minimap->camera_orientation.z;
        }
        if (t > 1.0f) t = 1.0f;
        minimap->camera_orientation = kart_chase_camera_interpolate(
            minimap->camera_orientation, target, t);
        minimap->camera_time_ms = frame_time_ms;
    }

    forward = minimap_forward_from_quaternion(minimap->camera_orientation);
    forward.z = 0.0f;
    forward = minimap_vec_normalize(forward);
    camera_y = minimap_vec_normalize((KartVec3){-forward.x, -forward.y, 1.0f});
    camera_x = minimap_vec_normalize(minimap_vec_cross(camera_y, forward));
    camera_z = minimap_vec_normalize(minimap_vec_cross(camera_x, camera_y));

    /* Undo the simulator's centred X mirror before applying ToMinimap. */
    map_x = (float)mapping->width * 0.5f +
            ((center_x - world_position.x) - mapping->origin_x) * mapping->scale;
    map_y = (float)mapping->height * 0.5f +
            ((center_y + world_position.y) - mapping->origin_y) * mapping->scale;
    camera_position = (KartVec3){
        map_x + camera_y.x * 60.0f,
        map_y + camera_y.y * 60.0f,
        camera_y.z * 60.0f,
    };

    output_pixels = (BYTE *)malloc((size_t)output_width * output_height * 4u);
    if (output_pixels == NULL) return;
    for (y = 0; y < output_height; ++y) {
        const float screen_z = 1.0f - 2.0f * ((float)y + 0.5f) / (float)output_height;
        for (x = 0; x < output_width; ++x) {
            /* The simulator's displayed minimap is horizontally reversed
               relative to the original camera target. Flip only the final
               screen X coordinate; keep the recovered camera and map-space
               transforms unchanged. */
            const float screen_x = 1.0f -
                2.0f * ((float)x + 0.5f) / (float)output_width;
            const KartVec3 ray = {
                -camera_y.x + camera_x.x * screen_x + camera_z.x * screen_z,
                -camera_y.y + camera_x.y * screen_x + camera_z.y * screen_z,
                -camera_y.z + camera_x.z * screen_x + camera_z.z * screen_z,
            };
            const float distance = ray.z != 0.0f ? -camera_position.z / ray.z : -1.0f;
            int source_x = 0;
            int source_y = 0;
            const BYTE *source_pixel;
            BYTE *output_pixel = output_pixels +
                ((size_t)y * output_width + x) * 4u;
            if (distance > 0.0f) {
                source_x = (int)floorf(camera_position.x + ray.x * distance);
                source_y = (int)floorf((float)mapping->height -
                    (camera_position.y + ray.y * distance));
            }
            /* TexProperty U/V address values are D3DTADDRESS_CLAMP (3). */
            if (source_x < 0) source_x = 0;
            if (source_x >= (int)mapping->width) source_x = (int)mapping->width - 1;
            if (source_y < 0) source_y = 0;
            if (source_y >= (int)mapping->height) source_y = (int)mapping->height - 1;
            source_pixel = (const BYTE *)source_bitmap.bmBits +
                (size_t)source_y * source_bitmap.bmWidthBytes + source_x * 4u;
            memcpy(output_pixel, source_pixel, 4u);
        }
    }

    memset(&output_info, 0, sizeof(output_info));
    output_info.bmiHeader.biSize = sizeof(output_info.bmiHeader);
    output_info.bmiHeader.biWidth = output_width;
    output_info.bmiHeader.biHeight = -output_height;
    output_info.bmiHeader.biPlanes = 1;
    output_info.bmiHeader.biBitCount = 32;
    output_info.bmiHeader.biCompression = BI_RGB;
    output_dc = CreateCompatibleDC(dc);
    dib_pixels = NULL;
    output_bitmap = CreateDIBSection(
        dc, &output_info, DIB_RGB_COLORS, (void **)&dib_pixels, NULL, 0);
    if (output_dc != NULL && output_bitmap != NULL && dib_pixels != NULL) {
        const BLENDFUNCTION blend = {
            AC_SRC_OVER, 0, 77, 0 /* minimap.1s TexProperty alpha = 0.3 */
        };
        memcpy(dib_pixels, output_pixels,
            (size_t)output_width * output_height * 4u);
        old_output_bitmap = SelectObject(output_dc, output_bitmap);
        AlphaBlend(dc, destination.left, destination.top,
            output_width, output_height, output_dc, 0, 0,
            output_width, output_height, blend);
        SelectObject(output_dc, old_output_bitmap);
    }
    if (output_bitmap != NULL) DeleteObject(output_bitmap);
    if (output_dc != NULL) DeleteDC(output_dc);
    free(output_pixels);

    /* FUN_00467550 gives `me` the current (unsmoothed) flattened heading.
       Project the three recovered minimap.1s vertices through the same camera. */
    {
        static const float marker_vertices[3][2] = {
            {16.587f, -16.396f}, {0.274f, 21.517f}, {-16.587f, -16.396f},
        };
        KartVec3 marker_forward = minimap_forward_from_quaternion(target);
        KartVec3 marker_x;
        HBRUSH brush;
        HPEN pen;
        HGDIOBJ old_brush;
        HGDIOBJ old_pen;
        int index;
        marker_forward.z = 0.0f;
        marker_forward = minimap_vec_normalize(marker_forward);
        marker_x = minimap_vec_normalize(
            minimap_vec_cross(marker_forward, (KartVec3){0.0f, 0.0f, 1.0f}));
        for (index = 0; index < 3; ++index) {
            const KartVec3 point = {
                map_x + KART_DEMO_MINIMAP_MARKER_SCALE *
                    (marker_x.x * marker_vertices[index][0] +
                     marker_forward.x * marker_vertices[index][1]),
                map_y + KART_DEMO_MINIMAP_MARKER_SCALE *
                    (marker_x.y * marker_vertices[index][0] +
                     marker_forward.y * marker_vertices[index][1]),
                0.1f,
            };
            const KartVec3 delta = {
                point.x - camera_position.x,
                point.y - camera_position.y,
                point.z - camera_position.z,
            };
            const float local_x = delta.x * camera_x.x + delta.y * camera_x.y +
                                  delta.z * camera_x.z;
            const float local_y = delta.x * camera_y.x + delta.y * camera_y.y +
                                  delta.z * camera_y.z;
            const float local_z = delta.x * camera_z.x + delta.y * camera_z.y +
                                  delta.z * camera_z.z;
            const float depth = -local_y;
            marker_points[index].x = destination.left + (LONG)(
                (0.5f - (local_x / depth) * 0.5f) * output_width);
            marker_points[index].y = destination.top + (LONG)(
                (0.5f - (local_z / depth) * 0.5f) * output_height);
        }
        brush = CreateSolidBrush(marker_colour);
        pen = CreatePen(PS_SOLID, 1, marker_colour);
        old_brush = SelectObject(dc, brush);
        old_pen = SelectObject(dc, pen);
        Polygon(dc, marker_points, 3);
        SelectObject(dc, old_brush);
        SelectObject(dc, old_pen);
        DeleteObject(brush);
        DeleteObject(pen);
    }
}

void kart_demo_draw_minimap_bitmap(
    HDC dc,
    RECT destination,
    const KartDemoMinimap *minimap)
{
    HDC source;
    HGDIOBJ old_bitmap;
    int old_mode;
    if (minimap == NULL || minimap->bitmap == NULL) return;
    source = CreateCompatibleDC(dc);
    old_bitmap = SelectObject(source, minimap->bitmap);
    old_mode = SetStretchBltMode(dc, HALFTONE);
    SetBrushOrgEx(dc, 0, 0, NULL);
    StretchBlt(
        dc,
        destination.left,
        destination.top,
        destination.right - destination.left,
        destination.bottom - destination.top,
        source,
        0,
        0,
        (int)minimap->width,
        (int)minimap->height,
        SRCCOPY);
    SetStretchBltMode(dc, old_mode);
    SelectObject(source, old_bitmap);
    DeleteDC(source);
}

/* Track position as a 0..1 panel coordinate, using the same axis flips as
   kart_demo_minimap_direction so a marker's position and heading always agree. */
static void normalized_track_point(
    const KartDemoTrackSpec *track,
    KartVec3 position,
    float *normalized_x,
    float *normalized_y)
{
    const float track_width = kart_demo_track_width(track);
    const float track_length = kart_demo_track_length(track);
    *normalized_x = kart_demo_track_mirror_x(track)
        ? 0.5f - position.x / track_width
        : position.x / track_width + 0.5f;
    /* Panel rows grow downward while the track's +Y axis maps upward on the
       authored minimap. Position and direction must use the same flip. */
    *normalized_y = 0.5f - position.y / track_length;
    if (*normalized_x < 0.0f) *normalized_x = 0.0f;
    if (*normalized_x > 1.0f) *normalized_x = 1.0f;
    if (*normalized_y < 0.0f) *normalized_y = 0.0f;
    if (*normalized_y > 1.0f) *normalized_y = 1.0f;
}

POINT kart_demo_minimap_bounds_point(
    RECT destination,
    const KartDemoTrackSpec *track,
    KartVec3 position)
{
    float normalized_x;
    float normalized_y;
    normalized_track_point(track, position, &normalized_x, &normalized_y);
    return (POINT){
        destination.left +
            (LONG)(normalized_x * (float)(destination.right - destination.left)),
        destination.top +
            (LONG)(normalized_y * (float)(destination.bottom - destination.top)),
    };
}

POINT kart_demo_minimap_kart_point(
    RECT destination,
    const KartDemoMinimap *minimap,
    const KartDemoTrackSpec *track,
    KartVec3 position)
{
    const OriginalMinimapMapping *mapping = original_mapping_for_track(track);
    float scene_x;
    float scene_y;
    if (mapping == NULL || minimap == NULL) return (POINT){0, 0};

    /* FUN_00467550: C + (world.xy - ToMinimap.origin) * scale.  The
       minimap.1s quad maps scene X directly to texture U and reverses scene Y
       for texture V. */
    scene_x = (float)mapping->width * 0.5f +
              (position.x - mapping->origin_x) * mapping->scale;
    scene_y = (float)mapping->height * 0.5f +
              (position.y - mapping->origin_y) * mapping->scale;
    return (POINT){
        destination.left + (LONG)(scene_x / (float)mapping->width *
            (destination.right - destination.left)),
        destination.top + (LONG)(((float)mapping->height - scene_y) /
            (float)mapping->height *
            (destination.bottom - destination.top)),
    };
}

KartVec3 kart_demo_minimap_direction(
    const KartDemoTrackSpec *track,
    KartVec3 world_direction)
{
    (void)track;
    world_direction.y = -world_direction.y;
    return world_direction;
}
