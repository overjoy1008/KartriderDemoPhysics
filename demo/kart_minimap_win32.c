#define COBJMACROS
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <objbase.h>
#include <wincodec.h>

#include "kart_minimap_win32.h"
#include "kart_minimap_resources.h"

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

_Static_assert(
    sizeof(MINIMAP_RESOURCE_IDS) / sizeof(MINIMAP_RESOURCE_IDS[0]) ==
        KART_DEMO_TRACK_COUNT,
    "one minimap resource id per track, in TRACKS[] order; use 0 for a track "
    "with no original minimap");

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

const KartDemoMinimap *kart_demo_minimap_for_track(
    const KartDemoMinimapSet *set,
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
    float normalized_x;
    float normalized_y;
    float pixel_x;
    float pixel_y;
    normalized_track_point(track, position, &normalized_x, &normalized_y);
    pixel_x = minimap->content_bounds.left +
              normalized_x * (minimap->content_bounds.right - minimap->content_bounds.left);
    pixel_y = minimap->content_bounds.top +
              normalized_y * (minimap->content_bounds.bottom - minimap->content_bounds.top);
    return (POINT){
        destination.left + (LONG)(pixel_x / minimap->width *
            (destination.right - destination.left)),
        destination.top + (LONG)(pixel_y / minimap->height *
            (destination.bottom - destination.top)),
    };
}

KartVec3 kart_demo_minimap_direction(
    const KartDemoTrackSpec *track,
    KartVec3 world_direction)
{
    if (kart_demo_track_mirror_x(track)) {
        world_direction.x = -world_direction.x;
    }
    world_direction.y = -world_direction.y;
    return world_direction;
}
