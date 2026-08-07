#ifndef KART_SCREENSHOT_WIN32_H
#define KART_SCREENSHOT_WIN32_H

/* Saves the window's client area next to the executable as a BMP.

   BMP rather than PNG so this needs no encoder: the DIB the demos already
   render into is written out with a header in front of it. */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <stdio.h>
#include <stdbool.h>

/* Writes shot-NNN.bmp beside the executable and returns the chosen name in
   `name_out`, which must hold at least MAX_PATH characters. */
static bool kart_demo_save_screenshot(HWND window, char *name_out, size_t name_size)
{
    RECT client;
    HDC window_dc = NULL;
    HDC memory_dc = NULL;
    HBITMAP bitmap = NULL;
    HGDIOBJ old_bitmap = NULL;
    BITMAPINFO info;
    BITMAPFILEHEADER file_header;
    BITMAPINFOHEADER info_header;
    unsigned char *pixels = NULL;
    char directory[MAX_PATH];
    char path[MAX_PATH];
    DWORD stride;
    DWORD image_size;
    FILE *output = NULL;
    unsigned int index;
    bool saved = false;
    int width;
    int height;

    if (!GetClientRect(window, &client)) return false;
    width = client.right - client.left;
    height = client.bottom - client.top;
    if (width <= 0 || height <= 0) return false;

    /* Beside the executable, not the working directory, so a shortcut launch
       still puts the file somewhere findable. */
    if (GetModuleFileNameA(NULL, directory, MAX_PATH) == 0) return false;
    {
        char *slash = strrchr(directory, '\\');
        if (slash != NULL) *slash = '\0';
    }
    for (index = 1; index < 10000u; ++index) {
        snprintf(path, sizeof(path), "%s\\shot-%03u.bmp", directory, index);
        if (GetFileAttributesA(path) == INVALID_FILE_ATTRIBUTES) break;
    }
    if (index >= 10000u) return false;

    window_dc = GetDC(window);
    if (window_dc == NULL) return false;
    memory_dc = CreateCompatibleDC(window_dc);
    if (memory_dc == NULL) goto cleanup;

    memset(&info, 0, sizeof(info));
    info.bmiHeader.biSize = sizeof(info.bmiHeader);
    info.bmiHeader.biWidth = width;
    /* Bottom-up, which is what a plain BMP stores. */
    info.bmiHeader.biHeight = height;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 24;
    info.bmiHeader.biCompression = BI_RGB;

    bitmap = CreateDIBSection(
        window_dc, &info, DIB_RGB_COLORS, (void **)&pixels, NULL, 0);
    if (bitmap == NULL || pixels == NULL) goto cleanup;
    old_bitmap = SelectObject(memory_dc, bitmap);
    if (!BitBlt(memory_dc, 0, 0, width, height, window_dc, 0, 0, SRCCOPY)) {
        goto cleanup;
    }

    stride = ((DWORD)width * 3u + 3u) & ~3u;
    image_size = stride * (DWORD)height;

    memset(&info_header, 0, sizeof(info_header));
    info_header.biSize = sizeof(info_header);
    info_header.biWidth = width;
    info_header.biHeight = height;
    info_header.biPlanes = 1;
    info_header.biBitCount = 24;
    info_header.biCompression = BI_RGB;
    info_header.biSizeImage = image_size;

    memset(&file_header, 0, sizeof(file_header));
    file_header.bfType = 0x4D42; /* "BM" */
    file_header.bfOffBits = sizeof(file_header) + sizeof(info_header);
    file_header.bfSize = file_header.bfOffBits + image_size;

    output = fopen(path, "wb");
    if (output == NULL) goto cleanup;
    saved = fwrite(&file_header, sizeof(file_header), 1, output) == 1 &&
            fwrite(&info_header, sizeof(info_header), 1, output) == 1 &&
            fwrite(pixels, 1, image_size, output) == image_size;
    fclose(output);
    if (!saved) {
        DeleteFileA(path);
    } else if (name_out != NULL && name_size > 0) {
        const char *slash = strrchr(path, '\\');
        snprintf(name_out, name_size, "%s", slash != NULL ? slash + 1 : path);
    }

cleanup:
    if (old_bitmap != NULL) SelectObject(memory_dc, old_bitmap);
    if (bitmap != NULL) DeleteObject(bitmap);
    if (memory_dc != NULL) DeleteDC(memory_dc);
    ReleaseDC(window, window_dc);
    return saved;
}

#endif
