#ifndef KART_DEMO_WIN32_UI_H
#define KART_DEMO_WIN32_UI_H

#include <windows.h>

#include "kart_demo_data.h"
#include "kart_gearbox.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>

/* Backdate applied when arming the countdown, see the reset paths. */
#define KART_DEMO_COUNTDOWN_PREROLL_MS 3000U

#define KART_DEMO_KART_MENU_BASE 1000U
#define KART_DEMO_TRACK_MENU_BASE 2000U

static void kart_demo_utf8_to_wide(
    const char *source,
    wchar_t *destination,
    int capacity)
{
    if (capacity <= 0) return;
    destination[0] = L'\0';
    if (source == NULL) return;
    if (MultiByteToWideChar(
            CP_UTF8, 0, source, -1, destination, capacity) == 0) {
        destination[0] = L'\0';
    }
    destination[capacity - 1] = L'\0';
}

static void kart_demo_text_out_utf8(
    HDC dc,
    int x,
    int y,
    const char *text)
{
    wchar_t wide[512];
    kart_demo_utf8_to_wide(text, wide, 512);
    TextOutW(dc, x, y, wide, (int)wcslen(wide));
}

static const KartDemoKartSpec *kart_demo_popup_select_kart(
    HWND window,
    const KartDemoKartSpec *current)
{
    HMENU menu = CreatePopupMenu();
    RECT window_rect;
    unsigned int i;
    UINT command;
    if (menu == NULL) {
        return current;
    }
    AppendMenuA(menu, MF_STRING | MF_DISABLED, 0, "SELECT KART");
    AppendMenuA(menu, MF_SEPARATOR, 0, NULL);
    for (i = 0; i < kart_demo_kart_count(); ++i) {
        const KartDemoKartSpec *spec = kart_demo_kart_at(i);
        UINT flags = MF_STRING;
        char label[128];
        if (i == 13) {
            flags |= MF_MENUBARBREAK;
        }
        if (spec == current) {
            flags |= MF_CHECKED;
        }
        snprintf(
            label,
            sizeof(label),
            "%s    %.3f x %.3f",
            spec->asset_name,
            spec->geometry.half_width * 2.0f,
            spec->geometry.half_length * 2.0f);
        AppendMenuA(
            menu, flags, KART_DEMO_KART_MENU_BASE + i, label);
    }
    GetWindowRect(window, &window_rect);
    SetForegroundWindow(window);
    command = TrackPopupMenu(
        menu,
        TPM_RETURNCMD | TPM_LEFTALIGN | TPM_TOPALIGN | TPM_RIGHTBUTTON,
        window_rect.left + 32,
        window_rect.top + 72,
        0,
        window,
        NULL);
    DestroyMenu(menu);
    PostMessage(window, WM_NULL, 0, 0);
    if (command >= KART_DEMO_KART_MENU_BASE &&
        command < KART_DEMO_KART_MENU_BASE + kart_demo_kart_count()) {
        return kart_demo_kart_at(command - KART_DEMO_KART_MENU_BASE);
    }
    return current;
}

static const KartDemoTrackSpec *kart_demo_popup_select_track(
    HWND window,
    const KartDemoTrackSpec *current)
{
    HMENU menu = CreatePopupMenu();
    RECT window_rect;
    unsigned int i;
    UINT command;
    if (menu == NULL) {
        return current;
    }
    AppendMenuW(menu, MF_STRING | MF_DISABLED, 0, L"트랙 선택");
    AppendMenuW(menu, MF_SEPARATOR, 0, NULL);
    for (i = 0; i < kart_demo_track_count(); ++i) {
        const KartDemoTrackSpec *spec = kart_demo_track_at(i);
        UINT flags = MF_STRING;
        char utf8_label[256];
        wchar_t label[256];
        if (spec == current) {
            flags |= MF_CHECKED;
        }
        if (spec->difficulty != 0) {
            snprintf(
                utf8_label,
                sizeof(utf8_label),
                "%s  [%s]  난이도 %u  (%s)",
                spec->display_name,
                spec->race_mode,
                spec->difficulty,
                spec->asset_name);
        } else {
            snprintf(
                utf8_label,
                sizeof(utf8_label),
                "%s  [%s]  난이도 ?  (%s)",
                spec->display_name,
                spec->race_mode,
                spec->asset_name);
        }
        kart_demo_utf8_to_wide(utf8_label, label, 256);
        AppendMenuW(
            menu, flags, KART_DEMO_TRACK_MENU_BASE + i, label);
    }
    GetWindowRect(window, &window_rect);
    SetForegroundWindow(window);
    command = TrackPopupMenu(
        menu,
        TPM_RETURNCMD | TPM_LEFTALIGN | TPM_TOPALIGN | TPM_RIGHTBUTTON,
        window_rect.left + 280,
        window_rect.top + 72,
        0,
        window,
        NULL);
    DestroyMenu(menu);
    PostMessage(window, WM_NULL, 0, 0);
    if (command >= KART_DEMO_TRACK_MENU_BASE &&
        command < KART_DEMO_TRACK_MENU_BASE + kart_demo_track_count()) {
        return kart_demo_track_at(command - KART_DEMO_TRACK_MENU_BASE);
    }
    return current;
}

static void kart_demo_draw_speedometer(
    HDC dc,
    RECT client,
    int speed_kmh,
    bool boost_active)
{
    const int panel_width = 238;
    const int panel_height = 94;
    const int margin = 18;
    RECT panel = {
        client.right - panel_width - margin,
        client.bottom - panel_height - margin,
        client.right - margin,
        client.bottom - margin,
    };
    RECT digits_rect = {
        panel.left + 8,
        panel.top + 4,
        panel.right - 65,
        panel.bottom - 5,
    };
    RECT unit_rect = {
        panel.right - 68,
        panel.top + 42,
        panel.right - 10,
        panel.bottom - 8,
    };
    HBRUSH panel_brush = CreateSolidBrush(RGB(12, 16, 22));
    HPEN panel_pen = CreatePen(
        PS_SOLID, 3, boost_active ? RGB(75, 225, 255) : RGB(110, 130, 145));
    HFONT digits_font = CreateFontA(
        -58, 0, 0, 0, FW_HEAVY, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        ANTIALIASED_QUALITY, FF_DONTCARE, "Segoe UI");
    HFONT unit_font = CreateFontA(
        -18, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        ANTIALIASED_QUALITY, FF_DONTCARE, "Segoe UI");
    HGDIOBJ old_brush = SelectObject(dc, panel_brush);
    HGDIOBJ old_pen = SelectObject(dc, panel_pen);
    HGDIOBJ old_font;
    char digits[16];
    static const char unit[] = "KM/H";

    if (panel.left < client.left + margin) {
        panel.left = client.left + margin;
    }
    Rectangle(dc, panel.left, panel.top, panel.right, panel.bottom);
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, boost_active ? RGB(90, 235, 255) : RGB(245, 248, 250));
    snprintf(digits, sizeof(digits), "%03d", speed_kmh);
    old_font = SelectObject(dc, digits_font);
    DrawTextA(dc, digits, -1, &digits_rect, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
    SelectObject(dc, unit_font);
    SetTextColor(dc, RGB(175, 195, 205));
    DrawTextA(dc, unit, -1, &unit_rect, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

    SelectObject(dc, old_font);
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(panel_brush);
    DeleteObject(panel_pen);
    DeleteObject(digits_font);
    DeleteObject(unit_font);
}

/* The engine note, drawn as a needle dial. There is no gear or crankshaft in
   the recovered engine, so this is not an RPM reading: it is the motor pitch
   the original's sound driver computes, plotted directly.

     ramp  = speed * 0.01171875 + 0.25
     pitch = speed >= 128 ? 1.5 : ramp

   Every number here is recovered (FUN_00452E60, see kart_engine_sound.h), which
   is why it is shown as a multiplier rather than dressed up as RPM. The ramp
   reaches about 1.75 just under speed 128 and then steps down to the 1.5 cap,
   so the needle jumps backwards out of the red band at that point. The dim
   marker is the uncapped ramp, and the driver only refreshes every 64 ms, so
   the needle holds between refreshes exactly as the sound does. */
static void kart_demo_draw_tachometer(
    HDC dc,
    RECT client,
    float pitch,
    float volume,
    float ramp,
    int gear)
{
    const float dial_min = 0.25f;
    const float dial_max = 1.75f;
    const float sweep_start = 210.0f;
    const float sweep_degrees = 240.0f;
    const float to_radians = 3.14159265358979323846f / 180.0f;
    const int panel_width = 238;
    const int panel_height = 132;
    const int margin = 18;
    /* Above the wheel-load panel, which sits above the speedometer. */
    const int bottom = client.bottom - margin - 94 - 8 - 118 - 8;
    RECT panel = {
        client.right - panel_width - margin,
        bottom - panel_height,
        client.right - margin,
        bottom,
    };
    const int center_x = (panel.left + panel.right) / 2;
    const int center_y = panel.top + 96;
    const int radius = 52;
    HBRUSH panel_brush = CreateSolidBrush(RGB(12, 16, 22));
    HPEN panel_pen = CreatePen(PS_SOLID, 1, RGB(54, 64, 73));
    HPEN dial_pen = CreatePen(PS_SOLID, 2, RGB(96, 110, 122));
    HPEN tick_pen = CreatePen(PS_SOLID, 1, RGB(150, 165, 180));
    HPEN band_pen = CreatePen(PS_SOLID, 4, RGB(215, 70, 70));
    HPEN needle_pen = CreatePen(PS_SOLID, 3, RGB(255, 210, 90));
    HPEN ramp_pen = CreatePen(PS_SOLID, 1, RGB(130, 145, 158));
    HFONT font = CreateFontA(
        -12, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        ANTIALIASED_QUALITY, FF_DONTCARE, "Segoe UI");
    HGDIOBJ old_brush = SelectObject(dc, panel_brush);
    HGDIOBJ old_pen = SelectObject(dc, panel_pen);
    HGDIOBJ old_font;
    static const char label[] = "RPM (motor pitch)";
    char text[48];
    int tick;

#define KART_TACH_ANGLE(value)                                                \
    ((sweep_start -                                                           \
      sweep_degrees * (((value) - dial_min) / (dial_max - dial_min))) *        \
     to_radians)
#define KART_TACH_X(angle, r) (center_x + (int)(cosf(angle) * (float)(r)))
#define KART_TACH_Y(angle, r) (center_y - (int)(sinf(angle) * (float)(r)))

    Rectangle(dc, panel.left, panel.top, panel.right, panel.bottom);
    old_font = SelectObject(dc, font);
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(180, 205, 215));
    TextOutA(dc, panel.left + 9, panel.top + 6, label, (int)strlen(label));

    /* The dial face, then the band the ramp climbs into before the cap drops
       it back to 1.5. */
    SelectObject(dc, dial_pen);
    {
        const float start = KART_TACH_ANGLE(dial_min);
        const float end = KART_TACH_ANGLE(dial_max);
        Arc(dc,
            center_x - radius, center_y - radius,
            center_x + radius, center_y + radius,
            KART_TACH_X(end, radius), KART_TACH_Y(end, radius),
            KART_TACH_X(start, radius), KART_TACH_Y(start, radius));
    }
    SelectObject(dc, band_pen);
    {
        const float start = KART_TACH_ANGLE(1.5f);
        const float end = KART_TACH_ANGLE(dial_max);
        Arc(dc,
            center_x - radius, center_y - radius,
            center_x + radius, center_y + radius,
            KART_TACH_X(end, radius), KART_TACH_Y(end, radius),
            KART_TACH_X(start, radius), KART_TACH_Y(start, radius));
    }
    SelectObject(dc, tick_pen);
    for (tick = 0; tick <= 6; ++tick) {
        const float value = dial_min + (dial_max - dial_min) * tick / 6.0f;
        const float angle = KART_TACH_ANGLE(value);
        MoveToEx(
            dc, KART_TACH_X(angle, radius - 9), KART_TACH_Y(angle, radius - 9),
            NULL);
        LineTo(dc, KART_TACH_X(angle, radius - 2), KART_TACH_Y(angle, radius - 2));
    }

    /* The uncapped ramp, so the backwards jump at the cap is visible. */
    if (ramp > pitch + 0.001f) {
        const float angle =
            KART_TACH_ANGLE(ramp > dial_max ? dial_max : ramp);
        SelectObject(dc, ramp_pen);
        MoveToEx(dc, KART_TACH_X(angle, radius - 16), KART_TACH_Y(angle, radius - 16), NULL);
        LineTo(dc, KART_TACH_X(angle, radius - 2), KART_TACH_Y(angle, radius - 2));
    }
    {
        const float clamped = pitch < dial_min
            ? dial_min : (pitch > dial_max ? dial_max : pitch);
        const float angle = KART_TACH_ANGLE(clamped);
        SelectObject(dc, needle_pen);
        MoveToEx(dc, center_x, center_y, NULL);
        LineTo(dc, KART_TACH_X(angle, radius - 6), KART_TACH_Y(angle, radius - 6));
    }

    SetTextColor(dc, RGB(255, 210, 90));
    snprintf(text, sizeof(text), "%.3fx", pitch);
    TextOutA(dc, panel.left + 12, panel.top + 26, text, (int)strlen(text));
    SetTextColor(dc, RGB(150, 165, 180));
    snprintf(text, sizeof(text), "vol %.2f", volume);
    TextOutA(dc, panel.left + 12, panel.top + 44, text, (int)strlen(text));
    if (gear > 0) {
        SetTextColor(dc, RGB(120, 215, 245));
        snprintf(text, sizeof(text), "GEAR %d/%d", gear, KART_GEAR_COUNT);
        TextOutA(dc, panel.right - 74, panel.top + 26, text, (int)strlen(text));
        SetTextColor(dc, RGB(150, 165, 180));
        snprintf(
            text, sizeof(text), "top %.2f",
            KART_GEAR_BANDS[KART_GEAR_COUNT - 1].high_pitch);
        TextOutA(dc, panel.right - 84, panel.top + 44, text, (int)strlen(text));
    } else {
        snprintf(text, sizeof(text), "1 gear");
        TextOutA(dc, panel.right - 62, panel.top + 26, text, (int)strlen(text));
        snprintf(text, sizeof(text), "cap 1.5");
        TextOutA(dc, panel.right - 62, panel.top + 44, text, (int)strlen(text));
    }

#undef KART_TACH_ANGLE
#undef KART_TACH_X
#undef KART_TACH_Y

    SelectObject(dc, old_font);
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(panel_brush);
    DeleteObject(panel_pen);
    DeleteObject(dial_pen);
    DeleteObject(tick_pen);
    DeleteObject(band_pen);
    DeleteObject(needle_pen);
    DeleteObject(ramp_pen);
    DeleteObject(font);
}

/* Drift gauge: a long bar across the bottom centre, with the booster it
   converts into at its right end. */
static void kart_demo_draw_gauge(
    HDC dc,
    RECT client,
    float ratio,
    unsigned int boosters,
    bool charging,
    const char *model_name)
{
    const int bar_width = 420;
    const int bar_height = 20;
    const int margin = 20;
    const int center_x = (client.left + client.right) / 2;
    RECT bar = {
        center_x - bar_width / 2,
        client.bottom - margin - bar_height,
        center_x + bar_width / 2,
        client.bottom - margin,
    };
    const int slot_width = 26;
    HBRUSH back_brush = CreateSolidBrush(RGB(14, 18, 24));
    HBRUSH fill_brush = CreateSolidBrush(RGB(255, 210, 90));
    HBRUSH slot_full_brush = CreateSolidBrush(RGB(255, 140, 40));
    HBRUSH slot_empty_brush = CreateSolidBrush(RGB(30, 38, 46));
    HPEN edge_pen = CreatePen(PS_SOLID, 1, RGB(86, 98, 108));
    HFONT font = CreateFontA(
        -12, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        ANTIALIASED_QUALITY, FF_DONTCARE, "Segoe UI");
    HGDIOBJ old_brush = SelectObject(dc, back_brush);
    HGDIOBJ old_pen = SelectObject(dc, edge_pen);
    HGDIOBJ old_font = SelectObject(dc, font);
    int fill_width;
    int slot;

    (void)charging;
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    fill_width = (int)((float)(bar_width - 4) * ratio);

    Rectangle(dc, bar.left, bar.top, bar.right, bar.bottom);
    if (fill_width > 0) {
        RECT fill = {
            bar.left + 2, bar.top + 2, bar.left + 2 + fill_width,
            bar.bottom - 2,
        };
        FillRect(dc, &fill, fill_brush);
    }
    for (slot = 0; slot < 2; ++slot) {
        const int left = bar.right + 10 + slot * (slot_width + 6);
        SelectObject(
            dc, (unsigned int)slot < boosters ? slot_full_brush
                                              : slot_empty_brush);
        Rectangle(dc, left, bar.top, left + slot_width, bar.bottom);
    }

    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(200, 212, 220));
    {
        RECT label = {bar.left, bar.top, bar.right, bar.bottom};
        DrawTextA(
            dc, model_name, -1, &label,
            DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    }

    SelectObject(dc, old_font);
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(back_brush);
    DeleteObject(fill_brush);
    DeleteObject(slot_full_brush);
    DeleteObject(slot_empty_brush);
    DeleteObject(edge_pen);
    DeleteObject(font);
}

/* Suspension load, drawn as the kart seen from above with each wheel's
   compression over it. Sits directly above the speedometer. Wheel order is the
   simulation's: 0 front-right, 1 front-left, 2 rear-right, 3 rear-left, with
   the nose of the figure pointing up. */
static void kart_demo_draw_wheel_load(
    HDC dc,
    RECT client,
    const float compression[4],
    bool grounded)
{
    const int panel_width = 238;
    const int panel_height = 118;
    const int margin = 18;
    /* The speedometer panel is 94 tall on the same margin. */
    const int bottom = client.bottom - margin - 94 - 8;
    RECT panel = {
        client.right - panel_width - margin,
        bottom - panel_height,
        client.right - margin,
        bottom,
    };
    const int center_x = (panel.left + panel.right) / 2;
    const int body_half_width = 26;
    const int body_top = panel.top + 30;
    const int body_bottom = panel.bottom - 12;
    const int wheel_width = 11;
    const int wheel_height = 20;
    /* Screen offsets per wheel, in the order above. */
    static const int WHEEL_SIDE[4] = {1, -1, 1, -1};
    static const int WHEEL_END[4] = {0, 0, 1, 1};
    HBRUSH panel_brush = CreateSolidBrush(RGB(12, 16, 22));
    HBRUSH body_brush = CreateSolidBrush(RGB(38, 46, 55));
    HPEN panel_pen = CreatePen(PS_SOLID, 1, RGB(54, 64, 73));
    HPEN body_pen = CreatePen(PS_SOLID, 1, RGB(96, 110, 122));
    HFONT font = CreateFontA(
        -12, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        ANTIALIASED_QUALITY, FF_DONTCARE, "Segoe UI");
    HGDIOBJ old_brush = SelectObject(dc, panel_brush);
    HGDIOBJ old_pen = SelectObject(dc, panel_pen);
    HGDIOBJ old_font;
    static const char label[] = "WHEEL LOAD";
    int i;

    Rectangle(dc, panel.left, panel.top, panel.right, panel.bottom);
    old_font = SelectObject(dc, font);
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, grounded ? RGB(180, 205, 215) : RGB(255, 140, 120));
    TextOutA(dc, panel.left + 9, panel.top + 6, label, (int)strlen(label));

    SelectObject(dc, body_brush);
    SelectObject(dc, body_pen);
    Rectangle(
        dc, center_x - body_half_width, body_top,
        center_x + body_half_width, body_bottom);
    /* A nose mark so the figure reads front-up. */
    MoveToEx(dc, center_x - 8, body_top + 7, NULL);
    LineTo(dc, center_x, body_top + 1);
    LineTo(dc, center_x + 8, body_top + 7);

    for (i = 0; i < 4; ++i) {
        const float value = compression[i];
        /* Uncompressed is 0 and fully compressed is 1; the original rests near
           0.5 on level ground. */
        const int shade = (int)(60.0f + 170.0f *
            (value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value)));
        const int x = center_x + WHEEL_SIDE[i] * (body_half_width + 3) -
                      (WHEEL_SIDE[i] < 0 ? wheel_width : 0);
        const int y = WHEEL_END[i] == 0
            ? body_top + 6
            : body_bottom - 6 - wheel_height;
        HBRUSH wheel_brush = CreateSolidBrush(
            value > 0.001f ? RGB(shade / 3, shade, shade / 2)
                           : RGB(70, 46, 50));
        char text[16];
        RECT text_rect;
        SelectObject(dc, wheel_brush);
        Rectangle(dc, x, y, x + wheel_width, y + wheel_height);
        snprintf(text, sizeof(text), "%.2f", value);
        text_rect.left = WHEEL_SIDE[i] < 0 ? panel.left + 6 : center_x + 30;
        text_rect.right = WHEEL_SIDE[i] < 0 ? center_x - 30 : panel.right - 6;
        text_rect.top = y - 1;
        text_rect.bottom = y + wheel_height;
        SetTextColor(
            dc, value > 0.001f ? RGB(235, 245, 240) : RGB(190, 130, 135));
        DrawTextA(
            dc, text, -1, &text_rect,
            (WHEEL_SIDE[i] < 0 ? DT_RIGHT : DT_LEFT) | DT_VCENTER |
            DT_SINGLELINE);
        SelectObject(dc, body_brush);
        DeleteObject(wheel_brush);
    }

    SelectObject(dc, old_font);
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(panel_brush);
    DeleteObject(body_brush);
    DeleteObject(panel_pen);
    DeleteObject(body_pen);
    DeleteObject(font);
}

/* Race-start overlay: the 3-2-1 digits while the countdown runs, then the START
   flash once it releases. Drawn at the middle of the client area in the same
   heavy face as the speedometer, with a drop shadow so it stays readable over a
   bright track. */
static void kart_demo_draw_countdown(
    HDC dc,
    RECT client,
    unsigned int remaining_ms,
    unsigned int start_notice_ms)
{
    char digits[8];
    const char *label;
    COLORREF colour;
    int height;
    HFONT font;
    HGDIOBJ old_font;
    RECT box;

    if (remaining_ms != 0) {
        const unsigned int seconds = (remaining_ms + 999u) / 1000u;
        if (seconds > 3u) return;
        snprintf(digits, sizeof(digits), "%u", seconds);
        label = digits;
        colour = RGB(245, 248, 250);
        height = -96;
    } else if (start_notice_ms != 0) {
        label = "START!";
        colour = RGB(120, 255, 155);
        height = -64;
    } else {
        return;
    }

    box.left = client.left;
    box.right = client.right;
    box.top = (client.top + client.bottom) / 2 - 130;
    box.bottom = box.top + 200;
    font = CreateFontA(
        height, 0, 0, 0, FW_HEAVY, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        ANTIALIASED_QUALITY, FF_DONTCARE, "Segoe UI");
    old_font = SelectObject(dc, font);
    SetBkMode(dc, TRANSPARENT);
    OffsetRect(&box, 4, 4);
    SetTextColor(dc, RGB(18, 22, 28));
    DrawTextA(dc, label, -1, &box, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    OffsetRect(&box, -4, -4);
    SetTextColor(dc, colour);
    DrawTextA(dc, label, -1, &box, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

    SelectObject(dc, old_font);
    DeleteObject(font);
}

#endif
