#ifndef KART_DEMO_WIN32_UI_H
#define KART_DEMO_WIN32_UI_H

#include <windows.h>

#include "kart_demo_data.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>

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

#endif
