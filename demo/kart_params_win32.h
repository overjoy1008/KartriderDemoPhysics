#ifndef KART_PARAMS_WIN32_H
#define KART_PARAMS_WIN32_H

/* Live editor for the kart's KartDynamicsConfig. Opened with P, it writes
   straight into the running simulation's config copy, so every field takes
   effect on the next step without a reset. */

#include <windows.h>

#include "kart_dynamics.h"
#include "kart_gauge.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#define KART_PARAM_DYNAMICS_COUNT 18
#define KART_PARAM_GAUGE_COUNT 4
#define KART_PARAM_FIELD_COUNT \
    (KART_PARAM_DYNAMICS_COUNT + KART_PARAM_GAUGE_COUNT)
#define KART_PARAM_EDIT_BASE 3000
#define KART_PARAM_APPLY 3100
#define KART_PARAM_RELOAD 3101
#define KART_PARAM_DEFAULTS 3102

typedef struct KartParamField {
    const char *label;
    size_t offset;
    int decimals;
} KartParamField;

/* Declaration order of KartDynamicsConfig, then of KartGaugeConfig. */
static const KartParamField KART_PARAM_FIELDS[KART_PARAM_FIELD_COUNT] = {
    {"mass", offsetof(KartDynamicsConfig, mass), 1},
    {"air friction", offsetof(KartDynamicsConfig, air_friction), 3},
    {"drag factor", offsetof(KartDynamicsConfig, drag_factor), 3},
    {"forward force", offsetof(KartDynamicsConfig, forward_accel_force), 1},
    {"reverse force", offsetof(KartDynamicsConfig, backward_accel_force), 1},
    {"grip brake", offsetof(KartDynamicsConfig, grip_brake_force), 1},
    {"slip brake", offsetof(KartDynamicsConfig, slip_brake_force), 1},
    {"steer angle deg", offsetof(KartDynamicsConfig, max_steer_angle_deg), 2},
    {"steer constraint", offsetof(KartDynamicsConfig, steer_constraint), 2},
    {"front grip", offsetof(KartDynamicsConfig, front_grip_factor), 3},
    {"rear grip", offsetof(KartDynamicsConfig, rear_grip_factor), 3},
    {"trigger factor", offsetof(KartDynamicsConfig, drift_trigger_factor), 3},
    {"trigger time", offsetof(KartDynamicsConfig, drift_trigger_time), 3},
    {"drift slip", offsetof(KartDynamicsConfig, drift_slip_factor), 3},
    {"escape force", offsetof(KartDynamicsConfig, drift_escape_force), 1},
    {"corner draw", offsetof(KartDynamicsConfig, corner_draw_factor), 3},
    {"drift lean", offsetof(KartDynamicsConfig, drift_lean_factor), 3},
    {"steer lean", offsetof(KartDynamicsConfig, steer_lean_factor), 3},
    {"gauge Kg", offsetof(KartGaugeConfig, charge_factor), 3},
    {"gauge full", offsetof(KartGaugeConfig, full_value), 1},
    {"gauge Ks", offsetof(KartGaugeConfig, suspension_gain), 3},
    {"gauge Ks max", offsetof(KartGaugeConfig, suspension_max), 3},
};

/* The window edits whatever these point at; the caller keeps ownership. */
static KartDynamicsConfig *g_kart_param_target = NULL;
static const KartDynamicsConfig *g_kart_param_defaults = NULL;
static KartGaugeConfig *g_kart_gauge_target = NULL;
static HWND g_kart_param_window = NULL;
static HWND g_kart_param_edits[KART_PARAM_FIELD_COUNT];

/* Rows past the dynamics block address the gauge config instead. */
static float *kart_param_slot(unsigned int i)
{
    void *base = i < KART_PARAM_DYNAMICS_COUNT
        ? (void *)g_kart_param_target
        : (void *)g_kart_gauge_target;
    if (base == NULL) return NULL;
    return (float *)((char *)base + KART_PARAM_FIELDS[i].offset);
}

static void kart_param_fill_edits(void)
{
    unsigned int i;
    char text[32];
    for (i = 0; i < KART_PARAM_FIELD_COUNT; ++i) {
        const float *slot = kart_param_slot(i);
        if (slot == NULL) continue;
        snprintf(
            text, sizeof(text), "%.*f",
            KART_PARAM_FIELDS[i].decimals, *slot);
        SetWindowTextA(g_kart_param_edits[i], text);
    }
}

/* Blank or unparseable text leaves the field alone rather than zeroing it. */
static void kart_param_apply_edits(void)
{
    unsigned int i;
    char text[32];
    for (i = 0; i < KART_PARAM_FIELD_COUNT; ++i) {
        float *slot = kart_param_slot(i);
        char *end = text;
        double parsed;
        if (slot == NULL) continue;
        if (GetWindowTextA(g_kart_param_edits[i], text, sizeof(text)) == 0) {
            continue;
        }
        parsed = strtod(text, &end);
        if (end == text) continue;
        *slot = (float)parsed;
    }
    kart_param_fill_edits();
}

static LRESULT CALLBACK kart_param_window_proc(
    HWND window, UINT message, WPARAM wparam, LPARAM lparam)
{
    switch (message) {
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case KART_PARAM_APPLY:
            kart_param_apply_edits();
            return 0;
        case KART_PARAM_RELOAD:
            kart_param_fill_edits();
            return 0;
        case KART_PARAM_DEFAULTS:
            if (g_kart_param_target != NULL && g_kart_param_defaults != NULL) {
                *g_kart_param_target = *g_kart_param_defaults;
                if (g_kart_gauge_target != NULL) {
                    *g_kart_gauge_target = kart_gauge_default_config();
                }
                kart_param_fill_edits();
            }
            return 0;
        default:
            break;
        }
        return 0;
    case WM_CLOSE:
        DestroyWindow(window);
        return 0;
    case WM_DESTROY:
        g_kart_param_window = NULL;
        return 0;
    default:
        break;
    }
    return DefWindowProc(window, message, wparam, lparam);
}

/* Opens the editor, or brings it forward when it is already up. */
static void kart_demo_params_open(
    HINSTANCE instance,
    HWND owner,
    KartDynamicsConfig *target,
    const KartDynamicsConfig *defaults,
    KartGaugeConfig *gauge)
{
    static const char CLASS_NAME[] = "KartPhysicsParamWindow";
    static bool registered = false;
    const int row_height = 26;
    const int label_width = 118;
    const int edit_width = 86;
    const int column_width = label_width + edit_width + 18;
    /* Two columns of dynamics, then the gauge coefficients in a third. */
    const int rows = KART_PARAM_DYNAMICS_COUNT / 2;
    const int width = column_width * 3 + 24;
    const int height = rows * row_height + 78;
    HFONT font = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
    unsigned int i;

    g_kart_param_target = target;
    g_kart_param_defaults = defaults;
    g_kart_gauge_target = gauge;
    if (g_kart_param_window != NULL) {
        kart_param_fill_edits();
        SetForegroundWindow(g_kart_param_window);
        return;
    }
    if (!registered) {
        WNDCLASSA window_class = {0};
        window_class.lpfnWndProc = kart_param_window_proc;
        window_class.hInstance = instance;
        window_class.lpszClassName = CLASS_NAME;
        window_class.hCursor = LoadCursor(NULL, IDC_ARROW);
        window_class.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
        if (!RegisterClassA(&window_class)) return;
        registered = true;
    }
    g_kart_param_window = CreateWindowExA(
        WS_EX_TOOLWINDOW,
        CLASS_NAME,
        "Kart parameters",
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU,
        CW_USEDEFAULT, CW_USEDEFAULT, width, height,
        owner, NULL, instance, NULL);
    if (g_kart_param_window == NULL) return;

    for (i = 0; i < KART_PARAM_FIELD_COUNT; ++i) {
        const int column = (int)i / rows;
        const int row = (int)i % rows;
        const int x = 12 + column * column_width;
        const int y = 10 + row * row_height;
        HWND label = CreateWindowExA(
            0, "STATIC", KART_PARAM_FIELDS[i].label,
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            x, y + 3, label_width, 18,
            g_kart_param_window, NULL, instance, NULL);
        g_kart_param_edits[i] = CreateWindowExA(
            WS_EX_CLIENTEDGE, "EDIT", "",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_LEFT | ES_AUTOHSCROLL,
            x + label_width, y, edit_width, 21,
            g_kart_param_window, (HMENU)(UINT_PTR)(KART_PARAM_EDIT_BASE + i),
            instance, NULL);
        SendMessage(label, WM_SETFONT, (WPARAM)font, TRUE);
        SendMessage(g_kart_param_edits[i], WM_SETFONT, (WPARAM)font, TRUE);
    }
    {
        static const struct {
            const char *label;
            int command;
        } BUTTONS[3] = {
            {"Apply", KART_PARAM_APPLY},
            {"Reload", KART_PARAM_RELOAD},
            {"Kart defaults", KART_PARAM_DEFAULTS},
        };
        const int y = 14 + rows * row_height;
        int x = 12;
        int button;
        for (button = 0; button < 3; ++button) {
            const int button_width = button == 2 ? 110 : 74;
            HWND control = CreateWindowExA(
                0, "BUTTON", BUTTONS[button].label,
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
                x, y, button_width, 24,
                g_kart_param_window,
                (HMENU)(UINT_PTR)BUTTONS[button].command, instance, NULL);
            SendMessage(control, WM_SETFONT, (WPARAM)font, TRUE);
            x += button_width + 8;
        }
    }
    kart_param_fill_edits();
    ShowWindow(g_kart_param_window, SW_SHOW);
    UpdateWindow(g_kart_param_window);
}

#endif
