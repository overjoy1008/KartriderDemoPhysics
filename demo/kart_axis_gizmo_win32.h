#ifndef KART_AXIS_GIZMO_WIN32_H
#define KART_AXIS_GIZMO_WIN32_H

/* A world-axis triad, drawn as a fixed screen-corner widget.

   The arrows always show the world X/Y/Z directions, never the kart's body
   axes, so the widget tells you how the world frame is oriented from the
   current viewpoint. Each demo supplies the screen-space direction of the three
   world axes; everything else is shared.

   An axis pointing nearly straight at or away from the viewer has no useful
   screen direction, so it is drawn with the usual "into/out of the page"
   symbol instead of an arrow. */

#include <windows.h>

#include <math.h>

#define KART_AXIS_GIZMO_RADIUS 34
#define KART_AXIS_GIZMO_MARGIN 14

typedef struct KartAxisGizmoAxis {
    float screen_x; /* screen right, in units of the gizmo radius */
    float screen_y; /* screen down */
    float toward;   /* +1 straight at the viewer, -1 straight away */
    COLORREF color;
    const char *label;
} KartAxisGizmoAxis;

static void kart_demo_axis_gizmo_arrow(
    HDC dc,
    POINT center,
    const KartAxisGizmoAxis *axis)
{
    const float length = sqrtf(
        axis->screen_x * axis->screen_x + axis->screen_y * axis->screen_y);
    const int tip_x = center.x + (int)(axis->screen_x * KART_AXIS_GIZMO_RADIUS);
    const int tip_y = center.y + (int)(axis->screen_y * KART_AXIS_GIZMO_RADIUS);
    HPEN pen = CreatePen(PS_SOLID, 2, axis->color);
    HGDIOBJ old_pen = SelectObject(dc, pen);
    HBRUSH brush = CreateSolidBrush(axis->color);
    HGDIOBJ old_brush = SelectObject(dc, brush);
    int label_x = tip_x;
    int label_y = tip_y;

    if (length < 0.18f) {
        /* Too close to the view direction to draw as an arrow. A ring with a
           filled centre reads as pointing at the viewer, a plain ring as
           pointing away. */
        const int ring = 7;
        SelectObject(dc, GetStockObject(NULL_BRUSH));
        Ellipse(dc, center.x - ring, center.y - ring,
                center.x + ring, center.y + ring);
        if (axis->toward >= 0.0f) {
            SelectObject(dc, brush);
            Ellipse(dc, center.x - 3, center.y - 3,
                    center.x + 3, center.y + 3);
        }
        /* Up and to the left, which is the quadrant the in-plane axes are
           least likely to occupy in either demo. */
        label_x = center.x - ring - 12;
        label_y = center.y - ring - 12;
    } else {
        const float unit_x = axis->screen_x / length;
        const float unit_y = axis->screen_y / length;
        const float head = 9.0f;
        const float width = 4.0f;
        POINT head_points[3];
        MoveToEx(dc, center.x, center.y, NULL);
        LineTo(dc, tip_x, tip_y);
        head_points[0].x = tip_x;
        head_points[0].y = tip_y;
        head_points[1].x = (int)(tip_x - unit_x * head - unit_y * width);
        head_points[1].y = (int)(tip_y - unit_y * head + unit_x * width);
        head_points[2].x = (int)(tip_x - unit_x * head + unit_y * width);
        head_points[2].y = (int)(tip_y - unit_y * head - unit_x * width);
        Polygon(dc, head_points, 3);
        label_x = (int)(tip_x + unit_x * 9.0f) - 4;
        label_y = (int)(tip_y + unit_y * 9.0f) - 7;
    }

    SetTextColor(dc, axis->color);
    TextOutA(dc, label_x, label_y, axis->label, 1);

    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(brush);
    DeleteObject(pen);
}

/* axis_x/axis_y/axis_toward are indexed world X, Y, Z. axis_y grows downward,
   matching screen coordinates. */
static void kart_demo_draw_axis_gizmo(
    HDC dc,
    RECT client,
    const float axis_x[3],
    const float axis_y[3],
    const float axis_toward[3])
{
    static const COLORREF COLORS[3] = {
        RGB(255, 96, 96),  /* X */
        RGB(120, 235, 120), /* Y */
        RGB(120, 170, 255), /* Z */
    };
    static const char *const LABELS[3] = {"X", "Y", "Z"};
    KartAxisGizmoAxis axes[3];
    /* Leaves room for an arrowhead plus its label in any direction. */
    const int span = KART_AXIS_GIZMO_RADIUS + 20;
    const POINT center = {
        client.left + KART_AXIS_GIZMO_MARGIN + span,
        client.bottom - KART_AXIS_GIZMO_MARGIN - span,
    };
    int order[3] = {0, 1, 2};
    int i;
    int j;
    int old_mode;

    for (i = 0; i < 3; ++i) {
        axes[i].screen_x = axis_x[i];
        axes[i].screen_y = axis_y[i];
        axes[i].toward = axis_toward[i];
        axes[i].color = COLORS[i];
        axes[i].label = LABELS[i];
    }
    /* Draw the axis furthest from the viewer first so nearer arrows overlap. */
    for (i = 0; i < 3; ++i) {
        for (j = i + 1; j < 3; ++j) {
            if (axes[order[j]].toward < axes[order[i]].toward) {
                const int swap = order[i];
                order[i] = order[j];
                order[j] = swap;
            }
        }
    }

    old_mode = SetBkMode(dc, TRANSPARENT);
    for (i = 0; i < 3; ++i) {
        kart_demo_axis_gizmo_arrow(dc, center, &axes[order[i]]);
    }
    /* Above the triad: the downward arrow would otherwise run into it. */
    SetTextColor(dc, RGB(150, 150, 160));
    TextOutA(dc, client.left + KART_AXIS_GIZMO_MARGIN, center.y - span - 4,
             "WORLD", 5);
    SetBkMode(dc, old_mode);
}

#endif
