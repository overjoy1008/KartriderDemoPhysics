#import <Cocoa/Cocoa.h>

#include "kart_demo_data.h"
#include "kart_simulation.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#define MAX_SKIDS 2048
#define SKID_LIFETIME_MS 15000

typedef struct MacSkid {
    float lx0, ly0, lx1, ly1;
    float rx0, ry0, rx1, ry1;
    unsigned int created_ms;
} MacSkid;

typedef struct MacDemoState {
    KartSimulationState kart;
    const KartDemoKartSpec *kart_spec;
    const KartDemoTrackSpec *track_spec;
    unsigned int simulation_time_ms;
    unsigned int skid_head;
    unsigned int skid_count;
    MacSkid skids[MAX_SKIDS];
    float previous_left_x, previous_left_y;
    float previous_right_x, previous_right_y;
    bool previous_skid_active;
    bool boost_active;
    bool drag_trigger_active;
} MacDemoState;

typedef struct MacCamera {
    KartVec3 position, right, up, forward;
    float focal;
} MacCamera;

typedef struct MacProjected {
    NSPoint point;
    float depth;
    bool visible;
} MacProjected;

static KartVec3 vadd(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static KartVec3 vsub(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static KartVec3 vscale(KartVec3 value, float amount)
{
    return (KartVec3){value.x * amount, value.y * amount, value.z * amount};
}

static float vdot(KartVec3 a, KartVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static KartVec3 vcross(KartVec3 a, KartVec3 b)
{
    return (KartVec3){
        a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}

static KartVec3 vnormalize(KartVec3 value)
{
    float length=sqrtf(vdot(value,value));
    return length>0 ? vscale(value,1.0f/length) : (KartVec3){0};
}

static void kart_axes(KartQuat q, KartVec3 *right, KartVec3 *forward, KartVec3 *up)
{
    const float xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
    const float xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
    const float wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;
    *right = (KartVec3){1-2*(yy+zz), 2*(xy+wz), 2*(xz-wy)};
    *forward = (KartVec3){-2*(xy-wz), -(1-2*(xx+zz)), -2*(yz+wx)};
    *up = (KartVec3){2*(xz+wy), 2*(yz-wx), 1-2*(xx+yy)};
}

static bool query_ground(void *user, KartVec3 start, KartVec3 delta, KartGroundHit *hit)
{
    float fraction;
    (void)user;
    if (delta.z >= 0 || start.z < 0 || start.z + delta.z > 0) return false;
    fraction = -start.z / delta.z;
    hit->point = (KartVec3){start.x + delta.x*fraction, start.y + delta.y*fraction, 0};
    hit->normal = (KartVec3){0, 0, 1};
    hit->surface_id = 1;
    return true;
}

static unsigned int query_walls(
    void *user, const KartSimulationState *kart,
    KartBodyContact *contacts, unsigned int capacity)
{
    MacDemoState *demo = (MacDemoState *)user;
    const float hw = kart_demo_track_width(demo->track_spec) * 0.5f;
    const float hh = kart_demo_track_length(demo->track_spec) * 0.5f;
    unsigned int count = 0;
#define CONTACT(nx, ny) do { if (count < capacity) { \
    contacts[count++] = (KartBodyContact){{(nx), (ny), 0}, 0.5f, 2}; } } while (0)
    if (kart->position.x >= hw && kart->linear_velocity.x > 0) CONTACT(-1, 0);
    if (kart->position.x <= -hw && kart->linear_velocity.x < 0) CONTACT(1, 0);
    if (kart->position.y >= hh && kart->linear_velocity.y > 0) CONTACT(0, -1);
    if (kart->position.y <= -hh && kart->linear_velocity.y < 0) CONTACT(0, 1);
#undef CONTACT
    return count;
}

static bool drift_active(const KartSimulationState *kart)
{
    return kart->drift.input_active || kart->drift.trigger_active ||
           kart->drift.slip_detected || kart->drift.linger_timer > 0;
}

static void reset_demo(MacDemoState *demo)
{
    if (!demo->kart_spec) demo->kart_spec = kart_demo_default_kart();
    if (!demo->track_spec) demo->track_spec = kart_demo_default_track();
    kart_simulation_init(&demo->kart, &demo->kart_spec->dynamics, &demo->kart_spec->geometry);
    demo->simulation_time_ms = 0;
    demo->skid_head = demo->skid_count = 0;
    demo->previous_skid_active = false;
    demo->boost_active = false;
    demo->drag_trigger_active = false;
}

static void update_skids(MacDemoState *demo)
{
    KartVec3 right, forward, up;
    const float speed = hypotf(demo->kart.linear_velocity.x, demo->kart.linear_velocity.y);
    float rear_x, rear_y, lx, ly, rx, ry;
    bool active;
    kart_axes(demo->kart.orientation, &right, &forward, &up);
    (void)up;
    rear_x = demo->kart.position.x - forward.x * demo->kart.geometry.half_length * 0.8f;
    rear_y = demo->kart.position.y - forward.y * demo->kart.geometry.half_length * 0.8f;
    lx = rear_x - right.x * demo->kart.geometry.half_width * 0.8f;
    ly = rear_y - right.y * demo->kart.geometry.half_width * 0.8f;
    rx = rear_x + right.x * demo->kart.geometry.half_width * 0.8f;
    ry = rear_y + right.y * demo->kart.geometry.half_width * 0.8f;
    active = demo->kart.grounded && speed > 5 && drift_active(&demo->kart);
    if (active && demo->previous_skid_active) {
        demo->skids[demo->skid_head] = (MacSkid){
            demo->previous_left_x, demo->previous_left_y, lx, ly,
            demo->previous_right_x, demo->previous_right_y, rx, ry,
            demo->simulation_time_ms};
        demo->skid_head = (demo->skid_head + 1) % MAX_SKIDS;
        if (demo->skid_count < MAX_SKIDS) demo->skid_count++;
    }
    demo->previous_left_x = lx; demo->previous_left_y = ly;
    demo->previous_right_x = rx; demo->previous_right_y = ry;
    demo->previous_skid_active = active;
}

static void stroke_line(NSPoint a, NSPoint b, NSColor *color, CGFloat width)
{
    NSBezierPath *path = [NSBezierPath bezierPath];
    [color setStroke];
    [path setLineWidth:width];
    [path moveToPoint:a];
    [path lineToPoint:b];
    [path stroke];
}

static NSColor *rgb(unsigned int red, unsigned int green, unsigned int blue)
{
    return [NSColor colorWithSRGBRed:(CGFloat)red / 255.0
                               green:(CGFloat)green / 255.0
                                blue:(CGFloat)blue / 255.0
                               alpha:1.0];
}

static NSColor *kart_body_color(const MacDemoState *demo)
{
    if (demo->boost_active) return rgb(65, 205, 255);
    if (drift_active(&demo->kart)) return rgb(255, 165, 35);
    return rgb(235, 65, 80);
}

static NSColor *skid_color(unsigned int age)
{
    if (age < 4000) return rgb(8, 10, 12);
    if (age < 9000) return rgb(23, 26, 29);
    return rgb(43, 47, 51);
}

static void fill_polygon(const NSPoint *points, unsigned int count, NSColor *fill, NSColor *stroke)
{
    NSBezierPath *path = [NSBezierPath bezierPath];
    if (!count) return;
    [path moveToPoint:points[0]];
    for (unsigned int i = 1; i < count; ++i) [path lineToPoint:points[i]];
    [path closePath];
    [fill setFill]; [path fill];
    [stroke setStroke]; [path setLineWidth:1.5]; [path stroke];
}

static MacCamera make_camera(const MacDemoState *demo, NSRect bounds)
{
    const KartVec3 world_up={0,0,1};
    KartVec3 body_right, body_forward, body_up, flat_forward, target;
    MacCamera camera;
    kart_axes(demo->kart.orientation,&body_right,&body_forward,&body_up);
    (void)body_right; (void)body_up;
    flat_forward=vnormalize((KartVec3){body_forward.x,body_forward.y,0});
    if(vdot(flat_forward,flat_forward)==0) flat_forward=(KartVec3){0,-1,0};
    target=vadd(demo->kart.position,vadd(vscale(flat_forward,4),(KartVec3){0,0,.8f}));
    camera.position=vadd(demo->kart.position,vadd(vscale(flat_forward,-11),(KartVec3){0,0,7}));
    camera.forward=vnormalize(vsub(target,camera.position));
    camera.right=vnormalize(vcross(world_up,camera.forward));
    camera.up=vcross(camera.forward,camera.right);
    camera.focal=(float)fmin(bounds.size.width,bounds.size.height)*.85f;
    return camera;
}

static MacProjected project(NSRect bounds, MacCamera camera, KartVec3 world)
{
    KartVec3 relative = vsub(world, camera.position);
    MacProjected result;
    result.depth = vdot(relative, camera.forward);
    result.visible = result.depth > 0.1f;
    result.point = result.visible ? NSMakePoint(
        NSMidX(bounds) + camera.focal * vdot(relative, camera.right) / result.depth,
        bounds.size.height * 0.52f - camera.focal * vdot(relative, camera.up) / result.depth)
        : NSZeroPoint;
    return result;
}

static void line3d(NSRect bounds, MacCamera camera, KartVec3 a, KartVec3 b, NSColor *color, CGFloat width)
{
    float da = vdot(vsub(a, camera.position), camera.forward);
    float db = vdot(vsub(b, camera.position), camera.forward);
    const float near = 0.12f;
    if (da <= near && db <= near) return;
    if (da <= near) {
        float t = (near-da)/(db-da);
        a = vadd(a, vscale(vsub(b,a), t));
    } else if (db <= near) {
        float t = (near-db)/(da-db);
        b = vadd(b, vscale(vsub(a,b), t));
    }
    MacProjected pa = project(bounds, camera, a), pb = project(bounds, camera, b);
    if (pa.visible && pb.visible) stroke_line(pa.point, pb.point, color, width);
}

@interface KartDemoView : NSView {
@private
    MacDemoState _demo;
    BOOL _keys[128];
    NSEventModifierFlags _modifiers;
    NSTimer *_timer;
    NSTimeInterval _previousTime;
}
@end

@implementation KartDemoView

- (BOOL)isFlipped { return YES; }
- (BOOL)acceptsFirstResponder { return YES; }

- (instancetype)initWithFrame:(NSRect)frame
{
    self = [super initWithFrame:frame];
    if (self) {
        memset(&_demo, 0, sizeof(_demo));
        memset(_keys, 0, sizeof(_keys));
        reset_demo(&_demo);
        _previousTime = [NSProcessInfo processInfo].systemUptime;
        _timer = [NSTimer scheduledTimerWithTimeInterval:1.0/60.0
            target:self selector:@selector(tick:) userInfo:nil repeats:YES];
    }
    return self;
}

- (void)dealloc { [_timer invalidate]; }

- (void)selectKart:(NSMenuItem *)sender
{
    _demo.kart_spec = kart_demo_kart_at(sender.tag);
    reset_demo(&_demo);
}

- (void)selectTrack:(NSMenuItem *)sender
{
    _demo.track_spec = kart_demo_track_at(sender.tag);
    reset_demo(&_demo);
}

- (void)showKartMenu
{
    NSMenu *menu = [[NSMenu alloc] initWithTitle:@"Select Kart"];
    for (unsigned int i=0; i<kart_demo_kart_count(); ++i) {
        const KartDemoKartSpec *spec = kart_demo_kart_at(i);
        NSString *title = [NSString stringWithFormat:@"%s   %.3f x %.3f",
            spec->asset_name, spec->geometry.half_width*2, spec->geometry.half_length*2];
        NSMenuItem *item = [[NSMenuItem alloc] initWithTitle:title action:@selector(selectKart:) keyEquivalent:@""];
        item.target = self; item.tag = i;
        item.state = spec == _demo.kart_spec ? NSControlStateValueOn : NSControlStateValueOff;
        [menu addItem:item];
    }
    [menu popUpMenuPositioningItem:nil atLocation:NSMakePoint(30,70) inView:self];
    _previousTime = [NSProcessInfo processInfo].systemUptime;
}

- (void)showTrackMenu
{
    NSMenu *menu = [[NSMenu alloc] initWithTitle:@"Select Track"];
    for (unsigned int i=0; i<kart_demo_track_count(); ++i) {
        const KartDemoTrackSpec *spec = kart_demo_track_at(i);
        NSString *title = [NSString stringWithFormat:@"%s (%s)   %.1f x %.1f",
            spec->display_name, spec->asset_name,
            kart_demo_track_width(spec), kart_demo_track_length(spec)];
        NSMenuItem *item = [[NSMenuItem alloc] initWithTitle:title action:@selector(selectTrack:) keyEquivalent:@""];
        item.target = self; item.tag = i;
        item.state = spec == _demo.track_spec ? NSControlStateValueOn : NSControlStateValueOff;
        [menu addItem:item];
    }
    [menu popUpMenuPositioningItem:nil atLocation:NSMakePoint(280,70) inView:self];
    _previousTime = [NSProcessInfo processInfo].systemUptime;
}

- (void)keyDown:(NSEvent *)event
{
    unsigned short key = event.keyCode;
    if (key == 40) { [self showKartMenu]; return; }
    if (key == 17) { [self showTrackMenu]; return; }
    if (key == 5) {
        kart_simulation_multiply_grounded_drag_scale(
            &_demo.kart, _demo.drag_trigger_active ? 0.25f : 4.0f);
        _demo.drag_trigger_active = !_demo.drag_trigger_active;
        return;
    }
    if (key < 128) _keys[key] = YES;
}

- (void)keyUp:(NSEvent *)event
{
    if (event.keyCode < 128) _keys[event.keyCode] = NO;
}

- (void)flagsChanged:(NSEvent *)event { _modifiers = event.modifierFlags; }

- (void)tick:(NSTimer *)timer
{
    NSTimeInterval now = [NSProcessInfo processInfo].systemUptime;
    unsigned int elapsed = (unsigned int)((now - _previousTime) * 1000.0);
    KartSimulationControls controls = {0};
    KartSimulationWorld world = {query_ground, query_walls, &_demo};
    (void)timer;
    _previousTime = now;
    if (elapsed > 50) elapsed = 50;
    controls.forward_input = _keys[126] ? 1 : 0;
    controls.reverse_input = _keys[125] ? 1 : 0;
    controls.steering_input = (_keys[123] ? -1.0f : 0) + (_keys[124] ? 1.0f : 0);
    controls.drift_input = (_modifiers & NSEventModifierFlagShift) != 0 || _keys[13];
    controls.boost_active = (_modifiers & NSEventModifierFlagCommand) != 0 || _keys[2];
    if (_keys[15]) reset_demo(&_demo);
    else if (elapsed) kart_simulate_milliseconds(&_demo.kart, &controls, &world, elapsed);
    _demo.boost_active = kart_any_boost_active(&_demo.kart.timed_boost, &_demo.kart.instant_boost);
    _demo.simulation_time_ms += elapsed;
    update_skids(&_demo);
    [self setNeedsDisplay:YES];
}

- (NSDictionary *)textAttributes:(CGFloat)size color:(NSColor *)color bold:(BOOL)bold
{
    NSFont *font = bold ? [NSFont boldSystemFontOfSize:size] : [NSFont systemFontOfSize:size];
    return @{NSFontAttributeName:font, NSForegroundColorAttributeName:color};
}

- (void)drawSpeedometer
{
    NSRect bounds = self.bounds;
    NSRect panel = NSMakeRect(bounds.size.width-256, bounds.size.height-112, 238, 94);
    NSBezierPath *box = [NSBezierPath bezierPathWithRoundedRect:panel xRadius:8 yRadius:8];
    [rgb(12,16,22) setFill]; [box fill];
    [(_demo.boost_active ? rgb(75,225,255) : rgb(110,130,145)) setStroke];
    [box setLineWidth:3]; [box stroke];
    int kmh = kart_speedometer_kmh(_demo.kart.linear_velocity);
    NSString *digits = [NSString stringWithFormat:@"%03d", kmh];
    [digits drawInRect:NSMakeRect(panel.origin.x+8,panel.origin.y+4,165,70)
        withAttributes:[self textAttributes:52 color:(_demo.boost_active?rgb(90,235,255):rgb(245,248,250)) bold:YES]];
    [@"KM/H" drawAtPoint:NSMakePoint(panel.origin.x+176,panel.origin.y+52)
        withAttributes:[self textAttributes:15 color:rgb(175,195,205) bold:YES]];
}

- (NSPoint)topPointX:(float)x y:(float)y scale:(float)scale
{
    return NSMakePoint(NSMidX(self.bounds)+(x-_demo.kart.position.x)*scale,
                       NSMidY(self.bounds)+(y-_demo.kart.position.y)*scale);
}

- (void)drawMinimap
{
    NSRect panel = NSMakeRect(self.bounds.size.width-236,16,220,190);
    [rgb(15,19,26) setFill]; NSRectFill(panel);
    NSBezierPath *panelPath=[NSBezierPath bezierPathWithRect:panel];
    [rgb(54,64,73) setStroke]; [panelPath setLineWidth:1]; [panelPath stroke];
    float tw=kart_demo_track_width(_demo.track_spec), th=kart_demo_track_length(_demo.track_spec);
    float scale=fminf(200.0f/tw,132.0f/th);
    float centerY=(NSMinY(panel)+42+NSMaxY(panel)-10)*0.5f;
    NSRect track=NSMakeRect(NSMidX(panel)-tw*scale/2,centerY-th*scale/2,tw*scale,th*scale);
    for(int division=1;division<4;division++) {
        float x=NSMinX(track)+track.size.width*division/4.0f;
        float y=NSMinY(track)+track.size.height*division/4.0f;
        stroke_line(NSMakePoint(x,NSMinY(track)),NSMakePoint(x,NSMaxY(track)),rgb(86,98,108),1);
        stroke_line(NSMakePoint(NSMinX(track),y),NSMakePoint(NSMaxX(track),y),rgb(86,98,108),1);
    }
    NSBezierPath *p=[NSBezierPath bezierPathWithRect:track]; [rgb(75,220,255) setStroke]; [p setLineWidth:3]; [p stroke];
    KartVec3 r,f,u; kart_axes(_demo.kart.orientation,&r,&f,&u); (void)u;
    NSPoint center=NSMakePoint(NSMidX(track)+_demo.kart.position.x*scale,centerY+_demo.kart.position.y*scale);
    NSPoint tri[3]={
        NSMakePoint(center.x+f.x*9,center.y+f.y*9),
        NSMakePoint(center.x-f.x*6+r.x*5,center.y-f.y*6+r.y*5),
        NSMakePoint(center.x-f.x*6-r.x*5,center.y-f.y*6-r.y*5)};
    fill_polygon(tri,3,rgb(255,180,45),rgb(255,235,175));
    [@"TRACK BOUNDS" drawAtPoint:NSMakePoint(NSMinX(panel)+8,NSMinY(panel)+6)
        withAttributes:[self textAttributes:12 color:rgb(180,205,215) bold:NO]];
    NSString *size=[NSString stringWithFormat:@"KART %s  %.3f x %.3f",
        _demo.kart_spec->asset_name,_demo.kart.geometry.half_width*2,
        _demo.kart.geometry.half_length*2];
    [size drawAtPoint:NSMakePoint(NSMinX(panel)+8,NSMinY(panel)+22)
        withAttributes:[self textAttributes:12 color:rgb(180,205,215) bold:NO]];
}

- (void)drawTopDown
{
    const float scale=fminf((self.bounds.size.width-90.0f)/110.0f,(self.bounds.size.height-90.0f)/76.0f);
    float left=_demo.kart.position.x-self.bounds.size.width/(2*scale);
    float right=_demo.kart.position.x+self.bounds.size.width/(2*scale);
    float top=_demo.kart.position.y-self.bounds.size.height/(2*scale);
    float bottom=_demo.kart.position.y+self.bounds.size.height/(2*scale);
    float hw=kart_demo_track_width(_demo.track_spec)/2, hh=kart_demo_track_length(_demo.track_spec)/2;
    NSPoint trackMin=[self topPointX:-hw y:-hh scale:scale];
    NSPoint trackMax=[self topPointX:hw y:hh scale:scale];
    NSRect trackRect=NSMakeRect(fmin(trackMin.x,trackMax.x),fmin(trackMin.y,trackMax.y),
        fabs(trackMax.x-trackMin.x),fabs(trackMax.y-trackMin.y));
    [rgb(50,58,65) setFill]; NSRectFill(trackRect);
    int gx=(int)floorf(left/10)*10, gy=(int)floorf(top/10)*10;
    for(float x=gx;x<=right;x+=10) stroke_line([self topPointX:x y:top scale:scale],[self topPointX:x y:bottom scale:scale],rgb(66,75,82),1);
    for(float y=gy;y<=bottom;y+=10) stroke_line([self topPointX:left y:y scale:scale],[self topPointX:right y:y scale:scale],rgb(66,75,82),1);
    NSPoint corners[4]={{0}};
    corners[0]=[self topPointX:-hw y:-hh scale:scale]; corners[1]=[self topPointX:hw y:-hh scale:scale];
    corners[2]=[self topPointX:hw y:hh scale:scale]; corners[3]=[self topPointX:-hw y:hh scale:scale];
    for(int i=0;i<4;i++) stroke_line(corners[i],corners[(i+1)%4],rgb(100,210,255),4);
    unsigned int start=(_demo.skid_head+MAX_SKIDS-_demo.skid_count)%MAX_SKIDS;
    for(unsigned int i=0;i<_demo.skid_count;i++) { MacSkid *s=&_demo.skids[(start+i)%MAX_SKIDS];
        unsigned int age=_demo.simulation_time_ms-s->created_ms;
        if(age>SKID_LIFETIME_MS) continue;
        NSColor *color=skid_color(age); CGFloat width=age<9000?3:2;
        stroke_line([self topPointX:s->lx0 y:s->ly0 scale:scale],[self topPointX:s->lx1 y:s->ly1 scale:scale],color,width);
        stroke_line([self topPointX:s->rx0 y:s->ry0 scale:scale],[self topPointX:s->rx1 y:s->ry1 scale:scale],color,width); }
    KartVec3 r,f,u; kart_axes(_demo.kart.orientation,&r,&f,&u); (void)u;
    NSPoint center=[self topPointX:_demo.kart.position.x y:_demo.kart.position.y scale:scale];
    if(_demo.boost_active) {
        NSPoint flame[3]={
            [self topPointX:_demo.kart.position.x-f.x*3.2f y:_demo.kart.position.y-f.y*3.2f scale:scale],
            [self topPointX:_demo.kart.position.x-f.x*.7f+r.x*.48f y:_demo.kart.position.y-f.y*.7f+r.y*.48f scale:scale],
            [self topPointX:_demo.kart.position.x-f.x*.7f-r.x*.48f y:_demo.kart.position.y-f.y*.7f-r.y*.48f scale:scale]};
        fill_polygon(flame,3,rgb(255,205,45),rgb(255,205,45));
    }
    NSPoint body[4]; float w=_demo.kart.geometry.half_width,l=_demo.kart.geometry.half_length;
    const float sx[4]={-w,w,w,-w}, sy[4]={-l,-l,l,l};
    for(int i=0;i<4;i++) body[i]=[self topPointX:_demo.kart.position.x+r.x*sx[i]+f.x*sy[i] y:_demo.kart.position.y+r.y*sx[i]+f.y*sy[i] scale:scale];
    fill_polygon(body,4,kart_body_color(&_demo),rgb(255,230,220));
    NSPoint nose=[self topPointX:_demo.kart.position.x+f.x*l y:_demo.kart.position.y+f.y*l scale:scale];
    stroke_line(center,nose,rgb(255,245,180),2);
    float speed=hypotf(_demo.kart.linear_velocity.x,_demo.kart.linear_velocity.y);
    if(speed>.01f) {
        float arrow=fminf(speed*.18f,9.0f);
        NSPoint velocity=[self topPointX:_demo.kart.position.x+_demo.kart.linear_velocity.x/speed*arrow
            y:_demo.kart.position.y+_demo.kart.linear_velocity.y/speed*arrow scale:scale];
        stroke_line(center,velocity,rgb(80,230,255),3);
    }
}

- (void)draw3D
{
    MacCamera camera=make_camera(&_demo,self.bounds);
    float hw=kart_demo_track_width(_demo.track_spec)/2, hh=kart_demo_track_length(_demo.track_spec)/2;
    const float gridRadius=140.0f;
    int x0=(int)floorf((_demo.kart.position.x-gridRadius)/10)*10;
    int y0=(int)floorf((_demo.kart.position.y-gridRadius)/10)*10;
    for(float x=x0;x<=_demo.kart.position.x+gridRadius;x+=10) line3d(self.bounds,camera,(KartVec3){x,fmaxf(-hh,_demo.kart.position.y-gridRadius),0},(KartVec3){x,fminf(hh,_demo.kart.position.y+gridRadius),0},rgb(86,98,108),1);
    for(float y=y0;y<=_demo.kart.position.y+gridRadius;y+=10) line3d(self.bounds,camera,(KartVec3){fmaxf(-hw,_demo.kart.position.x-gridRadius),y,0},(KartVec3){fminf(hw,_demo.kart.position.x+gridRadius),y,0},rgb(86,98,108),1);
    KartVec3 wall[4]={{-hw,-hh,0},{hw,-hh,0},{hw,hh,0},{-hw,hh,0}};
    for(int i=0;i<4;i++) { line3d(self.bounds,camera,wall[i],wall[(i+1)%4],rgb(75,220,255),4); wall[i].z=.8f; }
    for(int i=0;i<4;i++) line3d(self.bounds,camera,wall[i],wall[(i+1)%4],rgb(75,220,255),4);
    unsigned int start=(_demo.skid_head+MAX_SKIDS-_demo.skid_count)%MAX_SKIDS;
    for(unsigned int i=0;i<_demo.skid_count;i++) { MacSkid *s=&_demo.skids[(start+i)%MAX_SKIDS];
        unsigned int age=_demo.simulation_time_ms-s->created_ms;
        if(age>SKID_LIFETIME_MS) continue;
        NSColor *color=skid_color(age); CGFloat width=age<4000?4:(age<9000?3:2);
        line3d(self.bounds,camera,(KartVec3){s->lx0,s->ly0,.025f},(KartVec3){s->lx1,s->ly1,.025f},color,width);
        line3d(self.bounds,camera,(KartVec3){s->rx0,s->ry0,.025f},(KartVec3){s->rx1,s->ry1,.025f},color,width); }
    KartVec3 r,f,u; kart_axes(_demo.kart.orientation,&r,&f,&u);
    if(_demo.boost_active) {
        KartVec3 rear=vadd(_demo.kart.position,vadd(vscale(f,-_demo.kart.geometry.half_length),vscale(u,.35f)));
        KartVec3 flameWorld[3]={
            vadd(rear,vscale(r,_demo.kart.geometry.half_width*.42f)),
            vadd(rear,vscale(r,-_demo.kart.geometry.half_width*.42f)),
            vadd(rear,vadd(vscale(f,-2.8f),vscale(u,-.08f)))};
        NSPoint flame[3]; bool visible=true;
        for(int i=0;i<3;i++) { MacProjected p=project(self.bounds,camera,flameWorld[i]); flame[i]=p.point; visible=visible&&p.visible; }
        if(visible) fill_polygon(flame,3,rgb(255,198,35),rgb(90,225,255));
    }
    float w=_demo.kart.geometry.half_width,l=_demo.kart.geometry.half_length;
    float roofW=w*.8f,roofL=l*.75f,h=_demo.kart_spec->model_height;
    KartVec3 world[8]; MacProjected pp[8];
    for(int i=0;i<8;i++){ float sx=(i&1)?((i&4)?roofW:w):-((i&4)?roofW:w);
        float sy=(i&2)?((i&4)?roofL:l):-((i&4)?roofL:l), sz=(i&4)?h:0;
        world[i]=vadd(_demo.kart.position,vadd(vscale(r,sx),vadd(vscale(f,sy),vscale(u,sz+.15f)))); pp[i]=project(self.bounds,camera,world[i]); }
    const int edges[12][2]={{0,1},{1,3},{3,2},{2,0},{4,5},{5,7},{7,6},{6,4},{0,4},{1,5},{2,6},{3,7}};
    if(pp[4].visible&&pp[5].visible&&pp[7].visible&&pp[6].visible) {
        NSPoint roof[4]={pp[4].point,pp[5].point,pp[7].point,pp[6].point};
        fill_polygon(roof,4,kart_body_color(&_demo),rgb(255,230,220));
    }
    for(int i=0;i<12;i++) if(pp[edges[i][0]].visible&&pp[edges[i][1]].visible) stroke_line(pp[edges[i][0]].point,pp[edges[i][1]].point,rgb(255,230,220),2);
    KartVec3 origin=vadd(_demo.kart.position,vscale(u,1.0f));
    line3d(self.bounds,camera,origin,vadd(origin,vscale(f,4.0f)),rgb(255,245,180),2);
    float speed=sqrtf(vdot(_demo.kart.linear_velocity,_demo.kart.linear_velocity));
    if(speed>.01f) line3d(self.bounds,camera,origin,
        vadd(origin,vscale(_demo.kart.linear_velocity,fminf(speed*.18f,9.0f)/speed)),rgb(80,230,255),3);
}

- (void)drawRect:(NSRect)dirtyRect
{
    (void)dirtyRect;
#ifdef KART_DEMO_3D
    [rgb(19,24,33) setFill]; NSRectFill(self.bounds);
    [self draw3D];
#else
    [rgb(22,25,31) setFill]; NSRectFill(self.bounds);
    [self drawTopDown];
#endif
    [self drawMinimap];
    [self drawSpeedometer];
    KartVec3 bodyRight,bodyForward,bodyUp;
    kart_axes(_demo.kart.orientation,&bodyRight,&bodyForward,&bodyUp); (void)bodyUp;
#ifdef KART_DEMO_3D
    float speed=sqrtf(vdot(_demo.kart.linear_velocity,_demo.kart.linear_velocity));
#else
    float speed=hypotf(_demo.kart.linear_velocity.x,_demo.kart.linear_velocity.y);
#endif
    float forwardSpeed=vdot(_demo.kart.linear_velocity,bodyForward);
    float lateralSpeed=vdot(_demo.kart.linear_velocity,bodyRight);
    float slip=atan2f(fabsf(lateralSpeed),fabsf(forwardSpeed))*180.0f/3.14159265358979323846f;
    NSDictionary *normal=[self textAttributes:13 color:rgb(235,240,245) bold:NO];
    NSString *telemetry=[NSString stringWithFormat:@"speed %.2f m/s | slip %.1f deg | vf %.1f vs %.1f | AUTO %s",
        speed,slip,forwardSpeed,lateralSpeed,_demo.kart.drift.slip_detected?"ON":"off"];
    [telemetry drawAtPoint:NSMakePoint(16,12) withAttributes:normal];
    NSString *help=@"Arrows: drive  Shift/W: drift  Cmd/D: boost  K: kart list  T: track list  G: drag trigger  R: reset";
    [help drawAtPoint:NSMakePoint(16,32) withAttributes:normal];
#ifdef KART_DEMO_3D
    NSString *selection=[NSString stringWithFormat:@"%s (%s) %.1f x %.1f | kart %s %.3f x %.3f | h %.2f",
        _demo.track_spec->display_name,_demo.track_spec->asset_name,
        kart_demo_track_width(_demo.track_spec),kart_demo_track_length(_demo.track_spec),
        _demo.kart_spec->asset_name,_demo.kart.geometry.half_width*2,
        _demo.kart.geometry.half_length*2,_demo.kart.position.z];
#else
    NSString *selection=[NSString stringWithFormat:@"%s (%s) %.1f x %.1f | kart %s %.3f x %.3f",
        _demo.track_spec->display_name,_demo.track_spec->asset_name,
        kart_demo_track_width(_demo.track_spec),kart_demo_track_length(_demo.track_spec),
        _demo.kart_spec->asset_name,_demo.kart.geometry.half_width*2,
        _demo.kart.geometry.half_length*2];
#endif
    NSColor *selectionColor=_demo.kart.drift.slip_detected?rgb(255,185,55):rgb(180,195,205);
    [selection drawAtPoint:NSMakePoint(16,52)
        withAttributes:[self textAttributes:13 color:selectionColor bold:NO]];
    NSString *state=[NSString stringWithFormat:@"DRIFT %s | ITEM %s %.2fs | INSTANT READY %.2fs | INSTANT %s | drag x%.2f | skids %u",
        drift_active(&_demo.kart)?"ON":"off",_demo.kart.timed_boost.active?"ON":"off",
        (float)_demo.kart.timed_boost.remaining_ms*.001f,
        _demo.kart.instant_boost.opportunity_timer,
        _demo.kart.instant_boost.active?"ON":"off",
        _demo.kart.grounded_drag_scale,_demo.skid_count];
    NSColor *stateColor=_demo.boost_active?rgb(75,225,255):
        (drift_active(&_demo.kart)?rgb(255,185,55):rgb(180,195,205));
    [state drawAtPoint:NSMakePoint(16,72)
        withAttributes:[self textAttributes:13 color:stateColor bold:NO]];
}
@end

@interface KartDemoAppDelegate : NSObject <NSApplicationDelegate>
@property(strong) NSWindow *window;
@end

@implementation KartDemoAppDelegate
- (void)applicationDidFinishLaunching:(NSNotification *)notification
{
    (void)notification;
#ifdef KART_DEMO_3D
    NSString *title=@"KartRider Demo Physics - 3D";
#else
    NSString *title=@"KartRider Demo Physics - Top Down";
#endif
    self.window=[[NSWindow alloc] initWithContentRect:NSMakeRect(100,100,1280,760)
        styleMask:NSWindowStyleMaskTitled|NSWindowStyleMaskClosable|NSWindowStyleMaskResizable|NSWindowStyleMaskMiniaturizable
        backing:NSBackingStoreBuffered defer:NO];
    self.window.title=title;
    KartDemoView *view=[[KartDemoView alloc] initWithFrame:self.window.contentView.bounds];
    view.autoresizingMask=NSViewWidthSizable|NSViewHeightSizable;
    self.window.contentView=view;
    [self.window makeKeyAndOrderFront:nil];
    [self.window makeFirstResponder:view];
    [NSApp activateIgnoringOtherApps:YES];
}
- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication *)sender { (void)sender; return YES; }
@end

int main(void)
{
    @autoreleasepool {
        NSApplication *app=[NSApplication sharedApplication];
        [app setActivationPolicy:NSApplicationActivationPolicyRegular];
        KartDemoAppDelegate *delegate=[KartDemoAppDelegate new];
        app.delegate=delegate;
        NSMenu *bar=[NSMenu new], *appMenu=[NSMenu new];
        NSMenuItem *root=[NSMenuItem new]; [bar addItem:root]; [bar setSubmenu:appMenu forItem:root];
        [appMenu addItemWithTitle:@"Quit" action:@selector(terminate:) keyEquivalent:@"q"];
        app.mainMenu=bar;
        [app run];
    }
    return 0;
}
