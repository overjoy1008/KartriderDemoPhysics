#include "kart_demo_data.h"

#include <string.h>

#define DYNAMICS(mass_, air_, drag_, forward_, backward_, grip_brake_, slip_brake_, steer_, constraint_, front_grip_, rear_grip_, trigger_, trigger_time_, slip_, escape_, corner_) \
    { (mass_), (air_), (drag_), (forward_), (backward_), (grip_brake_), \
      (slip_brake_), (steer_), (constraint_), (front_grip_), (rear_grip_), \
      (trigger_), (trigger_time_), (slip_), (escape_), (corner_), 0.07f, 0.01f }

#define PRACTICE_DYNAMICS \
    DYNAMICS(100.0f, 3.0f, 0.740f, 2000.0f, 1500.0f, 1800.0f, 1200.0f, \
             10.0f, 22.0f, 5.0f, 5.0f, 0.2f, 0.2f, 0.2f, 1500.0f, 0.2f)
#define STANDARD_DYNAMICS \
    DYNAMICS(100.0f, 3.0f, 0.725f, 3300.0f, 2000.0f, 2000.0f, 1500.0f, \
             10.0f, 28.0f, 5.0f, 5.0f, 0.2f, 0.2f, 0.2f, 4000.0f, 0.05f)
#define MARATHON_DYNAMICS \
    DYNAMICS(100.0f, 3.0f, 0.622f, 3000.0f, 2000.0f, 1500.0f, 1500.0f, \
             10.0f, 26.0f, 5.0f, 5.0f, 0.2f, 0.2f, 0.2f, 4000.0f, 0.2f)
#define SABER_DYNAMICS \
    DYNAMICS(100.0f, 3.0f, 0.786f, 3550.0f, 2000.0f, 3000.0f, 1500.0f, \
             10.0f, 29.0f, 5.0f, 5.0f, 0.2f, 0.2f, 0.2f, 4000.0f, 0.0f)
#define SOLID_DYNAMICS \
    DYNAMICS(100.0f, 3.0f, 0.855f, 3800.0f, 2000.0f, 2500.0f, 1500.0f, \
             10.0f, 30.0f, 5.0f, 5.0f, 0.2f, 0.2f, 0.2f, 4000.0f, 0.0f)
#define KART(name_, dynamics_, half_width_, half_length_, height_) \
    { (name_), dynamics_, { (half_width_), (half_length_), 0.5f, 1.0f }, (height_) }

/* Exact model-root AABBs and parameter.xml values extracted from the 2004
   demo's kart.rho. The archive spells Cotton as "cotten". */
static const KartDemoKartSpec KARTS[] = {
    KART("practice1", PRACTICE_DYNAMICS, 0.7644821f, 0.81728435f, 0.81284374f),
    KART("burst1", PRACTICE_DYNAMICS, 0.7863699f, 0.86079025f, 0.81284374f),
    KART("burst2", STANDARD_DYNAMICS, 0.7489265f, 0.9987979f, 0.81284374f),
    KART("burst3", STANDARD_DYNAMICS, 0.8080695f, 1.06789325f, 0.81284374f),
    KART("burst4", STANDARD_DYNAMICS, 0.8378915f, 1.0619645f, 0.83439654f),
    KART("burst5", STANDARD_DYNAMICS, 0.8338487f, 1.1471490f, 0.9270710f),
    KART("cotten1", STANDARD_DYNAMICS, 0.7802979f, 0.8544110f, 0.81284374f),
    KART("cotten2", STANDARD_DYNAMICS, 0.7802979f, 0.98605945f, 0.81284374f),
    KART("cotten3", STANDARD_DYNAMICS, 0.8656463f, 1.09538875f, 0.81284374f),
    KART("cotten4", STANDARD_DYNAMICS, 0.8792389f, 1.0857897f, 0.87846375f),
    KART("cotten5", STANDARD_DYNAMICS, 0.87533175f, 1.13917575f, 0.9822382f),
    KART("marathon1", MARATHON_DYNAMICS, 0.74652725f, 0.8395150f, 0.81284374f),
    KART("marathon2", MARATHON_DYNAMICS, 0.78619005f, 0.9645016f, 0.81284374f),
    KART("marathon3", MARATHON_DYNAMICS, 0.90422895f, 1.0765728f, 0.81284374f),
    KART("marathon4", MARATHON_DYNAMICS, 0.84413935f, 1.1062925f, 0.81284374f),
    KART("marathon5", MARATHON_DYNAMICS, 0.8707624f, 1.24406455f, 0.9171259f),
    KART("saber1", SABER_DYNAMICS, 0.7815945f, 0.91753305f, 0.8148401f),
    KART("saber2", SABER_DYNAMICS, 0.7266946f, 1.04699875f, 0.81284374f),
    KART("saber3", SABER_DYNAMICS, 0.9616292f, 1.07968415f, 0.81284374f),
    KART("saber4", SABER_DYNAMICS, 0.91966365f, 1.1252806f, 0.83305377f),
    KART("saber5", SABER_DYNAMICS, 0.8918933f, 1.3687856f, 0.99534184f),
    KART("solid1", SOLID_DYNAMICS, 0.7267921f, 0.83080195f, 0.81284374f),
    KART("solid2", SOLID_DYNAMICS, 0.7744493f, 0.97958995f, 0.81284374f),
    KART("solid3", SOLID_DYNAMICS, 0.8325483f, 1.08305655f, 0.81284374f),
    KART("solid4", SOLID_DYNAMICS, 0.79691135f, 1.0517733f, 0.81284374f),
    KART("solid5", SOLID_DYNAMICS, 0.93279995f, 1.1436093f, 0.9673969f),
};

#define TRACK(asset_, display_, min_x_, min_y_, min_z_, max_x_, max_y_, max_z_) \
    { (asset_), (display_), { (min_x_), (min_y_), (min_z_) }, \
      { (max_x_), (max_y_), (max_z_) } }

/* Transformed mesh-vertex AABBs from each demo track's track.1s. */
static const KartDemoTrackSpec TRACKS[] = {
    TRACK("desert_I01", "Desert I01", 27.221329f, 17.29889f, 6.201988f, 976.4961f, 1240.95f, 131.7757f),
    TRACK("desert_I02", "Desert I02", 15.521851f, 21.887024f, 15.466524f, 716.0608f, 1257.2086f, 131.09953f),
    TRACK("desert_R01", "Desert R01", -75.36597f, 49.4375f, 5.913539f, 1133.6437f, 986.7229f, 152.13696f),
    TRACK("forest_I01", "Forest Log", 13.488026f, 27.247574f, 1.7524395f, 909.879f, 834.4304f, 118.12307f),
    TRACK("forest_I02", "Forest I02", 82.21716f, -140.41414f, -1.2710266f, 754.29016f, 1395.2924f, 113.337524f),
    TRACK("forest_R02", "Forest Zigzag", -7.913208f, -134.81958f, -0.71870804f, 1084.8955f, 1392.0508f, 172.86588f),
    TRACK("ice_I01", "Ice I01", 52.53003f, 60.53882f, 1.6540241f, 1107.8962f, 1165.5347f, 134.85002f),
    TRACK("ice_I02", "Ice Shark Tomb", 93.76453f, 94.95085f, 0.5250864f, 870.5864f, 873.0142f, 138.5321f),
    TRACK("ice_R01", "Ice R01", 136.81946f, 29.599762f, 11.456513f, 1597.5416f, 2003.4298f, 675.49866f),
    TRACK("village_C101", "Village Bank 1", 39.741577f, 68.00537f, 6.108156f, 847.272f, 618.7086f, 89.95568f),
    TRACK("village_C102", "Village Corners 1", 39.741577f, 29.840942f, 9.414543f, 917.69684f, 655.1831f, 89.955696f),
    TRACK("village_I01", "Village I01", 6.5500336f, 13.513728f, 0.34058666f, 651.5643f, 960.89734f, 87.23307f),
    TRACK("village_I02", "Village I02", -2.6714478f, -1809.6846f, -1.2980604f, 1667.4038f, 682.41016f, 49.306587f),
    TRACK("village_R01", "Village Overpass", 12.07431f, -111.33331f, 2.8498707f, 1239.2921f, 1600.7839f, 106.82852f),
    TRACK("village_R03", "Village R03", 23.362457f, 46.180664f, 8.646021f, 1213.144f, 1447.4727f, 71.47362f),
};

unsigned int kart_demo_kart_count(void)
{
    return (unsigned int)(sizeof(KARTS) / sizeof(KARTS[0]));
}

const KartDemoKartSpec *kart_demo_kart_at(unsigned int index)
{
    return index < kart_demo_kart_count() ? &KARTS[index] : NULL;
}

const KartDemoKartSpec *kart_demo_find_kart(const char *asset_name)
{
    unsigned int i;
    if (asset_name == NULL) return NULL;
    for (i = 0; i < kart_demo_kart_count(); ++i)
        if (strcmp(KARTS[i].asset_name, asset_name) == 0) return &KARTS[i];
    return NULL;
}

const KartDemoKartSpec *kart_demo_default_kart(void)
{
    return kart_demo_find_kart("burst3");
}

unsigned int kart_demo_track_count(void)
{
    return (unsigned int)(sizeof(TRACKS) / sizeof(TRACKS[0]));
}

const KartDemoTrackSpec *kart_demo_track_at(unsigned int index)
{
    return index < kart_demo_track_count() ? &TRACKS[index] : NULL;
}

const KartDemoTrackSpec *kart_demo_find_track(const char *asset_name)
{
    unsigned int i;
    if (asset_name == NULL) return NULL;
    for (i = 0; i < kart_demo_track_count(); ++i)
        if (strcmp(TRACKS[i].asset_name, asset_name) == 0) return &TRACKS[i];
    return NULL;
}

const KartDemoTrackSpec *kart_demo_default_track(void)
{
    return kart_demo_find_track("forest_I01");
}

float kart_demo_track_width(const KartDemoTrackSpec *track)
{
    return track != NULL ? track->maximum.x - track->minimum.x : 0.0f;
}

float kart_demo_track_length(const KartDemoTrackSpec *track)
{
    return track != NULL ? track->maximum.y - track->minimum.y : 0.0f;
}
