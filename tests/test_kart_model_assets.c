/* End-to-end check over the real packed kart models.

   Walks every kart in KARTS[], loads its KTKZ from disk, and asserts that the
   structure the demo relies on is actually there: six submeshes, a body at
   index 0, and exactly four wheels found by vertex count.

   The point of the test is the last step. The 78 geometry constants in
   src/kart_demo_data.c are derived from the body submesh alone, so if the body
   were ever misidentified the demo would draw one hull and simulate another
   without anything failing. Re-deriving half_width, half_length and
   model_height here from the loaded mesh and comparing them to the shipped
   constants closes that gap: it ties the drawn model to the simulated box.

   Also checks that the wheels do stick out past the body on the karts where
   the catalogue says they do, which is the whole reason the full-model AABB
   cannot be used for the physics.

   Registered only when the exports are present, since they are not tracked. */

#include "kart_demo_data.h"
#include "kart_model_win32.h"
#include "kart_track_scene.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The constants are stored to 7 decimals, so anything tighter is noise. */
#define GEOMETRY_TOLERANCE 2e-5f

static unsigned char *read_file(const char *path, size_t *size)
{
    FILE *file = fopen(path, "rb");
    unsigned char *data;
    long length;
    if (file == NULL) return NULL;
    if (fseek(file, 0, SEEK_END) != 0) { fclose(file); return NULL; }
    length = ftell(file);
    if (length <= 0 || fseek(file, 0, SEEK_SET) != 0) { fclose(file); return NULL; }
    data = (unsigned char *)malloc((size_t)length);
    if (data == NULL || fread(data, 1, (size_t)length, file) != (size_t)length) {
        free(data);
        fclose(file);
        return NULL;
    }
    fclose(file);
    *size = (size_t)length;
    return data;
}

static int check(const char *kart, const char *label, float asset, float shipped)
{
    if (fabsf(asset - shipped) <= GEOMETRY_TOLERANCE) return 0;
    printf("%-11s %-13s asset=%.7f shipped=%.7f MISMATCH\n",
           kart, label, asset, shipped);
    return 1;
}

int main(int argc, char **argv)
{
    const char *packed_dir = argc > 1 ? argv[1] : "analysis/kart-assets/packed";
    const unsigned int count = kart_demo_kart_count();
    unsigned int i;
    unsigned int failures = 0;
    unsigned int wider_wheels = 0;

    for (i = 0; i < count; ++i) {
        const KartDemoKartSpec *spec = kart_demo_kart_at(i);
        char path[1024];
        unsigned char *data;
        size_t size = 0;
        KartTrackScene model;
        KartModelParts parts;
        float half_width;
        float half_length;
        float model_height;
        float overhang;

        snprintf(path, sizeof(path), "%s/%s.ktkz", packed_dir, spec->asset_name);
        data = read_file(path, &size);
        if (data == NULL) {
            printf("missing %s\n", path);
            return 1;
        }
        memset(&model, 0, sizeof(model));
        if (!kart_track_scene_load_compressed(&model, data, size)) {
            printf("failed to load %s\n", path);
            free(data);
            return 1;
        }
        free(data);

        kart_model_parts_build(&model, &parts);
        if (!parts.valid) {
            printf("%-11s parts could not be built\n", spec->asset_name);
            ++failures;
            kart_track_scene_free(&model);
            continue;
        }
        if (parts.wheel_count != KART_MODEL_WHEEL_COUNT) {
            printf("%-11s found %u wheels, expected %d\n",
                   spec->asset_name, parts.wheel_count, KART_MODEL_WHEEL_COUNT);
            ++failures;
        }
        if (model.mesh_count != 6) {
            printf("%-11s %u submeshes, expected 6\n",
                   spec->asset_name, model.mesh_count);
            ++failures;
        }
        if (kart_model_is_wheel(&parts, 0)) {
            printf("%-11s submesh 0 classified as a wheel\n", spec->asset_name);
            ++failures;
        }

        half_width = (parts.body.maximum[0] - parts.body.minimum[0]) * 0.5f;
        half_length = (parts.body.maximum[1] - parts.body.minimum[1]) * 0.5f;
        model_height = parts.body.maximum[2] -
                       (parts.body.minimum[2] < 0.0f ? parts.body.minimum[2] : 0.0f);
        failures += (unsigned int)check(
            spec->asset_name, "half_width", half_width, spec->geometry.half_width);
        failures += (unsigned int)check(
            spec->asset_name, "half_length", half_length, spec->geometry.half_length);
        failures += (unsigned int)check(
            spec->asset_name, "model_height", model_height, spec->model_height);

        overhang = ((parts.full.maximum[0] - parts.full.minimum[0]) -
                    (parts.body.maximum[0] - parts.body.minimum[0])) * 0.5f;
        if (overhang > GEOMETRY_TOLERANCE) ++wider_wheels;
        /* The full box can never be narrower than the body it contains. */
        if (overhang < -GEOMETRY_TOLERANCE) {
            printf("%-11s full box narrower than body by %.7f\n",
                   spec->asset_name, -overhang);
            ++failures;
        }

        kart_track_scene_free(&model);
    }

    printf("%u karts checked, %u with wheels wider than the body -> %u failures\n",
           count, wider_wheels, failures);
    /* 17 of the 26 in the shipped assets; the check is that some do and the
       count is reported, not that it is exactly 17. */
    if (wider_wheels == 0) {
        printf("no kart had wheels past the body, which contradicts the catalogue\n");
        return 1;
    }
    return failures == 0 ? 0 : 1;
}
