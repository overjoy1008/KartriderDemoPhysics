/* Covers the raw DEFLATE decoder and the KTKZ container that carries the
   embedded track scenes.

   The compressed fixtures below are real streams emitted by a stock DEFLATE
   encoder, so this test needs no track assets. */

#include "kart_inflate.h"
#include "kart_track_scene.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define SCENE_KTRK_SIZE 248u

static void write_u32(unsigned char *out, unsigned int value)
{
    out[0] = (unsigned char)(value & 0xFFu);
    out[1] = (unsigned char)((value >> 8) & 0xFFu);
    out[2] = (unsigned char)((value >> 16) & 0xFFu);
    out[3] = (unsigned char)((value >> 24) & 0xFFu);
}

/* The smallest scene kart_track_scene_load_memory accepts: a header plus one
   mesh that declares no vertices. */
static size_t build_scene_ktrk(unsigned char *out)
{
    size_t offset;
    memset(out, 0, SCENE_KTRK_SIZE);
    memcpy(out, "KTRK", 4);
    write_u32(out + 4, KART_TRACK_SCENE_VERSION);  /* version */
    write_u32(out + 8, 1);  /* mesh_count */
    write_u32(out + 12, 0); /* total_vertex_count */
    write_u32(out + 16, 0); /* total_triangle_count */
    offset = 44;            /* header, including the zeroed bounds */
    memcpy(out + offset, "road", 4);
    offset += 96;
    memcpy(out + offset, "tile", 4);
    offset += 96;
    write_u32(out + offset, 1); /* flags: road candidate */
    write_u32(out + offset + 4, 0);
    write_u32(out + offset + 8, 0);
    return SCENE_KTRK_SIZE;
}

static void test_stored_block(void)
{
    /* One final stored block carrying "kart". */
    static const unsigned char stream[] = {
        0x01,                   /* BFINAL=1, BTYPE=00 */
        0x04, 0x00, 0xFB, 0xFF, /* LEN=4, NLEN=~4 */
        'k', 'a', 'r', 't',
    };
    unsigned char out[8];
    const size_t produced =
        kart_inflate_raw(out, sizeof(out), stream, sizeof(stream));
    assert(produced == 4);
    assert(memcmp(out, "kart", 4) == 0);
}

static void test_stored_block_rejects_bad_complement(void)
{
    static const unsigned char stream[] = {
        0x01, 0x04, 0x00, 0x00, 0x00, 'k', 'a', 'r', 't',
    };
    unsigned char out[8];
    assert(kart_inflate_raw(out, sizeof(out), stream, sizeof(stream)) == 0);
}

static void test_fixed_huffman_block(void)
{
    /* "aaaaaaaa": a literal 'a' then a match at distance 1, which exercises the
       overlapping back-reference path. */
    static const unsigned char stream[] = {0x4B, 0x4C, 0x84, 0x00, 0x00};
    unsigned char out[16];
    const size_t produced =
        kart_inflate_raw(out, sizeof(out), stream, sizeof(stream));
    assert(produced == 8);
    assert(memcmp(out, "aaaaaaaa", 8) == 0);
}

static void test_dynamic_huffman_block(void)
{
    /* A skewed alphabet, which makes the encoder emit its own code lengths
       (BTYPE=10) rather than fall back to the fixed tables. */
    static const unsigned char stream[] = {
        0x1D, 0x8C, 0x31, 0x12, 0x00, 0x30, 0x08, 0xC2, 0xDE, 0xCA, 0x90, 0x55,
        0x06, 0x99, 0xFA, 0xFA, 0xAA, 0x30, 0x71, 0xC9, 0x41, 0x09, 0x44, 0xDB,
        0x34, 0x14, 0x9E, 0x2A, 0x84, 0x99, 0xD1, 0xB0, 0x89, 0x49, 0xAF, 0x96,
        0x56, 0x3C, 0x92, 0x2A, 0x16, 0xDB, 0x1C, 0xC5, 0x91, 0x7B, 0xCD, 0x3A,
        0x3F, 0x75, 0x07, 0xC9, 0xC2, 0xF7, 0xF8,
    };
    static const char expected[] =
        "enaeeaesooeseeneoeoeateteesetaaeeeeeoetsnaeetsatoeenantoaeae"
        "aeteeeoeeotaoseeeeneeeeetnteteetteeoezze";
    unsigned char out[256];
    const size_t produced =
        kart_inflate_raw(out, sizeof(out), stream, sizeof(stream));
    assert(produced == sizeof(expected) - 1);
    assert(memcmp(out, expected, produced) == 0);
}

static void test_truncated_and_malformed_streams_are_rejected(void)
{
    static const unsigned char fixed[] = {0x4B, 0x4C, 0x84, 0x00, 0x00};
    static const unsigned char reserved_type[] = {0x07};
    unsigned char out[16];
    size_t i;
    assert(kart_inflate_raw(out, sizeof(out), reserved_type,
                            sizeof(reserved_type)) == 0);
    /* Every truncation must fail rather than return a partial buffer. */
    for (i = 1; i < sizeof(fixed); ++i) {
        assert(kart_inflate_raw(out, sizeof(out), fixed, i) == 0);
    }
    /* A destination too small for the output must fail, not overrun. */
    assert(kart_inflate_raw(out, 4, fixed, sizeof(fixed)) == 0);
    assert(kart_inflate_raw(out, sizeof(out), NULL, 4) == 0);
    assert(kart_inflate_raw(NULL, 8, fixed, sizeof(fixed)) == 0);
}

static void test_ktkz_container(void)
{
    unsigned char scene_bytes[SCENE_KTRK_SIZE];
    unsigned char container[8 + 5 + SCENE_KTRK_SIZE];
    KartTrackScene scene;
    const unsigned int size = (unsigned int)build_scene_ktrk(scene_bytes);

    /* "KTKZ", the payload size, then a stored-block copy of the scene. */
    memcpy(container, "KTKZ", 4);
    write_u32(container + 4, size);
    container[8] = 0x01;
    container[9] = (unsigned char)(size & 0xFFu);
    container[10] = (unsigned char)((size >> 8) & 0xFFu);
    container[11] = (unsigned char)(~size & 0xFFu);
    container[12] = (unsigned char)((~size >> 8) & 0xFFu);
    memcpy(container + 13, scene_bytes, size);

    memset(&scene, 0, sizeof(scene));
    assert(kart_track_scene_load_compressed(&scene, container, sizeof(container)));
    assert(scene.mesh_count == 1);
    assert(scene.meshes[0].flags == 1u);
    assert(strcmp(scene.meshes[0].name, "road") == 0);
    kart_track_scene_free(&scene);

    /* A short buffer, the wrong magic, and a size that disagrees with the
       stream must all be refused rather than partially loaded. */
    assert(!kart_track_scene_load_compressed(&scene, container, 4));
    container[0] = 'X';
    assert(!kart_track_scene_load_compressed(&scene, container, sizeof(container)));
    container[0] = 'K';
    write_u32(container + 4, size + 1u);
    assert(!kart_track_scene_load_compressed(&scene, container, sizeof(container)));
}

int main(void)
{
    test_stored_block();
    test_stored_block_rejects_bad_complement();
    test_fixed_huffman_block();
    test_dynamic_huffman_block();
    test_truncated_and_malformed_streams_are_rejected();
    test_ktkz_container();
    printf("inflate tests passed\n");
    return 0;
}
