#include "kart_track_texture.h"

#include "kart_inflate.h"

#include <stdlib.h>
#include <string.h>

/* Same ceiling as the scene loader, so a corrupt header cannot ask for a huge
   allocation before the payload is validated. */
#define KART_TRACK_TEXTURE_MAX_BYTES (256u * 1024u * 1024u)

typedef struct TextureReader {
    const unsigned char *cursor;
    size_t remaining;
} TextureReader;

static bool read_bytes(TextureReader *reader, void *destination, size_t size)
{
    if (size > reader->remaining) {
        return false;
    }
    memcpy(destination, reader->cursor, size);
    reader->cursor += size;
    reader->remaining -= size;
    return true;
}

static bool read_u32(TextureReader *reader, uint32_t *value)
{
    unsigned char bytes[4];
    if (!read_bytes(reader, bytes, sizeof(bytes))) {
        return false;
    }
    *value = (uint32_t)bytes[0] |
             ((uint32_t)bytes[1] << 8) |
             ((uint32_t)bytes[2] << 16) |
             ((uint32_t)bytes[3] << 24);
    return true;
}

void kart_track_texture_free(KartTrackTextureTable *table)
{
    uint32_t image_index;
    if (table == NULL) {
        return;
    }
    if (table->images != NULL) {
        for (image_index = 0; image_index < table->image_count; ++image_index) {
            free(table->images[image_index].texels);
            free(table->images[image_index].mask);
            free(table->images[image_index].alpha);
        }
    }
    free(table->images);
    free(table->entries);
    memset(table, 0, sizeof(*table));
}

bool kart_track_texture_load_memory(
    KartTrackTextureTable *table,
    const void *data,
    size_t size)
{
    TextureReader reader;
    char magic[4];
    uint32_t version;
    uint32_t index;

    if (table == NULL || data == NULL) {
        return false;
    }
    memset(table, 0, sizeof(*table));
    reader.cursor = (const unsigned char *)data;
    reader.remaining = size;
    if (!read_bytes(&reader, magic, sizeof(magic)) ||
        memcmp(magic, "KTEX", sizeof(magic)) != 0 ||
        !read_u32(&reader, &version) ||
        version != KART_TRACK_TEXTURE_VERSION ||
        !read_u32(&reader, &table->entry_count) ||
        !read_u32(&reader, &table->image_count) ||
        table->entry_count > 100000u || table->image_count > 100000u) {
        goto fail;
    }
    if (table->entry_count != 0u) {
        table->entries = (KartTrackTextureEntry *)calloc(
            table->entry_count, sizeof(*table->entries));
        if (table->entries == NULL) {
            goto fail;
        }
    }
    for (index = 0; index < table->entry_count; ++index) {
        KartTrackTextureEntry *entry = &table->entries[index];
        if (!read_bytes(&reader, entry->key, KART_TRACK_TEXTURE_KEY_BYTES) ||
            !read_u32(&reader, &entry->image) ||
            entry->image >= table->image_count) {
            goto fail;
        }
        entry->key[KART_TRACK_TEXTURE_KEY_BYTES - 1u] = '\0';
    }
    if (table->image_count != 0u) {
        table->images = (KartTrackTextureImage *)calloc(
            table->image_count, sizeof(*table->images));
        if (table->images == NULL) {
            goto fail;
        }
    }
    for (index = 0; index < table->image_count; ++index) {
        KartTrackTextureImage *image = &table->images[index];
        size_t texels;
        size_t mask_bytes;
        uint32_t texel;
        if (!read_u32(&reader, &image->width) ||
            !read_u32(&reader, &image->height) ||
            !read_u32(&reader, &image->flags) ||
            image->width == 0u || image->height == 0u ||
            image->width > 4096u || image->height > 4096u) {
            goto fail;
        }
        texels = (size_t)image->width * (size_t)image->height;
        mask_bytes = (texels + 7u) / 8u;
        image->texels = (uint16_t *)malloc(texels * sizeof(*image->texels));
        image->mask = (uint8_t *)malloc(mask_bytes);
        if (image->texels == NULL || image->mask == NULL) {
            goto fail;
        }
        for (texel = 0; texel < texels; ++texel) {
            unsigned char bytes[2];
            if (!read_bytes(&reader, bytes, sizeof(bytes))) {
                goto fail;
            }
            image->texels[texel] =
                (uint16_t)((uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8));
        }
        if (!read_bytes(&reader, image->mask, mask_bytes)) {
            goto fail;
        }
        if ((image->flags & KART_TRACK_TEXTURE_ALPHA8) != 0u) {
            image->alpha = (uint8_t *)malloc(texels);
            if (image->alpha == NULL ||
                !read_bytes(&reader, image->alpha, texels)) {
                goto fail;
            }
        }
    }
    return true;

fail:
    kart_track_texture_free(table);
    return false;
}

bool kart_track_texture_load_compressed(
    KartTrackTextureTable *table,
    const void *data,
    size_t size)
{
    const unsigned char *bytes = (const unsigned char *)data;
    uint32_t payload_size;
    unsigned char *payload;
    size_t produced;
    bool loaded;

    if (table == NULL || data == NULL || size < 8u) return false;
    if (memcmp(bytes, "KTXZ", 4) != 0) return false;
    payload_size = (uint32_t)bytes[4] |
                   ((uint32_t)bytes[5] << 8) |
                   ((uint32_t)bytes[6] << 16) |
                   ((uint32_t)bytes[7] << 24);
    if (payload_size == 0u || payload_size > KART_TRACK_TEXTURE_MAX_BYTES) {
        return false;
    }
    payload = (unsigned char *)malloc(payload_size);
    if (payload == NULL) return false;
    produced = kart_inflate_raw(payload, payload_size, bytes + 8, size - 8u);
    loaded = produced == (size_t)payload_size &&
             kart_track_texture_load_memory(table, payload, produced);
    free(payload);
    return loaded;
}

const KartTrackTextureImage *kart_track_texture_find(
    const KartTrackTextureTable *table,
    const char *theme,
    const char *name)
{
    char key[KART_TRACK_TEXTURE_KEY_BYTES];
    size_t theme_length;
    size_t name_length;
    uint32_t low;
    uint32_t high;

    if (table == NULL || table->entry_count == 0u ||
        theme == NULL || name == NULL || name[0] == '\0') {
        return NULL;
    }
    theme_length = strlen(theme);
    name_length = strlen(name);
    if (theme_length + name_length + 2u > KART_TRACK_TEXTURE_KEY_BYTES) {
        return NULL;
    }
    memcpy(key, theme, theme_length);
    key[theme_length] = '/';
    memcpy(key + theme_length + 1u, name, name_length + 1u);

    low = 0u;
    high = table->entry_count;
    while (low < high) {
        const uint32_t middle = low + (high - low) / 2u;
        const int order = strcmp(table->entries[middle].key, key);
        if (order == 0) {
            return &table->images[table->entries[middle].image];
        }
        if (order < 0) {
            low = middle + 1u;
        } else {
            high = middle;
        }
    }
    return NULL;
}
