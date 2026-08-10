#include "kart_inflate.h"

#include <string.h>

#define KART_INFLATE_MAX_BITS 15
#define KART_INFLATE_MAX_SYMBOLS 288

typedef struct BitReader {
    const unsigned char *data;
    size_t size;
    size_t position;
    unsigned int bit_buffer;
    unsigned int bit_count;
    bool failed;
} BitReader;

typedef struct Huffman {
    short count[KART_INFLATE_MAX_BITS + 1];
    short symbol[KART_INFLATE_MAX_SYMBOLS];
} Huffman;

static const unsigned short LENGTH_BASE[29] = {
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59,
    67, 83, 99, 115, 131, 163, 195, 227, 258};
static const unsigned char LENGTH_EXTRA[29] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3,
    4, 4, 4, 4, 5, 5, 5, 5, 0};
static const unsigned short DISTANCE_BASE[30] = {
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513,
    769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
static const unsigned char DISTANCE_EXTRA[30] = {
    0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8,
    9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
/* The order code lengths for the code-length alphabet arrive in. */
static const unsigned char CODE_LENGTH_ORDER[19] = {
    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};

static unsigned int read_bits(BitReader *reader, unsigned int count)
{
    unsigned int value;
    while (reader->bit_count < count) {
        if (reader->position >= reader->size) {
            reader->failed = true;
            return 0;
        }
        reader->bit_buffer |=
            (unsigned int)reader->data[reader->position++] << reader->bit_count;
        reader->bit_count += 8;
    }
    value = reader->bit_buffer & ((1u << count) - 1u);
    reader->bit_buffer >>= count;
    reader->bit_count -= count;
    return value;
}

/* Builds the canonical code table described in RFC 1951 section 3.2.2. */
static bool huffman_build(Huffman *huffman, const unsigned char *lengths, int count)
{
    short offsets[KART_INFLATE_MAX_BITS + 2];
    int symbol;
    int length;
    int left;
    for (length = 0; length <= KART_INFLATE_MAX_BITS; ++length) {
        huffman->count[length] = 0;
    }
    for (symbol = 0; symbol < count; ++symbol) {
        if (lengths[symbol] > KART_INFLATE_MAX_BITS) return false;
        huffman->count[lengths[symbol]]++;
    }
    if (huffman->count[0] == count) return true;
    /* Reject over-subscribed sets. Incomplete sets stay allowed: a stream may
       legitimately carry a single distance code, and decoding rejects any code
       that is actually unused. */
    left = 1;
    for (length = 1; length <= KART_INFLATE_MAX_BITS; ++length) {
        left <<= 1;
        left -= huffman->count[length];
        if (left < 0) return false;
    }
    offsets[1] = 0;
    for (length = 1; length < KART_INFLATE_MAX_BITS; ++length) {
        offsets[length + 1] = (short)(offsets[length] + huffman->count[length]);
    }
    for (symbol = 0; symbol < count; ++symbol) {
        if (lengths[symbol] != 0) {
            huffman->symbol[offsets[lengths[symbol]]++] = (short)symbol;
        }
    }
    return true;
}

static int huffman_decode(BitReader *reader, const Huffman *huffman)
{
    int code = 0;
    int first = 0;
    int index = 0;
    int length;
    for (length = 1; length <= KART_INFLATE_MAX_BITS; ++length) {
        int count;
        code |= (int)read_bits(reader, 1);
        if (reader->failed) return -1;
        count = huffman->count[length];
        if (code - first < count) return huffman->symbol[index + (code - first)];
        index += count;
        first = (first + count) << 1;
        code <<= 1;
    }
    return -1;
}

static bool inflate_codes(
    BitReader *reader,
    unsigned char *out,
    size_t out_size,
    size_t *out_position,
    const Huffman *literals,
    const Huffman *distances)
{
    for (;;) {
        int symbol = huffman_decode(reader, literals);
        if (symbol < 0) return false;
        if (symbol < 256) {
            if (*out_position >= out_size) return false;
            out[(*out_position)++] = (unsigned char)symbol;
        } else if (symbol == 256) {
            return true;
        } else {
            unsigned int length;
            unsigned int distance;
            size_t start;
            unsigned int i;
            symbol -= 257;
            if (symbol >= 29) return false;
            length = (unsigned int)LENGTH_BASE[symbol] +
                     read_bits(reader, LENGTH_EXTRA[symbol]);
            symbol = huffman_decode(reader, distances);
            if (symbol < 0 || symbol >= 30) return false;
            distance = (unsigned int)DISTANCE_BASE[symbol] +
                       read_bits(reader, DISTANCE_EXTRA[symbol]);
            if (reader->failed) return false;
            if (distance == 0 || (size_t)distance > *out_position) return false;
            if ((size_t)length > out_size - *out_position) return false;
            start = *out_position - distance;
            /* Overlapping copies are normal in DEFLATE, so copy byte by byte. */
            for (i = 0; i < length; ++i) {
                out[*out_position + i] = out[start + i];
            }
            *out_position += length;
        }
    }
}

static bool build_fixed_trees(Huffman *literals, Huffman *distances)
{
    unsigned char lengths[KART_INFLATE_MAX_SYMBOLS];
    int symbol;
    for (symbol = 0; symbol < 144; ++symbol) lengths[symbol] = 8;
    for (; symbol < 256; ++symbol) lengths[symbol] = 9;
    for (; symbol < 280; ++symbol) lengths[symbol] = 7;
    for (; symbol < 288; ++symbol) lengths[symbol] = 8;
    if (!huffman_build(literals, lengths, 288)) return false;
    for (symbol = 0; symbol < 30; ++symbol) lengths[symbol] = 5;
    return huffman_build(distances, lengths, 30);
}

static bool build_dynamic_trees(
    BitReader *reader,
    Huffman *literals,
    Huffman *distances)
{
    unsigned char lengths[KART_INFLATE_MAX_SYMBOLS + 30];
    Huffman code_lengths;
    unsigned int literal_count;
    unsigned int distance_count;
    unsigned int code_count;
    unsigned int i;
    literal_count = read_bits(reader, 5) + 257u;
    distance_count = read_bits(reader, 5) + 1u;
    code_count = read_bits(reader, 4) + 4u;
    if (reader->failed || literal_count > 286u || distance_count > 30u) return false;
    for (i = 0; i < 19u; ++i) lengths[CODE_LENGTH_ORDER[i]] = 0;
    for (i = 0; i < code_count; ++i) {
        lengths[CODE_LENGTH_ORDER[i]] = (unsigned char)read_bits(reader, 3);
    }
    if (reader->failed || !huffman_build(&code_lengths, lengths, 19)) return false;
    i = 0;
    while (i < literal_count + distance_count) {
        int symbol = huffman_decode(reader, &code_lengths);
        if (symbol < 0) return false;
        if (symbol < 16) {
            lengths[i++] = (unsigned char)symbol;
        } else {
            unsigned char value = 0;
            unsigned int repeat;
            if (symbol == 16) {
                if (i == 0) return false;
                value = lengths[i - 1];
                repeat = 3u + read_bits(reader, 2);
            } else if (symbol == 17) {
                repeat = 3u + read_bits(reader, 3);
            } else {
                repeat = 11u + read_bits(reader, 7);
            }
            if (reader->failed || i + repeat > literal_count + distance_count) {
                return false;
            }
            while (repeat-- != 0) lengths[i++] = value;
        }
    }
    if (lengths[256] == 0) return false;
    return huffman_build(literals, lengths, (int)literal_count) &&
           huffman_build(distances, lengths + literal_count, (int)distance_count);
}

static bool inflate_stored(
    BitReader *reader,
    unsigned char *out,
    size_t out_size,
    size_t *out_position)
{
    unsigned int length;
    unsigned int complement;
    /* Stored blocks resume on a byte boundary. */
    reader->bit_buffer = 0;
    reader->bit_count = 0;
    if (reader->position + 4u > reader->size) return false;
    length = (unsigned int)reader->data[reader->position] |
             ((unsigned int)reader->data[reader->position + 1] << 8);
    complement = (unsigned int)reader->data[reader->position + 2] |
                 ((unsigned int)reader->data[reader->position + 3] << 8);
    reader->position += 4u;
    if ((length ^ 0xFFFFu) != complement) return false;
    if (reader->position + length > reader->size) return false;
    if ((size_t)length > out_size - *out_position) return false;
    memcpy(out + *out_position, reader->data + reader->position, length);
    reader->position += length;
    *out_position += length;
    return true;
}

size_t kart_inflate_raw(
    void *destination,
    size_t destination_size,
    const void *source,
    size_t source_size)
{
    BitReader reader;
    unsigned char *out = (unsigned char *)destination;
    size_t out_position = 0;
    unsigned int final_block;

    if (destination == NULL || source == NULL) return 0;
    memset(&reader, 0, sizeof(reader));
    reader.data = (const unsigned char *)source;
    reader.size = source_size;

    do {
        unsigned int type;
        final_block = read_bits(&reader, 1);
        type = read_bits(&reader, 2);
        if (reader.failed) return 0;
        if (type == 0u) {
            if (!inflate_stored(&reader, out, destination_size, &out_position)) {
                return 0;
            }
        } else if (type == 1u || type == 2u) {
            Huffman literals;
            Huffman distances;
            const bool built = type == 1u
                ? build_fixed_trees(&literals, &distances)
                : build_dynamic_trees(&reader, &literals, &distances);
            if (!built) return 0;
            if (!inflate_codes(&reader, out, destination_size, &out_position,
                               &literals, &distances)) {
                return 0;
            }
        } else {
            return 0;
        }
    } while (final_block == 0u);

    return out_position;
}
