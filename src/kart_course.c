#include "kart_course.h"

#include "kart_track_collision.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* The original's course graph, gate crossing and lap counter.
   See docs/ORIGINAL_COURSE.md for the decompilation.

   Three things here look wrong until you check them against the original and
   are deliberately kept:

   - A node's `forward` list (+0x14) holds the nodes *behind* it and `backward`
     (+0x08) the nodes ahead. 0x00426470 tests the backward list for a positive
     crossing and the forward list for a negative one, so the names read
     inverted throughout.
   - The lap counter does not fire on the finish line. It fires when the number
     of times the kart crossed the *first* node equals the number of laps it
     has been credited with (0x00424b30). No other gate touches that count, so
     cutting the course silently stops laps from counting.
   - Nothing has a radius or a tolerance. The only threshold in the mechanism
     is zero (DAT_00571808). */

static KartVec3 add(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static KartVec3 subtract(KartVec3 a, KartVec3 b)
{
    return (KartVec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static KartVec3 scale(KartVec3 value, float amount)
{
    return (KartVec3){value.x * amount, value.y * amount, value.z * amount};
}

static float dot(KartVec3 a, KartVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static KartVec3 cross(KartVec3 a, KartVec3 b)
{
    return (KartVec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

static float length_of(KartVec3 value)
{
    return sqrtf(dot(value, value));
}

/* 0x00428500. The original divides by the length with no guard; a zero vector
   would give NaN there, so the guard here only affects inputs the original
   never produces. */
static KartVec3 normalize(KartVec3 value)
{
    const float length = length_of(value);
    return length > 0.0f ? scale(value, 1.0f / length) : (KartVec3){0.0f, 0.0f, 0.0f};
}

static KartVec3 read_point(const float source[3])
{
    return (KartVec3){source[0], source[1], source[2]};
}

/* --- construction ------------------------------------------------------- */

typedef struct Builder {
    KartCourse *course;
    unsigned int node_capacity;
    unsigned int gate_capacity;
    unsigned int link_capacity;
    unsigned int point_capacity;
    /* Tails of each node's two link lists, so appending keeps the original's
       insertion order rather than reversing it. */
    unsigned int *forward_tail;
    unsigned int *backward_tail;
    bool failed;
} Builder;

static void *grow(void *items, unsigned int *capacity, unsigned int needed, size_t size)
{
    unsigned int next = *capacity != 0 ? *capacity : 16u;
    void *replacement;
    if (needed <= *capacity) return items;
    while (next < needed) next *= 2u;
    replacement = realloc(items, (size_t)next * size);
    if (replacement == NULL) return NULL;
    *capacity = next;
    return replacement;
}

static unsigned int new_node(Builder *builder, unsigned int id)
{
    KartCourse *course = builder->course;
    const unsigned int index = course->node_count;
    KartCourseNode *nodes;
    unsigned int *forward_tail;
    unsigned int *backward_tail;
    unsigned int capacity = builder->node_capacity;
    if (builder->failed) return 0;
    nodes = grow(course->nodes, &builder->node_capacity, index + 1u,
                 sizeof *course->nodes);
    if (nodes == NULL) { builder->failed = true; return 0; }
    course->nodes = nodes;
    if (builder->node_capacity != capacity) {
        forward_tail = realloc(builder->forward_tail,
                               (size_t)builder->node_capacity * sizeof *forward_tail);
        backward_tail = realloc(builder->backward_tail,
                                (size_t)builder->node_capacity * sizeof *backward_tail);
        if (forward_tail == NULL || backward_tail == NULL) {
            free(forward_tail);
            free(backward_tail);
            builder->failed = true;
            return 0;
        }
        builder->forward_tail = forward_tail;
        builder->backward_tail = backward_tail;
    }
    course->nodes[index] = (KartCourseNode){
        id, KART_COURSE_NO_INDEX, KART_COURSE_NO_INDEX,
        course->point_count, 0u, 0.0f};
    builder->forward_tail[index] = KART_COURSE_NO_INDEX;
    builder->backward_tail[index] = KART_COURSE_NO_INDEX;
    ++course->node_count;
    return index;
}

static unsigned int new_gate(Builder *builder, const KartCourseGate *gate)
{
    KartCourse *course = builder->course;
    KartCourseGate *gates;
    if (builder->failed) return 0;
    gates = grow(course->gates, &builder->gate_capacity, course->gate_count + 1u,
                 sizeof *course->gates);
    if (gates == NULL) { builder->failed = true; return 0; }
    course->gates = gates;
    course->gates[course->gate_count] = *gate;
    return course->gate_count++;
}

static unsigned int new_link(Builder *builder, unsigned int gate, unsigned int node)
{
    KartCourse *course = builder->course;
    KartCourseLink *links;
    if (builder->failed) return 0;
    links = grow(course->links, &builder->link_capacity, course->link_count + 1u,
                 sizeof *course->links);
    if (links == NULL) { builder->failed = true; return 0; }
    course->links = links;
    course->links[course->link_count] = (KartCourseLink){
        gate, node, KART_COURSE_NO_INDEX};
    return course->link_count++;
}

static void append_link(
    Builder *builder,
    unsigned int node,
    bool forward,
    unsigned int gate,
    unsigned int target)
{
    const unsigned int link = new_link(builder, gate, target);
    unsigned int *head;
    unsigned int *tail;
    if (builder->failed) return;
    head = forward ? &builder->course->nodes[node].forward
                   : &builder->course->nodes[node].backward;
    tail = forward ? &builder->forward_tail[node] : &builder->backward_tail[node];
    if (*tail == KART_COURSE_NO_INDEX) {
        *head = link;
    } else {
        builder->course->links[*tail].next = link;
    }
    *tail = link;
}

static void append_point(Builder *builder, unsigned int node, KartVec3 position, KartVec3 direction)
{
    KartCourse *course = builder->course;
    KartCoursePoint *points;
    if (builder->failed) return;
    points = grow(course->points, &builder->point_capacity, course->point_count + 1u,
                  sizeof *course->points);
    if (points == NULL) { builder->failed = true; return; }
    course->points = points;
    /* Claimed on the first point rather than when the node is made: a branch's
       join node is created before the alternatives are built and only gets its
       centreline afterwards, so anything fixed at creation would point into
       the alternative's points instead. */
    if (course->nodes[node].point_count == 0) {
        course->nodes[node].point_first = course->point_count;
    }
    course->points[course->point_count++] = (KartCoursePoint){position, direction};
    ++course->nodes[node].point_count;
}

/* 0x00424e00's tail: the polyline length a node carries at +0x2c. */
static void measure_node(KartCourse *course, unsigned int node)
{
    const KartCourseNode *entry = &course->nodes[node];
    float total = 0.0f;
    unsigned int index;
    for (index = 1; index < entry->point_count; ++index) {
        const KartCoursePoint *previous = &course->points[entry->point_first + index - 1u];
        const KartCoursePoint *current = &course->points[entry->point_first + index];
        total += length_of(subtract(current->position, previous->position));
    }
    course->nodes[node].length = total;
}

static unsigned int find_element(
    const KartCourseElement *elements,
    unsigned int count,
    const char *name,
    unsigned int fallback)
{
    unsigned int index;
    if (name == NULL) return fallback;
    for (index = 0; index < count; ++index) {
        if (elements[index].name != NULL && strcmp(elements[index].name, name) == 0) {
            return index;
        }
    }
    return fallback;
}

typedef struct NodeList {
    unsigned int *items;
    unsigned int count;
    unsigned int capacity;
} NodeList;

static void list_clear(NodeList *list)
{
    list->count = 0;
}

static bool list_push(NodeList *list, unsigned int value)
{
    if (list->count == list->capacity) {
        const unsigned int next = list->capacity != 0 ? list->capacity * 2u : 8u;
        unsigned int *items = realloc(list->items, (size_t)next * sizeof *items);
        if (items == NULL) return false;
        list->items = items;
        list->capacity = next;
    }
    list->items[list->count++] = value;
    return true;
}

static void build_sections(
    Builder *builder,
    const KartCourseSection *sections,
    unsigned int section_count,
    unsigned int start_id,
    unsigned int *last_id,
    bool close_ring,
    NodeList *emitted);

/* One `road` tag. 0x00424e00, road case. */
static void build_road(
    Builder *builder,
    const KartCourseSection *section,
    unsigned int *current,
    NodeList *previous,
    unsigned int *last_id,
    NodeList *emitted)
{
    const KartCourseElement *elements = section->elements;
    const unsigned int count = section->element_count;
    const unsigned int reverse = section->reverse != 0 ? 1u : 0u;
    const unsigned int start = find_element(elements, count, section->start, 0u);
    const unsigned int end = find_element(elements, count, section->end, count - 1u);
    const unsigned int final = find_element(elements, count, section->final, count);
    unsigned int index = start;
    if (count == 0) { builder->failed = true; return; }
    for (;;) {
        const unsigned int next = reverse
            ? (index == 0u ? count : index) - 1u
            : (index == count - 1u ? 0u : index + 1u);
        const KartCourseElement *element = &elements[index];
        const KartCourseElement *point_source = &elements[reverse ? next : index];
        KartCourseGate gate;
        unsigned int gate_index;
        unsigned int face;
        unsigned int slot;
        if (element->record_count == 0 || point_source->record_count == 0) {
            builder->failed = true;
            return;
        }
        /* +0x04/+0x10/+0x1c and +0x28/+0x34/+0x40: corner 0, then the other two
           swapped when the road is walked backwards. */
        for (face = 0; face < 2; ++face) {
            static const unsigned int order[2][3] = {{0u, 1u, 2u}, {0u, 2u, 1u}};
            for (slot = 0; slot < 3; ++slot) {
                gate.face[face][slot] =
                    read_point(element->face[face][order[reverse][slot]]);
            }
        }
        /* +0x4c: record 0's direction, flipped when reverse. */
        gate.normal = scale(read_point(element->records[0].direction),
                            reverse ? -1.0f : 1.0f);
        gate.is_final = final == index;
        gate_index = new_gate(builder, &gate);

        if (previous->count == 0) {
            /* The dangling entry the ring closure or the enclosing branch
               fills in later. */
            append_link(builder, *current, true, gate_index, KART_COURSE_NO_INDEX);
        } else {
            unsigned int slot_index;
            for (slot_index = 0; slot_index < previous->count; ++slot_index) {
                const unsigned int prior = previous->items[slot_index];
                append_link(builder, prior, false, gate_index, *current);
                append_link(builder, *current, true, gate_index, prior);
            }
        }

        /* +0x20: the centreline. Walking backwards takes the *next* element's
           records, in reverse order and with every direction negated. */
        if (!reverse) {
            unsigned int record;
            for (record = 0; record < element->record_count; ++record) {
                append_point(builder, *current,
                             read_point(element->records[record].position),
                             read_point(element->records[record].direction));
            }
        } else {
            unsigned int record = point_source->record_count;
            while (record-- > 0) {
                append_point(builder, *current,
                             read_point(point_source->records[record].position),
                             scale(read_point(point_source->records[record].direction), -1.0f));
            }
        }
        measure_node(builder->course, *current);
        if (builder->failed) return;

        if (!list_push(emitted, *current)) { builder->failed = true; return; }
        list_clear(previous);
        if (!list_push(previous, *current)) { builder->failed = true; return; }
        *current = new_node(builder, *last_id + 1u);
        if (builder->failed) return;
        *last_id = builder->course->nodes[*current].id;
        if (index == end) break;
        index = next;
    }
}

/* One `branch` tag. Each alternative is built as its own course, then spliced
   between the node before the branch and the node after it. */
static void build_branch(
    Builder *builder,
    const KartCourseSection *section,
    unsigned int *current,
    NodeList *previous,
    unsigned int start_id,
    unsigned int *last_id,
    NodeList *emitted)
{
    unsigned int highest = start_id;
    unsigned int alternative;
    for (alternative = 0; alternative < section->alternative_count; ++alternative) {
        NodeList sub = {NULL, 0u, 0u};
        unsigned int sub_last = *last_id;
        unsigned int entry_gate;
        unsigned int index;
        unsigned int link;
        build_sections(builder, section->alternatives[alternative],
                       section->alternative_counts[alternative],
                       *last_id, &sub_last, false, &sub);
        if (builder->failed || sub.count == 0) {
            free(sub.items);
            builder->failed = true;
            return;
        }
        if (sub_last > highest) highest = sub_last;

        /* The sub-course's first node has one dangling forward link. Its gate
           is the branch entry, and both ends of it get wired to the nodes on
           either side of the branch. */
        entry_gate = builder->course->links[
            builder->course->nodes[sub.items[0]].forward].gate;
        builder->course->nodes[sub.items[0]].forward = KART_COURSE_NO_INDEX;
        builder->forward_tail[sub.items[0]] = KART_COURSE_NO_INDEX;
        for (index = 0; index < previous->count; ++index) {
            append_link(builder, sub.items[0], true, entry_gate, previous->items[index]);
            append_link(builder, previous->items[index], false, entry_gate, sub.items[0]);
        }

        /* The sub-course's last node is not kept: the node after the branch
           takes its place, so every alternative rejoins the same node. */
        for (link = builder->course->nodes[sub.items[sub.count - 1u]].forward;
             link != KART_COURSE_NO_INDEX;
             link = builder->course->links[link].next) {
            const unsigned int behind = builder->course->links[link].node;
            if (behind == KART_COURSE_NO_INDEX) continue;
            builder->course->links[builder->course->nodes[behind].backward].node = *current;
        }
        {
            const unsigned int exit = builder->course->nodes[sub.items[sub.count - 1u]].forward;
            if (exit != KART_COURSE_NO_INDEX) {
                append_link(builder, *current, true,
                            builder->course->links[exit].gate,
                            builder->course->links[exit].node);
            }
        }
        for (index = 0; index + 1u < sub.count; ++index) {
            if (!list_push(emitted, sub.items[index])) {
                free(sub.items);
                builder->failed = true;
                return;
            }
        }
        /* Only the first alternative's tail geometry is carried over, which is
           what makes the shared join node measurable at all. */
        if (alternative == 0) {
            const KartCourseNode *tail = &builder->course->nodes[sub.items[sub.count - 1u]];
            const unsigned int first = tail->point_first;
            const unsigned int total = tail->point_count;
            for (index = 0; index < total; ++index) {
                const KartCoursePoint point = builder->course->points[first + index];
                append_point(builder, *current, point.position, point.direction);
            }
            measure_node(builder->course, *current);
        }
        free(sub.items);
        if (builder->failed) return;
    }
    builder->course->nodes[*current].id = highest - 1u;
    if (!list_push(emitted, *current)) { builder->failed = true; return; }
    list_clear(previous);
    if (!list_push(previous, *current)) { builder->failed = true; return; }
    *current = new_node(builder, highest);
    if (builder->failed) return;
    *last_id = highest;
}

static void build_sections(
    Builder *builder,
    const KartCourseSection *sections,
    unsigned int section_count,
    unsigned int start_id,
    unsigned int *last_id,
    bool close_ring,
    NodeList *emitted)
{
    NodeList previous = {NULL, 0u, 0u};
    unsigned int current = new_node(builder, start_id);
    unsigned int index;
    *last_id = start_id;
    for (index = 0; index < section_count && !builder->failed; ++index) {
        if (sections[index].elements != NULL) {
            build_road(builder, &sections[index], &current, &previous, last_id, emitted);
        } else {
            build_branch(builder, &sections[index], &current, &previous,
                         start_id, last_id, emitted);
        }
    }
    if (!builder->failed && close_ring && emitted->count != 0 && previous.count != 0) {
        /* The dangling forward link of the first node now points at the last,
           and the last node gets a backward link into the first. That is the
           only thing that makes the course a loop. */
        const unsigned int first = emitted->items[0];
        const unsigned int last = previous.items[0];
        const unsigned int entry = builder->course->nodes[first].forward;
        if (entry != KART_COURSE_NO_INDEX) {
            builder->course->links[entry].node = last;
            append_link(builder, last, false, builder->course->links[entry].gate, first);
        }
    }
    free(previous.items);
}

bool kart_course_build(KartCourse *course, const KartCourseAsset *asset)
{
    Builder builder;
    NodeList emitted = {NULL, 0u, 0u};
    unsigned int last_id = 0;
    if (course == NULL) return false;
    memset(course, 0, sizeof *course);
    course->first_node = KART_COURSE_NO_INDEX;
    course->last_node = KART_COURSE_NO_INDEX;
    if (asset == NULL || asset->section_count == 0) return false;

    memset(&builder, 0, sizeof builder);
    builder.course = course;
    build_sections(&builder, asset->sections, asset->section_count, 0u, &last_id,
                   true, &emitted);
    free(builder.forward_tail);
    free(builder.backward_tail);
    if (builder.failed || emitted.count == 0) {
        free(emitted.items);
        kart_course_free(course);
        course->first_node = KART_COURSE_NO_INDEX;
        course->last_node = KART_COURSE_NO_INDEX;
        return false;
    }
    /* 0x00424e00 always has one node in hand that it has not published yet, so
       each section leaves a spare behind and a branch leaves one per
       alternative. The original's node vector holds only what it published, so
       the graph is compacted to that list here and the leftovers go away. */
    {
        unsigned int *remap = malloc((size_t)course->node_count * sizeof *remap);
        KartCourseNode *compact = malloc((size_t)emitted.count * sizeof *compact);
        unsigned int index;
        if (remap == NULL || compact == NULL) {
            free(remap);
            free(compact);
            free(emitted.items);
            kart_course_free(course);
            course->first_node = KART_COURSE_NO_INDEX;
            course->last_node = KART_COURSE_NO_INDEX;
            return false;
        }
        for (index = 0; index < course->node_count; ++index) {
            remap[index] = KART_COURSE_NO_INDEX;
        }
        for (index = 0; index < emitted.count; ++index) {
            remap[emitted.items[index]] = index;
            compact[index] = course->nodes[emitted.items[index]];
        }
        for (index = 0; index < course->link_count; ++index) {
            if (course->links[index].node != KART_COURSE_NO_INDEX) {
                course->links[index].node = remap[course->links[index].node];
            }
        }
        free(course->nodes);
        free(remap);
        course->nodes = compact;
        course->node_count = emitted.count;
    }
    course->first_node = 0u;
    course->last_node = course->node_count - 1u;
    free(emitted.items);

    /* 0x004240f0: the start pose, from the first node's first point. Column 1
       is the negated direction of travel, which is the kart's forward axis
       negated, so a kart placed with this basis faces down the course. */
    {
        const KartCourseNode *first = &course->nodes[course->first_node];
        const KartVec3 up = {0.0f, 0.0f, 1.0f}; /* DAT_005b16f0 */
        KartVec3 backward;
        KartVec3 right;
        KartVec3 lift;
        if (first->point_count == 0) return false;
        backward = scale(course->points[first->point_first].direction, -1.0f);
        right = normalize(cross(backward, up));
        lift = normalize(cross(right, backward));
        course->start_position = add(course->points[first->point_first].position,
                                     scale(backward, 0.5f));
        course->start_basis[0][0] = right.x;
        course->start_basis[1][0] = right.y;
        course->start_basis[2][0] = right.z;
        course->start_basis[0][1] = backward.x;
        course->start_basis[1][1] = backward.y;
        course->start_basis[2][1] = backward.z;
        course->start_basis[0][2] = lift.x;
        course->start_basis[1][2] = lift.y;
        course->start_basis[2][2] = lift.z;
    }
    return true;
}

void kart_course_free(KartCourse *course)
{
    if (course == NULL) return;
    free(course->nodes);
    free(course->gates);
    free(course->links);
    free(course->points);
    memset(course, 0, sizeof *course);
}

void kart_course_set_lap_count(KartCourse *course, unsigned int laps)
{
    if (course != NULL) course->lap_count = laps;
}

/* --- gate crossing (0x00425fe0) ----------------------------------------- */

int kart_course_gate_crossing(
    const KartCourseGate *gate,
    KartVec3 segment_start,
    KartVec3 segment_end)
{
    const KartVec3 direction = subtract(segment_end, segment_start);
    unsigned int face;
    if (gate == NULL) return 0;
    for (face = 0; face < 2; ++face) {
        if (!kart_track_segment_triangle_hit(
                segment_start, direction, gate->face[face][0],
                gate->face[face][1], gate->face[face][2])) {
            continue;
        }
        /* The only threshold in the whole mechanism, and it is zero. A grazing
           pass exactly along the gate plane therefore counts as forward. */
        return dot(direction, gate->normal) < 0.0f ? -1 : 1;
    }
    return 0;
}

/* --- progress (0x00426470, 0x00424b30) ---------------------------------- */

/* 0x00426670: how far into the current node the kart is, and which point of
   the node's polyline it is past. */
static void measure_progress(
    const KartCourse *course,
    KartCourseProgress *progress,
    KartVec3 position)
{
    const KartCourseNode *node = &course->nodes[progress->node];
    unsigned int index = 0;
    float travelled = 0.0f;
    while (index < node->point_count) {
        const KartCoursePoint *point = &course->points[node->point_first + index];
        if (dot(subtract(position, point->position), point->direction) < 0.0f) break;
        ++index;
    }
    progress->node_distance = 0.0f;
    if (index != 0) {
        unsigned int step;
        for (step = 0; step + 1u < index; ++step) {
            const KartCoursePoint *a = &course->points[node->point_first + step];
            const KartCoursePoint *b = &course->points[node->point_first + step + 1u];
            travelled += length_of(subtract(b->position, a->position));
        }
        {
            const KartCoursePoint *last = &course->points[node->point_first + index - 1u];
            travelled += dot(subtract(position, last->position), last->direction);
        }
        progress->node_distance = travelled;
    }
    progress->point = index != 0 ? index - 1u : 0u;
    if (progress->node_distance > node->length) {
        progress->node_distance = node->length;
    }
}

/* The kart's forward axis. 0x00426aa0 reads column 1 of the kart's basis and
   negates it, which is the same axis orientation_axes() reports in
   kart_track_collision.c. */
static KartVec3 orientation_forward(KartQuat q)
{
    return (KartVec3){
        -2.0f * (q.x * q.y - q.w * q.z),
        -(1.0f - 2.0f * (q.x * q.x + q.z * q.z)),
        -2.0f * (q.y * q.z + q.w * q.x),
    };
}

/* 0x00424b30's tail, record[10]. The kart is going the wrong way when the
   course direction at its node - the node ahead minus the node behind -
   opposes both where it points and where it is moving. Either one agreeing
   with the course clears the flag, so spinning out does not raise it. */
static bool wrong_way(
    const KartCourse *course,
    unsigned int node,
    KartQuat orientation,
    KartVec3 velocity)
{
    /* _DAT_005717b8. */
    const float limit = -0.5f;
    const KartVec3 facing = orientation_forward(orientation);
    const KartVec3 motion = normalize(velocity);
    unsigned int ahead;
    for (ahead = course->nodes[node].backward; ahead != KART_COURSE_NO_INDEX;
         ahead = course->links[ahead].next) {
        unsigned int behind;
        const unsigned int ahead_node = course->links[ahead].node;
        if (ahead_node == KART_COURSE_NO_INDEX ||
            course->nodes[ahead_node].point_count == 0) continue;
        for (behind = course->nodes[node].forward; behind != KART_COURSE_NO_INDEX;
             behind = course->links[behind].next) {
            const unsigned int behind_node = course->links[behind].node;
            KartVec3 travel;
            if (behind_node == KART_COURSE_NO_INDEX ||
                course->nodes[behind_node].point_count == 0) continue;
            travel = normalize(subtract(
                course->points[course->nodes[ahead_node].point_first].position,
                course->points[course->nodes[behind_node].point_first].position));
            if (dot(travel, facing) > limit) return false;
            if (dot(travel, motion) > limit) return false;
        }
    }
    return true;
}

void kart_course_progress_init(
    const KartCourse *course,
    KartCourseProgress *progress,
    KartVec3 position)
{
    if (progress == NULL) return;
    memset(progress, 0, sizeof *progress);
    if (course == NULL || course->node_count == 0) {
        progress->node = KART_COURSE_NO_INDEX;
        return;
    }
    /* 0x00424530 seeds every kart with the course's last node and no advance.
       The start pose sits behind the first node's gate, so the first crossing
       of that gate is what puts the kart on lap 1. */
    progress->node = course->last_node;
    progress->node_id = course->nodes[course->last_node].id;
    measure_progress(course, progress, position);
}

int kart_course_progress_step(
    const KartCourse *course,
    KartCourseProgress *progress,
    KartVec3 previous_position,
    KartVec3 position,
    KartQuat orientation,
    KartVec3 velocity,
    unsigned int time_ms)
{
    int advance = 0;
    unsigned int link;
    bool moved = false;
    if (course == NULL || progress == NULL || progress->node == KART_COURSE_NO_INDEX) {
        return 0;
    }
    /* 0x00426470. The original walks every segment of the kart's position
       trail; one simulation step is one segment of that trail. */
    for (link = course->nodes[progress->node].backward;
         link != KART_COURSE_NO_INDEX; link = course->links[link].next) {
        const unsigned int target = course->links[link].node;
        if (target == KART_COURSE_NO_INDEX) continue;
        if (kart_course_gate_crossing(&course->gates[course->links[link].gate],
                                      previous_position, position) <= 0) continue;
        if (target == course->first_node) {
            advance = 1;
        } else if (course->gates[course->links[link].gate].is_final &&
                   (unsigned int)progress->advance == course->lap_count) {
            /* The `final` gate closes the race for a course that does not end
               where it starts; only the last lap counts it. */
            advance = 1;
        }
        progress->node = target;
        moved = true;
        break;
    }
    if (!moved) {
        for (link = course->nodes[progress->node].forward;
             link != KART_COURSE_NO_INDEX; link = course->links[link].next) {
            const unsigned int target = course->links[link].node;
            if (target == KART_COURSE_NO_INDEX) continue;
            if (kart_course_gate_crossing(&course->gates[course->links[link].gate],
                                          previous_position, position) >= 0) continue;
            if (progress->node == course->first_node) advance = -1;
            progress->node = target;
            break;
        }
    }

    progress->lap_completed = false;
    if (advance == 1 && progress->advance == (int)progress->lap) {
        if (progress->lap == 0) {
            progress->lap_start_ms = time_ms;
            progress->lap = 1;
        } else {
            const unsigned int lap_ms = time_ms - progress->lap_start_ms;
            progress->best_lap_ms = progress->best_lap_ms != 0 &&
                                    progress->best_lap_ms < lap_ms
                ? progress->best_lap_ms : lap_ms;
            progress->lap_start_ms = time_ms;
            ++progress->lap;
            progress->lap_completed = true;
        }
    }
    progress->advance += advance;
    progress->node_id = course->nodes[progress->node].id;
    measure_progress(course, progress, position);
    progress->wrong_way = wrong_way(course, progress->node, orientation, velocity);
    return advance;
}

/* --- placement ---------------------------------------------------------- */

/* The quaternion for a basis whose columns are the original's. The kart's
   forward axis is column 1 negated (orientation_axes in
   kart_track_collision.c), so this and that stay each other's inverse. */
static KartQuat quaternion_from_basis(const float m[3][3])
{
    KartQuat q;
    const float trace = m[0][0] + m[1][1] + m[2][2];
    if (trace > 0.0f) {
        const float s = sqrtf(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (m[2][1] - m[1][2]) / s;
        q.y = (m[0][2] - m[2][0]) / s;
        q.z = (m[1][0] - m[0][1]) / s;
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const float s = sqrtf(1.0f + m[0][0] - m[1][1] - m[2][2]) * 2.0f;
        q.w = (m[2][1] - m[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (m[0][1] + m[1][0]) / s;
        q.z = (m[0][2] + m[2][0]) / s;
    } else if (m[1][1] > m[2][2]) {
        const float s = sqrtf(1.0f + m[1][1] - m[0][0] - m[2][2]) * 2.0f;
        q.w = (m[0][2] - m[2][0]) / s;
        q.x = (m[0][1] + m[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (m[1][2] + m[2][1]) / s;
    } else {
        const float s = sqrtf(1.0f + m[2][2] - m[0][0] - m[1][1]) * 2.0f;
        q.w = (m[1][0] - m[0][1]) / s;
        q.x = (m[0][2] + m[2][0]) / s;
        q.y = (m[1][2] + m[2][1]) / s;
        q.z = 0.25f * s;
    }
    return q;
}

void kart_course_start_pose(
    const KartCourse *course,
    unsigned int grid_index,
    KartVec3 *position,
    KartQuat *orientation)
{
    KartVec3 right;
    float offset;
    if (course == NULL || course->node_count == 0) return;
    /* 0x004260e0: even slots step out one way and odd slots the other, in
       steps of 2 (_DAT_0057180c is -2.0). Slot 0 sits on the course's own
       start pose. The ground snap the original follows this with is the
       caller's job; it needs the collision scene. */
    offset = (grid_index % 2u == 0u)
        ? (float)(grid_index / 2u) + (float)(grid_index / 2u)
        : (float)(grid_index / 2u + 1u) * -2.0f;
    right = (KartVec3){course->start_basis[0][0], course->start_basis[1][0],
                       course->start_basis[2][0]};
    if (position != NULL) {
        *position = add(course->start_position, scale(right, offset));
    }
    if (orientation != NULL) *orientation = quaternion_from_basis(course->start_basis);
}

bool kart_course_respawn_pose(
    const KartCourse *course,
    const KartCourseProgress *progress,
    KartVec3 *position,
    KartQuat *orientation)
{
    const KartCourseNode *node;
    const KartCoursePoint *point;
    const KartVec3 up = {0.0f, 0.0f, 1.0f}; /* DAT_005b16f0 */
    KartVec3 backward;
    KartVec3 right;
    KartVec3 lift;
    float basis[3][3];
    if (course == NULL || progress == NULL) return false;
    if (progress->node == KART_COURSE_NO_INDEX ||
        progress->node >= course->node_count) return false;
    node = &course->nodes[progress->node];
    if (node->point_count == 0) return false;
    /* 0x00424640 uses point 0 of the node the kart is in, not the point it is
       nearest. It also does not normalise the two cross products, so a banked
       road gives a basis that is not quite orthonormal; that is reproduced. */
    point = &course->points[node->point_first];
    backward = scale(point->direction, -1.0f);
    right = cross(backward, up);
    lift = cross(right, backward);
    basis[0][0] = right.x; basis[1][0] = right.y; basis[2][0] = right.z;
    basis[0][1] = backward.x; basis[1][1] = backward.y; basis[2][1] = backward.z;
    basis[0][2] = lift.x; basis[1][2] = lift.y; basis[2][2] = lift.z;
    if (position != NULL) *position = add(point->position, scale(backward, 0.5f));
    if (orientation != NULL) *orientation = quaternion_from_basis(basis);
    return true;
}
