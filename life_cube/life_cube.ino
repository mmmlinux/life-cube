/*
 * Conway's Game of Life on a rotating, bouncing 3D cube.
 * Waveshare ESP32-C5-LCD-1.47 (172 x 320 ST7789 SPI, driven landscape 320 x 172).
 *
 * Life runs on the *surface* of the cube as one connected topology: patterns
 * crawl across face edges and around corners instead of dying against a frame.
 * The cube spins on all three axes and bounces around the screen DVD-logo style.
 *
 * Board reference: https://github.com/waveshareteam/esp32-c5-lcd-1.47
 *
 * Build from the repo root (needs the esp32:esp32 core >= 3.3.11 and
 * Arduino_GFX 1.6.5+); see README.md for the full FQBN:
 *   arduino-cli compile -u -p /dev/cu.usbmodem3101 --fqbn esp32:esp32:esp32c5 life_cube
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#else
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#include <Arduino_GFX_Library.h>

#if __has_include(<esp_random.h>)
#include <esp_random.h>
#endif

// ---------------------------------------------------------------------------
// Display wiring - matches HARDWARE_REFERENCE.md / 01_lcd_panel_basic.ino
// ---------------------------------------------------------------------------
#define LCD_H_RES              (172)
#define LCD_V_RES              (320)
#define LCD_SPI_FREQ_HZ        (40 * 1000 * 1000)
#define LCD_SPI_SCLK           (7)
#define LCD_SPI_MOSI           (6)
#define LCD_SPI_MISO           (GFX_NOT_DEFINED)
#define LCD_SPI_CS             (23)
#define LCD_SPI_DC             (24)
#define LCD_SPI_RST            (26)
#define LCD_BACKLIGHT          (10)
#define LCD_X_GAP              (34)
#define LCD_Y_GAP              (0)
#define LCD_ROTATION_LANDSCAPE (3)

#define BACKLIGHT_LEDC_CH      (0)
#define BACKLIGHT_LEDC_FREQ_HZ (5000)
#define BACKLIGHT_LEDC_BITS    (8)

// Onboard WS2812B indicator - HARDWARE_REFERENCE.md / 04_ws2812_rgb.ino
#define RGB_LED_PIN (8)

#define SCREEN_W (320)
#define SCREEN_H (172)

static Arduino_DataBus *s_bus = new Arduino_ESP32SPI(
    LCD_SPI_DC, LCD_SPI_CS, LCD_SPI_SCLK, LCD_SPI_MOSI, LCD_SPI_MISO);

static Arduino_GFX *s_gfx = new Arduino_ST7789(
    s_bus, LCD_SPI_RST, LCD_ROTATION_LANDSCAPE, true,
    LCD_H_RES, LCD_V_RES,
    LCD_X_GAP, LCD_Y_GAP, LCD_X_GAP, LCD_Y_GAP);

// ---------------------------------------------------------------------------
// Offscreen sprite
//
// The cube is drawn into a square RAM canvas that gets blitted at the cube's
// current position, rather than into a full-screen framebuffer. 112x112x2 is
// 25 KB against 110 KB for full screen, on a board with no PSRAM, and it keeps
// the blit small: measured uncapped, render + blit runs at ~175 fps (5.7 ms a
// frame), which is why the loop paces itself rather than running flat out.
//
// The canvas is constructed with a null output device: it is never flush()ed
// (flush() would dereference that null output), only read back via
// getFramebuffer() and blitted by hand.
// ---------------------------------------------------------------------------
#define SPRITE     (112)
#define SPRITE_MID (SPRITE / 2.0f)

static Arduino_Canvas *s_spr = new Arduino_Canvas(SPRITE, SPRITE, nullptr);
static uint16_t *s_spr_fb = nullptr;


// ---------------------------------------------------------------------------
// Cube geometry / projection
//
// Model cube spans [-1,1]^3, camera sits at +Z distance CAM_D looking at the
// origin. PROJ_SCALE was picked by sweeping 216000 orientations on the host and
// taking the worst-case projected corner: max |X/(CAM_D-Z)| = 0.4804, so
// 0.4804 * 112 = 53.8 px from centre against a 56 px half-sprite. ~2 px spare.
// ---------------------------------------------------------------------------
#define CAM_D      (4.0f)
#define PROJ_SCALE (112.0f)

#define N           (12)                 // cells per face edge
#define FACE_CELLS  (N * N)
#define TOTAL_CELLS (6 * FACE_CELLS)
#define MAX_NBR     (8)
#define VERTS       ((N + 1) * (N + 1))  // projected grid vertices per face

// ---------------------------------------------------------------------------
// Life on the cube surface
// ---------------------------------------------------------------------------
#define SEED_DENSITY_PCT       (32)
#define GENERATION_INTERVAL_MS (170)
#define STAGNATION_HOLD_MS     (900)
#define HASH_RING_LEN          (12)
#define MAX_GENERATIONS_NO_LOOP (1500)

static uint8_t  s_cur[TOTAL_CELLS];
static uint8_t  s_next[TOTAL_CELLS];
static uint8_t  s_nbr_count[TOTAL_CELLS];
static int16_t  s_nbr[TOTAL_CELLS][MAX_NBR];

static uint32_t s_hash_ring[HASH_RING_LEN];
static uint8_t  s_hash_ring_pos = 0;
static uint32_t s_generation = 0;

// Motion state
//
// All rates are per *second*, not per frame: the render loop turns over at a
// few hundred fps if left alone, which would make a per-frame increment spin
// the cube into a blur. Motion is integrated against measured elapsed time and
// the frame rate is capped, so the animation looks the same regardless.
#define TARGET_FPS (60)
#define FRAME_US   (1000000 / TARGET_FPS)

#define SPIN_X_RAD_S (0.85f)   // deliberately incommensurate rates, so the
#define SPIN_Y_RAD_S (0.57f)   // cube never settles into a repeating tumble
#define SPIN_Z_RAD_S (0.35f)
#define DRIFT_X_PX_S (32.0f)
#define DRIFT_Y_PX_S (15.0f)

static float s_pos_x, s_pos_y, s_vel_x, s_vel_y;
static float s_ang_x = 0.0f, s_ang_y = 0.0f, s_ang_z = 0.0f;
static int   s_prev_x = -1, s_prev_y = -1;
// Longest inter-frame gap since boot, reported on the reseed line. Sticky
// rather than reset each time: a stall that happened ten seconds ago is exactly
// the thing worth knowing about, and a number that clears itself would hide it.
// Should sit just above the 16.7 ms frame budget - anything near 2000 ms means
// a USB CDC write is blocking the loop again (see the note in setup()).
static uint32_t s_worst_gap_us = 0;

// Palette, re-rolled on every reseed.
//
// The hue is randomised but saturation/value are fixed, so every roll lands on
// a colour that reads well on this panel. Picking raw random RGB instead would
// regularly produce muddy or near-black cells. The three tones share a hue so
// the cube stays one object rather than a colour clash.
static uint8_t s_live_rgb[3];
static uint8_t s_face_rgb[3];
static uint8_t s_edge_rgb[3];
static int     s_hue = -1;   // -1 until the first roll

// Scratch: projected vertex grid for the face being drawn
static int16_t s_vx[VERTS];
static int16_t s_vy[VERTS];

// ---------------------------------------------------------------------------
// Backlight
// ---------------------------------------------------------------------------
static void set_backlight(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    const uint32_t duty = (percent * ((1 << BACKLIGHT_LEDC_BITS) - 1)) / 100;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(LCD_BACKLIGHT, duty);
#else
    ledcWrite(BACKLIGHT_LEDC_CH, duty);
#endif
}

static void init_backlight(uint8_t percent)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(LCD_BACKLIGHT, BACKLIGHT_LEDC_FREQ_HZ, BACKLIGHT_LEDC_BITS);
#else
    ledcSetup(BACKLIGHT_LEDC_CH, BACKLIGHT_LEDC_FREQ_HZ, BACKLIGHT_LEDC_BITS);
    ledcAttachPin(LCD_BACKLIGHT, BACKLIGHT_LEDC_CH);
#endif
    set_backlight(percent);
}

// ---------------------------------------------------------------------------
// Cube surface topology
//
// Face f covers axis f/2 at sign (f&1 ? -1 : +1). Its two in-plane axes are
// u = (axis+1)%3 and v = (axis+2)%3, and cell (i,j) of that face sits at
// u = -1 + (2i+1)/N, v = -1 + (2j+1)/N. Rendering uses the same convention, so
// the texture and the topology always agree.
//
// Neighbours come from probing one cell-width out in each of the 8 directions
// and folding the resulting 3D point back onto the cube with a cube-map lookup
// (largest-magnitude component picks the face). Points that walk off an edge
// land on the adjacent face at the correct distance from the shared edge; the
// 24 cells at the cube's 8 corners correctly end up with 7 neighbours each.
// ---------------------------------------------------------------------------
static inline int face_id(int axis, int sign)
{
    return axis * 2 + (sign < 0 ? 1 : 0);
}

static int cubemap_cell(const float q[3])
{
    int m = 0;
    float best = fabsf(q[0]);
    if (fabsf(q[1]) > best) { best = fabsf(q[1]); m = 1; }
    if (fabsf(q[2]) > best) { best = fabsf(q[2]); m = 2; }

    const int sm = (q[m] < 0.0f) ? -1 : 1;
    const int u2 = (m + 1) % 3;
    const int v2 = (m + 2) % 3;
    const float inv = 1.0f / fabsf(q[m]);

    int i = (int)floorf((q[u2] * inv + 1.0f) * 0.5f * N);
    int j = (int)floorf((q[v2] * inv + 1.0f) * 0.5f * N);
    if (i < 0) { i = 0; } else if (i >= N) { i = N - 1; }
    if (j < 0) { j = 0; } else if (j >= N) { j = N - 1; }

    return face_id(m, sm) * FACE_CELLS + j * N + i;
}

static bool add_neighbor(int a, int b)
{
    if (a == b) {
        return false;
    }
    for (int k = 0; k < s_nbr_count[a]; k++) {
        if (s_nbr[a][k] == b) {
            return false;
        }
    }
    if (s_nbr_count[a] >= MAX_NBR) {
        return false;
    }
    s_nbr[a][s_nbr_count[a]++] = (int16_t)b;
    return true;
}

static void build_topology(void)
{
    memset(s_nbr_count, 0, sizeof(s_nbr_count));

    for (int f = 0; f < 6; f++) {
        const int axis = f / 2;
        const int sign = (f & 1) ? -1 : 1;
        const int u = (axis + 1) % 3;
        const int v = (axis + 2) % 3;

        for (int j = 0; j < N; j++) {
            for (int i = 0; i < N; i++) {
                const int idx = f * FACE_CELLS + j * N + i;
                const float cu = -1.0f + (2.0f * i + 1.0f) / N;
                const float cv = -1.0f + (2.0f * j + 1.0f) / N;

                for (int dj = -1; dj <= 1; dj++) {
                    for (int di = -1; di <= 1; di++) {
                        if (di == 0 && dj == 0) {
                            continue;
                        }
                        float q[3];
                        q[axis] = (float)sign;
                        q[u] = cu + di * (2.0f / N);
                        q[v] = cv + dj * (2.0f / N);
                        add_neighbor(idx, cubemap_cell(q));
                    }
                }
            }
        }
    }

    // A cube corner is a genuine argmax tie for the diagonal probe, so the
    // fold's axis-order tie-break could in principle make adjacency one-way.
    // Force symmetry rather than trusting it, then report what we ended up
    // with - one-way adjacency doesn't crash, it just quietly makes Life
    // misbehave near the edges.
    int added = 0;
    for (int a = 0; a < TOTAL_CELLS; a++) {
        const int cnt = s_nbr_count[a];
        for (int k = 0; k < cnt; k++) {
            if (add_neighbor(s_nbr[a][k], a)) {
                added++;
            }
        }
    }

    int hist[MAX_NBR + 1];
    memset(hist, 0, sizeof(hist));
    for (int a = 0; a < TOTAL_CELLS; a++) {
        hist[s_nbr_count[a]]++;
    }

    Serial.printf("topology: %d cells, %d symmetry fixups\n", TOTAL_CELLS, added);
    for (int d = 0; d <= MAX_NBR; d++) {
        if (hist[d]) {
            Serial.printf("  degree %d: %d cells%s\n", d, hist[d],
                          (d == 7) ? "  (cube corners)" : "");
        }
    }
}

// ---------------------------------------------------------------------------
// Palette
// ---------------------------------------------------------------------------
static void hsv_to_rgb(float h, float sat, float val, uint8_t *out)
{
    const float c = val * sat;
    const float hp = h / 60.0f;
    const float x = c * (1.0f - fabsf(fmodf(hp, 2.0f) - 1.0f));
    const float m = val - c;

    float r = 0.0f, g = 0.0f, b = 0.0f;
    switch ((int)hp % 6) {
        case 0: r = c; g = x;         break;
        case 1: r = x; g = c;         break;
        case 2:        g = c; b = x;  break;
        case 3:        g = x; b = c;  break;
        case 4: r = x;        b = c;  break;
        default: r = c;       b = x;  break;
    }

    out[0] = (uint8_t)lrintf((r + m) * 255.0f);
    out[1] = (uint8_t)lrintf((g + m) * 255.0f);
    out[2] = (uint8_t)lrintf((b + m) * 255.0f);
}

static void roll_palette(void)
{
    // Reject hues too close to the outgoing one, or a reseed can land on a
    // near-identical colour and read as "nothing happened".
    int hue;
    do {
        hue = (int)(esp_random() % 360);
        int delta = abs(hue - s_hue);
        if (delta > 180) {
            delta = 360 - delta;
        }
        if (s_hue < 0 || delta >= 60) {
            break;
        }
    } while (true);
    s_hue = hue;

    // Saturation/value match the original mint-green palette, so only the hue
    // moves: bright cells, a mid-tone edge, and a barely-lit face backdrop.
    hsv_to_rgb((float)hue, 0.69f, 1.00f, s_live_rgb);
    hsv_to_rgb((float)hue, 0.67f, 0.35f, s_edge_rgb);
    hsv_to_rgb((float)hue, 0.62f, 0.10f, s_face_rgb);

    // Logged from in here rather than at the call site: it cannot report a
    // stale palette, and it makes the boot roll visible too.
    Serial.printf("palette: hue=%d rgb=%d,%d,%d\n", s_hue,
                  s_live_rgb[0], s_live_rgb[1], s_live_rgb[2]);
}

// ---------------------------------------------------------------------------
// Life rules
// ---------------------------------------------------------------------------
static void seed_grid(uint8_t density_percent)
{
    for (int i = 0; i < TOTAL_CELLS; i++) {
        s_cur[i] = ((esp_random() % 100) < density_percent) ? 1 : 0;
    }
}

static int step_life(void)
{
    int live_count = 0;

    for (int i = 0; i < TOTAL_CELLS; i++) {
        int neighbors = 0;
        const int cnt = s_nbr_count[i];
        for (int k = 0; k < cnt; k++) {
            neighbors += s_cur[s_nbr[i][k]];
        }

        uint8_t next_alive;
        if (s_cur[i]) {
            next_alive = (neighbors == 2 || neighbors == 3) ? 1 : 0;
        } else {
            next_alive = (neighbors == 3) ? 1 : 0;
        }
        s_next[i] = next_alive;
        live_count += next_alive;
    }

    memcpy(s_cur, s_next, TOTAL_CELLS);
    return live_count;
}

static uint32_t hash_grid(void)
{
    uint32_t hash = 2166136261u;
    for (int i = 0; i < TOTAL_CELLS; i++) {
        hash ^= s_cur[i];
        hash *= 16777619u;
    }
    return hash;
}

static bool hash_seen_recently(uint32_t hash)
{
    for (int i = 0; i < HASH_RING_LEN; i++) {
        if (s_hash_ring[i] == hash) {
            return true;
        }
    }
    return false;
}

static void push_hash(uint32_t hash)
{
    s_hash_ring[s_hash_ring_pos] = hash;
    s_hash_ring_pos = (s_hash_ring_pos + 1) % HASH_RING_LEN;
}

static void start_new_generation(void)
{
    roll_palette();
    seed_grid(SEED_DENSITY_PCT);
    s_generation = 0;
    memset(s_hash_ring, 0, sizeof(s_hash_ring));
    s_hash_ring_pos = 0;
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
#define COLOR_BG (0x0000)

static inline uint16_t rgb565(int r, int g, int b)
{
    if (r < 0) r = 0; else if (r > 255) r = 255;
    if (g < 0) g = 0; else if (g > 255) g = 255;
    if (b < 0) b = 0; else if (b > 255) b = 255;
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static inline uint16_t shade(int r, int g, int b, float k)
{
    return rgb565((int)(r * k), (int)(g * k), (int)(b * k));
}

// Fills the quad v0-v1-v2-v3 (given as indices into the projected vertex grid).
// The diagonal is redrawn as a line because Arduino_GFX's scanline fill can
// leave a hairline seam between the two triangles of a quad.
static inline void fill_quad(int a, int b, int c, int d, uint16_t color)
{
    s_spr->fillTriangle(s_vx[a], s_vy[a], s_vx[b], s_vy[b], s_vx[c], s_vy[c], color);
    s_spr->fillTriangle(s_vx[a], s_vy[a], s_vx[c], s_vy[c], s_vx[d], s_vy[d], color);
    s_spr->drawLine(s_vx[a], s_vy[a], s_vx[c], s_vy[c], color);
}

static void render_cube(void)
{
    // R = Rz * Ry * Rx
    const float cx = cosf(s_ang_x), sx = sinf(s_ang_x);
    const float cy = cosf(s_ang_y), sy = sinf(s_ang_y);
    const float cz = cosf(s_ang_z), sz = sinf(s_ang_z);

    const float R[3][3] = {
        { cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx },
        { sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx },
        { -sy,     cy * sx,                cy * cx                },
    };

    // Light roughly over the viewer's shoulder.
    const float lx = 0.35f, ly = -0.45f, lz = 0.82f;

    s_spr->fillScreen(COLOR_BG);

    for (int f = 0; f < 6; f++) {
        const int axis = f / 2;
        const int sign = (f & 1) ? -1 : 1;
        const int u = (axis + 1) % 3;
        const int v = (axis + 2) % 3;

        // Face centre and outward normal coincide for a unit cube: both are
        // sign * (the axis'th column of R).
        const float nx = R[0][axis] * sign;
        const float ny = R[1][axis] * sign;
        const float nz = R[2][axis] * sign;

        // Exact perspective backface test: is the outward normal pointing away
        // from the ray that reaches this face from the camera at (0,0,CAM_D)?
        if (nx * nx + ny * ny + nz * (nz - CAM_D) >= 0.0f) {
            continue;
        }

        // Project this face's (N+1)x(N+1) vertex grid once, then let every cell
        // reuse the four corners it shares with its neighbours.
        for (int gj = 0; gj <= N; gj++) {
            const float gv = -1.0f + 2.0f * gj / N;
            for (int gi = 0; gi <= N; gi++) {
                const float gu = -1.0f + 2.0f * gi / N;

                float p[3];
                p[axis] = (float)sign;
                p[u] = gu;
                p[v] = gv;

                const float X = R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2];
                const float Y = R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2];
                const float Z = R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2];

                const float w = PROJ_SCALE / (CAM_D - Z);
                const int idx = gj * (N + 1) + gi;
                s_vx[idx] = (int16_t)lrintf(X * w + SPRITE_MID);
                s_vy[idx] = (int16_t)lrintf(Y * w + SPRITE_MID);
            }
        }

        float lambert = nx * lx + ny * ly + nz * lz;
        if (lambert < 0.0f) {
            lambert = 0.0f;
        }
        const float k = 0.34f + 0.66f * lambert;

        const uint16_t face_color = shade(s_face_rgb[0], s_face_rgb[1], s_face_rgb[2], k);
        const uint16_t live_color = shade(s_live_rgb[0], s_live_rgb[1], s_live_rgb[2], k);
        const uint16_t edge_color = shade(s_edge_rgb[0], s_edge_rgb[1], s_edge_rgb[2], k);

        // Face backdrop first; the gaps left between cell quads read as grid
        // lines against it.
        fill_quad(0, N, (N + 1) * (N + 1) - 1, N * (N + 1), face_color);

        const int base = f * FACE_CELLS;
        for (int j = 0; j < N; j++) {
            for (int i = 0; i < N; i++) {
                if (!s_cur[base + j * N + i]) {
                    continue;
                }
                const int a = j * (N + 1) + i;
                fill_quad(a, a + 1, a + N + 2, a + N + 1, live_color);
            }
        }

        // Silhouette/crease outline - cheap but does most of the work in
        // making the thing read as a solid cube.
        const int c0 = 0;
        const int c1 = N;
        const int c2 = (N + 1) * (N + 1) - 1;
        const int c3 = N * (N + 1);
        s_spr->drawLine(s_vx[c0], s_vy[c0], s_vx[c1], s_vy[c1], edge_color);
        s_spr->drawLine(s_vx[c1], s_vy[c1], s_vx[c2], s_vy[c2], edge_color);
        s_spr->drawLine(s_vx[c2], s_vy[c2], s_vx[c3], s_vy[c3], edge_color);
        s_spr->drawLine(s_vx[c3], s_vy[c3], s_vx[c0], s_vy[c0], edge_color);
    }
}

// Blits the sprite at (x,y) and erases whatever the previous blit left behind.
static void present(int x, int y)
{
    if (s_prev_x >= 0) {
        const int dx = x - s_prev_x;
        const int dy = y - s_prev_y;

        if (abs(dx) >= SPRITE || abs(dy) >= SPRITE) {
            s_gfx->fillRect(s_prev_x, s_prev_y, SPRITE, SPRITE, COLOR_BG);
        } else {
            // Old rect minus new rect is covered by one vertical and one
            // horizontal strip (they overlap in a corner; erasing twice is
            // harmless and far cheaper than computing the exact L).
            if (dx > 0) {
                s_gfx->fillRect(s_prev_x, s_prev_y, dx, SPRITE, COLOR_BG);
            } else if (dx < 0) {
                s_gfx->fillRect(x + SPRITE, s_prev_y, -dx, SPRITE, COLOR_BG);
            }
            if (dy > 0) {
                s_gfx->fillRect(s_prev_x, s_prev_y, SPRITE, dy, COLOR_BG);
            } else if (dy < 0) {
                s_gfx->fillRect(s_prev_x, y + SPRITE, SPRITE, -dy, COLOR_BG);
            }
        }
    }

    s_gfx->draw16bitRGBBitmap(x, y, s_spr_fb, SPRITE, SPRITE);
    s_prev_x = x;
    s_prev_y = y;
}

static void update_motion(float dt)
{
    s_ang_x += SPIN_X_RAD_S * dt;
    s_ang_y += SPIN_Y_RAD_S * dt;
    s_ang_z += SPIN_Z_RAD_S * dt;
    if (s_ang_x > (float)TWO_PI) s_ang_x -= (float)TWO_PI;
    if (s_ang_y > (float)TWO_PI) s_ang_y -= (float)TWO_PI;
    if (s_ang_z > (float)TWO_PI) s_ang_z -= (float)TWO_PI;

    const float max_x = (float)(SCREEN_W - SPRITE);
    const float max_y = (float)(SCREEN_H - SPRITE);

    s_pos_x += s_vel_x * dt;
    s_pos_y += s_vel_y * dt;

    if (s_pos_x < 0.0f)   { s_pos_x = 0.0f;   s_vel_x = -s_vel_x; }
    if (s_pos_x > max_x)  { s_pos_x = max_x;  s_vel_x = -s_vel_x; }
    if (s_pos_y < 0.0f)   { s_pos_y = 0.0f;   s_vel_y = -s_vel_y; }
    if (s_pos_y > max_y)  { s_pos_y = max_y;  s_vel_y = -s_vel_y; }
}

// Advances motion by real elapsed time, redraws, and paces to TARGET_FPS.
static void animate_frame(void)
{
    static uint32_t last_us = 0;
    const uint32_t start_us = micros();

    const uint32_t gap_us = start_us - last_us;
    const bool first_frame = (last_us == 0);
    last_us = start_us;
    if (!first_frame && gap_us > s_worst_gap_us) {
        s_worst_gap_us = gap_us;
    }

    float dt = gap_us * 1e-6f;
    if (dt > 0.1f) {
        dt = 0.1f;   // first frame, or a long stall - don't teleport the cube
    }

    update_motion(dt);
    render_cube();
    present((int)s_pos_x, (int)s_pos_y);

    const uint32_t spent = micros() - start_us;
    if (spent < FRAME_US) {
        const uint32_t remaining = FRAME_US - spent;
        if (remaining > 2000) {
            delay(remaining / 1000);
        } else {
            delayMicroseconds(remaining);
        }
    }
}

void setup(void)
{
    Serial.begin(115200);

    // Don't let logging stall the animation.
    //
    // On USB CDC, write() blocks up to max_consec_timeouts (20) *
    // tx_timeout_ms (100) = 2 SECONDS once the TX ring fills. That happens
    // whenever the board is plugged into a host that isn't draining the port
    // - i.e. any time you're watching the panel without a serial monitor
    // open. isPlugged() stays true, so the core treats it as host
    // backpressure and waits it out, freezing the render loop mid-tumble
    // every time the 5 s fps line fires.
    //
    // A zero timeout makes writes drop instead of wait: log lines are
    // best-effort, frames are not. Note `if (Serial)` is NOT a workaround -
    // it reports isCDC_Connected(), which stays true under backpressure.
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
#endif

    delay(100);
    Serial.println("ESP32-C5-LCD-1.47 Game of Life on a bouncing 3D cube");

    // Kill the onboard WS2812. It latches whatever it was last sent and holds
    // it with no refresh, so a single zero frame keeps it dark for the whole
    // run - and it also clears any colour left over by a previously flashed
    // sketch, or garbage the data line picked up while GPIO8 floated at reset.
    rgbLedWriteOrdered(RGB_LED_PIN, LED_COLOR_ORDER_RGB, 0, 0, 0);

    if (!s_gfx->begin(LCD_SPI_FREQ_HZ)) {
        Serial.println("LCD init failed");
        while (true) {
            delay(1000);
        }
    }
    s_gfx->invertDisplay(false);
    s_gfx->displayOn();
    s_gfx->fillScreen(COLOR_BG);
    init_backlight(80);

    // Null output device, so skip the output begin() and just allocate.
    if (!s_spr->begin(GFX_SKIP_OUTPUT_BEGIN)) {
        Serial.println("sprite canvas alloc failed");
        while (true) {
            delay(1000);
        }
    }
    s_spr_fb = s_spr->getFramebuffer();

    Serial.printf("sprite %dx%d (%u bytes), free heap %u\n",
                  SPRITE, SPRITE, (unsigned)(SPRITE * SPRITE * 2),
                  (unsigned)ESP.getFreeHeap());

    build_topology();

    randomSeed(esp_random());
    s_pos_x = (SCREEN_W - SPRITE) / 2.0f;
    s_pos_y = (SCREEN_H - SPRITE) / 2.0f;
    s_vel_x = (esp_random() & 1) ? DRIFT_X_PX_S : -DRIFT_X_PX_S;
    s_vel_y = (esp_random() & 1) ? DRIFT_Y_PX_S : -DRIFT_Y_PX_S;

    start_new_generation();
}

void loop(void)
{
    static uint32_t last_gen_ms = 0;
    const uint32_t now = millis();

    // Life advances on its own clock; the cube keeps spinning and bouncing at
    // full frame rate in between generations.
    if (now - last_gen_ms >= GENERATION_INTERVAL_MS) {
        last_gen_ms = now;

        const int live_count = step_life();
        s_generation++;

        const uint32_t hash = hash_grid();
        const bool looping = hash_seen_recently(hash);
        push_hash(hash);

        const bool dead = (live_count == 0);
        const bool stuck = (s_generation >= MAX_GENERATIONS_NO_LOOP);

        if (dead || looping || stuck) {
            Serial.printf("gen=%lu live=%d reason=%s max_frame_gap=%lums\n",
                          (unsigned long)s_generation, live_count,
                          dead ? "died-out" : looping ? "cycle-detected" : "max-generations",
                          (unsigned long)(s_worst_gap_us / 1000));

            // Hold the final state on screen, still spinning, before reseeding.
            const uint32_t hold_until = millis() + STAGNATION_HOLD_MS;
            while ((int32_t)(millis() - hold_until) < 0) {
                animate_frame();
            }
            start_new_generation();
            last_gen_ms = millis();
        }
    }

    animate_frame();
}
