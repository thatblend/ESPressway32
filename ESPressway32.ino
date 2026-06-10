#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Preferences.h>
#include <math.h>

// User Config
// Leave false for the normal T-Display-S3 orientation. Set true to rotate the screen 180 degrees.
#define DISPLAY_FLIP_180 false
#define DISPLAY_ROTATION (DISPLAY_FLIP_180 ? 1 : 3)
// HUD performance readout (bottom-left corner).
#define SHOW_FPS true           // show the frames-per-second counter
#define SHOW_FRAME_TIMING false // also show CPU render time per frame in ms

// Game Constants
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 170
#define NUM_SEGMENTS 400
#define SEGMENT_LENGTH 4.0f
#define ROAD_WIDTH 5.0f
#define CURB_WIDTH 0.6f
#define MAX_SPEED 240.0f // km/h
#define TARGET_FPS 60
#define FRAME_TIME_US (1000000UL / TARGET_FPS)
#define BRAKE_DECEL_BASE 140.0f
#define BRAKE_DECEL_HIGH_SPEED_BONUS 140.0f
#define TURN_ROLL_AMOUNT 0.04f
#define TURN_STEER_VISUAL_AMOUNT 0.16f
#define TURN_VISUAL_RESPONSE 7.5f
#define PLAYER_CAR_CAMERA_Y -1.05f
#define PLAYER_CAR_CAMERA_Z 5.4f
#define ROAD_DRAW_SEGMENTS 42
#define SCENERY_DRAW_SEGMENTS 30
#define MODEL_MAX_VERTICES 256
#define MODEL_MAX_FACES 384
#define NUM_OPPONENTS 4
// Panel write clock. 20 MHz is the known-good speed for the T-Display-S3's
// ST7789. Higher values (e.g. 24-32 MHz) may work on some units but can show
// as shimmering static / missing screen regions -- raise only in small steps.
#define LCD_BUS_WRITE_HZ 20000000
// Async DMA frame push (double buffered). Set to 0 to fall back to blocking
// pushSprite if the display ever misbehaves -- the game then runs slower but
// uses the exact same proven transfer path as the original code.
#define USE_DMA_PUSH 1
#define FOG_COLOR 0xAE7C            // hazy horizon blue-grey (RGB565)
#define FOG_START_SEGMENT 12        // road segments before distance fog kicks in
#define GROUND_HALF_WIDTH 46.0f     // lateral reach of the terrain skirt per side
#define GROUND_EDGE_DROP 1.1f       // outer terrain edge dips below the road for soft hillsides
#define MAX_PARTICLES 28
#define MINIMAP_POINTS 80

// 3D Math Structures
struct Point3D {
    float x, y, z;
};

struct Point2D {
    float x, y;
};

struct Face {
    uint8_t indices[4];
    uint8_t num_vertices;
    uint8_t flags;  // bit0: double-sided (exempt from backface culling)
    uint16_t color; // 0xFFFF means use base color
};

struct TrackSegment {
    float x, y, z;       // Center point in world coordinates
    float rx, rz;        // Right vector (normalized)
    float fx, fy, fz;    // Forward vector (normalized)
    float curve;         // Track curvature
    float pitch;         // Track height change
    bool is_alternate;   // Alternating colors for lines/curbs
};

// Global Rendering Variables
class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789 _panel;
    lgfx::Bus_Parallel8 _bus;
    lgfx::Light_PWM _light;

public:
    LGFX() {
        auto bus_cfg = _bus.config();
        bus_cfg.freq_write = LCD_BUS_WRITE_HZ;
        bus_cfg.pin_wr = 8;
        bus_cfg.pin_rd = 9;
        bus_cfg.pin_rs = 7;
        bus_cfg.pin_d0 = 39;
        bus_cfg.pin_d1 = 40;
        bus_cfg.pin_d2 = 41;
        bus_cfg.pin_d3 = 42;
        bus_cfg.pin_d4 = 45;
        bus_cfg.pin_d5 = 46;
        bus_cfg.pin_d6 = 47;
        bus_cfg.pin_d7 = 48;
        _bus.config(bus_cfg);
        _panel.setBus(&_bus);

        auto panel_cfg = _panel.config();
        panel_cfg.pin_cs = 6;
        panel_cfg.pin_rst = 5;
        panel_cfg.pin_busy = -1;
        panel_cfg.panel_width = 170;
        panel_cfg.panel_height = 320;
        panel_cfg.memory_width = 240;
        panel_cfg.memory_height = 320;
        panel_cfg.offset_x = 35;
        panel_cfg.offset_y = 0;
        panel_cfg.offset_rotation = 0;
        panel_cfg.dummy_read_pixel = 8;
        panel_cfg.dummy_read_bits = 1;
        panel_cfg.readable = false;
        panel_cfg.rgb_order = false;
        panel_cfg.invert = true;
        panel_cfg.dlen_16bit = false;
        panel_cfg.bus_shared = false; // bus is exclusive to the panel; keeps DMA pushes async
        _panel.config(panel_cfg);

        auto light_cfg = _light.config();
        light_cfg.pin_bl = 38;
        light_cfg.invert = false;
        light_cfg.freq = 44100;
        light_cfg.pwm_channel = 0;
        _light.config(light_cfg);
        _panel.setLight(&_light);

        setPanel(&_panel);
    }
};

LGFX tft;
// Two full-screen framebuffers: while one streams to the panel over DMA, the
// next frame renders into the other. All draw code keeps using the name
// "sprite" through the alias below; loop() flips fb_idx after each push.
LGFX_Sprite fb[2] = { LGFX_Sprite(&tft), LGFX_Sprite(&tft) };
uint8_t fb_idx = 0;
bool use_dma = false;
#define sprite fb[fb_idx]
Preferences prefs;

float cam_x = 0.0f;
float cam_y = 0.0f;
float cam_z = 0.0f;
float cam_yaw = 0.0f;
float cam_pitch = 0.0f;
float cam_cos_yaw = 1.0f;
float cam_sin_yaw = 0.0f;
float cam_cos_pitch = 1.0f;
float cam_sin_pitch = 0.0f;

float center_x = 160.0f;
float center_y = 85.0f;
float fov = 180.0f;

// Sun/Light source direction in world space (normalized)
const float light_dir_x = 0.577f;
const float light_dir_y = 0.707f;
const float light_dir_z = -0.408f;

// Active light direction in WORLD space (the sun during the race, animated in
// the menu). draw3DModel rotates it into model space once per call, so each
// face is lit with a single dot product against its precomputed normal.
float g_light_x = 0.577f;
float g_light_y = 0.707f;
float g_light_z = -0.408f;

// Track array
TrackSegment track[NUM_SEGMENTS];

// 3D model data: winding-corrected, outward-facing, with precomputed unit
// face normals for one-dot-product lighting and exact backface culling.
// The data tables themselves live at the BOTTOM of this file so the game
// code stays together; regenerate them with tools/gen_models.py.
// ===== GENERATED MODEL DECLARATIONS (tools/gen_models.py) -- do not hand-edit =====
#define CAR_NUM_VERTICES 215
#define CAR_NUM_FACES 368
extern const Point3D car_vertices[CAR_NUM_VERTICES];
extern const Face car_faces[CAR_NUM_FACES];
extern const Point3D car_normals[CAR_NUM_FACES];
#define LOD_CAR_NUM_VERTICES 32
#define LOD_CAR_NUM_FACES 23
extern const Point3D lod_car_vertices[LOD_CAR_NUM_VERTICES];
extern const Face lod_car_faces[LOD_CAR_NUM_FACES];
extern const Point3D lod_car_normals[LOD_CAR_NUM_FACES];
#define TREE_NUM_VERTICES 13
#define TREE_NUM_FACES 9
extern const Point3D tree_vertices[TREE_NUM_VERTICES];
extern const Face tree_faces[TREE_NUM_FACES];
extern const Point3D tree_normals[TREE_NUM_FACES];
#define BILLBOARD_NUM_VERTICES 8
#define BILLBOARD_NUM_FACES 1
extern const Point3D billboard_vertices[BILLBOARD_NUM_VERTICES];
extern const Face billboard_faces[BILLBOARD_NUM_FACES];
extern const Point3D billboard_normals[BILLBOARD_NUM_FACES];
#define BRIDGE_NUM_VERTICES 24
#define BRIDGE_NUM_FACES 12
extern const Point3D bridge_vertices[BRIDGE_NUM_VERTICES];
extern const Face bridge_faces[BRIDGE_NUM_FACES];
extern const Face gantry_faces[BRIDGE_NUM_FACES];
extern const Point3D bridge_normals[BRIDGE_NUM_FACES];
// ===== END GENERATED MODEL DECLARATIONS =====

// Player State Variables
float player_segment_float = 0.0f;
float player_w = 0.0f;          // Lateral offset (-2.5 to 2.5 on road)
float player_speed = 0.0f;      // km/h
float player_roll = 0.0f;       // Visual tilt when turning
float player_pitch = 0.0f;      // Visual nose up/down following the road grade
float player_steer_angle = 0.0f;// Front wheel steer angle
float wheel_spin_angle = 0.0f;  // Spin value
int player_laps = 0;
bool player_braking = false;    // both buttons held -> show brake lights

unsigned long lap_start_ms = 0;
unsigned long race_start_ms = 0;
unsigned long race_finish_ms = 0;
unsigned long best_time_ms = 99999999;
unsigned long current_lap_ms = 0;

int screen_shake_timer = 0;
unsigned long last_time = 0;
unsigned long last_frame_us = 0;
unsigned long fps_window_start_us = 0;
int fps_window_frames = 0;
float measured_fps = 0.0f;
float perf_frame_ms = 0.0f;
float perf_render_ms = 0.0f;
float perf_push_ms = 0.0f;

// Opponents
struct Opponent {
    float segment;
    float lateral_pos;
    float speed;
    uint16_t color;
    float target_lateral;
    float lane_change_timer; // seconds until the next lane change
    int laps;
};

Opponent opponents[NUM_OPPONENTS] = {
    { 2.0f, -0.9f, 0.0f, 0xFFE0, -0.9f, 0, 0 }, // Yellow
    { 3.0f,  0.9f, 0.0f, 0x07FF,  0.9f, 0, 0 }, // Cyan
    { 4.0f, -0.9f, 0.0f, 0xFD20, -0.9f, 0, 0 }, // Orange
    { 5.0f,  0.9f, 0.0f, 0xF81F,  0.9f, 0, 0 }  // Magenta
};
const float opponent_base_speed[NUM_OPPONENTS] = { 175.0f, 168.0f, 171.0f, 164.0f };

// Screen-space particles: dirt spray off-road, sparks on collisions.
struct Particle {
    float x, y, vx, vy;
    int16_t life;
    uint16_t color;
};
Particle particles[MAX_PARTICLES];
int particle_cursor = 0;

// True top-down minimap outline, precomputed from the track at startup.
int16_t mm_px[MINIMAP_POINTS];
int16_t mm_py[MINIMAP_POINTS];

// Per-draw-distance fog blend factor (0..32 toward FOG_COLOR).
uint8_t fog_table[ROAD_DRAW_SEGMENTS + 2];

enum GameState {
    START_SCREEN,
    COUNTDOWN,
    PLAYING,
    FINISHED
};

GameState current_state = START_SCREEN;
unsigned long countdown_start_ms = 0;

// Function Declarations
void generateTrack();
void resetRaceState();
void updatePhysics(float dt);
void renderTrackAndObjects();
void drawHUD();
void drawCountdown();
void drawFinished();
void drawStartScreen();
void drawMenuGarage(float t);
void drawMenuGarageProps(float t);
void drawMenuQuad3D(float x0, float y0, float z0, float x1, float y1, float z1,
                    float x2, float y2, float z2, float x3, float y3, float z3,
                    uint16_t color);
void drawMenuEllipse(int cx, int cy, int w, int h, uint16_t color);
void drawMenuShadow(int cx, int cy, int w, int h);
void drawBackground(int horizon_y, float yaw);
void drawClouds(int horizon_y, float yaw);
void drawMountains(int horizon_y, float yaw);
void drawQuad(float sx0, float sy0, float sx1, float sy1, float sx2, float sy2, float sx3, float sy3, uint16_t color);
void draw3DModel(const Point3D* vertices, int num_vertices,
                 const Face* faces, const Point3D* normals, int num_faces,
                 float pos_x, float pos_y, float pos_z,
                 float rot_x, float rot_y, float rot_z,
                 float scale, uint16_t base_color, uint8_t fog_a);
void drawCarRearGlass(float pos_x, float pos_y, float pos_z,
                      float rot_x, float rot_y, float rot_z,
                      float scale, bool braking);
void drawBillboard(float pos_x, float pos_y, float pos_z, float rot_y, float scale, uint16_t color, uint8_t fog_a);
void drawTreeImpostor(float pos_x, float pos_y, float pos_z, float scale, uint8_t fog_a);
void drawOpponentBrakeLights(float pos_x, float pos_y, float pos_z, float yaw, bool braking);
void drawStartFinishGantry(int seg, uint8_t fog_a);
void drawTracksideLake(int seg, uint8_t fog_a);
void drawSegmentScenery(int seg, uint8_t fog_a);
void drawSegmentOpponents(int seg, uint8_t fog_a);
bool projectPoint(float wx, float wy, float wz, float& sx, float& sy, float& sz);
void getTrackPosition(float seg_float, float lateral_pos, float& out_x, float& out_y, float& out_z, float& out_rx, float& out_rz);
void cameraRelativeToWorld(float local_x, float local_y, float local_z, float& out_x, float& out_y, float& out_z);
void updateCameraTrig();
float stableJitter(int seg, int side, int index, int salt);
float approachFloat(float current, float target, float response, float dt);
void buildMinimap();
void buildFogTable();
void spawnParticle(float x, float y, float vx, float vy, int life, uint16_t color);
void spawnImpactSparks(int cx, int cy, float dir);
void updateAndDrawParticles();
void shadowEllipse(int cx, int cy, int rx, int ry, uint8_t darken);
void fillGradientRows(int y_start, int y_end, int span_start, int span_len,
                      int r0, int g0, int b0, int r1, int g1, int b1);

// Integer RGB565 shading: i32 is 0..32 (32 = full brightness).
static inline uint16_t shade565(uint16_t c, uint8_t i32) {
    uint32_t rb = ((uint32_t)(c & 0xF81F) * i32) >> 5;
    uint32_t g  = ((uint32_t)(c & 0x07E0) * i32) >> 5;
    return (uint16_t)((rb & 0xF81F) | (g & 0x07E0));
}

// Integer RGB565 blend: alpha32 is 0..32 toward color b.
static inline uint16_t blend565(uint16_t a, uint16_t b, uint8_t alpha32) {
    uint32_t ia = 32 - alpha32;
    uint32_t rb = ((uint32_t)(a & 0xF81F) * ia + (uint32_t)(b & 0xF81F) * alpha32) >> 5;
    uint32_t g  = ((uint32_t)(a & 0x07E0) * ia + (uint32_t)(b & 0x07E0) * alpha32) >> 5;
    return (uint16_t)((rb & 0xF81F) | (g & 0x07E0));
}

static inline uint16_t pack565(int r, int g, int b) {
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

// Direction-preserving pull-in of a projected point toward the screen
// center. Used to shrink stretched off-screen geometry whose edges are
// visually tolerant (e.g. grass-on-grass), so the rasterizer walks less.
static inline void pullInRadial(float& sx, float& sy, float limit) {
    float ox = sx - center_x;
    float oy = sy - center_y;
    float ax = fabsf(ox);
    float ay = fabsf(oy);
    float m = (ax > ay) ? ax : ay;
    if (m > limit) {
        float k = limit / m;
        sx = center_x + ox * k;
        sy = center_y + oy * k;
    }
}

// Setup Function
void setup() {
    setCpuFrequencyMhz(240);
    
    // Enable Screen Power
    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH);
    
    // Enable Screen Backlight
    pinMode(38, OUTPUT);
    digitalWrite(38, HIGH);
    
    // Set active-low buttons
    pinMode(0, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
    
    prefs.begin("esp-racer", false);
    best_time_ms = prefs.getULong("bestRace", 99999999UL);
    
    // Initialize Display
    tft.init();
    tft.setRotation(DISPLAY_ROTATION); // Landscape mode, optionally flipped 180 degrees.
    tft.setBrightness(255);
    
    // Two framebuffers in internal (DMA-capable) RAM. If the second
    // allocation fails we fall back to single-buffer blocking pushes.
    fb[0].setColorDepth(16);
    fb[1].setColorDepth(16);
    bool fb0_ok = (fb[0].createSprite(SCREEN_WIDTH, SCREEN_HEIGHT) != nullptr);
    use_dma = USE_DMA_PUSH && fb0_ok && (fb[1].createSprite(SCREEN_WIDTH, SCREEN_HEIGHT) != nullptr);
    if (use_dma) {
        tft.initDMA();
        tft.startWrite(); // hold the bus transaction so DMA pushes stay asynchronous
    }

    // Procedurally Generate Track
    generateTrack();
    buildMinimap();
    buildFogTable();
    resetRaceState();
    
    // Timers
    lap_start_ms = millis();
    last_time = millis();
    last_frame_us = micros();
    fps_window_start_us = last_frame_us;
}

// Main Game Loop
void loop() {
    unsigned long now_us = micros();
    unsigned long elapsed_us = now_us - last_frame_us;
    
    // Cap frame starts to the 60 Hz budget.
    if (elapsed_us < FRAME_TIME_US) {
        delay(1);
        return;
    }
    last_frame_us = now_us;
    last_time = millis();
    float dt = elapsed_us / 1000000.0f;
    
    // Limit dt to prevent massive jumps during delays
    if (dt > 0.1f) dt = 0.1f;
    
    if (current_state == START_SCREEN) {
        drawStartScreen();
        
        bool left = (digitalRead(0) == LOW);
        bool right = (digitalRead(14) == LOW);
        if (left || right) {
            current_state = COUNTDOWN;
            countdown_start_ms = millis();
            lap_start_ms = millis();
        }
    } else {
        // Update Game Physics
        updatePhysics(dt);
        
        // Render 3D Race Scene
        renderTrackAndObjects();
        
        // Draw HUD Overlays
        if (current_state == PLAYING || current_state == COUNTDOWN || current_state == FINISHED) {
            drawHUD();
        }
        
        if (current_state == COUNTDOWN) {
            drawCountdown();
        } else if (current_state == FINISHED) {
            drawFinished();
        }
    }
    
    // Push the frame buffer to the physical screen. With DMA the call only
    // kicks off the transfer; the next frame renders into the other buffer
    // while this one streams out, so the push costs ~0 CPU time.
    unsigned long before_push_us = micros();
    if (use_dma) {
        tft.waitDMA(); // previous frame must be fully out before reusing the channel
        tft.pushImageDMA(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT,
                         (const lgfx::swap565_t*)sprite.getBuffer());
        fb_idx ^= 1;
    } else {
        sprite.pushSprite(0, 0);
    }
    unsigned long after_push_us = micros();
    
    perf_frame_ms = elapsed_us * 0.001f;
    perf_render_ms = (before_push_us - now_us) * 0.001f;
    perf_push_ms = (after_push_us - before_push_us) * 0.001f;
    
    fps_window_frames++;
    unsigned long fps_elapsed_us = after_push_us - fps_window_start_us;
    if (fps_elapsed_us >= 1000000UL) {
        measured_fps = (fps_window_frames * 1000000.0f) / (float)fps_elapsed_us;
        fps_window_frames = 0;
        fps_window_start_us = after_push_us;
    }
}

// Procedural Track Generation
void generateTrack() {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float heading = 0.0f;
    float elevation = 0.0f;
    int seg_count = 0;
    
    // High-level track design
    struct SegmentDef {
        int length;
        float curve;
        float pitch;
    };
    
    // Pitch values create rolling hills; they are chosen so the summed
    // elevation returns to ~0 over the lap (the closure pass absorbs the rest).
    SegmentDef track_layout[] = {
        { 40,  0.0f,      0.0f },      // Start straight (flat)
        { 40,  0.03927f,  0.0040f },   // 90 Right, climbing
        { 30,  0.0f,     -0.0050f },   // Straight over the crest
        { 40, -0.03927f, -0.0040f },   // 90 Left, descending into the valley
        { 50,  0.0f,      0.0030f },   // Straight, climbing back to level
        { 40,  0.03927f,  0.0042f },   // 90 Right, second climb
        { 30,  0.0f,     -0.0056f },   // Straight, fast drop
        { 40,  0.07854f,  0.0f },      // 180 Right hairpin (flat)
        { 50,  0.0f,     -0.0030f },   // Back straight, gentle descent
        { 40,  0.03927f,  0.00375f }   // Final 90 Right, climbing to the line
    };
    
    int num_defs = sizeof(track_layout) / sizeof(SegmentDef);
    
    for (int l = 0; l < num_defs; l++) {
        for (int s = 0; s < track_layout[l].length; s++) {
            if (seg_count >= NUM_SEGMENTS) break;
            
            heading += track_layout[l].curve;
            elevation += track_layout[l].pitch;

            x += sinf(heading) * SEGMENT_LENGTH;
            z += cosf(heading) * SEGMENT_LENGTH;
            y += elevation * SEGMENT_LENGTH;
            
            track[seg_count].x = x;
            track[seg_count].y = y;
            track[seg_count].z = z;
            track[seg_count].curve = track_layout[l].curve;
            track[seg_count].pitch = track_layout[l].pitch;
            track[seg_count].is_alternate = ((seg_count / 3) % 2 == 0);
            
            seg_count++;
        }
    }
    
    // Seamless Track Closure Math
    float err_x = track[seg_count-1].x + sinf(heading)*SEGMENT_LENGTH - track[0].x;
    float err_y = track[seg_count-1].y + elevation*SEGMENT_LENGTH - track[0].y;
    float err_z = track[seg_count-1].z + cosf(heading)*SEGMENT_LENGTH - track[0].z;
    
    for (int i = 0; i < seg_count; i++) {
        float factor = (float)i / (float)seg_count;
        track[i].x -= err_x * factor;
        track[i].y -= err_y * factor;
        track[i].z -= err_z * factor;
    }
    
    // Precalculate local segment forward and right vectors
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        int next = (i + 1) % NUM_SEGMENTS;
        
        float fx = track[next].x - track[i].x;
        float fy = track[next].y - track[i].y;
        float fz = track[next].z - track[i].z;
        float len = sqrtf(fx*fx + fy*fy + fz*fz);

        track[i].fx = fx / len;
        track[i].fy = fy / len;
        track[i].fz = fz / len;

        // Right vector is perpendicular in the horizontal plane
        float rx = fz;
        float rz = -fx;
        float len_r = sqrtf(rx*rx + rz*rz);
        
        track[i].rx = rx / len_r;
        track[i].rz = rz / len_r;
    }
}

void resetRaceState() {
    player_segment_float = 0.0f;
    player_w = 0.0f;
    player_speed = 0.0f;
    player_roll = 0.0f;
    player_pitch = 0.0f;
    player_steer_angle = 0.0f;
    wheel_spin_angle = 0.0f;
    player_laps = 0;
    race_start_ms = 0;
    race_finish_ms = 0;
    current_lap_ms = 0;
    screen_shake_timer = 0;
    player_braking = false;

    opponents[0] = Opponent{ 2.0f, -0.9f, 0.0f, 0xFFE0, -0.9f, 0, 0 };
    opponents[1] = Opponent{ 3.0f,  0.9f, 0.0f, 0x07FF,  0.9f, 0, 0 };
    opponents[2] = Opponent{ 4.0f, -0.9f, 0.0f, 0xFD20, -0.9f, 0, 0 };
    opponents[3] = Opponent{ 5.0f,  0.9f, 0.0f, 0xF81F,  0.9f, 0, 0 };

    for (int i = 0; i < MAX_PARTICLES; i++) {
        particles[i].life = 0;
    }
}

// Physics Loop
void updatePhysics(float dt) {
    if (current_state == PLAYING) {
        // Read active-low buttons
        bool steer_left = (digitalRead(0) == LOW);
        bool steer_right = (digitalRead(14) == LOW);
        
        // Speed Auto-Acceleration and Friction
        bool off_road = (fabsf(player_w) > 2.5f);
        player_braking = (steer_left && steer_right && player_speed > 1.0f);
        if (steer_left && steer_right) {
            // Braking
            float speed_ratio = player_speed / MAX_SPEED;
            if (speed_ratio < 0.0f) speed_ratio = 0.0f;
            if (speed_ratio > 1.0f) speed_ratio = 1.0f;
            float brake_decel = BRAKE_DECEL_BASE + BRAKE_DECEL_HIGH_SPEED_BONUS * speed_ratio;
            player_speed -= brake_decel * dt;
            if (player_speed < 0.0f) player_speed = 0.0f;
        } else if (off_road) {
            // Speed limits on grass
            if (player_speed > 80.0f) {
                player_speed -= 120.0f * dt; // Slow down rapidly
            } else {
                player_speed += 12.0f * dt;
                if (player_speed > 80.0f) player_speed = 80.0f;
            }
        } else {
            // Normal track acceleration
            bool cornering = (steer_left != steer_right);
            float accel_rate = 22.0f;
            if (player_speed < 100.0f) accel_rate = 38.0f; // Boost off-the-line acceleration
            if (player_speed >= 150.0f) accel_rate *= 0.5f;
            if (cornering) accel_rate *= 0.33f;
            player_speed += accel_rate * dt;
            if (player_speed > MAX_SPEED) player_speed = MAX_SPEED;
        }

        // Dirt spray while off-road
        if (off_road && player_speed > 30.0f) {
            for (int d = 0; d < 2; d++) {
                spawnParticle(160 + random(-42, 43), 124 + random(0, 12),
                              random(-12, 13) * 0.1f, -random(8, 28) * 0.1f,
                              random(12, 26), (d & 1) ? 0x9367 : 0x4DE8);
            }
        }

        // Steer physics & body bank
        float steer_rate = 3.6f;
        float target_roll = 0.0f;
        float target_steer_angle = 0.0f;
        if (steer_left && !steer_right) {
            player_w -= steer_rate * dt;
            target_roll = -TURN_ROLL_AMOUNT;
            target_steer_angle = -TURN_STEER_VISUAL_AMOUNT;
        } else if (steer_right && !steer_left) {
            player_w += steer_rate * dt;
            target_roll = TURN_ROLL_AMOUNT;
            target_steer_angle = TURN_STEER_VISUAL_AMOUNT;
        }
        player_roll = approachFloat(player_roll, target_roll, TURN_VISUAL_RESPONSE, dt);
        player_steer_angle = approachFloat(player_steer_angle, target_steer_angle, TURN_VISUAL_RESPONSE, dt);
        
        // Centrifugal curve physics. dt-scaled (8.4 * dt == the old 0.14 per
        // frame at 60 fps) so cornering difficulty no longer depends on the
        // frame rate -- per-frame application made corners easier at low fps.
        int p_seg = (int)player_segment_float % NUM_SEGMENTS;
        float curve = track[p_seg].curve;
        float force = curve * (player_speed / 50.0f) * (player_speed / 50.0f) * 8.4f * dt;
        player_w -= force;

        // Tilt the car nose up/down to follow the local road grade
        // (interpolated between segments, eased so crests feel smooth).
        int pitch_next = (p_seg + 1) % NUM_SEGMENTS;
        float pitch_frac = player_segment_float - (int)player_segment_float;
        float slope_fy = track[p_seg].fy * (1.0f - pitch_frac) + track[pitch_next].fy * pitch_frac;
        player_pitch = approachFloat(player_pitch, -asinf(slope_fy), 5.0f, dt);
        
        // Crash boundary checking
        if (player_w > 3.8f) {
            player_w = 3.8f;
            player_speed *= 0.5f; // lose speed
            screen_shake_timer = 12; // shake camera
            spawnImpactSparks(212, 122, -1.0f);
        } else if (player_w < -3.8f) {
            player_w = -3.8f;
            player_speed *= 0.5f;
            screen_shake_timer = 12;
            spawnImpactSparks(108, 122, 1.0f);
        }
        
        // Spin wheels
        wheel_spin_angle += 0.05f * player_speed * dt;
        if (wheel_spin_angle > 2.0f * PI) wheel_spin_angle -= 2.0f * PI;
        
        // Track position advancement
        player_segment_float += (player_speed / 3.6f) * dt / SEGMENT_LENGTH;
        if (player_segment_float >= NUM_SEGMENTS) {
            player_segment_float -= NUM_SEGMENTS;
            player_laps++;
            
            // Register Lap completed
            unsigned long now_ms = millis();
            current_lap_ms = now_ms - lap_start_ms;
            lap_start_ms = now_ms;
            
            // Race Finish Line (3 Laps)
            if (player_laps >= 3) {
                current_state = FINISHED;
                race_finish_ms = (race_start_ms > 0) ? (now_ms - race_start_ms) : 0;
                if (race_finish_ms > 0 && race_finish_ms < best_time_ms) {
                    best_time_ms = race_finish_ms;
                    prefs.putULong("bestRace", best_time_ms);
                }
                player_speed = 0.0f;
            }
        }
    } else if (current_state == COUNTDOWN) {
        player_speed = 0.0f;
        player_roll = 0.0f;
        player_steer_angle = 0.0f;
        player_braking = false;
        
        if (millis() - countdown_start_ms > 4000) {
            current_state = PLAYING;
            lap_start_ms = millis();
            race_start_ms = lap_start_ms;
            race_finish_ms = 0;
        }
    }
    
    // Update Opponent AI
    if (current_state == PLAYING) {
        for (int i = 0; i < NUM_OPPONENTS; i++) {
            Opponent& opp = opponents[i];
        
            opp.segment += (opp.speed / 3.6f) * dt / SEGMENT_LENGTH;
            if (opp.segment >= NUM_SEGMENTS) {
                opp.segment -= NUM_SEGMENTS;
                opp.laps++;
            }
            
            // AI lane changing timer (dt-based, so AI behaves the same at any fps)
            if (opp.lane_change_timer <= 0.0f) {
                opp.target_lateral = random(-14, 15) / 10.0f; // lane position [-1.4, 1.4]
                opp.lane_change_timer = random(17, 50) * 0.1f; // 1.7 - 5.0 seconds
            } else {
                opp.lane_change_timer -= dt;
            }
            opp.lateral_pos = approachFloat(opp.lateral_pos, opp.target_lateral, 1.2f, dt);

            // AI deceleration on curves (3.9/s == the old 0.065 per frame at 60 fps)
            int opp_seg = (int)opp.segment % NUM_SEGMENTS;
            float curve = fabsf(track[opp_seg].curve);
            float base_speed = opponent_base_speed[i];
            float target_speed = (curve > 0.015f) ? (base_speed - 38.0f) : base_speed;
            opp.speed = approachFloat(opp.speed, target_speed, 3.9f, dt);
            
            // Collision: Player vs Opponent
            float dist_seg = fabsf(player_segment_float - opp.segment);
            if (dist_seg > NUM_SEGMENTS / 2.0f) dist_seg = NUM_SEGMENTS - dist_seg;
            
            if (dist_seg < 0.6f) { // close in track segment
                float dist_w = fabsf(player_w - opp.lateral_pos);
                if (dist_w < 0.7f) { // close laterally
                    player_speed *= 0.75f; // player loses speed
                    opp.speed *= 0.8f;     // opponent loses speed
                    screen_shake_timer = 8;
                    spawnImpactSparks(160 + (player_w > opp.lateral_pos ? -18 : 18), 112,
                                      player_w > opp.lateral_pos ? 1.0f : -1.0f);
                    
                    // bounce apart
                    if (player_w > opp.lateral_pos) {
                        player_w += 0.25f;
                    } else {
                        player_w -= 0.25f;
                    }
                }
            }
        }
    }
}

// 3D Scene Rendering Engine
void renderTrackAndObjects() {
    float cam_segment_float = player_segment_float - 1.2f;
    if (cam_segment_float < 0.0f) cam_segment_float += NUM_SEGMENTS;
    
    // Find Camera 3D Position
    float cam_rx, cam_rz;
    getTrackPosition(cam_segment_float, player_w * 0.95f, cam_x, cam_y, cam_z, cam_rx, cam_rz);
    cam_y += 1.2f; // elevation above track
    
    // Find Camera Look-at Target 3D Position
    float target_segment_float = player_segment_float + 2.0f;
    if (target_segment_float >= NUM_SEGMENTS) target_segment_float -= NUM_SEGMENTS;
    
    float target_x, target_y, target_z, target_rx, target_rz;
    getTrackPosition(target_segment_float, player_w, target_x, target_y, target_z, target_rx, target_rz);
    target_y += 0.35f;
    
    // Yaw/Pitch Camera Vectors
    float dx = target_x - cam_x;
    float dy = target_y - cam_y;
    float dz = target_z - cam_z;
    
    cam_yaw = atan2f(dx, dz);
    cam_pitch = atan2f(dy, sqrtf(dx*dx + dz*dz));
    updateCameraTrig();
    
    // Apply Crash Camera Shake
    if (screen_shake_timer > 0) {
        center_x = 160.0f + random(-3, 4);
        center_y = 85.0f + random(-3, 4);
        screen_shake_timer--;
    } else {
        center_x = 160.0f;
        center_y = 85.0f;
    }
    
    // The race is lit by the fixed world-space sun (the menu animates this).
    g_light_x = light_dir_x;
    g_light_y = light_dir_y;
    g_light_z = light_dir_z;
    
    // Render Parallax Environment
    int horizon_y = (int)(center_y + cam_pitch * fov);
    drawBackground(horizon_y, cam_yaw);
    
    // Render 3D Road Segments using Painter's Algorithm (Far to Near)
    int start_i = (int)cam_segment_float;
    float half_w = ROAD_WIDTH * 0.5f;
    float curb_w = half_w + CURB_WIDTH;
    float line_w = 0.1f;
    
    struct RoadProjection {
        float l_curb_x, l_curb_y;
        float l_road_x, l_road_y;
        float r_road_x, r_road_y;
        float r_curb_x, r_curb_y;
        float line_l_x, line_l_y;
        float line_r_x, line_r_y;
        float l_grnd_x, l_grnd_y;
        float r_grnd_x, r_grnd_y;
        bool offview;
    };
    
    static RoadProjection road_proj[ROAD_DRAW_SEGMENTS + 2];
    for (int i = 0; i <= ROAD_DRAW_SEGMENTS + 1; i++) {
        int seg = (start_i + i) % NUM_SEGMENTS;
        float scratch_z;
        
        projectPoint(track[seg].x - track[seg].rx * curb_w, track[seg].y, track[seg].z - track[seg].rz * curb_w,
                     road_proj[i].l_curb_x, road_proj[i].l_curb_y, scratch_z);
        projectPoint(track[seg].x - track[seg].rx * half_w, track[seg].y, track[seg].z - track[seg].rz * half_w,
                     road_proj[i].l_road_x, road_proj[i].l_road_y, scratch_z);
        projectPoint(track[seg].x + track[seg].rx * half_w, track[seg].y, track[seg].z + track[seg].rz * half_w,
                     road_proj[i].r_road_x, road_proj[i].r_road_y, scratch_z);
        projectPoint(track[seg].x + track[seg].rx * curb_w, track[seg].y, track[seg].z + track[seg].rz * curb_w,
                     road_proj[i].r_curb_x, road_proj[i].r_curb_y, scratch_z);
        projectPoint(track[seg].x - track[seg].rx * line_w, track[seg].y, track[seg].z - track[seg].rz * line_w,
                     road_proj[i].line_l_x, road_proj[i].line_l_y, scratch_z);
        projectPoint(track[seg].x + track[seg].rx * line_w, track[seg].y, track[seg].z + track[seg].rz * line_w,
                     road_proj[i].line_r_x, road_proj[i].line_r_y, scratch_z);
        // Terrain skirt edges, following track elevation. The width adapts to
        // the segment's distance (just enough to cover the screen laterally
        // at that depth), and each side's corner is then clipped against a
        // slightly expanded view frustum IN CAMERA SPACE. Camera-space
        // coordinates are exactly linear along the lateral ray, so every
        // frustum plane yields a closed-form width cap. This is what keeps
        // corner geometry small: merely keeping a corner in front of the
        // near plane still lets it project thousands of pixels off-screen on
        // tight corners (the screen-covering "grass mountain" bug).
        {
            // Segment centerline in camera space (same transform as projectPoint).
            float wxc = track[seg].x - cam_x;
            float wyc = track[seg].y - cam_y;
            float wzc = track[seg].z - cam_z;
            float rz1 = wxc * cam_sin_yaw + wzc * cam_cos_yaw;
            float xc = wxc * cam_cos_yaw - wzc * cam_sin_yaw;
            float yc = wyc * cam_cos_pitch - rz1 * cam_sin_pitch;
            float zc = wyc * cam_sin_pitch + rz1 * cam_cos_pitch;

            // View culling: on corners (especially the hairpin) many segments
            // that are ahead along the track are behind or far beside the
            // camera. Rasterizing their off-screen quads is pure waste, so
            // flag them; the draw loop skips whole strips when both ends are
            // off-view. The side cone (|x| <= 3*z + 8) is wider than the
            // skirt frustum so visible geometry is never clipped by this.
            road_proj[i].offview = (i >= 2) &&
                                   ((zc < 0.05f) || (fabsf(xc) > zc * 3.0f + 8.0f));

            float zw = (zc < 0.1f) ? 0.1f : zc;
            float ground_w = (ROAD_WIDTH * 0.5f + CURB_WIDTH + 1.5f) + zw * (300.0f / fov);
            if (ground_w > GROUND_HALF_WIDTH) ground_w = GROUND_HALF_WIDTH;
            float gw_l = ground_w, gw_r = ground_w;

            if (i >= 2) {
                // Camera-space direction of one unit along the segment's right vector.
                float q   = track[seg].rx * cam_sin_yaw + track[seg].rz * cam_cos_yaw;
                float dxr = track[seg].rx * cam_cos_yaw - track[seg].rz * cam_sin_yaw;
                float dyr = -q * cam_sin_pitch;
                float dzr =  q * cam_cos_pitch;

                // Expanded frustum: |x| <= KX*z (~+/-470 px), |y| <= KY*z
                // (~+/-360 px), z >= Z_SAFE. Each plane: A + s*t*B >= 0.
                // (Segments 0-1 keep full stretch: they fill the bottom of
                // the screen by design, exactly like the road quads.)
                const float KX = 2.6f;
                const float KY = 2.0f;
                const float Z_SAFE = 0.35f;
                float A[5] = { zc - Z_SAFE,
                               KX * zc - xc, KX * zc + xc,
                               KY * zc - yc, KY * zc + yc };
                float B[5] = { dzr,
                               KX * dzr - dxr, KX * dzr + dxr,
                               KY * dzr - dyr, KY * dzr + dyr };
                for (int side = 0; side < 2; side++) {
                    float s = side ? 1.0f : -1.0f;
                    float t = ground_w;
                    for (int p = 0; p < 5; p++) {
                        float Bp = s * B[p];
                        if (A[p] + Bp * t < 0.0f) {
                            t = (Bp < -1e-5f) ? (-A[p] / Bp) : 0.05f;
                            if (t < 0.05f) t = 0.05f;
                        }
                    }
                    if (side) gw_r = t; else gw_l = t;
                }
            }

            float drop_l = GROUND_EDGE_DROP * (gw_l / GROUND_HALF_WIDTH);
            float drop_r = GROUND_EDGE_DROP * (gw_r / GROUND_HALF_WIDTH);
            projectPoint(track[seg].x - track[seg].rx * gw_l, track[seg].y - drop_l,
                         track[seg].z - track[seg].rz * gw_l,
                         road_proj[i].l_grnd_x, road_proj[i].l_grnd_y, scratch_z);
            projectPoint(track[seg].x + track[seg].rx * gw_r, track[seg].y - drop_r,
                         track[seg].z + track[seg].rz * gw_r,
                         road_proj[i].r_grnd_x, road_proj[i].r_grnd_y, scratch_z);
            // Near-fill skirts (segments 0-1) only need to reach past the
            // screen edge; their borders are grass-on-grass, so pulling the
            // stretched points way in is invisible and much cheaper to fill.
            // Frustum-clipped corners are already inside this radius.
            pullInRadial(road_proj[i].l_grnd_x, road_proj[i].l_grnd_y, 700.0f);
            pullInRadial(road_proj[i].r_grnd_x, road_proj[i].r_grnd_y, 700.0f);
        }
    }
    
    for (int i = ROAD_DRAW_SEGMENTS; i >= 0; i--) {
        int seg = (start_i + i) % NUM_SEGMENTS;
        RoadProjection& near_p = road_proj[i];
        RoadProjection& far_p = road_proj[i + 1];
        if (near_p.offview && far_p.offview) continue; // strip entirely outside the view
        uint8_t fog_a = fog_table[i];

        // Colors, faded into the horizon haze with distance. The fog also
        // hides the draw-distance pop-in at the last segments.
        uint16_t road_color = track[seg].is_alternate ? 0x39E7 : 0x4208; // alternating greys
        uint16_t curb_color = track[seg].is_alternate ? 0xF800 : 0xFFFF; // alternating red/white
        uint16_t line_color = 0xFFFF;
        if (fog_a) {
            road_color = blend565(road_color, FOG_COLOR, fog_a);
            curb_color = blend565(curb_color, FOG_COLOR, fog_a);
            line_color = blend565(line_color, FOG_COLOR, fog_a);
        }

        // Terrain skirts: wide grass polygons attached to the road edges and
        // following the track's elevation, so hills are solid ground instead
        // of a floating ribbon. Two alternating greens give the classic
        // striped-infield look; the flat gradient backdrop only remains
        // visible near the horizon, where fog blends everything together.
        uint16_t grass_color = track[seg].is_alternate ? 0x3C45 : 0x33C4;
        if (fog_a) grass_color = blend565(grass_color, FOG_COLOR, fog_a);
        drawQuad(near_p.l_grnd_x, near_p.l_grnd_y, near_p.l_curb_x, near_p.l_curb_y,
                 far_p.l_curb_x, far_p.l_curb_y, far_p.l_grnd_x, far_p.l_grnd_y, grass_color);
        drawQuad(near_p.r_curb_x, near_p.r_curb_y, near_p.r_grnd_x, near_p.r_grnd_y,
                 far_p.r_grnd_x, far_p.r_grnd_y, far_p.r_curb_x, far_p.r_curb_y, grass_color);

        // Draw Curbs
        drawQuad(near_p.l_curb_x, near_p.l_curb_y, near_p.l_road_x, near_p.l_road_y,
                 far_p.l_road_x, far_p.l_road_y, far_p.l_curb_x, far_p.l_curb_y, curb_color);
        drawQuad(near_p.r_road_x, near_p.r_road_y, near_p.r_curb_x, near_p.r_curb_y,
                 far_p.r_curb_x, far_p.r_curb_y, far_p.r_road_x, far_p.r_road_y, curb_color);
                 
        // Draw Road surface
        drawQuad(near_p.l_road_x, near_p.l_road_y, near_p.r_road_x, near_p.r_road_y,
                 far_p.r_road_x, far_p.r_road_y, far_p.l_road_x, far_p.l_road_y, road_color);
                 
        // Draw Dashed white line
        if ((seg / 2) % 2 == 0) {
            drawQuad(near_p.line_l_x, near_p.line_l_y, near_p.line_r_x, near_p.line_r_y,
                     far_p.line_r_x, far_p.line_r_y, far_p.line_l_x, far_p.line_l_y, line_color);
        }
        
        // Start/finish checker strip
        if (seg == 0) {
            for (int c = 0; c < 8; c++) {
                float t0 = c * 0.125f;
                float t1 = (c + 1) * 0.125f;
                float n0x = near_p.l_road_x + (near_p.r_road_x - near_p.l_road_x) * t0;
                float n0y = near_p.l_road_y + (near_p.r_road_y - near_p.l_road_y) * t0;
                float n1x = near_p.l_road_x + (near_p.r_road_x - near_p.l_road_x) * t1;
                float n1y = near_p.l_road_y + (near_p.r_road_y - near_p.l_road_y) * t1;
                float f0x = far_p.l_road_x + (far_p.r_road_x - far_p.l_road_x) * t0;
                float f0y = far_p.l_road_y + (far_p.r_road_y - far_p.l_road_y) * t0;
                float f1x = far_p.l_road_x + (far_p.r_road_x - far_p.l_road_x) * t1;
                float f1y = far_p.l_road_y + (far_p.r_road_y - far_p.l_road_y) * t1;
                uint16_t check_color = (c & 1) ? 0x0000 : 0xFFFF;
                if (fog_a) check_color = blend565(check_color, FOG_COLOR, fog_a);
                drawQuad(n0x, n0y, n1x, n1y, f1x, f1y, f0x, f0y, check_color);
            }
        }

        if (i <= SCENERY_DRAW_SEGMENTS) {
            // Keep object draw distance unchanged while testing a longer road horizon.
            drawSegmentScenery(seg, fog_a);
            drawSegmentOpponents(seg, fog_a);
        }
    }
    
    // Render Player Car in the foreground last (forces it on top)
    if (current_state == PLAYING || current_state == COUNTDOWN || current_state == FINISHED) {
        float p_yaw = cam_yaw; // Lock to camera for solid 3D perspective

        // Bouncing hover/engine vibration
        float bounce_y = 0.0f;
        if (player_speed > 0.0f && current_state == PLAYING) {
            bounce_y = 0.018f * sinf(millis() * 0.065f * (player_speed / MAX_SPEED + 0.3f));
        }

        float p_x, p_y, p_z;
        cameraRelativeToWorld(0.0f, PLAYER_CAR_CAMERA_Y + bounce_y, PLAYER_CAR_CAMERA_Z, p_x, p_y, p_z);

        // Ground shadow first; the car is camera-locked so the shadow is a
        // fixed-position translucent darkening of whatever is under it.
        shadowEllipse((int)center_x + (int)(player_roll * 90.0f), 129, 30, 12, 10);

        // Draw body
        draw3DModel(car_vertices, CAR_NUM_VERTICES, car_faces, car_normals, CAR_NUM_FACES,
                    p_x, p_y, p_z,
                    player_pitch, p_yaw + player_steer_angle, player_roll,
                    1.0f, 0x021F, 0); // Subaru blue
        drawCarRearGlass(p_x, p_y, p_z,
                         player_pitch, p_yaw + player_steer_angle, player_roll,
                         1.0f, player_braking && current_state == PLAYING);

        updateAndDrawParticles();
    }
}

// Side of Road Scenery Placement
void drawSegmentScenery(int seg, uint8_t fog_a) {
    float side_offset = 3.8f;
    float bill_offset = 4.2f;
    bool forest_zone = (seg >= 68 && seg < 112) ||
                       (seg >= 186 && seg < 232) ||
                       (seg >= 304 && seg < 344);

    if (seg == 0) {
        drawStartFinishGantry(seg, fog_a);
    }
    // The lake lives in the hairpin zone (segments 270-309), the one stretch
    // of track that is dead flat -- a water surface on a grade looks wrong.
    if (seg == 290) {
        drawTracksideLake(seg, fog_a);
    }

    // Draw Bridge Arch (every 40 segments)
    if (seg % 40 == 20) {
        draw3DModel(bridge_vertices, BRIDGE_NUM_VERTICES, bridge_faces, bridge_normals, BRIDGE_NUM_FACES,
                    track[seg].x, track[seg].y, track[seg].z,
                    0.0f, atan2f(track[seg].fx, track[seg].fz), 0.0f,
                    1.0f, 0x5AEB, fog_a);
    }

    // Draw 3D Grass
    if (seg % 2 == 0) {
        static const float grass_offsets[] = { 1.2f, 2.5f, 4.0f };
        uint16_t blade_color = fog_a ? blend565(0x2D84, FOG_COLOR, fog_a) : 0x2D84;
        for (int side = -1; side <= 1; side += 2) {
            for (int go = 0; go < 3; go++) {
                float jitter = stableJitter(seg, side, go, 0);
                float offset = ROAD_WIDTH / 2.0f + grass_offsets[go] + jitter;
                float gx = track[seg].x + track[seg].rx * side * offset;
                float gz = track[seg].z + track[seg].rz * side * offset;
                float sx, sy, sz;
                if (projectPoint(gx, track[seg].y, gz, sx, sy, sz)) {
                    if (sz > 2.0f && sz < 40.0f) {
                        int h = (int)(25.0f / sz);
                        sprite.drawLine((int16_t)sx, (int16_t)sy, (int16_t)sx - h/2, (int16_t)sy - h, blade_color);
                        sprite.drawLine((int16_t)sx, (int16_t)sy, (int16_t)sx + h/2, (int16_t)sy - h, blade_color);
                    }
                }
            }
        }
    }

    // Draw Tree Left (every 8 segments)
    if (seg % 8 == 0) {
        float tx = track[seg].x - track[seg].rx * side_offset;
        float ty = track[seg].y;
        float tz = track[seg].z - track[seg].rz * side_offset;

        draw3DModel(tree_vertices, TREE_NUM_VERTICES, tree_faces, tree_normals, TREE_NUM_FACES,
                    tx, ty, tz,
                    0.0f, 0.0f, 0.0f,
                    0.9f + 0.15f * sinf((float)seg), 0x04C0, fog_a);
    }

    // Draw Tree Right (every 8 segments, offset by 4)
    if (seg % 8 == 4) {
        float tx = track[seg].x + track[seg].rx * side_offset;
        float ty = track[seg].y;
        float tz = track[seg].z + track[seg].rz * side_offset;

        draw3DModel(tree_vertices, TREE_NUM_VERTICES, tree_faces, tree_normals, TREE_NUM_FACES,
                    tx, ty, tz,
                    0.0f, 0.0f, 0.0f,
                    0.9f + 0.15f * cosf((float)seg), 0x04C0, fog_a);
    }

    // Dense forest pockets in a few parts of the lap
    if (forest_zone) {
        static const float forest_offsets[] = { 5.1f, 6.8f };
        for (int side = -1; side <= 1; side += 2) {
            int side_index = (side > 0) ? 1 : 0;
            for (int fo = 0; fo < 2; fo++) {
                if (((seg + fo + side_index) & 1) != 0) continue;

                float offset = forest_offsets[fo] + stableJitter(seg, side, fo, 5) * 0.8f;
                float tx = track[seg].x + track[seg].rx * side * offset;
                float ty = track[seg].y;
                float tz = track[seg].z + track[seg].rz * side * offset;
                float scale = 0.80f + 0.14f * ((seg + fo + side_index) % 3);

                drawTreeImpostor(tx, ty, tz, scale, fog_a);
            }
        }
    }

    // Draw Billboard Left (every 16 segments)
    if (seg % 16 == 5) {
        float bx = track[seg].x - track[seg].rx * bill_offset;
        float by = track[seg].y;
        float bz = track[seg].z - track[seg].rz * bill_offset;
        float rot_y = atan2f(track[seg].fx, track[seg].fz) + 1.5707f; // face road

        drawBillboard(bx, by, bz, rot_y, 0.8f, 0x001F, fog_a); // Blue
    }

    // Draw Billboard Right (every 16 segments)
    if (seg % 16 == 13) {
        float bx = track[seg].x + track[seg].rx * bill_offset;
        float by = track[seg].y;
        float bz = track[seg].z + track[seg].rz * bill_offset;
        float rot_y = atan2f(track[seg].fx, track[seg].fz) - 1.5707f; // face road

        drawBillboard(bx, by, bz, rot_y, 0.8f, 0xF800, fog_a); // Red
    }
}

void drawStartFinishGantry(int seg, uint8_t fog_a) {
    float yaw = atan2f(track[seg].fx, track[seg].fz);
    draw3DModel(bridge_vertices, BRIDGE_NUM_VERTICES, gantry_faces, bridge_normals, BRIDGE_NUM_FACES,
                track[seg].x, track[seg].y, track[seg].z,
                0.0f, yaw, 0.0f,
                1.05f, 0xFFFF, fog_a);
}

void drawTracksideLake(int seg, uint8_t fog_a) {
    static const float oval_x[] = { 1.00f, 0.70f, 0.00f, -0.70f, -1.00f, -0.70f, 0.00f, 0.70f };
    static const float oval_z[] = { 0.00f, 0.70f, 1.00f,  0.70f,  0.00f, -0.70f, -1.00f, -0.70f };
    
    const int side = -1;
    float center_offset = ROAD_WIDTH * 0.5f + 9.2f;
    float center_forward = 3.5f;
    float shore_rx = 5.2f;
    float shore_rz = 9.5f;
    float water_rx = 4.4f;
    float water_rz = 8.0f;
    float cxw = track[seg].x + track[seg].rx * side * center_offset + track[seg].fx * center_forward;
    float cyw = track[seg].y - 0.04f;
    float czw = track[seg].z + track[seg].rz * side * center_offset + track[seg].fz * center_forward;
    
    Point2D shore[8];
    Point2D water[8];
    float center_sx, center_sy, center_sz;
    if (!projectPoint(cxw, cyw, czw, center_sx, center_sy, center_sz)) return;
    // The near gate must exceed the shore radius (9.5): otherwise shore
    // vertices can cross behind the camera while the center is still ahead,
    // and their near-plane-stretched projections fan wedge triangles across
    // the screen. By z=11 the lake is fully off-screen to the side anyway.
    if (center_sz <= 11.0f || center_sz > 52.0f) return;
    
    for (int i = 0; i < 8; i++) {
        float sx_axis = track[seg].rx * side * oval_x[i];
        float sz_axis = track[seg].rz * side * oval_x[i];
        float fx_axis = track[seg].fx * oval_z[i];
        float fz_axis = track[seg].fz * oval_z[i];
        float scratch_z;
        
        projectPoint(cxw + sx_axis * shore_rx + fx_axis * shore_rz,
                     cyw, czw + sz_axis * shore_rx + fz_axis * shore_rz,
                     shore[i].x, shore[i].y, scratch_z);
        projectPoint(cxw + sx_axis * water_rx + fx_axis * water_rz,
                     cyw + 0.01f, czw + sz_axis * water_rx + fz_axis * water_rz,
                     water[i].x, water[i].y, scratch_z);
    }
    
    uint16_t shore_color = fog_a ? blend565(0xA5A6, FOG_COLOR, fog_a) : 0xA5A6;
    for (int i = 0; i < 8; i++) {
        int next = (i + 1) & 7;
        sprite.fillTriangle((int16_t)center_sx, (int16_t)center_sy,
                            (int16_t)shore[i].x, (int16_t)shore[i].y,
                            (int16_t)shore[next].x, (int16_t)shore[next].y,
                            shore_color);
    }
    for (int i = 0; i < 8; i++) {
        int next = (i + 1) & 7;
        uint16_t water_color = (i & 1) ? 0x04BF : 0x039F;
        if (fog_a) water_color = blend565(water_color, FOG_COLOR, fog_a);
        sprite.fillTriangle((int16_t)center_sx, (int16_t)center_sy,
                            (int16_t)water[i].x, (int16_t)water[i].y,
                            (int16_t)water[next].x, (int16_t)water[next].y,
                            water_color);
    }
    
    sprite.drawLine((int16_t)water[1].x, (int16_t)water[1].y,
                    (int16_t)water[3].x, (int16_t)water[3].y, 0x8EFF);
    sprite.drawLine((int16_t)water[0].x, (int16_t)water[0].y,
                    (int16_t)water[2].x, (int16_t)water[2].y, 0x5D9F);
    sprite.drawLine((int16_t)shore[5].x, (int16_t)shore[5].y,
                    (int16_t)shore[7].x, (int16_t)shore[7].y, 0x2D84);
}

// Render Opponents
void drawSegmentOpponents(int seg, uint8_t fog_a) {
    for (int i = 0; i < NUM_OPPONENTS; i++) {
        int opp_seg = (int)opponents[i].segment % NUM_SEGMENTS;
        if (opp_seg == seg) {
            float ox, oy, oz, orx, orz;
            getTrackPosition(opponents[i].segment, opponents[i].lateral_pos, ox, oy, oz, orx, orz);

            float opp_yaw = atan2f(track[opp_seg].fx, track[opp_seg].fz);

            // Translucent ground shadow keeps the car visually planted.
            float ssx, ssy, ssz;
            projectPoint(ox, oy + 0.02f, oz, ssx, ssy, ssz);
            if (ssz > 1.5f && ssz < 26.0f) {
                int srx = (int)(0.80f * fov / ssz);
                int sry = srx / 3;
                if (sry < 2) sry = 2;
                shadowEllipse((int)ssx, (int)ssy, srx, sry, 9);
            }

            draw3DModel(lod_car_vertices, LOD_CAR_NUM_VERTICES, lod_car_faces, lod_car_normals, LOD_CAR_NUM_FACES,
                        ox, oy, oz,
                        0.0f, opp_yaw, 0.0f,
                        1.0f, opponents[i].color, fog_a);

            bool braking = fabsf(track[opp_seg].curve) > 0.015f;
            drawOpponentBrakeLights(ox, oy, oz, opp_yaw, braking);
        }
    }
}

// 3D Object Renderer.
// - One fused matrix A = Camera * ModelRotation * scale per call, so each
//   vertex is a single 3x3 multiply-add instead of two chained rotations.
// - Exact backface culling using the precomputed model-space face normals:
//   a face is visible iff the eye is on its front side (dot(n, eye - v0) > 0).
//   Double-sided faces (open sheets like the billboard) are never culled.
// - Lighting is one dot product per face against the model-space light, and
//   shading/fog use integer RGB565 math. Culled/behind faces never get sorted.
void draw3DModel(const Point3D* vertices, int num_vertices,
                 const Face* faces, const Point3D* normals, int num_faces,
                 float pos_x, float pos_y, float pos_z,
                 float rot_x, float rot_y, float rot_z,
                 float scale, uint16_t base_color, uint8_t fog_a) {
    static Point2D projected[MODEL_MAX_VERTICES];
    static float camera_z[MODEL_MAX_VERTICES];

    struct FaceRenderData {
        float avg_z;
        uint16_t index;
        uint16_t color;
    };
    static FaceRenderData face_data[MODEL_MAX_FACES];

    if (num_vertices > MODEL_MAX_VERTICES || num_faces > MODEL_MAX_FACES) return;

    float cx = 1.0f, sx = 0.0f, cy = 1.0f, sy = 0.0f, cz = 1.0f, sz = 0.0f;
    if (rot_x != 0.0f) { cx = cosf(rot_x); sx = sinf(rot_x); }
    if (rot_y != 0.0f) { cy = cosf(rot_y); sy = sinf(rot_y); }
    if (rot_z != 0.0f) { cz = cosf(rot_z); sz = sinf(rot_z); }

    // Model rotation matrix M = Ry * Rx * Rz (roll, then pitch, then yaw).
    float m00 = cy * cz + sy * sx * sz, m01 = -cy * sz + sy * sx * cz, m02 = sy * cx;
    float m10 = cx * sz,                m11 = cx * cz,                 m12 = -sx;
    float m20 = -sy * cz + cy * sx * sz, m21 = sy * sz + cy * sx * cz, m22 = cy * cx;

    // Camera matrix C (yaw then pitch, same convention as projectPoint).
    float cyc = cam_cos_yaw, syc = cam_sin_yaw, cp = cam_cos_pitch, sp = cam_sin_pitch;
    float c00 = cyc,       c02 = -syc;
    float c10 = -syc * sp, c11 = cp, c12 = -cyc * sp;
    float c20 = syc * cp,  c21 = sp, c22 = cyc * cp;

    // Fused vertex transform: cam = A * v + b, with A = C * M * scale.
    float a00 = (c00 * m00 + c02 * m20) * scale;
    float a01 = (c00 * m01 + c02 * m21) * scale;
    float a02 = (c00 * m02 + c02 * m22) * scale;
    float a10 = (c10 * m00 + c11 * m10 + c12 * m20) * scale;
    float a11 = (c10 * m01 + c11 * m11 + c12 * m21) * scale;
    float a12 = (c10 * m02 + c11 * m12 + c12 * m22) * scale;
    float a20 = (c20 * m00 + c21 * m10 + c22 * m20) * scale;
    float a21 = (c20 * m01 + c21 * m11 + c22 * m21) * scale;
    float a22 = (c20 * m02 + c21 * m12 + c22 * m22) * scale;

    float dx = pos_x - cam_x, dy = pos_y - cam_y, dz = pos_z - cam_z;
    float b0 = c00 * dx + c02 * dz;
    float b1 = c10 * dx + c11 * dy + c12 * dz;
    float b2 = c20 * dx + c21 * dy + c22 * dz;

    // Whole model at/behind the camera plane: every on-screen part would be
    // clipped face-by-face anyway, and anything this close but off-axis
    // projects far off-screen. Skip the entire transform.
    if (b2 < 0.4f) return;

    // Eye position and light direction in model space (M is orthonormal, so
    // its inverse is the transpose; uniform scale only affects the eye).
    float inv_s = 1.0f / scale;
    float ex = (m00 * -dx + m10 * -dy + m20 * -dz) * inv_s;
    float ey = (m01 * -dx + m11 * -dy + m21 * -dz) * inv_s;
    float ez = (m02 * -dx + m12 * -dy + m22 * -dz) * inv_s;
    float lx = m00 * g_light_x + m10 * g_light_y + m20 * g_light_z;
    float ly = m01 * g_light_x + m11 * g_light_y + m21 * g_light_z;
    float lz = m02 * g_light_x + m12 * g_light_y + m22 * g_light_z;

    for (int i = 0; i < num_vertices; i++) {
        float vx = vertices[i].x, vy = vertices[i].y, vz = vertices[i].z;
        float zc = a20 * vx + a21 * vy + a22 * vz + b2;
        camera_z[i] = zc;
        if (zc > 0.1f) {
            float inv = fov / zc;
            projected[i].x = center_x + (a00 * vx + a01 * vy + a02 * vz + b0) * inv;
            projected[i].y = center_y - (a10 * vx + a11 * vy + a12 * vz + b1) * inv;
        }
    }

    // Cull, light, and gather visible faces.
    int count = 0;
    for (int i = 0; i < num_faces; i++) {
        const Face& f = faces[i];
        const Point3D& n = normals[i];
        const Point3D& p0 = vertices[f.indices[0]];

        float facing = n.x * (ex - p0.x) + n.y * (ey - p0.y) + n.z * (ez - p0.z);
        float light_sign = 1.0f;
        if (facing <= 0.0f) {
            if (!(f.flags & 1)) continue; // backface of a closed surface: invisible
            light_sign = -1.0f;           // double-sided sheet: light the visible side
        }

        float sum_z = 0.0f;
        bool behind = false;
        for (int v = 0; v < f.num_vertices; v++) {
            float z = camera_z[f.indices[v]];
            if (z <= 0.1f) { behind = true; break; }
            sum_z += z;
        }
        if (behind) continue;

        float diff = (n.x * lx + n.y * ly + n.z * lz) * light_sign;
        if (diff < 0.0f) diff = 0.0f;
        int i32 = (int)(19.2f + 13.4f * diff); // ambient 0.60 + diffuse 0.42, in 1/32 steps
        if (i32 > 32) i32 = 32;

        uint16_t col = (f.color == 0xFFFF) ? base_color : f.color;
        col = shade565(col, (uint8_t)i32);
        if (fog_a) col = blend565(col, FOG_COLOR, fog_a);

        face_data[count].avg_z = sum_z / f.num_vertices;
        face_data[count].index = (uint16_t)i;
        face_data[count].color = col;
        count++;
    }

    // Painter's sort: furthest faces first.
    for (int gap = count / 2; gap > 0; gap /= 2) {
        for (int i = gap; i < count; i++) {
            FaceRenderData temp = face_data[i];
            int j = i;
            while (j >= gap && face_data[j - gap].avg_z < temp.avg_z) {
                face_data[j] = face_data[j - gap];
                j -= gap;
            }
            face_data[j] = temp;
        }
    }

    for (int fi = 0; fi < count; fi++) {
        const Face& f = faces[face_data[fi].index];
        uint16_t shaded = face_data[fi].color;
        int i0 = f.indices[0];
        int i1 = f.indices[1];
        int i2 = f.indices[2];

        sprite.fillTriangle((int16_t)(projected[i0].x + 0.5f), (int16_t)(projected[i0].y + 0.5f),
                            (int16_t)(projected[i1].x + 0.5f), (int16_t)(projected[i1].y + 0.5f),
                            (int16_t)(projected[i2].x + 0.5f), (int16_t)(projected[i2].y + 0.5f),
                            shaded);
        if (f.num_vertices == 4) {
            int i3 = f.indices[3];
            sprite.fillTriangle((int16_t)(projected[i0].x + 0.5f), (int16_t)(projected[i0].y + 0.5f),
                                (int16_t)(projected[i2].x + 0.5f), (int16_t)(projected[i2].y + 0.5f),
                                (int16_t)(projected[i3].x + 0.5f), (int16_t)(projected[i3].y + 0.5f),
                                shaded);
        }
    }
}

void drawCarRearGlass(float pos_x, float pos_y, float pos_z,
                      float rot_x, float rot_y, float rot_z,
                      float scale, bool braking) {
    // 4 glass corners, then 2 tail light positions.
    static const Point3D rear_points[6] = {
        { -0.42f, 1.025f, -0.79f },
        {  0.42f, 1.025f, -0.79f },
        {  0.24f, 0.742f, -1.275f },
        { -0.24f, 0.742f, -1.275f },
        { -0.47f, 0.470f, -1.660f },
        {  0.47f, 0.470f, -1.660f }
    };

    Point2D projected[6];
    float depth[6];

    float cx = cosf(rot_x), sx = sinf(rot_x);
    float cy = cosf(rot_y), sy = sinf(rot_y);
    float cz = cosf(rot_z), sz = sinf(rot_z);

    for (int i = 0; i < 6; i++) {
        float lx = rear_points[i].x * scale;
        float ly = rear_points[i].y * scale;
        float lz = rear_points[i].z * scale;
        
        float x1 = lx * cz - ly * sz;
        float y1 = lx * sz + ly * cz;
        float z1 = lz;
        
        float x2 = x1;
        float y2 = y1 * cx - z1 * sx;
        float z2 = y1 * sx + z1 * cx;
        
        float wx = x2 * cy + z2 * sy + pos_x;
        float wy = y2 + pos_y;
        float wz = -x2 * sy + z2 * cy + pos_z;
        
        float cx_cam = wx - cam_x;
        float cy_cam = wy - cam_y;
        float cz_cam = wz - cam_z;
        
        float rx_cam1 = cx_cam * cam_cos_yaw - cz_cam * cam_sin_yaw;
        float rz_cam1 = cx_cam * cam_sin_yaw + cz_cam * cam_cos_yaw;
        float ry_cam1 = cy_cam;
        
        float rx_cam = rx_cam1;
        float ry_cam = ry_cam1 * cam_cos_pitch - rz_cam1 * cam_sin_pitch;
        float rz_cam = ry_cam1 * cam_sin_pitch + rz_cam1 * cam_cos_pitch;
        
        if (rz_cam <= 0.1f) return;
        depth[i] = rz_cam;
        projected[i].x = center_x + (rx_cam * fov / rz_cam);
        projected[i].y = center_y - (ry_cam * fov / rz_cam);
    }

    sprite.fillTriangle((int16_t)projected[0].x, (int16_t)projected[0].y,
                        (int16_t)projected[1].x, (int16_t)projected[1].y,
                        (int16_t)projected[2].x, (int16_t)projected[2].y,
                        0x0000);
    sprite.fillTriangle((int16_t)projected[0].x, (int16_t)projected[0].y,
                        (int16_t)projected[2].x, (int16_t)projected[2].y,
                        (int16_t)projected[3].x, (int16_t)projected[3].y,
                        0x0000);

    sprite.drawLine((int16_t)projected[0].x, (int16_t)projected[0].y,
                    (int16_t)projected[1].x, (int16_t)projected[1].y, 0x39E7);
    sprite.drawLine((int16_t)projected[0].x, (int16_t)projected[0].y,
                    (int16_t)projected[3].x, (int16_t)projected[3].y, 0x2104);
    sprite.drawLine((int16_t)projected[1].x, (int16_t)projected[1].y,
                    (int16_t)projected[2].x, (int16_t)projected[2].y, 0x2104);

    // Tail lights when the player brakes.
    if (braking) {
        for (int i = 4; i <= 5; i++) {
            int r = (int)(0.075f * fov / depth[i]);
            if (r < 1) r = 1;
            if (r > 5) r = 5;
            int x = (int)projected[i].x;
            int y = (int)projected[i].y;
            sprite.fillRect(x - r, y - r / 2 - 1, r * 2 + 1, r + 2, 0xF800);
            sprite.fillRect(x - r / 2, y - 1, r + 1, 2, 0xFFE0);
        }
    }
}

void drawTreeImpostor(float pos_x, float pos_y, float pos_z, float scale, uint8_t fog_a) {
    float sx, sy, sz;
    projectPoint(pos_x, pos_y, pos_z, sx, sy, sz);
    if (sz <= 2.0f || sz > 44.0f) return;

    int h = (int)(460.0f * scale / sz);
    if (h < 4) return;
    if (h > 48) h = 48;

    int x = (int)sx;
    int y = (int)sy;
    int half_w = h / 4;
    if (x < -h || x > SCREEN_WIDTH + h || y < -h || y > SCREEN_HEIGHT + h) return;

    int trunk_h = h / 3;
    int trunk_w = h / 10;
    if (trunk_w < 1) trunk_w = 1;
    int leaf_base_y = y - trunk_h;
    int top_y = y - h;

    uint16_t trunk_color = 0x3920;
    uint16_t leaf_dark = 0x0340;
    uint16_t leaf_mid = 0x04C0;
    if (fog_a) {
        trunk_color = blend565(trunk_color, FOG_COLOR, fog_a);
        leaf_dark = blend565(leaf_dark, FOG_COLOR, fog_a);
        leaf_mid = blend565(leaf_mid, FOG_COLOR, fog_a);
    }

    sprite.fillRect(x - trunk_w / 2, leaf_base_y, trunk_w, trunk_h, trunk_color);
    sprite.fillTriangle(x - half_w, leaf_base_y,
                        x + half_w, leaf_base_y,
                        x, top_y,
                        leaf_dark);
    sprite.fillTriangle(x - half_w * 3 / 4, leaf_base_y - h / 4,
                        x + half_w * 3 / 4, leaf_base_y - h / 4,
                        x, top_y + h / 6,
                        leaf_mid);
}

void drawOpponentBrakeLights(float pos_x, float pos_y, float pos_z, float yaw, bool braking) {
    float cy = cosf(yaw);
    float sy = sinf(yaw);
    
    float cam_dx = cam_x - pos_x;
    float cam_dz = cam_z - pos_z;
    float cam_local_z = cam_dx * sy + cam_dz * cy;
    if (cam_local_z > 0.35f) return;
    
    uint16_t lamp_color = braking ? 0xF800 : 0x7800;
    uint16_t core_color = braking ? 0xFBE0 : 0xA000;
    
    for (int side = -1; side <= 1; side += 2) {
        float lx = 0.34f * side;
        float ly = 0.30f;
        float lz = -1.34f;
        
        float wx = pos_x + lx * cy + lz * sy;
        float wy = pos_y + ly;
        float wz = pos_z - lx * sy + lz * cy;
        
        float sx, sy_screen, sz;
        projectPoint(wx, wy, wz, sx, sy_screen, sz);
        if (sz <= 0.1f || sz > 34.0f) continue;
        
        int r = (int)(16.0f / sz);
        if (r < 1) r = 1;
        if (r > 3) r = 3;
        int x = (int)sx;
        int y = (int)sy_screen;
        if (x < -4 || x > SCREEN_WIDTH + 4 || y < -4 || y > SCREEN_HEIGHT + 4) continue;
        
        sprite.fillRect(x - r, y - r, r * 2 + 1, r * 2 + 1, lamp_color);
        if (braking && r > 1) {
            sprite.fillRect(x, y, 1, 1, core_color);
        }
    }
}

// Billboard drawing
void drawBillboard(float pos_x, float pos_y, float pos_z, float rot_y, float scale, uint16_t color, uint8_t fog_a) {
    // 1. Draw the board face
    draw3DModel(billboard_vertices, 8, billboard_faces, billboard_normals, BILLBOARD_NUM_FACES,
                pos_x, pos_y, pos_z,
                0.0f, rot_y, 0.0f,
                scale, color, fog_a);

    // 2. Draw support posts as lines (simpler, faster)
    float c_y = cosf(rot_y), s_y = sinf(rot_y);
    float lp0_x = -1.5f * scale * c_y + pos_x;
    float lp0_y = pos_y;
    float lp0_z = 1.5f * scale * s_y + pos_z;
    float lp1_x = -1.5f * scale * c_y + pos_x;
    float lp1_y = 1.4f * scale + pos_y;
    float lp1_z = 1.5f * scale * s_y + pos_z;
    
    float rp0_x = 1.5f * scale * c_y + pos_x;
    float rp0_y = pos_y;
    float rp0_z = -1.5f * scale * s_y + pos_z;
    float rp1_x = 1.5f * scale * c_y + pos_x;
    float rp1_y = 1.4f * scale + pos_y;
    float rp1_z = -1.5f * scale * s_y + pos_z;
    
    float sx0, sy0, sz0, sx1, sy1, sz1;
    projectPoint(lp0_x, lp0_y, lp0_z, sx0, sy0, sz0);
    projectPoint(lp1_x, lp1_y, lp1_z, sx1, sy1, sz1);
    if (sz0 > 0.4f && sz1 > 0.4f) {
        sprite.drawLine((int16_t)sx0, (int16_t)sy0, (int16_t)sx1, (int16_t)sy1, 0x4A69);
    }
    projectPoint(rp0_x, rp0_y, rp0_z, sx0, sy0, sz0);
    projectPoint(rp1_x, rp1_y, rp1_z, sx1, sy1, sz1);
    if (sz0 > 0.4f && sz1 > 0.4f) {
        sprite.drawLine((int16_t)sx0, (int16_t)sy0, (int16_t)sx1, (int16_t)sy1, 0x4A69);
    }
    
    // 3. Draw Billboard Text (Removed to keep scene clean)
}

// 3D Point Projection Helper
bool projectPoint(float wx, float wy, float wz, float& sx, float& sy, float& sz) {
    float cx = wx - cam_x;
    float cy = wy - cam_y;
    float cz = wz - cam_z;

    // Yaw
    float rx1 = cx * cam_cos_yaw - cz * cam_sin_yaw;
    float rz1 = cx * cam_sin_yaw + cz * cam_cos_yaw;
    float ry1 = cy;
    
    // Pitch
    float rx = rx1;
    float ry = ry1 * cam_cos_pitch - rz1 * cam_sin_pitch;
    float rz = ry1 * cam_sin_pitch + rz1 * cam_cos_pitch;
    
    sz = rz;
    
    // Near-Plane Z-Clamping Hack
    // If the point is behind the camera, clamp it to the glass.
    // The perspective divide will stretch it way off-screen, perfectly filling the bottom gap!
    if (rz <= 0.1f) {
        rz = 0.1f;
    }
    
    sx = center_x + (rx * fov / rz);
    sy = center_y - (ry * fov / rz);

    // Radial clamp for far-stretched projections. The near-plane stretch
    // trick above can fling coordinates to tens of thousands of pixels,
    // which would overflow the rasterizer's int16 coordinates (wrapping into
    // garbage triangles). Scaling the offset vector UNIFORMLY preserves the
    // point's direction from the screen center exactly -- clamping x and y
    // independently would bend edge slopes (visible as road edges kinking
    // inward at the screen border).
    float ox = sx - center_x;
    float oy = sy - center_y;
    float ax = fabsf(ox);
    float ay = fabsf(oy);
    float m = (ax > ay) ? ax : ay;
    if (m > 2500.0f) {
        float k = 2500.0f / m;
        sx = center_x + ox * k;
        sy = center_y + oy * k;
    }
    return true;
}

// Get Interpolated Track Center Position and Right Vectors
void getTrackPosition(float seg_float, float lateral_pos, float& out_x, float& out_y, float& out_z, float& out_rx, float& out_rz) {
    int seg = (int)seg_float % NUM_SEGMENTS;
    float frac = seg_float - (int)seg_float;
    int next = (seg + 1) % NUM_SEGMENTS;
    
    float cx = track[seg].x * (1.0f - frac) + track[next].x * frac;
    float cy = track[seg].y * (1.0f - frac) + track[next].y * frac;
    float cz = track[seg].z * (1.0f - frac) + track[next].z * frac;
    
    out_rx = track[seg].rx * (1.0f - frac) + track[next].rx * frac;
    out_rz = track[seg].rz * (1.0f - frac) + track[next].rz * frac;
    float len = sqrtf(out_rx*out_rx + out_rz*out_rz);
    out_rx /= len;
    out_rz /= len;
    
    out_x = cx + out_rx * lateral_pos;
    out_y = cy;
    out_z = cz + out_rz * lateral_pos;
}

void cameraRelativeToWorld(float local_x, float local_y, float local_z, float& out_x, float& out_y, float& out_z) {
    // Undo camera pitch, then camera yaw. This lets player-car placement stay screen-stable.
    float ry1 = local_y * cam_cos_pitch + local_z * cam_sin_pitch;
    float rz1 = -local_y * cam_sin_pitch + local_z * cam_cos_pitch;
    float rx1 = local_x;
    
    float wx = rx1 * cam_cos_yaw + rz1 * cam_sin_yaw;
    float wz = -rx1 * cam_sin_yaw + rz1 * cam_cos_yaw;
    
    out_x = cam_x + wx;
    out_y = cam_y + ry1;
    out_z = cam_z + wz;
}

void updateCameraTrig() {
    cam_cos_yaw = cosf(cam_yaw);
    cam_sin_yaw = sinf(cam_yaw);
    cam_cos_pitch = cosf(cam_pitch);
    cam_sin_pitch = sinf(cam_pitch);
}

float stableJitter(int seg, int side, int index, int salt) {
    uint32_t hash = (uint32_t)seg * 1103515245UL;
    hash ^= (uint32_t)(side + 2) * 2654435761UL;
    hash ^= (uint32_t)(index + 1) * 2246822519UL;
    hash ^= (uint32_t)(salt + 1) * 3266489917UL;
    hash ^= hash >> 16;
    hash *= 2246822519UL;
    hash ^= hash >> 13;
    
    return ((int)(hash % 9UL) - 4) * 0.1f;
}

float approachFloat(float current, float target, float response, float dt) {
    float blend = response * dt;
    if (blend > 1.0f) blend = 1.0f;
    if (blend < 0.0f) blend = 0.0f;
    return current + (target - current) * blend;
}

// Per-scanline dithered vertical gradient written straight into the
// framebuffer (sprite pixels are stored byte-swapped, hence the bswap).
// Rows [y_start, y_end) take their blend position from (y - span_start) / span_len.
void fillGradientRows(int y_start, int y_end, int span_start, int span_len,
                      int r0, int g0, int b0, int r1, int g1, int b1) {
    if (y_start < 0) y_start = 0;
    if (y_end > SCREEN_HEIGHT) y_end = SCREEN_HEIGHT;
    if (y_start >= y_end) return;
    if (span_len < 1) span_len = 1;
    uint16_t* buf = (uint16_t*)sprite.getBuffer();
    float inv = 1.0f / (float)span_len;

    for (int y = y_start; y < y_end; y++) {
        float t = (float)(y - span_start) * inv;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        int r = r0 + (int)((r1 - r0) * t);
        int g = g0 + (int)((g1 - g0) * t);
        int b = b0 + (int)((b1 - b0) * t);
        // Two colors half a quantization step apart, alternated per pixel in a
        // checker pattern: cheap ordered dithering that kills banding.
        uint16_t ca = pack565(r, g, b);
        int r2 = r + 4; if (r2 > 255) r2 = 255;
        int g2 = g + 2; if (g2 > 255) g2 = 255;
        int b2 = b + 4; if (b2 > 255) b2 = 255;
        uint16_t cb = pack565(r2, g2, b2);
        uint16_t sa = __builtin_bswap16(ca);
        uint16_t sb = __builtin_bswap16(cb);
        uint32_t pat = (y & 1) ? ((uint32_t)sa << 16) | sb : ((uint32_t)sb << 16) | sa;
        uint32_t* row = (uint32_t*)(buf + y * SCREEN_WIDTH);
        for (int x = 0; x < SCREEN_WIDTH / 2; x++) row[x] = pat;
    }
}

// Render background (Gradient sky, sun, scrolling mountains, green grass)
void drawBackground(int horizon_y, float yaw) {
    int sky_height = horizon_y;
    if (sky_height < 0) sky_height = 0;
    if (sky_height > SCREEN_HEIGHT) sky_height = SCREEN_HEIGHT;

    // Smooth dithered sky gradient: deep blue up high to pale haze at the
    // horizon (which matches FOG_COLOR so the road fades seamlessly into it).
    if (sky_height > 0) {
        fillGradientRows(0, sky_height, 0, (horizon_y > 0 ? horizon_y : 1),
                         10, 64, 200,
                         172, 211, 235);
    }
    
    // Sun
    float cylinder_w = 2.0f * PI * 200.0f;
    float sun_pos = -yaw * 200.0f;
    while (sun_pos <= -160.0f) sun_pos += cylinder_w;
    while (sun_pos > cylinder_w - 160.0f) sun_pos -= cylinder_w;
    int sun_x = (int)sun_pos;
    int sun_y = horizon_y - 56;
    if (sun_y < 18) sun_y = 18;
    
    if (sun_y > -50 && sun_x > -100 && sun_x < 420) {
        float center_factor = 1.0f - (abs(sun_x - 160) / 200.0f);
        if (center_factor < 0.0f) center_factor = 0.0f;
        if (center_factor > 1.0f) center_factor = 1.0f;
        
        int sun_r = 13 + (int)(5 * center_factor);
        sprite.fillCircle(sun_x, sun_y, sun_r, 0xFFE0);
        sprite.fillCircle(sun_x, sun_y, sun_r - 4, 0xFFFF);
    }
    
    // Distant clouds drift in one steady wind direction behind the mountains
    drawClouds(horizon_y, yaw);
    
    // Mountains
    drawMountains(horizon_y, yaw);
    
    // Grass: hazy pale green at the horizon, deepening toward the foreground.
    int grass_y = horizon_y;
    if (grass_y < 0) grass_y = 0;
    if (grass_y < SCREEN_HEIGHT) {
        fillGradientRows(grass_y, SCREEN_HEIGHT, grass_y, SCREEN_HEIGHT - grass_y,
                         150, 188, 165,
                         56, 134, 40);
    }
}

// Slow cloud bank visible only when facing one world direction
void drawClouds(int horizon_y, float yaw) {
    if (horizon_y < 34) return;
    
    struct Cloud {
        float yaw_offset;
        int y_offset;
        int w;
        int h;
    };
    
    static const Cloud clouds[] = {
        { -0.36f, 42, 34, 10 },
        {  0.02f, 55, 46, 12 },
        {  0.33f, 36, 38, 9 }
    };
    
    // Two cloud banks on opposite sides of the world. The second bank
    // mirrors the offsets and tweaks sizes/heights so it reads as different
    // weather rather than a copy of the first.
    static const float bank_yaw[2] = { PI, 0.0f };
    const float visible_half_angle = 0.92f;
    // Triangle-wave wind: clouds drift one way, ease to a stop, and drift
    // back -- no teleport when the old sawtooth wrapped around.
    float phase = fmodf(millis() * 0.000035f, 0.56f);
    float wind = (phase < 0.28f) ? phase : (0.56f - phase);

    for (int bi = 0; bi < 6; bi++) {
        int b = bi / 3;
        int i = bi % 3;
        float yaw_off = b ? (-clouds[i].yaw_offset + 0.15f) : clouds[i].yaw_offset;
        float cloud_yaw = bank_yaw[b] + yaw_off + wind;
        float delta = cloud_yaw - yaw;
        while (delta > PI) delta -= 2.0f * PI;
        while (delta < -PI) delta += 2.0f * PI;
        if (fabsf(delta) > visible_half_angle) continue;

        int cx = 160 + (int)(delta * 185.0f);
        int cy = horizon_y - clouds[i].y_offset + (b ? 7 : 0);
        int cw = clouds[i].w - (b ? 7 : 0);
        int ch = clouds[i].h - (b ? 2 : 0);
        
        if (cy < 8 || cy > horizon_y - 6) continue;
        if (cx + cw < 0 || cx - cw > SCREEN_WIDTH) continue;
        
        int r0 = ch / 2;
        int r1 = ch * 2 / 3;
        int r2 = ch / 2;
        uint16_t shadow = 0xE73C;
        uint16_t white = 0xFFFF;
        
        sprite.fillRect(cx - cw / 2, cy, cw, ch / 2, shadow);
        sprite.fillCircle(cx - cw / 3, cy, r0, shadow);
        sprite.fillCircle(cx, cy - ch / 4, r1, white);
        sprite.fillCircle(cx + cw / 3, cy, r2, shadow);
        sprite.fillRect(cx - cw / 3, cy - ch / 3, cw * 2 / 3, ch / 2, white);
    }
}

// Distant Scrolling Mountains
void drawMountains(int horizon_y, float yaw) {
    const int panorama_w = 960;
    int offset = (int)(-yaw * 153.0f) % panorama_w;
    if (offset < 0) offset += panorama_w;
    
    struct Mountain {
        int rx;
        int w, h;
        uint16_t color;
        uint16_t shadow;
        uint8_t snow_style;
    };
    static const Mountain mountains[] = {
        {  35, 120, 20, 0x8498, 0x63B3, 0 },
        { 145,  95, 32, 0x7497, 0x5B92, 0 },
        { 275, 150, 62, 0x6C58, 0x4B31, 2 },
        { 395,  85, 24, 0x7C99, 0x5BB4, 0 },
        { 500, 145, 28, 0x84FA, 0x63D5, 0 },
        { 620, 115, 52, 0x6C78, 0x4B52, 3 },
        { 700, 105, 46, 0x7478, 0x5332, 1 },
        { 820, 170, 24, 0x8D1B, 0x6C16, 0 },
        { 920, 100, 38, 0x74B8, 0x5B93, 1 }
    };
    
    for (int i = 0; i < 9; i++) {
        for (int copy = -1; copy <= 1; copy++) {
            int mx = offset + mountains[i].rx + copy * panorama_w;
            int my = horizon_y;
            
            int x0 = mx - mountains[i].w / 2;
            int y0 = my;
            int x1 = mx;
            int y1 = my - mountains[i].h;
            int x2 = mx + mountains[i].w / 2;
            int y2 = my;
            
            if (x2 >= 0 && x0 < 320) {
                // Distant peaks sit in the haze layer.
                uint16_t m_color = blend565(mountains[i].color, FOG_COLOR, 9);
                uint16_t m_shadow = blend565(mountains[i].shadow, FOG_COLOR, 9);
                sprite.fillTriangle(x0, y0, x1, y1, x2, y2, m_color);
                if (i & 1) {
                    sprite.fillTriangle(x0, y0, x1, y1, x1, y0, m_shadow);
                } else {
                    sprite.fillTriangle(x1, y1, x2, y2, x1, y0, m_shadow);
                }

                // Snow caps
                if (mountains[i].snow_style != 0) {
                    uint16_t snow = blend565(0xFFFF, FOG_COLOR, 7);
                    int sx0 = x1 - (x1 - x0) * 0.35f;
                    int sy0 = y1 + mountains[i].h * 0.35f;
                    int sx2 = x1 + (x2 - x1) * 0.35f;
                    if (mountains[i].snow_style == 1) {
                        sprite.fillTriangle(sx0, sy0, x1, y1, x1, sy0, snow);
                    } else if (mountains[i].snow_style == 2) {
                        sprite.fillTriangle(x1, sy0, x1, y1, sx2, sy0, snow);
                    } else {
                        sprite.fillTriangle(sx0, sy0, x1, y1, sx2, sy0, snow);
                    }
                }
            }
        }
    }
}

// 2D Quad Drawer helper
void drawQuad(float sx0, float sy0, float sx1, float sy1, float sx2, float sy2, float sx3, float sy3, uint16_t color) {
    sprite.fillTriangle((int16_t)sx0, (int16_t)sy0,
                        (int16_t)sx1, (int16_t)sy1,
                        (int16_t)sx2, (int16_t)sy2, color);
    sprite.fillTriangle((int16_t)sx0, (int16_t)sy0,
                        (int16_t)sx2, (int16_t)sy2,
                        (int16_t)sx3, (int16_t)sy3, color);
}

// Translucent ground shadow: darkens the framebuffer pixels under an
// ellipse instead of painting a solid blob, so road markings stay visible.
void shadowEllipse(int cx, int cy, int rx, int ry, uint8_t darken) {
    if (rx <= 0 || ry <= 0) return;
    uint16_t* buf = (uint16_t*)sprite.getBuffer();
    uint8_t keep = 32 - darken;
    for (int dy = -ry; dy <= ry; dy++) {
        int y = cy + dy;
        if (y < 0 || y >= SCREEN_HEIGHT) continue;
        float fy = (float)dy / (float)ry;
        int span = (int)(rx * sqrtf(1.0f - fy * fy));
        int x0 = cx - span; if (x0 < 0) x0 = 0;
        int x1 = cx + span; if (x1 >= SCREEN_WIDTH) x1 = SCREEN_WIDTH - 1;
        uint16_t* p = buf + y * SCREEN_WIDTH + x0;
        for (int x = x0; x <= x1; x++, p++) {
            uint16_t c = __builtin_bswap16(*p);
            *p = __builtin_bswap16(shade565(c, keep));
        }
    }
}

// Screen-space particles (dirt, sparks)
void spawnParticle(float x, float y, float vx, float vy, int life, uint16_t color) {
    particles[particle_cursor] = { x, y, vx, vy, (int16_t)life, color };
    particle_cursor = (particle_cursor + 1) % MAX_PARTICLES;
}

void spawnImpactSparks(int cx, int cy, float dir) {
    for (int i = 0; i < 8; i++) {
        uint16_t c = (i & 1) ? 0xFFE0 : ((i & 2) ? 0xFD20 : 0xFFFF);
        spawnParticle(cx + random(-6, 7), cy + random(-8, 9),
                      dir * (0.8f + random(0, 21) * 0.1f), -random(0, 25) * 0.1f,
                      8 + random(0, 9), c);
    }
}

void updateAndDrawParticles() {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle& p = particles[i];
        if (p.life <= 0) continue;
        p.life--;
        p.x += p.vx;
        p.y += p.vy;
        p.vy += 0.18f; // gravity
        int size = (p.life > 8) ? 2 : 1;
        sprite.fillRect((int)p.x, (int)p.y, size, size, p.color);
    }
}

// Fit the real track outline into a small box at the top center of the HUD.
void buildMinimap() {
    float minx = 1e9f, maxx = -1e9f, minz = 1e9f, maxz = -1e9f;
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        if (track[i].x < minx) minx = track[i].x;
        if (track[i].x > maxx) maxx = track[i].x;
        if (track[i].z < minz) minz = track[i].z;
        if (track[i].z > maxz) maxz = track[i].z;
    }
    float cx = (minx + maxx) * 0.5f;
    float cz = (minz + maxz) * 0.5f;
    float sx = 68.0f / (maxx - minx);
    float sz = 40.0f / (maxz - minz);
    float s = (sx < sz) ? sx : sz;
    for (int p = 0; p < MINIMAP_POINTS; p++) {
        int seg = p * (NUM_SEGMENTS / MINIMAP_POINTS);
        mm_px[p] = (int16_t)(275.0f + (track[seg].x - cx) * s); // top-right corner
        mm_py[p] = (int16_t)(27.0f - (track[seg].z - cz) * s);
    }
}

void buildFogTable() {
    for (int i = 0; i <= ROAD_DRAW_SEGMENTS + 1; i++) {
        float t = (float)(i - FOG_START_SEGMENT) / (float)(ROAD_DRAW_SEGMENTS - FOG_START_SEGMENT);
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        fog_table[i] = (uint8_t)(t * t * 30.0f);
    }
}

// UI Head-Up Display Overlay
void drawHUD() {
    sprite.setTextDatum(TL_DATUM);
    sprite.setTextSize(1);
    
    // Lap Counter
    sprite.setTextColor(0xFFFF);
    int display_lap = player_laps >= 3 ? 3 : player_laps + 1;
    char lap_str[12];
    snprintf(lap_str, sizeof(lap_str), "LAP: %d / 3", display_lap);
    sprite.drawString(lap_str, 10, 8);
    
    // Race Time
    unsigned long elapsed = 0;
    if (current_state == PLAYING) {
        elapsed = (race_start_ms > 0) ? (millis() - race_start_ms) : 0;
    } else if (current_state == FINISHED) {
        elapsed = race_finish_ms;
    }
    
    int mins = elapsed / 60000;
    int secs = (elapsed % 60000) / 1000;
    int ms = (elapsed % 1000) / 10;
    char time_str[18];
    snprintf(time_str, sizeof(time_str), "TIME: %02d:%02d.%02d", mins, secs, ms);
    sprite.drawString(time_str, 10, 20);
    
    // Position ranking
    float player_dist = player_segment_float + player_laps * NUM_SEGMENTS;

    int rank = 1;
    for (int i = 0; i < NUM_OPPONENTS; i++) {
        float opp_dist = opponents[i].segment + opponents[i].laps * NUM_SEGMENTS;
        if (opp_dist > player_dist) rank++;
    }
    
    // One-line position readout under LAP and TIME
    sprite.setTextSize(1);
    sprite.setTextColor(0xFCE0); // Gold
    const char* pos_suffix = "th";
    if (rank == 1) pos_suffix = "st";
    else if (rank == 2) pos_suffix = "nd";
    else if (rank == 3) pos_suffix = "rd";
    char rank_str[10];
    snprintf(rank_str, sizeof(rank_str), "POS %d%s", rank, pos_suffix);
    sprite.drawString(rank_str, 10, 32);
    
    // Speedometer
    sprite.setTextDatum(BR_DATUM);
    sprite.setTextSize(2);
    sprite.setTextColor(0xFFFF);
    char speed_str[6];
    snprintf(speed_str, sizeof(speed_str), "%d", (int)player_speed);
    sprite.drawString(speed_str, 310, 150);
    sprite.setTextSize(1);
    sprite.setTextColor(0x7BEF);
    sprite.drawString("km/h", 310, 166);
    
    // Tachometer RPM Bar
    int tach_w = 112;
    int tach_x = (SCREEN_WIDTH - tach_w) / 2;
    int tach_y = 156;
    int tach_h = 10;
    sprite.drawRect(tach_x, tach_y, tach_w, tach_h, 0x5AEB);
    
    float speed_ratio = player_speed / MAX_SPEED;
    int fill_w = (int)(speed_ratio * (tach_w - 4));
    if (fill_w < 0) fill_w = 0;
    if (fill_w > tach_w - 4) fill_w = tach_w - 4;
    
    uint16_t rpm_color = 0x07E0; // Green
    if (speed_ratio > 0.85f) rpm_color = 0xF800;      // Redline
    else if (speed_ratio > 0.60f) rpm_color = 0xFBE0; // Orange
    
    if (fill_w > 0) {
        sprite.fillRect(tach_x + 2, tach_y + 2, fill_w, tach_h - 4, rpm_color);
    }
    
    for (int tick = 1; tick <= 5; tick++) {
        int tx = tach_x + (tick * (tach_w / 6));
        sprite.drawFastVLine(tx, tach_y + 2, tach_h - 4, 0x3186);
    }
    
    // True track-shape minimap with live car positions
    for (int p = 0; p < MINIMAP_POINTS; p++) {
        int q = (p + 1) % MINIMAP_POINTS;
        sprite.drawLine(mm_px[p], mm_py[p], mm_px[q], mm_py[q], 0x39E7);
    }
    sprite.fillRect(mm_px[0] - 1, mm_py[0] - 1, 3, 3, 0xFFFF); // start/finish
    int seg_per_point = NUM_SEGMENTS / MINIMAP_POINTS;
    for (int i = 0; i < NUM_OPPONENTS; i++) {
        int p = ((int)opponents[i].segment / seg_per_point) % MINIMAP_POINTS;
        sprite.fillCircle(mm_px[p], mm_py[p], 1, opponents[i].color);
    }
    int pp = ((int)player_segment_float / seg_per_point) % MINIMAP_POINTS;
    sprite.fillCircle(mm_px[pp], mm_py[pp], 2, 0xF800);
    
    // Performance readout, configured in User Config at the top of the file.
    // Render ms = CPU time per frame before the push; " SB" marks the
    // single-buffer fallback (no DMA) -- both useful when diagnosing.
    if (SHOW_FPS || SHOW_FRAME_TIMING) {
        sprite.setTextDatum(BL_DATUM);
        sprite.setTextSize(1);
        sprite.setTextColor(0x07E0);
        char perf_str[24];
        int len = 0;
        if (SHOW_FPS) {
            int fps10 = (int)(measured_fps * 10.0f + 0.5f);
            len += snprintf(perf_str + len, sizeof(perf_str) - len,
                            "FPS:%d.%d", fps10 / 10, fps10 % 10);
        }
        if (SHOW_FRAME_TIMING) {
            int r10 = (int)(perf_render_ms * 10.0f + 0.5f);
            len += snprintf(perf_str + len, sizeof(perf_str) - len,
                            "%s%d.%dms", len ? " " : "", r10 / 10, r10 % 10);
        }
        if (!use_dma) {
            snprintf(perf_str + len, sizeof(perf_str) - len, " SB");
        }
        sprite.drawString(perf_str, 10, 166);
    }
}

// Countdown overlay
void drawCountdown() {
    unsigned long elapsed = millis() - countdown_start_ms;
    sprite.setTextDatum(MC_DATUM);
    sprite.setTextColor(0xFFFF);
    
    if (elapsed < 1000) {
        sprite.setTextSize(5);
        sprite.drawString("3", 160, 85);
    } else if (elapsed < 2000) {
        sprite.setTextSize(5);
        sprite.drawString("2", 160, 85);
    } else if (elapsed < 3000) {
        sprite.setTextSize(5);
        sprite.drawString("1", 160, 85);
    } else if (elapsed < 4000) {
        sprite.setTextSize(5);
        sprite.setTextColor(0x07E0);
        sprite.drawString("GO!", 160, 85);
    }
}

// Race Finished UI overlay
void drawFinished() {
    sprite.fillRect(40, 25, 240, 120, 0x10A2); // translucent dark card
    sprite.drawRect(40, 25, 240, 120, 0x07FF); // cyan border
    
    sprite.setTextDatum(MC_DATUM);
    sprite.setTextSize(3);
    sprite.setTextColor(0xFCE0); // gold
    sprite.drawString("FINISHED!", 160, 46);

    // Final race position. Opponents freeze the moment the player finishes
    // (their AI only runs in PLAYING), so this stays stable on screen.
    float player_dist = player_segment_float + player_laps * NUM_SEGMENTS;
    int rank = 1;
    for (int i = 0; i < NUM_OPPONENTS; i++) {
        if (opponents[i].segment + opponents[i].laps * NUM_SEGMENTS > player_dist) rank++;
    }
    const char* pos_suffix = "th";
    if (rank == 1) pos_suffix = "st";
    else if (rank == 2) pos_suffix = "nd";
    else if (rank == 3) pos_suffix = "rd";
    char pos_str[24];
    snprintf(pos_str, sizeof(pos_str), "RACE POSITION: %d%s", rank, pos_suffix);
    sprite.setTextSize(1);
    sprite.setTextColor(0xFCE0);
    sprite.drawString(pos_str, 160, 64);

    sprite.setTextColor(0xFFFF);
    
    int mins = race_finish_ms / 60000;
    int secs = (race_finish_ms % 60000) / 1000;
    int ms = (race_finish_ms % 1000) / 10;
    char current_str[32];
    snprintf(current_str, sizeof(current_str), "CURRENT: %02d:%02d.%02d", mins, secs, ms);
    sprite.drawString(current_str, 160, 78);
    
    char best_str[32];
    if (best_time_ms != 99999999) {
        mins = best_time_ms / 60000;
        secs = (best_time_ms % 60000) / 1000;
        ms = (best_time_ms % 1000) / 10;
        snprintf(best_str, sizeof(best_str), "BEST TIME: %02d:%02d.%02d", mins, secs, ms);
    } else {
        snprintf(best_str, sizeof(best_str), "BEST TIME: --:--.--");
    }
    sprite.drawString(best_str, 160, 96);
    
    sprite.setTextColor(0x07FF);
    sprite.drawString("PRESS ANY BUTTON TO PLAY AGAIN", 160, 124);
    
    bool left = (digitalRead(0) == LOW);
    bool right = (digitalRead(14) == LOW);
    
    static bool released = false;
    if (!left && !right) released = true;
    
    if (released && (left || right)) {
        released = false;
        
        resetRaceState();
        current_state = COUNTDOWN;
        countdown_start_ms = millis();
    }
}

// 3D Start Screen (dark garage, rotating car demo)
void drawStartScreen() {
    float t = millis() * 0.001f;
    
    center_x = 160.0f;
    center_y = 90.0f;
    cam_x = 0.0f;
    cam_y = 1.28f;
    cam_z = 0.0f;
    cam_yaw = 0.0f;
    cam_pitch = -0.13f;
    updateCameraTrig();
    
    drawMenuGarage(t);
    drawMenuGarageProps(t);
    drawMenuShadow(160, 124, 118, 20);
    
    // Render slowly rotating car body
    float rot_y = t * 0.72f;
    float car_x_val = 0.0f;
    float car_y_val = 0.16f + 0.025f * sinf(t * 2.7f);
    float car_z_val = 4.15f;

    // Animated showroom light, in world space.
    g_light_x = 0.18f + 0.18f * sinf(t * 0.8f);
    g_light_y = 0.98f;
    g_light_z = -0.18f;

    draw3DModel(car_vertices, CAR_NUM_VERTICES, car_faces, car_normals, CAR_NUM_FACES,
                car_x_val, car_y_val, car_z_val,
                0.0f, rot_y, 0.026f * sinf(t * 1.55f),
                1.22f, 0x021F, 0); // Subaru blue
    drawCarRearGlass(car_x_val, car_y_val, car_z_val,
                     0.0f, rot_y, 0.026f * sinf(t * 1.55f),
                     1.22f, false);
                
    // UI titles
    sprite.setTextDatum(MC_DATUM);
    sprite.setTextSize(3);
    sprite.setTextColor(0x0000);
    sprite.drawString("ESPressway32", 162, 24);
    sprite.setTextColor(0x2C7F);
    sprite.drawString("ESPressway32", 160, 22);
    
    char menu_best_str[24];
    if (best_time_ms != 99999999) {
        int b_mins = best_time_ms / 60000;
        int b_secs = (best_time_ms % 60000) / 1000;
        int b_ms = (best_time_ms % 1000) / 10;
        snprintf(menu_best_str, sizeof(menu_best_str), "BEST TIME: %02d:%02d.%02d", b_mins, b_secs, b_ms);
    } else {
        snprintf(menu_best_str, sizeof(menu_best_str), "BEST TIME: --:--.--");
    }
    sprite.setTextSize(1);
    sprite.setTextColor(0x0000);
    sprite.drawString(menu_best_str, 161, 45);
    sprite.setTextColor(0xFCE0);
    sprite.drawString(menu_best_str, 160, 44);
    
    if ((millis() / 500) % 2 == 0) {
        sprite.setTextSize(1);
        sprite.setTextColor(0x0000);
        sprite.drawString("PRESS ANY BUTTON", 161, 146);
        sprite.setTextColor(0xFFFF);
        sprite.drawString("PRESS ANY BUTTON", 160, 145);
    }
    
    sprite.setTextSize(1);
    sprite.setTextColor(0x0000);
    sprite.drawString("STEER LEFT (GPIO 0)  |  STEER RIGHT (GPIO 14)", 161, 159);
    sprite.setTextColor(0x5AEB);
    sprite.drawString("STEER LEFT (GPIO 0)  |  STEER RIGHT (GPIO 14)", 160, 158);
}

void drawMenuGarage(float t) {
    sprite.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, 0x18E3);
    
    // Dark simplified garage shell
    drawMenuQuad3D(-8.4f, 0.0f, 12.4f,  8.4f, 0.0f, 12.4f,  8.4f, 5.2f, 12.4f, -8.4f, 5.2f, 12.4f, 0x4A69);
    drawMenuQuad3D(-8.4f, 0.0f,  0.9f, -8.4f, 0.0f, 12.4f, -8.4f, 5.2f, 12.4f, -8.4f, 5.2f,  0.9f, 0x39E7);
    drawMenuQuad3D( 8.4f, 0.0f, 12.4f,  8.4f, 0.0f,  0.9f,  8.4f, 5.2f,  0.9f,  8.4f, 5.2f, 12.4f, 0x39E7);
    drawMenuQuad3D(-8.4f, 5.2f,  0.9f,  8.4f, 5.2f,  0.9f,  8.4f, 5.2f, 12.4f, -8.4f, 5.2f, 12.4f, 0x2945);
    
    // Subtle wall panel grid
    for (int i = 0; i <= 6; i++) {
        float x = -7.2f + i * 2.4f;
        drawMenuQuad3D(x - 0.025f, 0.45f, 12.32f, x + 0.025f, 0.45f, 12.32f,
                       x + 0.025f, 4.45f, 12.32f, x - 0.025f, 4.45f, 12.32f, 0x2104);
    }
    for (int j = 0; j < 3; j++) {
        float y = 1.05f + j * 1.05f;
        drawMenuQuad3D(-8.0f, y, 12.30f, 8.0f, y, 12.30f,
                       8.0f, y + 0.035f, 12.30f, -8.0f, y + 0.035f, 12.30f, 0x3186);
    }
    
    // Dark concrete floor tiles with simple seams
    for (int z = 11; z >= 0; z--) {
        float z0 = 0.85f + z * 0.95f;
        float z1 = z0 + 0.95f;
        for (int x = -5; x < 5; x++) {
            float x0 = x * 1.68f;
            float x1 = x0 + 1.68f;
            uint16_t tile = ((x + z) & 1) ? 0x3186 : 0x39E7;
            drawMenuQuad3D(x0, 0.0f, z0, x1, 0.0f, z0, x1, 0.0f, z1, x0, 0.0f, z1, tile);
        }
    }
    for (int x = -5; x <= 5; x++) {
        float wx = x * 1.68f;
        float sx0, sy0, sz0, sx1, sy1, sz1;
        projectPoint(wx, 0.012f, 0.95f, sx0, sy0, sz0);
        projectPoint(wx, 0.012f, 12.1f, sx1, sy1, sz1);
        sprite.drawLine((int16_t)sx0, (int16_t)sy0, (int16_t)sx1, (int16_t)sy1, 0x2104);
    }
    for (int z = 1; z <= 12; z++) {
        float wz = 0.85f + z * 0.95f;
        float sx0, sy0, sz0, sx1, sy1, sz1;
        projectPoint(-8.4f, 0.012f, wz, sx0, sy0, sz0);
        projectPoint( 8.4f, 0.012f, wz, sx1, sy1, sz1);
        sprite.drawLine((int16_t)sx0, (int16_t)sy0, (int16_t)sx1, (int16_t)sy1, 0x2104);
    }
    
    // A few overhead panels keep the car readable without turning the menu into a showroom.
    for (int col = -1; col <= 1; col++) {
        float x = col * 3.2f;
        drawMenuQuad3D(x - 1.05f, 5.08f, 3.2f, x + 1.05f, 5.08f, 3.2f,
                       x + 0.82f, 5.08f, 4.40f, x - 0.82f, 5.08f, 4.40f, 0x632C);
        drawMenuQuad3D(x - 0.78f, 5.04f, 3.38f, x + 0.78f, 5.04f, 3.38f,
                       x + 0.58f, 5.04f, 4.15f, x - 0.58f, 5.04f, 4.15f, 0xC638);
    }
    
    // Closed roll-up garage door
    drawMenuQuad3D(-3.95f, 0.55f, 12.14f,  3.95f, 0.55f, 12.14f,  3.95f, 4.25f, 12.14f, -3.95f, 4.25f, 12.14f, 0x2104);
    drawMenuQuad3D(-3.55f, 0.78f, 12.09f,  3.55f, 0.78f, 12.09f,  3.55f, 4.02f, 12.09f, -3.55f, 4.02f, 12.09f, 0x4208);
    for (int s = 0; s < 8; s++) {
        float y = 0.86f + s * 0.38f;
        uint16_t strip = (s & 1) ? 0x39E7 : 0x4A69;
        drawMenuQuad3D(-3.45f, y, 12.05f, 3.45f, y, 12.05f, 3.45f, y + 0.28f, 12.05f, -3.45f, y + 0.28f, 12.05f, strip);
        drawMenuQuad3D(-3.45f, y + 0.29f, 12.03f, 3.45f, y + 0.29f, 12.03f, 3.45f, y + 0.33f, 12.03f, -3.45f, y + 0.33f, 12.03f, 0x2104);
    }
    drawMenuQuad3D(-3.82f, 0.55f, 12.02f, -3.65f, 0.55f, 12.02f, -3.65f, 4.25f, 12.02f, -3.82f, 4.25f, 12.02f, 0x18E3);
    drawMenuQuad3D( 3.65f, 0.55f, 12.02f,  3.82f, 0.55f, 12.02f,  3.82f, 4.25f, 12.02f,  3.65f, 4.25f, 12.02f, 0x18E3);
    for (int w = 0; w < 3; w++) {
        float x0 = -2.55f + w * 1.75f;
        drawMenuQuad3D(x0, 3.42f, 11.98f, x0 + 1.10f, 3.42f, 11.98f, x0 + 1.10f, 3.78f, 11.98f, x0, 3.78f, 11.98f, 0x6B6D);
    }
    drawMenuQuad3D(-0.22f, 1.24f, 11.96f, 0.22f, 1.24f, 11.96f, 0.22f, 1.36f, 11.96f, -0.22f, 1.36f, 11.96f, 0xA514);
    
    // Background shelves, boxes, and garage details
    for (int side = -1; side <= 1; side += 2) {
        float x0 = side < 0 ? -7.55f : 4.55f;
        float x1 = side < 0 ? -4.55f : 7.55f;
        drawMenuQuad3D(x0, 0.62f, 12.08f, x1, 0.62f, 12.08f, x1, 3.58f, 12.08f, x0, 3.58f, 12.08f, 0x2945);
        for (int shelf = 0; shelf < 2; shelf++) {
            float y = 1.18f + shelf * 1.08f;
            drawMenuQuad3D(x0 + 0.12f, y, 12.00f, x1 - 0.12f, y, 12.00f, x1 - 0.12f, y + 0.10f, 12.00f, x0 + 0.12f, y + 0.10f, 12.00f, 0x6B4D);
        }
        drawMenuQuad3D(x0 + 0.38f, 1.28f, 11.96f, x0 + 1.18f, 1.28f, 11.96f, x0 + 1.18f, 1.78f, 11.96f, x0 + 0.38f, 1.78f, 11.96f, 0x8B00);
        drawMenuQuad3D(x0 + 1.38f, 1.28f, 11.96f, x0 + 2.18f, 1.28f, 11.96f, x0 + 2.18f, 1.70f, 11.96f, x0 + 1.38f, 1.70f, 11.96f, 0x4208);
        drawMenuQuad3D(x0 + 0.55f, 2.36f, 11.96f, x0 + 1.42f, 2.36f, 11.96f, x0 + 1.42f, 2.78f, 11.96f, x0 + 0.55f, 2.78f, 11.96f, 0x0012);
    }
    
    // Central turntable and muted reflected light
    drawMenuEllipse(160, 130, 150, 30, 0x2945);
    drawMenuEllipse(160, 128, 130, 24, 0x4A69);
    drawMenuEllipse(160, 126, 108, 18, 0x39E7);
    drawMenuEllipse(160, 124, 76, 10, 0x18E3);
    
    int sweep = (int)(sinf(t * 1.55f) * 42.0f);
    sprite.drawFastHLine(104 + sweep, 116, 50, 0x4A9F);
    sprite.drawFastHLine(174 - sweep, 135, 44, 0x8C51);
    sprite.drawFastHLine(110, 141, 100, 0x632C);
}

void drawMenuGarageProps(float t) {
    // Garage props: tire stacks, work cabinets, and compact side shelves
    for (int side = -1; side <= 1; side += 2) {
        float x = side * 6.45f;
        drawMenuQuad3D(x - side * 0.18f, 0.0f, 3.0f, x + side * 0.18f, 0.0f, 3.0f,
                       x + side * 0.18f, 1.55f, 3.0f, x - side * 0.18f, 1.55f, 3.0f, 0x3186);
        drawMenuQuad3D(x - side * 0.22f, 1.65f, 3.0f, x + side * 0.22f, 1.65f, 3.0f,
                       x + side * 0.22f, 1.86f, 3.0f, x - side * 0.22f, 1.86f, 3.0f, 0x6B4D);
        
        float cabinet_x = side * 7.25f;
        drawMenuQuad3D(cabinet_x - side * 0.45f, 0.0f, 6.6f, cabinet_x + side * 0.45f, 0.0f, 6.6f,
                       cabinet_x + side * 0.45f, 1.25f, 6.6f, cabinet_x - side * 0.45f, 1.25f, 6.6f, side < 0 ? 0x3000 : 0x0010);
        drawMenuQuad3D(cabinet_x - side * 0.40f, 0.30f, 6.55f, cabinet_x + side * 0.40f, 0.30f, 6.55f,
                       cabinet_x + side * 0.40f, 0.38f, 6.55f, cabinet_x - side * 0.40f, 0.38f, 6.55f, 0x8C51);
        drawMenuQuad3D(cabinet_x - side * 0.40f, 0.72f, 6.55f, cabinet_x + side * 0.40f, 0.72f, 6.55f,
                       cabinet_x + side * 0.40f, 0.80f, 6.55f, cabinet_x - side * 0.40f, 0.80f, 6.55f, 0x8C51);
        
        int tire_x = side < 0 ? 50 : 270;
        for (int tire = 0; tire < 3; tire++) {
            drawMenuEllipse(tire_x, 133 - tire * 8, 26, 10, 0x0000);
            drawMenuEllipse(tire_x, 133 - tire * 8, 15, 5, 0x3186);
        }
        int shelf_x = side < 0 ? 38 : 282;
        sprite.drawFastVLine(shelf_x, 90, 40, 0x528A);
        sprite.drawFastVLine(shelf_x + side * 26, 90, 40, 0x528A);
        sprite.drawFastHLine(side < 0 ? shelf_x : shelf_x + side * 26, 101, 26, 0x528A);
        sprite.drawFastHLine(side < 0 ? shelf_x : shelf_x + side * 26, 116, 26, 0x528A);
        sprite.fillRect(side < 0 ? shelf_x + 4 : shelf_x - 22, 93, 10, 7, 0x7A40);
        sprite.fillRect(side < 0 ? shelf_x + 15 : shelf_x - 11, 107, 9, 8, 0x0010);
    }
    
    // Muted reflections around the rotating car.
    sprite.drawFastHLine(118, 109, 84, 0x8C51);
    sprite.drawFastHLine(132, 113, 56, 0x632C);
    sprite.drawFastHLine(141, 117, 38, 0x528A);
}

void drawMenuQuad3D(float x0, float y0, float z0, float x1, float y1, float z1,
                    float x2, float y2, float z2, float x3, float y3, float z3,
                    uint16_t color) {
    float sx0, sy0, sz0, sx1, sy1, sz1, sx2, sy2, sz2, sx3, sy3, sz3;
    projectPoint(x0, y0, z0, sx0, sy0, sz0);
    projectPoint(x1, y1, z1, sx1, sy1, sz1);
    projectPoint(x2, y2, z2, sx2, sy2, sz2);
    projectPoint(x3, y3, z3, sx3, sy3, sz3);
    drawQuad(sx0, sy0, sx1, sy1, sx2, sy2, sx3, sy3, color);
}

void drawMenuEllipse(int cx, int cy, int w, int h, uint16_t color) {
    int half_h = h / 2;
    for (int y = -half_h; y <= half_h; y++) {
        int span = w - (abs(y) * w) / (half_h + 1);
        if (span < 2) continue;
        sprite.drawFastHLine(cx - span / 2, cy + y, span, color);
    }
}

void drawMenuShadow(int cx, int cy, int w, int h) {
    int half_h = h / 2;
    for (int y = -half_h; y <= half_h; y++) {
        int span = w - (abs(y) * w) / (half_h + 1);
        if (span < 2) continue;
        uint16_t color = (abs(y) < half_h / 2) ? 0x1082 : 0x2104;
        sprite.drawFastHLine(cx - span / 2, cy + y, span, color);
    }
}

// ===== GENERATED MODEL DATA (tools/gen_models.py) -- do not hand-edit =====
const Point3D car_vertices[215] = {
    {  0.5200f,  0.3950f, -0.7610f },
    {  0.5200f,  0.2180f, -0.7290f },
    {  0.5200f,  0.0620f, -0.8190f },
    {  0.5200f,  0.0000f, -0.9880f },
    {  0.5200f,  0.0610f, -1.1570f },
    {  0.5200f,  0.2170f, -1.2480f },
    {  0.5200f,  0.3950f, -1.2170f },
    {  0.5200f,  0.5100f, -1.0790f },
    {  0.5200f,  0.5110f, -0.8990f },
    {  0.7060f,  0.3950f, -0.7610f },
    {  0.7060f,  0.2180f, -0.7290f },
    {  0.7060f,  0.0620f, -0.8190f },
    {  0.7060f,  0.0000f, -0.9880f },
    {  0.7060f,  0.0610f, -1.1570f },
    {  0.7060f,  0.2170f, -1.2480f },
    {  0.7060f,  0.3950f, -1.2170f },
    {  0.7060f,  0.5100f, -1.0790f },
    {  0.7060f,  0.5110f, -0.8990f },
    { -0.8140f,  0.6580f,  0.5360f },
    { -0.8010f,  0.7260f,  0.5360f },
    { -0.8010f,  0.7260f,  0.4730f },
    { -0.8140f,  0.6480f,  0.4730f },
    { -0.6130f,  0.6580f,  0.5360f },
    { -0.6090f,  0.6480f,  0.4730f },
    { -0.5940f,  0.7130f,  0.4730f },
    { -0.5960f,  0.7120f,  0.5360f },
    { -0.6180f,  0.7090f, -1.1400f },
    { -0.6260f,  0.6410f, -0.6420f },
    { -0.6350f,  0.6380f,  0.6660f },
    { -0.4340f,  1.0200f,  0.0170f },
    { -0.6040f,  0.3160f,  1.6020f },
    { -0.7010f,  0.4010f,  1.2510f },
    { -0.6890f,  0.3170f,  1.2960f },
    { -0.5950f,  0.0750f,  1.5970f },
    { -0.5440f,  0.3830f,  1.5540f },
    { -0.5310f,  0.4900f,  1.5320f },
    { -0.6720f,  0.1070f, -1.2940f },
    { -0.5830f,  0.1050f, -1.6090f },
    { -0.5940f,  0.4300f, -1.6200f },
    { -0.6720f,  0.0920f,  0.7170f },
    { -0.6770f,  0.0830f, -0.7080f },
    { -0.5470f,  0.6140f, -1.6180f },
    { -0.5910f,  0.7280f, -1.3140f },
    { -0.2000f,  0.7320f, -1.2950f },
    { -0.4710f,  0.7280f, -1.3130f },
    { -0.5060f,  0.7280f, -1.5820f },
    { -0.4710f,  0.7280f, -1.5930f },
    { -0.7030f,  0.4470f,  1.2160f },
    { -0.6770f,  0.6150f, -1.1390f },
    { -0.6950f,  0.5890f, -0.6570f },
    { -0.5540f,  0.5920f,  1.3660f },
    { -0.7030f,  0.5280f,  1.0960f },
    { -0.6690f,  0.0790f,  1.3070f },
    { -0.7070f,  0.3270f, -0.7090f },
    { -0.7010f,  0.3170f,  0.7120f },
    { -0.6840f,  0.5710f,  0.7110f },
    { -0.7120f,  0.4550f, -0.7740f },
    { -0.7030f,  0.4470f,  0.7790f },
    { -0.7030f,  0.5280f,  0.9210f },
    { -0.7060f,  0.4670f, -1.1990f },
    { -0.7120f,  0.5340f, -1.0390f },
    { -0.7150f,  0.5340f, -0.9200f },
    { -0.7060f,  0.3270f, -1.2710f },
    { -0.5170f,  0.8760f, -1.4510f },
    { -0.5060f,  0.8770f, -1.6390f },
    { -0.4690f,  0.8390f, -1.4170f },
    { -0.4700f,  0.8390f, -1.6310f },
    { -0.0010f,  0.3830f,  1.6660f },
    { -0.0010f,  0.4930f,  1.6420f },
    { -0.0010f,  0.1050f, -1.6910f },
    { -0.0010f,  0.4280f, -1.7250f },
    { -0.0010f,  0.6160f, -1.6570f },
    { -0.0010f,  0.7340f, -1.6110f },
    {  0.8140f,  0.6720f,  0.5360f },
    {  0.8140f,  0.6630f,  0.4730f },
    {  0.8010f,  0.7400f,  0.4730f },
    {  0.8010f,  0.7400f,  0.5360f },
    {  0.6120f,  0.6720f,  0.5360f },
    {  0.6200f,  0.6630f,  0.4730f },
    {  0.5900f,  0.7260f,  0.4650f },
    {  0.5890f,  0.7270f,  0.5290f },
    {  0.1990f,  0.7320f, -1.2950f },
    {  0.4520f,  1.0350f, -0.7560f },
    {  0.6210f,  0.7090f, -1.1400f },
    {  0.6240f,  0.6410f, -0.6420f },
    {  0.4400f,  1.0200f,  0.0170f },
    {  0.6340f,  0.6380f,  0.6660f },
    {  0.6030f,  0.3160f,  1.6020f },
    {  0.6870f,  0.3170f,  1.2960f },
    {  0.6990f,  0.4010f,  1.2510f },
    {  0.5930f,  0.0750f,  1.5970f },
    { -0.0010f,  0.3170f,  1.7060f },
    { -0.0010f,  0.0740f,  1.7250f },
    {  0.5420f,  0.3830f,  1.5540f },
    {  0.5300f,  0.4900f,  1.5320f },
    {  0.6700f,  0.1070f, -1.2940f },
    {  0.5830f,  0.1050f, -1.6090f },
    { -0.0010f,  0.4290f, -1.6490f },
    {  0.5890f,  0.4350f, -1.6050f },
    {  0.6700f,  0.0920f,  0.7170f },
    {  0.6750f,  0.0830f, -0.7080f },
    {  0.5460f,  0.6140f, -1.6180f },
    {  0.5710f,  0.7280f, -1.3140f },
    {  0.4690f,  0.7280f, -1.3130f },
    {  0.4690f,  0.7280f, -1.5930f },
    {  0.5040f,  0.7280f, -1.5820f },
    {  0.7010f,  0.4470f,  1.2160f },
    {  0.6930f,  0.5890f, -0.6570f },
    {  0.6750f,  0.6150f, -1.1390f },
    {  0.5520f,  0.5920f,  1.3660f },
    {  0.7010f,  0.5280f,  1.0960f },
    {  0.6680f,  0.0790f,  1.3070f },
    {  0.7050f,  0.3270f, -0.7090f },
    {  0.6990f,  0.3170f,  0.7120f },
    {  0.6820f,  0.5710f,  0.7110f },
    {  0.7100f,  0.4550f, -0.7740f },
    {  0.7010f,  0.5280f,  0.9210f },
    {  0.7010f,  0.4470f,  0.7790f },
    {  0.7040f,  0.4670f, -1.1990f },
    {  0.7130f,  0.5340f, -0.9200f },
    {  0.7100f,  0.5340f, -1.0390f },
    {  0.7040f,  0.3270f, -1.2710f },
    {  0.5040f,  0.8770f, -1.6390f },
    {  0.5160f,  0.8760f, -1.4510f },
    {  0.4680f,  0.8390f, -1.4170f },
    {  0.4680f,  0.8390f, -1.6310f },
    { -0.4570f,  1.0070f,  0.1350f },
    {  0.4550f,  1.0070f,  0.1350f },
    { -0.4400f,  1.0490f, -0.7030f },
    { -0.4540f,  1.0350f, -0.7560f },
    {  0.4460f,  1.0490f, -0.7030f },
    { -0.0010f,  0.6660f,  0.7980f },
    {  0.5370f,  0.7280f, -1.5730f },
    {  0.6000f,  0.6140f, -1.5650f },
    {  0.6370f,  0.4320f, -1.5610f },
    {  0.6300f,  0.1060f, -1.5650f },
    { -0.5070f,  0.7280f, -1.5680f },
    { -0.5990f,  0.6140f, -1.5660f },
    { -0.6370f,  0.4330f, -1.5640f },
    { -0.6300f,  0.1060f, -1.5620f },
    {  0.4320f,  0.0920f,  0.7170f },
    {  0.4320f,  0.0790f,  1.3070f },
    { -0.4650f,  0.0920f,  0.7170f },
    { -0.4660f,  0.0790f,  1.3070f },
    {  0.4510f,  0.1070f, -1.2940f },
    {  0.4530f,  0.0830f, -0.7080f },
    { -0.4760f,  0.1070f, -1.2940f },
    { -0.4770f,  0.0830f, -0.7080f },
    {  0.3130f,  0.3160f,  1.6700f },
    {  0.3130f,  0.0750f,  1.6820f },
    {  0.3130f,  0.3830f,  1.6120f },
    {  0.3130f,  0.4910f,  1.5880f },
    { -0.2850f,  0.0750f,  1.6780f },
    { -0.2850f,  0.3100f,  1.6650f },
    { -0.2850f,  0.3830f,  1.6090f },
    { -0.2850f,  0.4910f,  1.5850f },
    { -0.2850f,  0.1050f, -1.6820f },
    { -0.2850f,  0.4280f, -1.7210f },
    {  0.2950f,  0.1050f, -1.6800f },
    {  0.2950f,  0.4330f, -1.7130f },
    { -0.0010f,  0.5930f,  1.3720f },
    {  0.5090f,  0.3950f,  1.2370f },
    {  0.5090f,  0.2180f,  1.2690f },
    {  0.5090f,  0.0620f,  1.1790f },
    {  0.5090f,  0.0000f,  1.0100f },
    {  0.5090f,  0.0610f,  0.8400f },
    {  0.5090f,  0.2170f,  0.7500f },
    {  0.5090f,  0.3950f,  0.7810f },
    {  0.5090f,  0.5100f,  0.9190f },
    {  0.5090f,  0.5110f,  1.0990f },
    {  0.6950f,  0.3950f,  1.2370f },
    {  0.6950f,  0.2180f,  1.2690f },
    {  0.6950f,  0.0620f,  1.1790f },
    {  0.6950f,  0.0000f,  1.0100f },
    {  0.6950f,  0.0610f,  0.8400f },
    {  0.6950f,  0.2170f,  0.7500f },
    {  0.6950f,  0.3950f,  0.7810f },
    {  0.6950f,  0.5100f,  0.9190f },
    {  0.6950f,  0.5110f,  1.0990f },
    { -0.5090f,  0.3950f,  1.2370f },
    { -0.5090f,  0.2180f,  1.2690f },
    { -0.5090f,  0.0620f,  1.1790f },
    { -0.5090f,  0.0000f,  1.0100f },
    { -0.5090f,  0.0610f,  0.8400f },
    { -0.5090f,  0.2170f,  0.7500f },
    { -0.5090f,  0.3950f,  0.7810f },
    { -0.5090f,  0.5100f,  0.9190f },
    { -0.5090f,  0.5110f,  1.0990f },
    { -0.6950f,  0.3950f,  1.2370f },
    { -0.6950f,  0.2180f,  1.2690f },
    { -0.6950f,  0.0620f,  1.1790f },
    { -0.6950f,  0.0000f,  1.0100f },
    { -0.6950f,  0.0610f,  0.8400f },
    { -0.6950f,  0.2170f,  0.7500f },
    { -0.6950f,  0.3950f,  0.7810f },
    { -0.6950f,  0.5100f,  0.9190f },
    { -0.6950f,  0.5110f,  1.0990f },
    { -0.5180f,  0.3950f, -0.7610f },
    { -0.5180f,  0.2180f, -0.7290f },
    { -0.5180f,  0.0620f, -0.8190f },
    { -0.5180f,  0.0000f, -0.9880f },
    { -0.5180f,  0.0610f, -1.1570f },
    { -0.5180f,  0.2170f, -1.2480f },
    { -0.5180f,  0.3950f, -1.2170f },
    { -0.5180f,  0.5100f, -1.0790f },
    { -0.5180f,  0.5110f, -0.8990f },
    { -0.7040f,  0.3950f, -0.7610f },
    { -0.7040f,  0.2180f, -0.7290f },
    { -0.7040f,  0.0620f, -0.8190f },
    { -0.7040f,  0.0000f, -0.9880f },
    { -0.7040f,  0.0610f, -1.1570f },
    { -0.7040f,  0.2170f, -1.2480f },
    { -0.7040f,  0.3950f, -1.2170f },
    { -0.7040f,  0.5100f, -1.0790f },
    { -0.7040f,  0.5110f, -0.8990f }
};

const Face car_faces[368] = {
    { {  3, 12, 11,  0}, 3, 0, 0x0000 },
    { { 11,  2,  3,  0}, 3, 0, 0x0000 },
    { {  5, 14, 13,  0}, 3, 0, 0x0000 },
    { { 13,  4,  5,  0}, 3, 0, 0x0000 },
    { {  7, 16, 15,  0}, 3, 0, 0x0000 },
    { { 15,  6,  7,  0}, 3, 0, 0x0000 },
    { {  8, 17, 16,  0}, 3, 0, 0x0000 },
    { { 16,  7,  8,  0}, 3, 0, 0x0000 },
    { {  0,  9, 17,  0}, 3, 0, 0x0000 },
    { { 17,  8,  0,  0}, 3, 0, 0x0000 },
    { {  2,  1,  0,  0}, 3, 0, 0x0000 },
    { {  0,  3,  2,  0}, 3, 0, 0x0000 },
    { {  3,  0,  8,  0}, 3, 0, 0x0000 },
    { {  8,  4,  3,  0}, 3, 0, 0x0000 },
    { {  4,  8,  7,  0}, 3, 0, 0x0000 },
    { {  7,  5,  4,  0}, 3, 0, 0x0000 },
    { {  6,  5,  7,  0}, 3, 0, 0x0000 },
    { {  9,  0,  1,  0}, 3, 0, 0x0000 },
    { {  1, 10,  9,  0}, 3, 0, 0x0000 },
    { { 10,  1,  2,  0}, 3, 0, 0x0000 },
    { {  2, 11, 10,  0}, 3, 0, 0x0000 },
    { { 12,  3,  4,  0}, 3, 0, 0x0000 },
    { {  4, 13, 12,  0}, 3, 0, 0x0000 },
    { { 14,  5,  6,  0}, 3, 0, 0x0000 },
    { {  6, 15, 14,  0}, 3, 0, 0x0000 },
    { { 11, 12,  9,  0}, 3, 0, 0x0000 },
    { {  9, 10, 11,  0}, 3, 0, 0x0000 },
    { { 12, 13, 17,  0}, 3, 0, 0x0000 },
    { { 17,  9, 12,  0}, 3, 0, 0x0000 },
    { { 13, 14, 16,  0}, 3, 0, 0x0000 },
    { { 16, 17, 13,  0}, 3, 0, 0x0000 },
    { { 15, 16, 14,  0}, 3, 0, 0x0000 },
    { { 18, 20, 21,  0}, 3, 0, 0x0008 },
    { { 20, 18, 19,  0}, 3, 0, 0x0008 },
    { { 22, 21, 23,  0}, 3, 0, 0x0008 },
    { { 21, 22, 18,  0}, 3, 0, 0x0008 },
    { { 23, 20, 24,  0}, 3, 0, 0x0008 },
    { { 20, 23, 21,  0}, 3, 0, 0x0008 },
    { { 24, 19, 25,  0}, 3, 0, 0x0008 },
    { { 19, 24, 20,  0}, 3, 0, 0x0008 },
    { { 25, 18, 22,  0}, 3, 0, 0x0008 },
    { { 18, 25, 19,  0}, 3, 0, 0x0008 },
    { { 43, 26,129,  0}, 3, 0, 0xFFFF },
    { { 27,128,129,  0}, 3, 0, 0x0008 },
    { {128,126, 29,  0}, 3, 0, 0xFFFF },
    { { 27,126,128,  0}, 3, 0, 0x0008 },
    { { 27, 28,126,  0}, 3, 0, 0x0008 },
    { { 30, 31, 32,  0}, 3, 1, 0xFFFF },
    { { 33,153, 30,  0}, 3, 1, 0xFFFF },
    { {153, 33,152,  0}, 3, 1, 0xFFFF },
    { { 34,153,154,  0}, 3, 1, 0xFFFF },
    { {153, 34, 30,  0}, 3, 1, 0xFFFF },
    { { 35,154,155,  0}, 3, 1, 0xFFFF },
    { {154, 35, 34,  0}, 3, 1, 0xFFFF },
    { {135,158, 96,  0}, 3, 1, 0xFFFF },
    { {135, 69,158,  0}, 3, 1, 0xFFFF },
    { { 95, 69,135,  0}, 3, 1, 0xFFFF },
    { { 95,156, 69,  0}, 3, 1, 0xFFFF },
    { { 95, 37,156,  0}, 3, 1, 0xFFFF },
    { { 95,139, 37,  0}, 3, 1, 0xFFFF },
    { {144,139, 95,  0}, 3, 1, 0xFFFF },
    { {146,139,144,  0}, 3, 1, 0xFFFF },
    { { 36,139,146,  0}, 3, 1, 0xFFFF },
    { {157, 97, 70,  0}, 3, 0, 0xFFFF },
    { { 97,157, 38,  0}, 3, 0, 0xFFFF },
    { { 37,157,156,  0}, 3, 1, 0xFFFF },
    { {157, 37, 38,  0}, 3, 1, 0xFFFF },
    { { 38, 71, 97,  0}, 3, 1, 0xFFFF },
    { { 71, 38, 41,  0}, 3, 1, 0xFFFF },
    { { 42, 43, 44,  0}, 3, 0, 0xFFFF },
    { { 43, 42, 26,  0}, 3, 0, 0xFFFF },
    { { 71, 46, 72,  0}, 3, 0, 0xFFFF },
    { { 41, 46, 71,  0}, 3, 0, 0xFFFF },
    { { 41, 45, 46,  0}, 3, 0, 0xFFFF },
    { { 30, 47, 31,  0}, 3, 1, 0xFFFF },
    { { 47, 30, 34,  0}, 3, 1, 0xFFFF },
    { { 48, 27, 26,  0}, 3, 0, 0xFFFF },
    { { 27, 48, 49,  0}, 3, 0, 0x0008 },
    { { 50, 68,160,  0}, 3, 0, 0xFFFF },
    { { 50,155, 68,  0}, 3, 1, 0xFFFF },
    { { 50, 35,155,  0}, 3, 1, 0xFFFF },
    { { 35, 47, 34,  0}, 3, 1, 0xFFFF },
    { { 47, 35, 51,  0}, 3, 1, 0xFFFF },
    { { 50, 51, 35,  0}, 3, 1, 0xFFFF },
    { { 51, 50, 28,  0}, 3, 0, 0x0008 },
    { { 52, 30, 32,  0}, 3, 1, 0xFFFF },
    { { 30, 52, 33,  0}, 3, 1, 0xFFFF },
    { { 53, 39, 54,  0}, 3, 1, 0xFFFF },
    { { 39, 53, 40,  0}, 3, 1, 0xFFFF },
    { { 28, 49, 55,  0}, 3, 0, 0x0008 },
    { { 49, 28, 27,  0}, 3, 0, 0x0008 },
    { { 41,136, 45,  0}, 3, 0, 0xFFFF },
    { {136, 41,137,  0}, 3, 0, 0xFFFF },
    { {129, 26, 27,  0}, 3, 0, 0x0008 },
    { { 56, 53, 49,  0}, 3, 1, 0xFFFF },
    { { 53, 55, 49,  0}, 3, 1, 0xFFFF },
    { { 55, 53, 54,  0}, 3, 1, 0xFFFF },
    { { 57, 55, 54,  0}, 3, 1, 0xFFFF },
    { { 55, 57, 58,  0}, 3, 1, 0xFFFF },
    { { 28, 58, 51,  0}, 3, 0, 0x0008 },
    { { 58, 28, 55,  0}, 3, 0, 0x0008 },
    { { 49, 61, 56,  0}, 3, 1, 0xFFFF },
    { { 61, 49, 60,  0}, 3, 0, 0xFFFF },
    { { 38,137, 41,  0}, 3, 1, 0xFFFF },
    { {137, 38,138,  0}, 3, 1, 0xFFFF },
    { { 45, 63, 64,  0}, 3, 0, 0xFFFF },
    { { 63,136, 42,  0}, 3, 0, 0xFFFF },
    { { 45,136, 63,  0}, 3, 0, 0xFFFF },
    { {102,124,103,  0}, 3, 0, 0xFFFF },
    { {124,102,123,  0}, 3, 0, 0xFFFF },
    { { 66, 45, 64,  0}, 3, 0, 0xFFFF },
    { { 45, 66, 46,  0}, 3, 0, 0xFFFF },
    { { 46, 65, 44,  0}, 3, 0, 0xFFFF },
    { { 65, 46, 66,  0}, 3, 0, 0xFFFF },
    { { 29,130,128,  0}, 3, 0, 0xFFFF },
    { {130, 29, 85,  0}, 3, 0, 0xFFFF },
    { { 50,131, 28,  0}, 3, 0, 0x0008 },
    { {131, 50,160,  0}, 3, 0, 0xFFFF },
    { { 64,123,122,  0}, 3, 0, 0x0008 },
    { {123, 64, 63,  0}, 3, 0, 0x0008 },
    { { 43, 46, 44,  0}, 3, 0, 0xFFFF },
    { { 43, 72, 46,  0}, 3, 0, 0x0008 },
    { {104, 81,103,  0}, 3, 0, 0xFFFF },
    { { 72, 81,104,  0}, 3, 0, 0x0008 },
    { { 43, 81, 72,  0}, 3, 0, 0x0008 },
    { { 73, 75, 76,  0}, 3, 0, 0x0008 },
    { { 75, 73, 74,  0}, 3, 0, 0x0008 },
    { { 77, 74, 73,  0}, 3, 0, 0x0008 },
    { { 74, 77, 78,  0}, 3, 0, 0x0008 },
    { { 78, 75, 74,  0}, 3, 0, 0x0008 },
    { { 75, 78, 79,  0}, 3, 0, 0x0008 },
    { { 79, 76, 75,  0}, 3, 0, 0x0008 },
    { { 76, 79, 80,  0}, 3, 0, 0x0008 },
    { { 80, 73, 76,  0}, 3, 0, 0x0008 },
    { { 73, 80, 77,  0}, 3, 0, 0x0008 },
    { { 81, 82, 83,  0}, 3, 0, 0xFFFF },
    { { 86, 85,127,  0}, 3, 0, 0x0008 },
    { { 84, 85, 86,  0}, 3, 0, 0x0008 },
    { { 85, 82,130,  0}, 3, 0, 0xFFFF },
    { { 84, 82, 85,  0}, 3, 0, 0x0008 },
    { { 87, 88, 89,  0}, 3, 1, 0xFFFF },
    { { 90,148,149,  0}, 3, 1, 0xFFFF },
    { {148, 90, 87,  0}, 3, 1, 0xFFFF },
    { {148, 93,150,  0}, 3, 1, 0xFFFF },
    { { 93,148, 87,  0}, 3, 1, 0xFFFF },
    { {150, 94,151,  0}, 3, 1, 0xFFFF },
    { { 94,150, 93,  0}, 3, 1, 0xFFFF },
    { { 97,159, 70,  0}, 3, 0, 0xFFFF },
    { {159, 97, 98,  0}, 3, 0, 0xFFFF },
    { { 99,145,100,  0}, 3, 1, 0xFFFF },
    { { 99,147,145,  0}, 3, 1, 0xFFFF },
    { { 99, 40,147,  0}, 3, 1, 0xFFFF },
    { { 40,142, 39,  0}, 3, 1, 0xFFFF },
    { { 40,140,142,  0}, 3, 1, 0xFFFF },
    { { 99,140, 40,  0}, 3, 1, 0xFFFF },
    { { 96,159, 98,  0}, 3, 1, 0xFFFF },
    { {159, 96,158,  0}, 3, 1, 0xFFFF },
    { { 71, 98, 97,  0}, 3, 1, 0xFFFF },
    { { 98, 71,101,  0}, 3, 1, 0xFFFF },
    { {102, 81, 83,  0}, 3, 0, 0xFFFF },
    { { 81,102,103,  0}, 3, 0, 0xFFFF },
    { {101,104,105,  0}, 3, 0, 0xFFFF },
    { {104, 71, 72,  0}, 3, 0, 0xFFFF },
    { {101, 71,104,  0}, 3, 0, 0xFFFF },
    { { 89, 93, 87,  0}, 3, 0, 0xFFFF },
    { { 93, 89,106,  0}, 3, 1, 0xFFFF },
    { { 84,108, 83,  0}, 3, 0, 0xFFFF },
    { {108, 84,107,  0}, 3, 0, 0x0008 },
    { {109,151, 94,  0}, 3, 1, 0xFFFF },
    { {109, 68,151,  0}, 3, 1, 0xFFFF },
    { {109,160, 68,  0}, 3, 0, 0xFFFF },
    { {109,110, 86,  0}, 3, 0, 0x0008 },
    { {110,109, 94,  0}, 3, 1, 0xFFFF },
    { {111, 87, 90,  0}, 3, 1, 0xFFFF },
    { { 87,111, 88,  0}, 3, 1, 0xFFFF },
    { {112, 99,100,  0}, 3, 1, 0xFFFF },
    { { 99,112,113,  0}, 3, 1, 0xFFFF },
    { { 86,107, 84,  0}, 3, 0, 0x0008 },
    { {107, 86,114,  0}, 3, 0, 0x0008 },
    { {108,102, 83,  0}, 3, 0, 0xFFFF },
    { {102,133,132,  0}, 3, 0, 0xFFFF },
    { {108,133,102,  0}, 3, 0, 0xFFFF },
    { { 82, 84, 83,  0}, 3, 0, 0x0008 },
    { {115,107,112,  0}, 3, 1, 0xFFFF },
    { {112,114,113,  0}, 3, 1, 0xFFFF },
    { {114,112,107,  0}, 3, 1, 0xFFFF },
    { {117,114,116,  0}, 3, 1, 0xFFFF },
    { {114,117,113,  0}, 3, 1, 0xFFFF },
    { { 86,116,114,  0}, 3, 0, 0x0008 },
    { {116, 86,110,  0}, 3, 0, 0x0008 },
    { {120,108,107,  0}, 3, 0, 0xFFFF },
    { {108,134,133,  0}, 3, 1, 0xFFFF },
    { {134,108,118,  0}, 3, 1, 0xFFFF },
    { {135,121, 95,  0}, 3, 0, 0xFFFF },
    { {135,118,121,  0}, 3, 1, 0xFFFF },
    { {135,134,118,  0}, 3, 1, 0xFFFF },
    { {132,123,102,  0}, 3, 0, 0xFFFF },
    { {105,123,132,  0}, 3, 0, 0xFFFF },
    { {105,122,123,  0}, 3, 0, 0xFFFF },
    { {104,124,125,  0}, 3, 0, 0xFFFF },
    { {124,104,103,  0}, 3, 0, 0xFFFF },
    { { 43, 82, 81,  0}, 3, 0, 0x0008 },
    { { 82, 43,129,  0}, 3, 0, 0xFFFF },
    { { 52,152, 33,  0}, 3, 1, 0xFFFF },
    { { 52, 92,152,  0}, 3, 1, 0xFFFF },
    { { 52,149, 92,  0}, 3, 1, 0xFFFF },
    { { 52, 90,149,  0}, 3, 1, 0xFFFF },
    { { 90,141,111,  0}, 3, 1, 0xFFFF },
    { { 90,143,141,  0}, 3, 1, 0xFFFF },
    { { 52,143, 90,  0}, 3, 1, 0xFFFF },
    { {126,131,127,  0}, 3, 0, 0x0008 },
    { { 66,124, 65,  0}, 3, 0, 0x0008 },
    { {124, 66,125,  0}, 3, 0, 0x0008 },
    { { 85,126,127,  0}, 3, 0, 0xFFFF },
    { {126, 85, 29,  0}, 3, 0, 0xFFFF },
    { { 82,128,130,  0}, 3, 0, 0xFFFF },
    { {128, 82,129,  0}, 3, 0, 0xFFFF },
    { {127,131, 86,  0}, 3, 0, 0x0008 },
    { {131,126, 28,  0}, 3, 0, 0x0008 },
    { {138, 37,139,  0}, 3, 1, 0xFFFF },
    { { 37,138, 38,  0}, 3, 1, 0xFFFF },
    { { 94,106,110,  0}, 3, 1, 0xFFFF },
    { {106, 94, 93,  0}, 3, 1, 0xFFFF },
    { {119,107,115,  0}, 3, 1, 0xFFFF },
    { {107,119,120,  0}, 3, 0, 0xFFFF },
    { { 60, 48, 59,  0}, 3, 0, 0xFFFF },
    { {125,105,104,  0}, 3, 0, 0xFFFF },
    { {105,125,122,  0}, 3, 0, 0xFFFF },
    { { 66,122,125,  0}, 3, 0, 0x0008 },
    { {122, 66, 64,  0}, 3, 0, 0x0008 },
    { { 42, 65, 63,  0}, 3, 0, 0xFFFF },
    { { 65, 42, 44,  0}, 3, 0, 0xFFFF },
    { { 63,124,123,  0}, 3, 0, 0x0008 },
    { {124, 63, 65,  0}, 3, 0, 0x0008 },
    { {132,101,105,  0}, 3, 0, 0xFFFF },
    { {101,132,133,  0}, 3, 0, 0xFFFF },
    { {133, 98,101,  0}, 3, 0, 0xFFFF },
    { { 98,133,134,  0}, 3, 1, 0xFFFF },
    { {134, 96, 98,  0}, 3, 1, 0xFFFF },
    { { 96,134,135,  0}, 3, 1, 0xFFFF },
    { { 42, 48, 26,  0}, 3, 0, 0xFFFF },
    { { 42,137, 48,  0}, 3, 0, 0xFFFF },
    { {136,137, 42,  0}, 3, 0, 0xFFFF },
    { { 48,138, 59,  0}, 3, 1, 0xFFFF },
    { {138, 48,137,  0}, 3, 1, 0xFFFF },
    { {138, 62, 59,  0}, 3, 1, 0xFFFF },
    { {138, 36, 62,  0}, 3, 1, 0xFFFF },
    { {138,139, 36,  0}, 3, 1, 0xFFFF },
    { {108,120,118,  0}, 3, 0, 0xFFFF },
    { { 48, 60, 49,  0}, 3, 0, 0xFFFF },
    { {142,141,143,  0}, 3, 1, 0xFFFF },
    { {141,142,140,  0}, 3, 1, 0xFFFF },
    { {144,147,146,  0}, 3, 1, 0xFFFF },
    { {147,144,145,  0}, 3, 1, 0xFFFF },
    { { 92,148, 91,  0}, 3, 1, 0xFFFF },
    { {148, 92,149,  0}, 3, 1, 0xFFFF },
    { {148, 67, 91,  0}, 3, 1, 0xFFFF },
    { { 67,148,150,  0}, 3, 1, 0xFFFF },
    { {150, 68, 67,  0}, 3, 1, 0xFFFF },
    { { 68,150,151,  0}, 3, 1, 0xFFFF },
    { { 91,152, 92,  0}, 3, 1, 0xFFFF },
    { {152, 91,153,  0}, 3, 1, 0xFFFF },
    { { 67,153, 91,  0}, 3, 1, 0xFFFF },
    { {153, 67,154,  0}, 3, 1, 0xFFFF },
    { { 68,154, 67,  0}, 3, 1, 0xFFFF },
    { {154, 68,155,  0}, 3, 1, 0xFFFF },
    { { 69,157, 70,  0}, 3, 1, 0xFFFF },
    { {157, 69,156,  0}, 3, 1, 0xFFFF },
    { {159, 69, 70,  0}, 3, 1, 0xFFFF },
    { { 69,159,158,  0}, 3, 1, 0xFFFF },
    { {131,109, 86,  0}, 3, 0, 0x0008 },
    { {109,131,160,  0}, 3, 0, 0xFFFF },
    { {164,173,172,  0}, 3, 0, 0x0000 },
    { {172,163,164,  0}, 3, 0, 0x0000 },
    { {166,175,174,  0}, 3, 0, 0x0000 },
    { {174,165,166,  0}, 3, 0, 0x0000 },
    { {168,177,176,  0}, 3, 0, 0x0000 },
    { {176,167,168,  0}, 3, 0, 0x0000 },
    { {169,178,177,  0}, 3, 0, 0x0000 },
    { {177,168,169,  0}, 3, 0, 0x0000 },
    { {161,170,178,  0}, 3, 0, 0x0000 },
    { {178,169,161,  0}, 3, 0, 0x0000 },
    { {163,162,161,  0}, 3, 0, 0x0000 },
    { {161,164,163,  0}, 3, 0, 0x0000 },
    { {164,161,169,  0}, 3, 0, 0x0000 },
    { {169,165,164,  0}, 3, 0, 0x0000 },
    { {165,169,168,  0}, 3, 0, 0x0000 },
    { {168,166,165,  0}, 3, 0, 0x0000 },
    { {167,166,168,  0}, 3, 0, 0x0000 },
    { {170,161,162,  0}, 3, 0, 0x0000 },
    { {162,171,170,  0}, 3, 0, 0x0000 },
    { {171,162,163,  0}, 3, 0, 0x0000 },
    { {163,172,171,  0}, 3, 0, 0x0000 },
    { {173,164,165,  0}, 3, 0, 0x0000 },
    { {165,174,173,  0}, 3, 0, 0x0000 },
    { {175,166,167,  0}, 3, 0, 0x0000 },
    { {167,176,175,  0}, 3, 0, 0x0000 },
    { {172,173,170,  0}, 3, 0, 0x0000 },
    { {170,171,172,  0}, 3, 0, 0x0000 },
    { {173,174,178,  0}, 3, 0, 0x0000 },
    { {178,170,173,  0}, 3, 0, 0x0000 },
    { {174,175,177,  0}, 3, 0, 0x0000 },
    { {177,178,174,  0}, 3, 0, 0x0000 },
    { {176,177,175,  0}, 3, 0, 0x0000 },
    { {182,190,191,  0}, 3, 0, 0x0000 },
    { {190,182,181,  0}, 3, 0, 0x0000 },
    { {184,192,193,  0}, 3, 0, 0x0000 },
    { {192,184,183,  0}, 3, 0, 0x0000 },
    { {186,194,195,  0}, 3, 0, 0x0000 },
    { {194,186,185,  0}, 3, 0, 0x0000 },
    { {187,195,196,  0}, 3, 0, 0x0000 },
    { {195,187,186,  0}, 3, 0, 0x0000 },
    { {179,196,188,  0}, 3, 0, 0x0000 },
    { {196,179,187,  0}, 3, 0, 0x0000 },
    { {181,179,180,  0}, 3, 0, 0x0000 },
    { {179,181,182,  0}, 3, 0, 0x0000 },
    { {182,187,179,  0}, 3, 0, 0x0000 },
    { {187,182,183,  0}, 3, 0, 0x0000 },
    { {183,186,187,  0}, 3, 0, 0x0000 },
    { {186,183,184,  0}, 3, 0, 0x0000 },
    { {185,186,184,  0}, 3, 0, 0x0000 },
    { {188,180,179,  0}, 3, 0, 0x0000 },
    { {180,188,189,  0}, 3, 0, 0x0000 },
    { {189,181,180,  0}, 3, 0, 0x0000 },
    { {181,189,190,  0}, 3, 0, 0x0000 },
    { {191,183,182,  0}, 3, 0, 0x0000 },
    { {183,191,192,  0}, 3, 0, 0x0000 },
    { {193,185,184,  0}, 3, 0, 0x0000 },
    { {185,193,194,  0}, 3, 0, 0x0000 },
    { {190,188,191,  0}, 3, 0, 0x0000 },
    { {188,190,189,  0}, 3, 0, 0x0000 },
    { {191,196,192,  0}, 3, 0, 0x0000 },
    { {196,191,188,  0}, 3, 0, 0x0000 },
    { {192,195,193,  0}, 3, 0, 0x0000 },
    { {195,192,196,  0}, 3, 0, 0x0000 },
    { {194,193,195,  0}, 3, 0, 0x0000 },
    { {200,208,209,  0}, 3, 0, 0x0000 },
    { {208,200,199,  0}, 3, 0, 0x0000 },
    { {202,210,211,  0}, 3, 0, 0x0000 },
    { {210,202,201,  0}, 3, 0, 0x0000 },
    { {204,212,213,  0}, 3, 0, 0x0000 },
    { {212,204,203,  0}, 3, 0, 0x0000 },
    { {205,213,214,  0}, 3, 0, 0x0000 },
    { {213,205,204,  0}, 3, 0, 0x0000 },
    { {197,214,206,  0}, 3, 0, 0x0000 },
    { {214,197,205,  0}, 3, 0, 0x0000 },
    { {199,197,198,  0}, 3, 0, 0x0000 },
    { {197,199,200,  0}, 3, 0, 0x0000 },
    { {200,205,197,  0}, 3, 0, 0x0000 },
    { {205,200,201,  0}, 3, 0, 0x0000 },
    { {201,204,205,  0}, 3, 0, 0x0000 },
    { {204,201,202,  0}, 3, 0, 0x0000 },
    { {203,204,202,  0}, 3, 0, 0x0000 },
    { {206,198,197,  0}, 3, 0, 0x0000 },
    { {198,206,207,  0}, 3, 0, 0x0000 },
    { {207,199,198,  0}, 3, 0, 0x0000 },
    { {199,207,208,  0}, 3, 0, 0x0000 },
    { {209,201,200,  0}, 3, 0, 0x0000 },
    { {201,209,210,  0}, 3, 0, 0x0000 },
    { {211,203,202,  0}, 3, 0, 0x0000 },
    { {203,211,212,  0}, 3, 0, 0x0000 },
    { {208,206,209,  0}, 3, 0, 0x0000 },
    { {206,208,207,  0}, 3, 0, 0x0000 },
    { {209,214,210,  0}, 3, 0, 0x0000 },
    { {214,209,206,  0}, 3, 0, 0x0000 },
    { {210,213,211,  0}, 3, 0, 0x0000 },
    { {213,210,214,  0}, 3, 0, 0x0000 },
    { {212,211,213,  0}, 3, 0, 0x0000 }
};

const Point3D car_normals[368] = {
    {  0.0000f, -0.9388f,  0.3444f },
    {  0.0000f, -0.9388f,  0.3444f },
    {  0.0000f, -0.5039f, -0.8638f },
    { -0.0000f, -0.5039f, -0.8638f },
    {  0.0000f,  0.7682f, -0.6402f },
    {  0.0000f,  0.7682f, -0.6402f },
    {  0.0000f,  1.0000f, -0.0056f },
    {  0.0000f,  1.0000f, -0.0056f },
    { -0.0000f,  0.7655f,  0.6435f },
    {  0.0000f,  0.7655f,  0.6435f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f, -0.0000f,  0.0000f },
    {  0.0000f,  0.1779f,  0.9840f },
    { -0.0000f,  0.1779f,  0.9840f },
    {  0.0000f, -0.4997f,  0.8662f },
    {  0.0000f, -0.4997f,  0.8662f },
    { -0.0000f, -0.9406f, -0.3395f },
    {  0.0000f, -0.9406f, -0.3395f },
    {  0.0000f,  0.1716f, -0.9852f },
    {  0.0000f,  0.1716f, -0.9852f },
    {  1.0000f, -0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f, -0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f, -0.0000f,  0.0000f },
    {  1.0000f,  0.0000f, -0.0000f },
    {  1.0000f,  0.0000f, -0.0000f },
    { -0.9861f,  0.1643f, -0.0261f },
    { -0.9822f,  0.1878f,  0.0000f },
    {  0.0000f, -0.9876f,  0.1568f },
    {  0.0000f, -0.9876f,  0.1568f },
    {  0.0000f,  0.0000f, -1.0000f },
    {  0.0000f, -0.0000f, -1.0000f },
    {  0.0681f,  0.9975f,  0.0180f },
    {  0.0627f,  0.9980f,  0.0000f },
    {  0.0000f,  0.0000f,  1.0000f },
    {  0.0000f,  0.0000f,  1.0000f },
    { -0.2516f,  0.7882f, -0.5617f },
    { -0.8958f,  0.4269f,  0.1239f },
    { -0.3249f,  0.9449f,  0.0408f },
    { -0.9098f,  0.4151f,  0.0023f },
    { -0.9036f,  0.4283f, -0.0052f },
    { -0.9635f,  0.0057f,  0.2677f },
    { -0.1942f, -0.0276f,  0.9806f },
    { -0.2524f,  0.0534f,  0.9661f },
    { -0.1661f,  0.6002f,  0.7824f },
    { -0.1347f,  0.6540f,  0.7444f },
    { -0.2066f,  0.2122f,  0.9551f },
    { -0.2026f,  0.2208f,  0.9541f },
    { -0.0076f, -0.9995f,  0.0308f },
    { -0.0004f, -1.0000f,  0.0098f },
    {  0.0009f, -1.0000f,  0.0036f },
    {  0.0002f, -1.0000f,  0.0048f },
    {  0.0008f, -1.0000f,  0.0032f },
    { -0.0030f, -0.9998f,  0.0183f },
    {  0.0000f, -1.0000f,  0.0037f },
    {  0.0000f, -1.0000f,  0.0037f },
    {  0.0000f, -1.0000f,  0.0037f },
    { -0.0002f,  0.9999f, -0.0132f },
    {  0.0008f,  0.9999f, -0.0172f },
    { -0.2363f, -0.1165f, -0.9647f },
    { -0.3107f, -0.0427f, -0.9496f },
    { -0.0489f, -0.0427f, -0.9979f },
    { -0.0713f,  0.0291f, -0.9970f },
    {  0.0021f,  0.9692f, -0.2463f },
    { -0.0153f,  0.9942f,  0.1062f },
    { -0.0403f,  0.3629f, -0.9310f },
    { -0.0698f,  0.2578f, -0.9637f },
    { -0.2775f,  0.3786f, -0.8830f },
    { -0.9333f,  0.1908f,  0.3041f },
    { -0.5284f,  0.7535f,  0.3912f },
    { -0.8452f,  0.5311f,  0.0589f },
    { -0.6045f,  0.7963f,  0.0204f },
    { -0.0055f,  0.9377f,  0.3473f },
    { -0.1115f,  0.8433f,  0.5258f },
    { -0.1180f,  0.8387f,  0.5317f },
    { -0.8720f,  0.1980f,  0.4477f },
    { -0.7772f,  0.5216f,  0.3521f },
    { -0.8373f,  0.4064f,  0.3657f },
    { -0.5662f,  0.8156f,  0.1191f },
    { -0.9613f, -0.0685f,  0.2668f },
    { -0.9683f, -0.0413f,  0.2465f },
    { -0.9918f, -0.1278f,  0.0033f },
    { -0.9925f, -0.1220f,  0.0043f },
    { -0.8025f,  0.5965f,  0.0143f },
    { -0.6015f,  0.7988f, -0.0023f },
    { -0.9320f,  0.3562f, -0.0666f },
    { -0.6174f,  0.4874f, -0.6174f },
    { -0.9113f,  0.4098f,  0.0413f },
    { -0.9926f,  0.0218f,  0.1193f },
    { -0.9990f,  0.0440f,  0.0086f },
    { -0.9978f,  0.0668f,  0.0047f },
    { -0.9856f,  0.0653f, -0.1562f },
    { -0.9911f,  0.1157f, -0.0660f },
    { -0.8506f,  0.5258f,  0.0000f },
    { -0.7883f,  0.6129f,  0.0542f },
    { -0.9955f,  0.0736f,  0.0603f },
    { -0.8998f,  0.4357f, -0.0227f },
    { -0.6949f,  0.1851f, -0.6949f },
    { -0.7792f,  0.1569f, -0.6068f },
    { -0.9980f, -0.0224f, -0.0585f },
    { -0.9337f,  0.1810f, -0.3088f },
    { -0.9974f, -0.0111f, -0.0712f },
    {  0.0072f,  0.6837f,  0.7297f },
    { -0.0030f,  0.6787f,  0.7344f },
    { -0.1672f, -0.3523f, -0.9208f },
    { -0.2852f, -0.3081f, -0.9076f },
    {  0.9998f, -0.0180f,  0.0000f },
    {  0.9999f, -0.0106f, -0.0047f },
    {  0.0000f,  0.9992f,  0.0402f },
    {  0.0000f,  0.9992f,  0.0402f },
    { -0.0590f,  0.9956f,  0.0723f },
    { -0.0032f,  0.9920f,  0.1262f },
    { -0.0000f,  1.0000f,  0.0053f },
    {  0.0000f,  1.0000f,  0.0053f },
    { -0.0148f,  0.9999f,  0.0000f },
    { -0.0128f,  0.9999f, -0.0018f },
    {  0.0148f,  0.9999f, -0.0000f },
    {  0.0128f,  0.9999f, -0.0018f },
    { -0.0000f,  1.0000f,  0.0063f },
    {  0.9822f,  0.1878f,  0.0000f },
    {  0.9858f,  0.1664f, -0.0238f },
    {  0.0000f, -0.9899f,  0.1414f },
    {  0.0000f, -0.9899f,  0.1414f },
    {  0.0000f,  0.0000f, -1.0000f },
    {  0.0446f, -0.1049f, -0.9935f },
    { -0.0662f,  0.9978f,  0.0000f },
    { -0.0607f,  0.9980f, -0.0165f },
    { -0.0326f, -0.0062f,  0.9994f },
    {  0.0000f,  0.1263f,  0.9920f },
    {  0.2490f,  0.7896f, -0.5608f },
    {  0.9380f,  0.3368f, -0.0821f },
    {  0.8956f,  0.4449f, -0.0058f },
    {  0.9453f,  0.3256f,  0.0210f },
    {  0.9139f,  0.4053f,  0.0221f },
    {  0.9643f,  0.0041f,  0.2647f },
    {  0.2902f,  0.0476f,  0.9558f },
    {  0.2282f, -0.0297f,  0.9732f },
    {  0.1881f,  0.6428f,  0.7426f },
    {  0.1692f,  0.6712f,  0.7217f },
    {  0.2452f,  0.2103f,  0.9464f },
    {  0.2394f,  0.2212f,  0.9454f },
    { -0.0164f,  0.9998f, -0.0132f },
    { -0.0110f,  0.9999f,  0.0115f },
    {  0.0000f, -1.0000f,  0.0063f },
    {  0.0000f, -1.0000f,  0.0063f },
    {  0.0000f, -1.0000f,  0.0063f },
    {  0.0000f, -1.0000f,  0.0063f },
    {  0.0000f, -1.0000f,  0.0063f },
    {  0.0000f, -1.0000f,  0.0063f },
    {  0.3448f,  0.0051f, -0.9387f },
    {  0.2382f, -0.0972f, -0.9663f },
    {  0.0747f, -0.0426f, -0.9963f },
    {  0.0708f, -0.0553f, -0.9960f },
    {  0.0160f,  0.9944f,  0.1040f },
    { -0.0025f,  0.9677f, -0.2521f },
    {  0.2772f,  0.3807f, -0.8821f },
    {  0.0403f,  0.3629f, -0.9310f },
    {  0.0696f,  0.2583f, -0.9635f },
    {  0.5635f,  0.7543f,  0.3368f },
    {  0.8373f,  0.3076f,  0.4521f },
    {  0.8651f,  0.4976f,  0.0627f },
    {  0.6045f,  0.7963f,  0.0204f },
    {  0.1411f,  0.8350f,  0.5318f },
    {  0.0900f,  0.8665f,  0.4911f },
    {  0.0055f,  0.9377f,  0.3473f },
    {  0.5674f,  0.8147f,  0.1200f },
    {  0.8357f,  0.4115f,  0.3636f },
    {  0.9673f, -0.0453f,  0.2495f },
    {  0.9624f, -0.0646f,  0.2640f },
    {  0.9925f, -0.1220f,  0.0043f },
    {  0.9918f, -0.1278f,  0.0033f },
    {  0.6015f,  0.7989f, -0.0028f },
    {  0.8083f,  0.5886f,  0.0142f },
    {  0.8519f,  0.4874f, -0.1916f },
    {  0.8729f,  0.4743f, -0.1146f },
    {  0.8310f,  0.5363f, -0.1476f },
    {  0.9098f,  0.4119f,  0.0508f },
    {  0.9926f,  0.0218f,  0.1193f },
    {  0.9978f,  0.0668f,  0.0047f },
    {  0.9990f,  0.0440f,  0.0086f },
    {  0.9911f,  0.1157f, -0.0660f },
    {  0.9856f,  0.0653f, -0.1562f },
    {  0.7950f,  0.6044f,  0.0518f },
    {  0.8540f,  0.5202f, -0.0000f },
    {  0.9240f,  0.3821f, -0.0139f },
    {  0.9663f,  0.1927f, -0.1706f },
    {  0.9430f,  0.2659f, -0.2002f },
    {  0.9801f, -0.1364f, -0.1442f },
    {  0.9080f,  0.1916f, -0.3726f },
    {  0.9834f, -0.0189f, -0.1802f },
    {  0.9623f,  0.2407f, -0.1263f },
    {  0.2022f,  0.6399f, -0.7414f },
    {  0.9977f, -0.0244f, -0.0638f },
    { -1.0000f, -0.0090f,  0.0000f },
    { -1.0000f, -0.0090f,  0.0000f },
    {  0.0000f,  0.8717f, -0.4900f },
    {  0.0000f,  0.8717f, -0.4900f },
    {  0.0039f, -0.9999f, -0.0148f },
    { -0.0021f, -1.0000f, -0.0086f },
    {  0.0013f, -0.9999f, -0.0140f },
    { -0.0018f, -1.0000f, -0.0059f },
    {  0.0000f, -0.9999f, -0.0138f },
    {  0.0000f, -0.9999f, -0.0138f },
    {  0.0000f, -0.9999f, -0.0138f },
    { -0.0000f,  0.8893f,  0.4574f },
    {  0.0000f, -1.0000f,  0.0000f },
    {  0.0000f, -1.0000f, -0.0000f },
    {  0.0000f,  0.9940f,  0.1095f },
    {  0.0000f,  0.9940f,  0.1095f },
    {  0.0000f,  0.9668f, -0.2554f },
    {  0.0000f,  0.9668f, -0.2554f },
    {  0.1471f,  0.8348f,  0.5305f },
    { -0.1473f,  0.8347f,  0.5306f },
    { -0.7072f, -0.0195f, -0.7068f },
    { -0.7935f, -0.0474f, -0.6067f },
    {  0.7790f,  0.5197f,  0.3508f },
    {  0.8741f,  0.1900f,  0.4471f },
    {  0.9955f,  0.0736f,  0.0603f },
    {  0.8998f,  0.4357f, -0.0227f },
    { -0.9598f,  0.2441f, -0.1382f },
    {  0.2852f, -0.3081f, -0.9076f },
    {  0.1672f, -0.3523f, -0.9208f },
    {  0.0000f, -0.2060f, -0.9785f },
    { -0.0000f, -0.2060f, -0.9785f },
    {  0.0028f,  0.6786f,  0.7345f },
    { -0.0061f,  0.6838f,  0.7297f },
    { -0.0000f,  0.6766f,  0.7363f },
    {  0.0000f,  0.6766f,  0.7363f },
    {  0.2442f,  0.3727f, -0.8953f },
    {  0.6638f,  0.3194f, -0.6763f },
    {  0.6958f,  0.1157f, -0.7089f },
    {  0.6748f,  0.1212f, -0.7279f },
    {  0.6756f, -0.0033f, -0.7373f },
    {  0.6835f, -0.0057f, -0.7299f },
    { -0.8451f,  0.5296f, -0.0733f },
    { -0.8906f,  0.4243f, -0.1637f },
    { -0.7556f,  0.6054f, -0.2499f },
    { -0.9421f,  0.2669f, -0.2030f },
    { -0.9637f,  0.2004f, -0.1765f },
    { -0.9760f,  0.0997f, -0.1938f },
    { -0.9558f, -0.1197f, -0.2684f },
    { -0.9877f, -0.0221f, -0.1547f },
    {  0.9598f,  0.2441f, -0.1382f },
    { -0.9240f,  0.3821f, -0.0139f },
    {  0.0000f, -0.9998f, -0.0220f },
    {  0.0000f, -0.9998f, -0.0220f },
    { -0.0000f, -0.9992f, -0.0409f },
    {  0.0000f, -0.9992f, -0.0409f },
    {  0.1138f,  0.0774f,  0.9905f },
    {  0.1354f,  0.0493f,  0.9896f },
    {  0.0992f,  0.5157f,  0.8510f },
    {  0.1289f,  0.6490f,  0.7498f },
    {  0.1657f,  0.2102f,  0.9635f },
    {  0.1669f,  0.2139f,  0.9625f },
    { -0.1625f,  0.0769f,  0.9837f },
    { -0.1440f,  0.0547f,  0.9881f },
    { -0.1350f,  0.5136f,  0.8474f },
    { -0.1573f,  0.6011f,  0.7836f },
    { -0.1924f,  0.2092f,  0.9588f },
    { -0.1937f,  0.2128f,  0.9577f },
    { -0.0140f, -0.1047f, -0.9944f },
    { -0.0314f, -0.1198f, -0.9923f },
    {  0.0420f, -0.1046f, -0.9936f },
    {  0.0370f, -0.1000f, -0.9943f },
    {  0.0589f,  0.9956f,  0.0723f },
    {  0.0032f,  0.9920f,  0.1262f },
    {  0.0000f, -0.9388f,  0.3444f },
    {  0.0000f, -0.9388f,  0.3444f },
    {  0.0000f, -0.4997f, -0.8662f },
    { -0.0000f, -0.4997f, -0.8662f },
    {  0.0000f,  0.7682f, -0.6402f },
    {  0.0000f,  0.7682f, -0.6402f },
    {  0.0000f,  1.0000f, -0.0056f },
    {  0.0000f,  1.0000f, -0.0056f },
    { -0.0000f,  0.7655f,  0.6435f },
    {  0.0000f,  0.7655f,  0.6435f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f, -0.0000f,  0.0000f },
    {  0.0000f,  0.1779f,  0.9840f },
    { -0.0000f,  0.1779f,  0.9840f },
    {  0.0000f, -0.4997f,  0.8662f },
    {  0.0000f, -0.4997f,  0.8662f },
    { -0.0000f, -0.9412f, -0.3377f },
    {  0.0000f, -0.9412f, -0.3377f },
    {  0.0000f,  0.1716f, -0.9852f },
    {  0.0000f,  0.1716f, -0.9852f },
    {  1.0000f, -0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f, -0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f, -0.0000f,  0.0000f },
    {  1.0000f,  0.0000f, -0.0000f },
    {  1.0000f,  0.0000f, -0.0000f },
    {  0.0000f, -0.9388f,  0.3444f },
    {  0.0000f, -0.9388f,  0.3444f },
    { -0.0000f, -0.4997f, -0.8662f },
    {  0.0000f, -0.4997f, -0.8662f },
    {  0.0000f,  0.7682f, -0.6402f },
    {  0.0000f,  0.7682f, -0.6402f },
    {  0.0000f,  1.0000f, -0.0056f },
    {  0.0000f,  1.0000f, -0.0056f },
    {  0.0000f,  0.7655f,  0.6435f },
    { -0.0000f,  0.7655f,  0.6435f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f, -0.0000f },
    { -0.0000f,  0.1779f,  0.9840f },
    {  0.0000f,  0.1779f,  0.9840f },
    {  0.0000f, -0.4997f,  0.8662f },
    {  0.0000f, -0.4997f,  0.8662f },
    {  0.0000f, -0.9412f, -0.3377f },
    { -0.0000f, -0.9412f, -0.3377f },
    {  0.0000f,  0.1716f, -0.9852f },
    {  0.0000f,  0.1716f, -0.9852f },
    { -1.0000f,  0.0000f, -0.0000f },
    { -1.0000f, -0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f, -0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f, -0.0000f,  0.0000f },
    { -1.0000f, -0.0000f,  0.0000f },
    {  0.0000f, -0.9388f,  0.3444f },
    {  0.0000f, -0.9388f,  0.3444f },
    { -0.0000f, -0.5039f, -0.8638f },
    {  0.0000f, -0.5039f, -0.8638f },
    {  0.0000f,  0.7682f, -0.6402f },
    {  0.0000f,  0.7682f, -0.6402f },
    {  0.0000f,  1.0000f, -0.0056f },
    {  0.0000f,  1.0000f, -0.0056f },
    {  0.0000f,  0.7655f,  0.6435f },
    { -0.0000f,  0.7655f,  0.6435f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f, -0.0000f },
    { -0.0000f,  0.1779f,  0.9840f },
    {  0.0000f,  0.1779f,  0.9840f },
    {  0.0000f, -0.4997f,  0.8662f },
    {  0.0000f, -0.4997f,  0.8662f },
    {  0.0000f, -0.9406f, -0.3395f },
    { -0.0000f, -0.9406f, -0.3395f },
    {  0.0000f,  0.1716f, -0.9852f },
    {  0.0000f,  0.1716f, -0.9852f },
    { -1.0000f,  0.0000f, -0.0000f },
    { -1.0000f, -0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f, -0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f, -0.0000f,  0.0000f },
    { -1.0000f, -0.0000f,  0.0000f }
};

const Point3D lod_car_vertices[32] = {
    { -0.6000f,  0.1000f,  1.4000f },
    { -0.5000f,  0.4000f,  1.2000f },
    { -0.5000f,  0.4500f,  0.5000f },
    { -0.4000f,  0.8000f,  0.1000f },
    { -0.4000f,  0.7500f, -0.6000f },
    { -0.5000f,  0.5000f, -1.2000f },
    { -0.6000f,  0.2000f, -1.3000f },
    { -0.6000f,  0.1000f, -1.3000f },
    {  0.6000f,  0.1000f,  1.4000f },
    {  0.5000f,  0.4000f,  1.2000f },
    {  0.5000f,  0.4500f,  0.5000f },
    {  0.4000f,  0.8000f,  0.1000f },
    {  0.4000f,  0.7500f, -0.6000f },
    {  0.5000f,  0.5000f, -1.2000f },
    {  0.6000f,  0.2000f, -1.3000f },
    {  0.6000f,  0.1000f, -1.3000f },
    { -0.6500f, -0.0500f,  1.1000f },
    { -0.6500f,  0.2500f,  1.1000f },
    { -0.6500f,  0.2500f,  0.5000f },
    { -0.6500f, -0.0500f,  0.5000f },
    { -0.6500f, -0.0500f, -0.6000f },
    { -0.6500f,  0.2500f, -0.6000f },
    { -0.6500f,  0.2500f, -1.2000f },
    { -0.6500f, -0.0500f, -1.2000f },
    {  0.6500f, -0.0500f,  1.1000f },
    {  0.6500f,  0.2500f,  1.1000f },
    {  0.6500f,  0.2500f,  0.5000f },
    {  0.6500f, -0.0500f,  0.5000f },
    {  0.6500f, -0.0500f, -0.6000f },
    {  0.6500f,  0.2500f, -0.6000f },
    {  0.6500f,  0.2500f, -1.2000f },
    {  0.6500f, -0.0500f, -1.2000f }
};

const Face lod_car_faces[23] = {
    { {  0,  8,  9,  1}, 4, 0, 0x3186 },
    { {  1,  9, 10,  2}, 4, 0, 0xFFFF },
    { {  2, 10, 11,  3}, 4, 0, 0x0008 },
    { {  3, 11, 12,  4}, 4, 0, 0xFFFF },
    { {  4, 12, 13,  5}, 4, 0, 0x0008 },
    { {  5, 13, 14,  6}, 4, 0, 0xFFFF },
    { {  6, 14, 15,  7}, 4, 0, 0x3186 },
    { {  0,  1,  2,  0}, 3, 0, 0xFFFF },
    { {  2,  3,  4,  0}, 3, 0, 0xFFFF },
    { {  4,  5,  6,  0}, 3, 0, 0xFFFF },
    { {  0,  2,  4,  0}, 3, 0, 0xFFFF },
    { {  0,  4,  6,  0}, 3, 0, 0xFFFF },
    { {  0,  6,  7,  0}, 3, 0, 0x18C3 },
    { {  8, 10,  9,  0}, 3, 0, 0xFFFF },
    { { 10, 12, 11,  0}, 3, 0, 0xFFFF },
    { { 12, 14, 13,  0}, 3, 0, 0xFFFF },
    { {  8, 12, 10,  0}, 3, 0, 0xFFFF },
    { {  8, 14, 12,  0}, 3, 0, 0xFFFF },
    { {  8, 15, 14,  0}, 3, 0, 0x18C3 },
    { { 16, 17, 18, 19}, 4, 1, 0x0000 },
    { { 20, 21, 22, 23}, 4, 1, 0x0000 },
    { { 24, 25, 26, 27}, 4, 1, 0x0000 },
    { { 28, 29, 30, 31}, 4, 1, 0x0000 }
};

const Point3D lod_car_normals[23] = {
    {  0.0000f,  0.5547f,  0.8321f },
    {  0.0000f,  0.9975f,  0.0712f },
    {  0.0000f,  0.7526f,  0.6585f },
    {  0.0000f,  0.9975f, -0.0712f },
    {  0.0000f,  0.9231f, -0.3846f },
    {  0.0000f,  0.3162f, -0.9487f },
    {  0.0000f,  0.0000f, -1.0000f },
    { -0.9436f,  0.3303f,  0.0236f },
    { -0.9667f,  0.2553f, -0.0182f },
    { -0.9513f,  0.3069f,  0.0307f },
    { -0.9843f,  0.1712f, -0.0428f },
    { -0.9446f,  0.3280f,  0.0121f },
    { -1.0000f,  0.0000f,  0.0000f },
    {  0.9436f,  0.3303f,  0.0236f },
    {  0.9667f,  0.2553f, -0.0182f },
    {  0.9513f,  0.3069f,  0.0307f },
    {  0.9843f,  0.1712f, -0.0428f },
    {  0.9446f,  0.3280f,  0.0121f },
    {  1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    { -1.0000f,  0.0000f,  0.0000f }
};

const Point3D tree_vertices[13] = {
    { -0.1200f,  0.0000f, -0.1200f },
    {  0.1200f,  0.0000f, -0.1200f },
    {  0.1200f,  0.0000f,  0.1200f },
    { -0.1200f,  0.0000f,  0.1200f },
    { -0.1200f,  0.8000f, -0.1200f },
    {  0.1200f,  0.8000f, -0.1200f },
    {  0.1200f,  0.8000f,  0.1200f },
    { -0.1200f,  0.8000f,  0.1200f },
    { -0.7000f,  0.8000f, -0.7000f },
    {  0.7000f,  0.8000f, -0.7000f },
    {  0.7000f,  0.8000f,  0.7000f },
    { -0.7000f,  0.8000f,  0.7000f },
    {  0.0000f,  2.8000f,  0.0000f }
};

const Face tree_faces[9] = {
    { {  0,  4,  5,  1}, 4, 0, 0x51E0 },
    { {  1,  5,  6,  2}, 4, 0, 0x51E0 },
    { {  2,  6,  7,  3}, 4, 0, 0x51E0 },
    { {  3,  7,  4,  0}, 4, 0, 0x51E0 },
    { {  8,  9, 10, 11}, 4, 0, 0x04A0 },
    { {  8, 12,  9,  0}, 3, 0, 0x0640 },
    { {  9, 12, 10,  0}, 3, 0, 0x0640 },
    { { 10, 12, 11,  0}, 3, 0, 0x0640 },
    { { 11, 12,  8,  0}, 3, 0, 0x0640 }
};

const Point3D tree_normals[9] = {
    {  0.0000f,  0.0000f, -1.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  0.0000f,  0.0000f,  1.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    {  0.0000f, -1.0000f,  0.0000f },
    {  0.0000f,  0.3304f, -0.9439f },
    {  0.9439f,  0.3304f, -0.0000f },
    {  0.0000f,  0.3304f,  0.9439f },
    { -0.9439f,  0.3304f,  0.0000f }
};

const Point3D billboard_vertices[8] = {
    { -1.5000f,  0.0000f,  0.0000f },
    { -1.5000f,  1.4000f,  0.0000f },
    {  1.5000f,  0.0000f,  0.0000f },
    {  1.5000f,  1.4000f,  0.0000f },
    { -1.8000f,  1.3000f,  0.0000f },
    {  1.8000f,  1.3000f,  0.0000f },
    {  1.8000f,  2.4000f,  0.0000f },
    { -1.8000f,  2.4000f,  0.0000f }
};

const Face billboard_faces[1] = {
    { {  4,  5,  6,  7}, 4, 1, 0xFFFF }
};

const Point3D billboard_normals[1] = {
    {  0.0000f,  0.0000f,  1.0000f }
};

const Point3D bridge_vertices[24] = {
    { -3.4000f,  0.0000f, -0.4000f },
    { -2.9000f,  0.0000f, -0.4000f },
    { -2.9000f,  3.5000f, -0.4000f },
    { -3.4000f,  3.5000f, -0.4000f },
    { -3.4000f,  0.0000f,  0.4000f },
    { -2.9000f,  0.0000f,  0.4000f },
    { -2.9000f,  3.5000f,  0.4000f },
    { -3.4000f,  3.5000f,  0.4000f },
    {  2.9000f,  0.0000f, -0.4000f },
    {  3.4000f,  0.0000f, -0.4000f },
    {  3.4000f,  3.5000f, -0.4000f },
    {  2.9000f,  3.5000f, -0.4000f },
    {  2.9000f,  0.0000f,  0.4000f },
    {  3.4000f,  0.0000f,  0.4000f },
    {  3.4000f,  3.5000f,  0.4000f },
    {  2.9000f,  3.5000f,  0.4000f },
    { -3.4000f,  3.5000f, -0.4000f },
    {  3.4000f,  3.5000f, -0.4000f },
    {  3.4000f,  4.3000f, -0.4000f },
    { -3.4000f,  4.3000f, -0.4000f },
    { -3.4000f,  3.5000f,  0.4000f },
    {  3.4000f,  3.5000f,  0.4000f },
    {  3.4000f,  4.3000f,  0.4000f },
    { -3.4000f,  4.3000f,  0.4000f }
};

const Face bridge_faces[12] = {
    { {  0,  3,  2,  1}, 4, 1, 0x5AEB },
    { {  4,  5,  6,  7}, 4, 1, 0x5AEB },
    { {  0,  4,  7,  3}, 4, 1, 0x3186 },
    { {  1,  2,  6,  5}, 4, 1, 0x3186 },
    { {  8, 11, 10,  9}, 4, 1, 0x5AEB },
    { { 12, 13, 14, 15}, 4, 1, 0x5AEB },
    { {  8, 12, 15, 11}, 4, 1, 0x3186 },
    { {  9, 10, 14, 13}, 4, 1, 0x3186 },
    { { 16, 19, 18, 17}, 4, 1, 0xF800 },
    { { 20, 21, 22, 23}, 4, 1, 0xF800 },
    { { 19, 23, 22, 18}, 4, 1, 0xFBE0 },
    { { 16, 17, 21, 20}, 4, 1, 0x5AEB }
};

const Face gantry_faces[12] = {
    { {  0,  3,  2,  1}, 4, 1, 0xFFFF },
    { {  4,  5,  6,  7}, 4, 1, 0xFFFF },
    { {  0,  4,  7,  3}, 4, 1, 0x3186 },
    { {  1,  2,  6,  5}, 4, 1, 0x3186 },
    { {  8, 11, 10,  9}, 4, 1, 0xFFFF },
    { { 12, 13, 14, 15}, 4, 1, 0xFFFF },
    { {  8, 12, 15, 11}, 4, 1, 0x3186 },
    { {  9, 10, 14, 13}, 4, 1, 0x3186 },
    { { 16, 19, 18, 17}, 4, 1, 0xFFFF },
    { { 20, 21, 22, 23}, 4, 1, 0xFFFF },
    { { 19, 23, 22, 18}, 4, 1, 0x0000 },
    { { 16, 17, 21, 20}, 4, 1, 0x5AEB }
};

const Point3D bridge_normals[12] = {
    {  0.0000f,  0.0000f, -1.0000f },
    {  0.0000f,  0.0000f,  1.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  0.0000f,  0.0000f, -1.0000f },
    {  0.0000f,  0.0000f,  1.0000f },
    { -1.0000f,  0.0000f,  0.0000f },
    {  1.0000f,  0.0000f,  0.0000f },
    {  0.0000f,  0.0000f, -1.0000f },
    {  0.0000f,  0.0000f,  1.0000f },
    {  0.0000f,  1.0000f,  0.0000f },
    {  0.0000f, -1.0000f,  0.0000f }
};

// gantry_faces shares bridge geometry; use bridge_normals with it.
// ===== END GENERATED MODEL DATA =====
