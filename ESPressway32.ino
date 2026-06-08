#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Preferences.h>
#include <math.h>

// User Config
// Leave false for the normal T-Display-S3 orientation. Set true to rotate the screen 180 degrees.
#define DISPLAY_FLIP_180 false
#define DISPLAY_ROTATION (DISPLAY_FLIP_180 ? 1 : 3)

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

// 3D Math Structures
struct Point3D {
    float x, y, z;
};

struct Point2D {
    float x, y;
};

struct Face {
    int indices[4];
    int num_vertices;
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
        bus_cfg.freq_write = 20000000;
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
        panel_cfg.bus_shared = true;
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
LGFX_Sprite sprite(&tft);
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

// Camera space light direction (pre-rotated per frame)
float cam_light_x = 0.577f;
float cam_light_y = 0.707f;
float cam_light_z = -0.408f;

// Track array
TrackSegment track[NUM_SEGMENTS];

// 3D Low-Poly Models
// Mesh converted from the downloaded Sketchfab Subaru Impreza 22B source OBJ.
// Coordinates are normalized for the ESP32 flat-shaded renderer.
#define CAR_NUM_VERTICES 215
#define CAR_NUM_FACES 368

const Point3D car_vertices[CAR_NUM_VERTICES] = {
    {  0.520f,  0.395f, -0.761f }, // 0
    {  0.520f,  0.218f, -0.729f }, // 1
    {  0.520f,  0.062f, -0.819f }, // 2
    {  0.520f,  0.000f, -0.988f }, // 3
    {  0.520f,  0.061f, -1.157f }, // 4
    {  0.520f,  0.217f, -1.248f }, // 5
    {  0.520f,  0.395f, -1.217f }, // 6
    {  0.520f,  0.510f, -1.079f }, // 7
    {  0.520f,  0.511f, -0.899f }, // 8
    {  0.706f,  0.395f, -0.761f }, // 9
    {  0.706f,  0.218f, -0.729f }, // 10
    {  0.706f,  0.062f, -0.819f }, // 11
    {  0.706f,  0.000f, -0.988f }, // 12
    {  0.706f,  0.061f, -1.157f }, // 13
    {  0.706f,  0.217f, -1.248f }, // 14
    {  0.706f,  0.395f, -1.217f }, // 15
    {  0.706f,  0.510f, -1.079f }, // 16
    {  0.706f,  0.511f, -0.899f }, // 17
    { -0.814f,  0.658f,  0.536f }, // 18
    { -0.801f,  0.726f,  0.536f }, // 19
    { -0.801f,  0.726f,  0.473f }, // 20
    { -0.814f,  0.648f,  0.473f }, // 21
    { -0.613f,  0.658f,  0.536f }, // 22
    { -0.609f,  0.648f,  0.473f }, // 23
    { -0.594f,  0.713f,  0.473f }, // 24
    { -0.596f,  0.712f,  0.536f }, // 25
    { -0.618f,  0.709f, -1.140f }, // 26
    { -0.626f,  0.641f, -0.642f }, // 27
    { -0.635f,  0.638f,  0.666f }, // 28
    { -0.434f,  1.020f,  0.017f }, // 29
    { -0.604f,  0.316f,  1.602f }, // 30
    { -0.701f,  0.401f,  1.251f }, // 31
    { -0.689f,  0.317f,  1.296f }, // 32
    { -0.595f,  0.075f,  1.597f }, // 33
    { -0.544f,  0.383f,  1.554f }, // 34
    { -0.531f,  0.490f,  1.532f }, // 35
    { -0.672f,  0.107f, -1.294f }, // 36
    { -0.583f,  0.105f, -1.609f }, // 37
    { -0.594f,  0.430f, -1.620f }, // 38
    { -0.672f,  0.092f,  0.717f }, // 39
    { -0.677f,  0.083f, -0.708f }, // 40
    { -0.547f,  0.614f, -1.618f }, // 41
    { -0.591f,  0.728f, -1.314f }, // 42
    { -0.200f,  0.732f, -1.295f }, // 43
    { -0.471f,  0.728f, -1.313f }, // 44
    { -0.506f,  0.728f, -1.582f }, // 45
    { -0.471f,  0.728f, -1.593f }, // 46
    { -0.703f,  0.447f,  1.216f }, // 47
    { -0.677f,  0.615f, -1.139f }, // 48
    { -0.695f,  0.589f, -0.657f }, // 49
    { -0.554f,  0.592f,  1.366f }, // 50
    { -0.703f,  0.528f,  1.096f }, // 51
    { -0.669f,  0.079f,  1.307f }, // 52
    { -0.707f,  0.327f, -0.709f }, // 53
    { -0.701f,  0.317f,  0.712f }, // 54
    { -0.684f,  0.571f,  0.711f }, // 55
    { -0.712f,  0.455f, -0.774f }, // 56
    { -0.703f,  0.447f,  0.779f }, // 57
    { -0.703f,  0.528f,  0.921f }, // 58
    { -0.706f,  0.467f, -1.199f }, // 59
    { -0.712f,  0.534f, -1.039f }, // 60
    { -0.715f,  0.534f, -0.920f }, // 61
    { -0.706f,  0.327f, -1.271f }, // 62
    { -0.517f,  0.876f, -1.451f }, // 63
    { -0.506f,  0.877f, -1.639f }, // 64
    { -0.469f,  0.839f, -1.417f }, // 65
    { -0.470f,  0.839f, -1.631f }, // 66
    { -0.001f,  0.383f,  1.666f }, // 67
    { -0.001f,  0.493f,  1.642f }, // 68
    { -0.001f,  0.105f, -1.691f }, // 69
    { -0.001f,  0.428f, -1.725f }, // 70
    { -0.001f,  0.616f, -1.657f }, // 71
    { -0.001f,  0.734f, -1.611f }, // 72
    {  0.814f,  0.672f,  0.536f }, // 73
    {  0.814f,  0.663f,  0.473f }, // 74
    {  0.801f,  0.740f,  0.473f }, // 75
    {  0.801f,  0.740f,  0.536f }, // 76
    {  0.612f,  0.672f,  0.536f }, // 77
    {  0.620f,  0.663f,  0.473f }, // 78
    {  0.590f,  0.726f,  0.465f }, // 79
    {  0.589f,  0.727f,  0.529f }, // 80
    {  0.199f,  0.732f, -1.295f }, // 81
    {  0.452f,  1.035f, -0.756f }, // 82
    {  0.621f,  0.709f, -1.140f }, // 83
    {  0.624f,  0.641f, -0.642f }, // 84
    {  0.440f,  1.020f,  0.017f }, // 85
    {  0.634f,  0.638f,  0.666f }, // 86
    {  0.603f,  0.316f,  1.602f }, // 87
    {  0.687f,  0.317f,  1.296f }, // 88
    {  0.699f,  0.401f,  1.251f }, // 89
    {  0.593f,  0.075f,  1.597f }, // 90
    { -0.001f,  0.317f,  1.706f }, // 91
    { -0.001f,  0.074f,  1.725f }, // 92
    {  0.542f,  0.383f,  1.554f }, // 93
    {  0.530f,  0.490f,  1.532f }, // 94
    {  0.670f,  0.107f, -1.294f }, // 95
    {  0.583f,  0.105f, -1.609f }, // 96
    { -0.001f,  0.429f, -1.649f }, // 97
    {  0.589f,  0.435f, -1.605f }, // 98
    {  0.670f,  0.092f,  0.717f }, // 99
    {  0.675f,  0.083f, -0.708f }, // 100
    {  0.546f,  0.614f, -1.618f }, // 101
    {  0.571f,  0.728f, -1.314f }, // 102
    {  0.469f,  0.728f, -1.313f }, // 103
    {  0.469f,  0.728f, -1.593f }, // 104
    {  0.504f,  0.728f, -1.582f }, // 105
    {  0.701f,  0.447f,  1.216f }, // 106
    {  0.693f,  0.589f, -0.657f }, // 107
    {  0.675f,  0.615f, -1.139f }, // 108
    {  0.552f,  0.592f,  1.366f }, // 109
    {  0.701f,  0.528f,  1.096f }, // 110
    {  0.668f,  0.079f,  1.307f }, // 111
    {  0.705f,  0.327f, -0.709f }, // 112
    {  0.699f,  0.317f,  0.712f }, // 113
    {  0.682f,  0.571f,  0.711f }, // 114
    {  0.710f,  0.455f, -0.774f }, // 115
    {  0.701f,  0.528f,  0.921f }, // 116
    {  0.701f,  0.447f,  0.779f }, // 117
    {  0.704f,  0.467f, -1.199f }, // 118
    {  0.713f,  0.534f, -0.920f }, // 119
    {  0.710f,  0.534f, -1.039f }, // 120
    {  0.704f,  0.327f, -1.271f }, // 121
    {  0.504f,  0.877f, -1.639f }, // 122
    {  0.516f,  0.876f, -1.451f }, // 123
    {  0.468f,  0.839f, -1.417f }, // 124
    {  0.468f,  0.839f, -1.631f }, // 125
    { -0.457f,  1.007f,  0.135f }, // 126
    {  0.455f,  1.007f,  0.135f }, // 127
    { -0.440f,  1.049f, -0.703f }, // 128
    { -0.454f,  1.035f, -0.756f }, // 129
    {  0.446f,  1.049f, -0.703f }, // 130
    { -0.001f,  0.666f,  0.798f }, // 131
    {  0.537f,  0.728f, -1.573f }, // 132
    {  0.600f,  0.614f, -1.565f }, // 133
    {  0.637f,  0.432f, -1.561f }, // 134
    {  0.630f,  0.106f, -1.565f }, // 135
    { -0.507f,  0.728f, -1.568f }, // 136
    { -0.599f,  0.614f, -1.566f }, // 137
    { -0.637f,  0.433f, -1.564f }, // 138
    { -0.630f,  0.106f, -1.562f }, // 139
    {  0.432f,  0.092f,  0.717f }, // 140
    {  0.432f,  0.079f,  1.307f }, // 141
    { -0.465f,  0.092f,  0.717f }, // 142
    { -0.466f,  0.079f,  1.307f }, // 143
    {  0.451f,  0.107f, -1.294f }, // 144
    {  0.453f,  0.083f, -0.708f }, // 145
    { -0.476f,  0.107f, -1.294f }, // 146
    { -0.477f,  0.083f, -0.708f }, // 147
    {  0.313f,  0.316f,  1.670f }, // 148
    {  0.313f,  0.075f,  1.682f }, // 149
    {  0.313f,  0.383f,  1.612f }, // 150
    {  0.313f,  0.491f,  1.588f }, // 151
    { -0.285f,  0.075f,  1.678f }, // 152
    { -0.285f,  0.310f,  1.665f }, // 153
    { -0.285f,  0.383f,  1.609f }, // 154
    { -0.285f,  0.491f,  1.585f }, // 155
    { -0.285f,  0.105f, -1.682f }, // 156
    { -0.285f,  0.428f, -1.721f }, // 157
    {  0.295f,  0.105f, -1.680f }, // 158
    {  0.295f,  0.433f, -1.713f }, // 159
    { -0.001f,  0.593f,  1.372f }, // 160
    {  0.509f,  0.395f,  1.237f }, // 161
    {  0.509f,  0.218f,  1.269f }, // 162
    {  0.509f,  0.062f,  1.179f }, // 163
    {  0.509f,  0.000f,  1.010f }, // 164
    {  0.509f,  0.061f,  0.840f }, // 165
    {  0.509f,  0.217f,  0.750f }, // 166
    {  0.509f,  0.395f,  0.781f }, // 167
    {  0.509f,  0.510f,  0.919f }, // 168
    {  0.509f,  0.511f,  1.099f }, // 169
    {  0.695f,  0.395f,  1.237f }, // 170
    {  0.695f,  0.218f,  1.269f }, // 171
    {  0.695f,  0.062f,  1.179f }, // 172
    {  0.695f,  0.000f,  1.010f }, // 173
    {  0.695f,  0.061f,  0.840f }, // 174
    {  0.695f,  0.217f,  0.750f }, // 175
    {  0.695f,  0.395f,  0.781f }, // 176
    {  0.695f,  0.510f,  0.919f }, // 177
    {  0.695f,  0.511f,  1.099f }, // 178
    { -0.509f,  0.395f,  1.237f }, // 179
    { -0.509f,  0.218f,  1.269f }, // 180
    { -0.509f,  0.062f,  1.179f }, // 181
    { -0.509f,  0.000f,  1.010f }, // 182
    { -0.509f,  0.061f,  0.840f }, // 183
    { -0.509f,  0.217f,  0.750f }, // 184
    { -0.509f,  0.395f,  0.781f }, // 185
    { -0.509f,  0.510f,  0.919f }, // 186
    { -0.509f,  0.511f,  1.099f }, // 187
    { -0.695f,  0.395f,  1.237f }, // 188
    { -0.695f,  0.218f,  1.269f }, // 189
    { -0.695f,  0.062f,  1.179f }, // 190
    { -0.695f,  0.000f,  1.010f }, // 191
    { -0.695f,  0.061f,  0.840f }, // 192
    { -0.695f,  0.217f,  0.750f }, // 193
    { -0.695f,  0.395f,  0.781f }, // 194
    { -0.695f,  0.510f,  0.919f }, // 195
    { -0.695f,  0.511f,  1.099f }, // 196
    { -0.518f,  0.395f, -0.761f }, // 197
    { -0.518f,  0.218f, -0.729f }, // 198
    { -0.518f,  0.062f, -0.819f }, // 199
    { -0.518f,  0.000f, -0.988f }, // 200
    { -0.518f,  0.061f, -1.157f }, // 201
    { -0.518f,  0.217f, -1.248f }, // 202
    { -0.518f,  0.395f, -1.217f }, // 203
    { -0.518f,  0.510f, -1.079f }, // 204
    { -0.518f,  0.511f, -0.899f }, // 205
    { -0.704f,  0.395f, -0.761f }, // 206
    { -0.704f,  0.218f, -0.729f }, // 207
    { -0.704f,  0.062f, -0.819f }, // 208
    { -0.704f,  0.000f, -0.988f }, // 209
    { -0.704f,  0.061f, -1.157f }, // 210
    { -0.704f,  0.217f, -1.248f }, // 211
    { -0.704f,  0.395f, -1.217f }, // 212
    { -0.704f,  0.510f, -1.079f }, // 213
    { -0.704f,  0.511f, -0.899f } // 214
};

const Face car_faces[CAR_NUM_FACES] = {
    { {  3,  12,  11,   0}, 3, 0x0000 }, // 0: impreza_wheel
    { { 11,   2,   3,   0}, 3, 0x0000 }, // 1: impreza_wheel
    { {  5,  14,  13,   0}, 3, 0x0000 }, // 2: impreza_wheel
    { { 13,   4,   5,   0}, 3, 0x0000 }, // 3: impreza_wheel
    { {  7,  16,  15,   0}, 3, 0x0000 }, // 4: impreza_wheel
    { { 15,   6,   7,   0}, 3, 0x0000 }, // 5: impreza_wheel
    { {  8,  17,  16,   0}, 3, 0x0000 }, // 6: impreza_wheel
    { { 16,   7,   8,   0}, 3, 0x0000 }, // 7: impreza_wheel
    { {  0,   9,  17,   0}, 3, 0x0000 }, // 8: impreza_wheel
    { { 17,   8,   0,   0}, 3, 0x0000 }, // 9: impreza_wheel
    { {  2,   1,   0,   0}, 3, 0x0000 }, // 10: impreza_wheel
    { {  0,   3,   2,   0}, 3, 0x0000 }, // 11: impreza_wheel
    { {  3,   0,   8,   0}, 3, 0x0000 }, // 12: impreza_wheel
    { {  8,   4,   3,   0}, 3, 0x0000 }, // 13: impreza_wheel
    { {  4,   8,   7,   0}, 3, 0x0000 }, // 14: impreza_wheel
    { {  7,   5,   4,   0}, 3, 0x0000 }, // 15: impreza_wheel
    { {  6,   5,   7,   0}, 3, 0x0000 }, // 16: impreza_wheel
    { {  9,   0,   1,   0}, 3, 0x0000 }, // 17: impreza_wheel
    { {  1,  10,   9,   0}, 3, 0x0000 }, // 18: impreza_wheel
    { { 10,   1,   2,   0}, 3, 0x0000 }, // 19: impreza_wheel
    { {  2,  11,  10,   0}, 3, 0x0000 }, // 20: impreza_wheel
    { { 12,   3,   4,   0}, 3, 0x0000 }, // 21: impreza_wheel
    { {  4,  13,  12,   0}, 3, 0x0000 }, // 22: impreza_wheel
    { { 14,   5,   6,   0}, 3, 0x0000 }, // 23: impreza_wheel
    { {  6,  15,  14,   0}, 3, 0x0000 }, // 24: impreza_wheel
    { { 11,  12,   9,   0}, 3, 0x0000 }, // 25: impreza_wheel
    { {  9,  10,  11,   0}, 3, 0x0000 }, // 26: impreza_wheel
    { { 12,  13,  17,   0}, 3, 0x0000 }, // 27: impreza_wheel
    { { 17,   9,  12,   0}, 3, 0x0000 }, // 28: impreza_wheel
    { { 13,  14,  16,   0}, 3, 0x0000 }, // 29: impreza_wheel
    { { 16,  17,  13,   0}, 3, 0x0000 }, // 30: impreza_wheel
    { { 15,  16,  14,   0}, 3, 0x0000 }, // 31: impreza_wheel
    { { 18,  20,  21,   0}, 3, 0x0008 }, // 32: imprezachassis
    { { 20,  18,  19,   0}, 3, 0x0008 }, // 33: imprezachassis
    { { 22,  21,  23,   0}, 3, 0x0008 }, // 34: imprezachassis
    { { 21,  22,  18,   0}, 3, 0x0008 }, // 35: imprezachassis
    { { 23,  20,  24,   0}, 3, 0x0008 }, // 36: imprezachassis
    { { 20,  23,  21,   0}, 3, 0x0008 }, // 37: imprezachassis
    { { 24,  19,  25,   0}, 3, 0x0008 }, // 38: imprezachassis
    { { 19,  24,  20,   0}, 3, 0x0008 }, // 39: imprezachassis
    { { 25,  18,  22,   0}, 3, 0x0008 }, // 40: imprezachassis
    { { 18,  25,  19,   0}, 3, 0x0008 }, // 41: imprezachassis
    { { 43,  26, 129,   0}, 3, 0xFFFF }, // 42: imprezachassis
    { { 27, 128, 129,   0}, 3, 0x0008 }, // 43: imprezachassis
    { {128, 126,  29,   0}, 3, 0xFFFF }, // 44: imprezachassis
    { { 27, 126, 128,   0}, 3, 0x0008 }, // 45: imprezachassis
    { { 27,  28, 126,   0}, 3, 0x0008 }, // 46: imprezachassis
    { { 30,  31,  32,   0}, 3, 0xFFFF }, // 47: imprezachassis
    { { 33, 153,  30,   0}, 3, 0xFFFF }, // 48: imprezachassis
    { {153,  33, 152,   0}, 3, 0xFFFF }, // 49: imprezachassis
    { { 34, 153, 154,   0}, 3, 0xFFFF }, // 50: imprezachassis
    { {153,  34,  30,   0}, 3, 0xFFFF }, // 51: imprezachassis
    { { 35, 154, 155,   0}, 3, 0xFFFF }, // 52: imprezachassis
    { {154,  35,  34,   0}, 3, 0xFFFF }, // 53: imprezachassis
    { {135, 158,  96,   0}, 3, 0xFFFF }, // 54: imprezachassis
    { {135,  69, 158,   0}, 3, 0xFFFF }, // 55: imprezachassis
    { { 95,  69, 135,   0}, 3, 0xFFFF }, // 56: imprezachassis
    { { 95, 156,  69,   0}, 3, 0xFFFF }, // 57: imprezachassis
    { { 95,  37, 156,   0}, 3, 0xFFFF }, // 58: imprezachassis
    { { 95, 139,  37,   0}, 3, 0xFFFF }, // 59: imprezachassis
    { {144, 139,  95,   0}, 3, 0xFFFF }, // 60: imprezachassis
    { {146, 139, 144,   0}, 3, 0xFFFF }, // 61: imprezachassis
    { { 36, 139, 146,   0}, 3, 0xFFFF }, // 62: imprezachassis
    { {157,  97,  70,   0}, 3, 0xFFFF }, // 63: imprezachassis
    { { 97, 157,  38,   0}, 3, 0xFFFF }, // 64: imprezachassis
    { { 37, 157, 156,   0}, 3, 0xFFFF }, // 65: imprezachassis
    { {157,  37,  38,   0}, 3, 0xFFFF }, // 66: imprezachassis
    { { 38,  71,  97,   0}, 3, 0xFFFF }, // 67: imprezachassis
    { { 71,  38,  41,   0}, 3, 0xFFFF }, // 68: imprezachassis
    { { 42,  43,  44,   0}, 3, 0xFFFF }, // 69: imprezachassis
    { { 43,  42,  26,   0}, 3, 0xFFFF }, // 70: imprezachassis
    { { 71,  46,  72,   0}, 3, 0xFFFF }, // 71: imprezachassis
    { { 41,  46,  71,   0}, 3, 0xFFFF }, // 72: imprezachassis
    { { 41,  45,  46,   0}, 3, 0xFFFF }, // 73: imprezachassis
    { { 30,  47,  31,   0}, 3, 0xFFFF }, // 74: imprezachassis
    { { 47,  30,  34,   0}, 3, 0xFFFF }, // 75: imprezachassis
    { { 48,  27,  26,   0}, 3, 0xFFFF }, // 76: imprezachassis
    { { 27,  48,  49,   0}, 3, 0x0008 }, // 77: imprezachassis
    { { 50,  68, 160,   0}, 3, 0xFFFF }, // 78: imprezachassis
    { { 50, 155,  68,   0}, 3, 0xFFFF }, // 79: imprezachassis
    { { 50,  35, 155,   0}, 3, 0xFFFF }, // 80: imprezachassis
    { { 35,  47,  34,   0}, 3, 0xFFFF }, // 81: imprezachassis
    { { 47,  35,  51,   0}, 3, 0xFFFF }, // 82: imprezachassis
    { { 50,  51,  35,   0}, 3, 0xFFFF }, // 83: imprezachassis
    { { 51,  50,  28,   0}, 3, 0x0008 }, // 84: imprezachassis
    { { 52,  30,  32,   0}, 3, 0xFFFF }, // 85: imprezachassis
    { { 30,  52,  33,   0}, 3, 0xFFFF }, // 86: imprezachassis
    { { 53,  39,  54,   0}, 3, 0xFFFF }, // 87: imprezachassis
    { { 39,  53,  40,   0}, 3, 0xFFFF }, // 88: imprezachassis
    { { 28,  49,  55,   0}, 3, 0x0008 }, // 89: imprezachassis
    { { 49,  28,  27,   0}, 3, 0x0008 }, // 90: imprezachassis
    { { 41, 136,  45,   0}, 3, 0xFFFF }, // 91: imprezachassis
    { {136,  41, 137,   0}, 3, 0xFFFF }, // 92: imprezachassis
    { {129,  26,  27,   0}, 3, 0x0008 }, // 93: imprezachassis
    { { 56,  53,  49,   0}, 3, 0xFFFF }, // 94: imprezachassis
    { { 53,  55,  49,   0}, 3, 0xFFFF }, // 95: imprezachassis
    { { 55,  53,  54,   0}, 3, 0xFFFF }, // 96: imprezachassis
    { { 57,  55,  54,   0}, 3, 0xFFFF }, // 97: imprezachassis
    { { 55,  57,  58,   0}, 3, 0xFFFF }, // 98: imprezachassis
    { { 28,  58,  51,   0}, 3, 0x0008 }, // 99: imprezachassis
    { { 58,  28,  55,   0}, 3, 0x0008 }, // 100: imprezachassis
    { { 49,  61,  56,   0}, 3, 0xFFFF }, // 101: imprezachassis
    { { 61,  49,  60,   0}, 3, 0xFFFF }, // 102: imprezachassis
    { { 38, 137,  41,   0}, 3, 0xFFFF }, // 103: imprezachassis
    { {137,  38, 138,   0}, 3, 0xFFFF }, // 104: imprezachassis
    { { 45,  63,  64,   0}, 3, 0xFFFF }, // 105: imprezachassis
    { { 63, 136,  42,   0}, 3, 0xFFFF }, // 106: imprezachassis
    { { 45, 136,  63,   0}, 3, 0xFFFF }, // 107: imprezachassis
    { {102, 124, 103,   0}, 3, 0xFFFF }, // 108: imprezachassis
    { {124, 102, 123,   0}, 3, 0xFFFF }, // 109: imprezachassis
    { { 66,  45,  64,   0}, 3, 0xFFFF }, // 110: imprezachassis
    { { 45,  66,  46,   0}, 3, 0xFFFF }, // 111: imprezachassis
    { { 46,  65,  44,   0}, 3, 0xFFFF }, // 112: imprezachassis
    { { 65,  46,  66,   0}, 3, 0xFFFF }, // 113: imprezachassis
    { { 29, 130, 128,   0}, 3, 0xFFFF }, // 114: imprezachassis
    { {130,  29,  85,   0}, 3, 0xFFFF }, // 115: imprezachassis
    { { 50, 131,  28,   0}, 3, 0x0008 }, // 116: imprezachassis
    { {131,  50, 160,   0}, 3, 0xFFFF }, // 117: imprezachassis
    { { 64, 123, 122,   0}, 3, 0x0008 }, // 118: imprezachassis
    { {123,  64,  63,   0}, 3, 0x0008 }, // 119: imprezachassis
    { { 43,  46,  44,   0}, 3, 0xFFFF }, // 120: imprezachassis
    { { 43,  72,  46,   0}, 3, 0x0008 }, // 121: imprezachassis
    { {104,  81, 103,   0}, 3, 0xFFFF }, // 122: imprezachassis
    { { 72,  81, 104,   0}, 3, 0x0008 }, // 123: imprezachassis
    { { 43,  81,  72,   0}, 3, 0x0008 }, // 124: imprezachassis
    { { 73,  75,  76,   0}, 3, 0x0008 }, // 125: imprezachassis
    { { 75,  73,  74,   0}, 3, 0x0008 }, // 126: imprezachassis
    { { 77,  74,  73,   0}, 3, 0x0008 }, // 127: imprezachassis
    { { 74,  77,  78,   0}, 3, 0x0008 }, // 128: imprezachassis
    { { 78,  75,  74,   0}, 3, 0x0008 }, // 129: imprezachassis
    { { 75,  78,  79,   0}, 3, 0x0008 }, // 130: imprezachassis
    { { 79,  76,  75,   0}, 3, 0x0008 }, // 131: imprezachassis
    { { 76,  79,  80,   0}, 3, 0x0008 }, // 132: imprezachassis
    { { 80,  73,  76,   0}, 3, 0x0008 }, // 133: imprezachassis
    { { 73,  80,  77,   0}, 3, 0x0008 }, // 134: imprezachassis
    { { 81,  82,  83,   0}, 3, 0xFFFF }, // 135: imprezachassis
    { { 86,  85, 127,   0}, 3, 0x0008 }, // 136: imprezachassis
    { { 84,  85,  86,   0}, 3, 0x0008 }, // 137: imprezachassis
    { { 85,  82, 130,   0}, 3, 0xFFFF }, // 138: imprezachassis
    { { 84,  82,  85,   0}, 3, 0x0008 }, // 139: imprezachassis
    { { 87,  88,  89,   0}, 3, 0xFFFF }, // 140: imprezachassis
    { { 90, 148, 149,   0}, 3, 0xFFFF }, // 141: imprezachassis
    { {148,  90,  87,   0}, 3, 0xFFFF }, // 142: imprezachassis
    { {148,  93, 150,   0}, 3, 0xFFFF }, // 143: imprezachassis
    { { 93, 148,  87,   0}, 3, 0xFFFF }, // 144: imprezachassis
    { {150,  94, 151,   0}, 3, 0xFFFF }, // 145: imprezachassis
    { { 94, 150,  93,   0}, 3, 0xFFFF }, // 146: imprezachassis
    { { 97, 159,  70,   0}, 3, 0xFFFF }, // 147: imprezachassis
    { {159,  97,  98,   0}, 3, 0xFFFF }, // 148: imprezachassis
    { { 99, 145, 100,   0}, 3, 0xFFFF }, // 149: imprezachassis
    { { 99, 147, 145,   0}, 3, 0xFFFF }, // 150: imprezachassis
    { { 99,  40, 147,   0}, 3, 0xFFFF }, // 151: imprezachassis
    { { 40, 142,  39,   0}, 3, 0xFFFF }, // 152: imprezachassis
    { { 40, 140, 142,   0}, 3, 0xFFFF }, // 153: imprezachassis
    { { 99, 140,  40,   0}, 3, 0xFFFF }, // 154: imprezachassis
    { { 96, 159,  98,   0}, 3, 0xFFFF }, // 155: imprezachassis
    { {159,  96, 158,   0}, 3, 0xFFFF }, // 156: imprezachassis
    { { 71,  98,  97,   0}, 3, 0xFFFF }, // 157: imprezachassis
    { { 98,  71, 101,   0}, 3, 0xFFFF }, // 158: imprezachassis
    { {102,  81,  83,   0}, 3, 0xFFFF }, // 159: imprezachassis
    { { 81, 102, 103,   0}, 3, 0xFFFF }, // 160: imprezachassis
    { {101, 104, 105,   0}, 3, 0xFFFF }, // 161: imprezachassis
    { {104,  71,  72,   0}, 3, 0xFFFF }, // 162: imprezachassis
    { {101,  71, 104,   0}, 3, 0xFFFF }, // 163: imprezachassis
    { { 89,  93,  87,   0}, 3, 0xFFFF }, // 164: imprezachassis
    { { 93,  89, 106,   0}, 3, 0xFFFF }, // 165: imprezachassis
    { { 84, 108,  83,   0}, 3, 0xFFFF }, // 166: imprezachassis
    { {108,  84, 107,   0}, 3, 0x0008 }, // 167: imprezachassis
    { {109, 151,  94,   0}, 3, 0xFFFF }, // 168: imprezachassis
    { {109,  68, 151,   0}, 3, 0xFFFF }, // 169: imprezachassis
    { {109, 160,  68,   0}, 3, 0xFFFF }, // 170: imprezachassis
    { {109, 110,  86,   0}, 3, 0x0008 }, // 171: imprezachassis
    { {110, 109,  94,   0}, 3, 0xFFFF }, // 172: imprezachassis
    { {111,  87,  90,   0}, 3, 0xFFFF }, // 173: imprezachassis
    { { 87, 111,  88,   0}, 3, 0xFFFF }, // 174: imprezachassis
    { {112,  99, 100,   0}, 3, 0xFFFF }, // 175: imprezachassis
    { { 99, 112, 113,   0}, 3, 0xFFFF }, // 176: imprezachassis
    { { 86, 107,  84,   0}, 3, 0x0008 }, // 177: imprezachassis
    { {107,  86, 114,   0}, 3, 0x0008 }, // 178: imprezachassis
    { {108, 102,  83,   0}, 3, 0xFFFF }, // 179: imprezachassis
    { {102, 133, 132,   0}, 3, 0xFFFF }, // 180: imprezachassis
    { {108, 133, 102,   0}, 3, 0xFFFF }, // 181: imprezachassis
    { { 82,  84,  83,   0}, 3, 0x0008 }, // 182: imprezachassis
    { {115, 107, 112,   0}, 3, 0xFFFF }, // 183: imprezachassis
    { {112, 114, 113,   0}, 3, 0xFFFF }, // 184: imprezachassis
    { {114, 112, 107,   0}, 3, 0xFFFF }, // 185: imprezachassis
    { {117, 114, 116,   0}, 3, 0xFFFF }, // 186: imprezachassis
    { {114, 117, 113,   0}, 3, 0xFFFF }, // 187: imprezachassis
    { { 86, 116, 114,   0}, 3, 0x0008 }, // 188: imprezachassis
    { {116,  86, 110,   0}, 3, 0x0008 }, // 189: imprezachassis
    { {120, 108, 107,   0}, 3, 0xFFFF }, // 190: imprezachassis
    { {108, 134, 133,   0}, 3, 0xFFFF }, // 191: imprezachassis
    { {134, 108, 118,   0}, 3, 0xFFFF }, // 192: imprezachassis
    { {135, 121,  95,   0}, 3, 0xFFFF }, // 193: imprezachassis
    { {135, 118, 121,   0}, 3, 0xFFFF }, // 194: imprezachassis
    { {135, 134, 118,   0}, 3, 0xFFFF }, // 195: imprezachassis
    { {132, 123, 102,   0}, 3, 0xFFFF }, // 196: imprezachassis
    { {105, 123, 132,   0}, 3, 0xFFFF }, // 197: imprezachassis
    { {105, 122, 123,   0}, 3, 0xFFFF }, // 198: imprezachassis
    { {104, 124, 125,   0}, 3, 0xFFFF }, // 199: imprezachassis
    { {124, 104, 103,   0}, 3, 0xFFFF }, // 200: imprezachassis
    { { 43,  82,  81,   0}, 3, 0x0008 }, // 201: imprezachassis
    { { 82,  43, 129,   0}, 3, 0xFFFF }, // 202: imprezachassis
    { { 52, 152,  33,   0}, 3, 0xFFFF }, // 203: imprezachassis
    { { 52,  92, 152,   0}, 3, 0xFFFF }, // 204: imprezachassis
    { { 52, 149,  92,   0}, 3, 0xFFFF }, // 205: imprezachassis
    { { 52,  90, 149,   0}, 3, 0xFFFF }, // 206: imprezachassis
    { { 90, 141, 111,   0}, 3, 0xFFFF }, // 207: imprezachassis
    { { 90, 143, 141,   0}, 3, 0xFFFF }, // 208: imprezachassis
    { { 52, 143,  90,   0}, 3, 0xFFFF }, // 209: imprezachassis
    { {126, 131, 127,   0}, 3, 0x0008 }, // 210: imprezachassis
    { { 66, 124,  65,   0}, 3, 0x0008 }, // 211: imprezachassis
    { {124,  66, 125,   0}, 3, 0x0008 }, // 212: imprezachassis
    { { 85, 126, 127,   0}, 3, 0xFFFF }, // 213: imprezachassis
    { {126,  85,  29,   0}, 3, 0xFFFF }, // 214: imprezachassis
    { { 82, 128, 130,   0}, 3, 0xFFFF }, // 215: imprezachassis
    { {128,  82, 129,   0}, 3, 0xFFFF }, // 216: imprezachassis
    { {127, 131,  86,   0}, 3, 0x0008 }, // 217: imprezachassis
    { {131, 126,  28,   0}, 3, 0x0008 }, // 218: imprezachassis
    { {138,  37, 139,   0}, 3, 0xFFFF }, // 219: imprezachassis
    { { 37, 138,  38,   0}, 3, 0xFFFF }, // 220: imprezachassis
    { { 94, 106, 110,   0}, 3, 0xFFFF }, // 221: imprezachassis
    { {106,  94,  93,   0}, 3, 0xFFFF }, // 222: imprezachassis
    { {119, 107, 115,   0}, 3, 0xFFFF }, // 223: imprezachassis
    { {107, 119, 120,   0}, 3, 0xFFFF }, // 224: imprezachassis
    { { 60,  48,  59,   0}, 3, 0xFFFF }, // 225: imprezachassis
    { {125, 105, 104,   0}, 3, 0xFFFF }, // 226: imprezachassis
    { {105, 125, 122,   0}, 3, 0xFFFF }, // 227: imprezachassis
    { { 66, 122, 125,   0}, 3, 0x0008 }, // 228: imprezachassis
    { {122,  66,  64,   0}, 3, 0x0008 }, // 229: imprezachassis
    { { 42,  65,  63,   0}, 3, 0xFFFF }, // 230: imprezachassis
    { { 65,  42,  44,   0}, 3, 0xFFFF }, // 231: imprezachassis
    { { 63, 124, 123,   0}, 3, 0x0008 }, // 232: imprezachassis
    { {124,  63,  65,   0}, 3, 0x0008 }, // 233: imprezachassis
    { {132, 101, 105,   0}, 3, 0xFFFF }, // 234: imprezachassis
    { {101, 132, 133,   0}, 3, 0xFFFF }, // 235: imprezachassis
    { {133,  98, 101,   0}, 3, 0xFFFF }, // 236: imprezachassis
    { { 98, 133, 134,   0}, 3, 0xFFFF }, // 237: imprezachassis
    { {134,  96,  98,   0}, 3, 0xFFFF }, // 238: imprezachassis
    { { 96, 134, 135,   0}, 3, 0xFFFF }, // 239: imprezachassis
    { { 42,  48,  26,   0}, 3, 0xFFFF }, // 240: imprezachassis
    { { 42, 137,  48,   0}, 3, 0xFFFF }, // 241: imprezachassis
    { {136, 137,  42,   0}, 3, 0xFFFF }, // 242: imprezachassis
    { { 48, 138,  59,   0}, 3, 0xFFFF }, // 243: imprezachassis
    { {138,  48, 137,   0}, 3, 0xFFFF }, // 244: imprezachassis
    { {138,  62,  59,   0}, 3, 0xFFFF }, // 245: imprezachassis
    { {138,  36,  62,   0}, 3, 0xFFFF }, // 246: imprezachassis
    { {138, 139,  36,   0}, 3, 0xFFFF }, // 247: imprezachassis
    { {108, 120, 118,   0}, 3, 0xFFFF }, // 248: imprezachassis
    { { 48,  60,  49,   0}, 3, 0xFFFF }, // 249: imprezachassis
    { {142, 141, 143,   0}, 3, 0xFFFF }, // 250: imprezachassis
    { {141, 142, 140,   0}, 3, 0xFFFF }, // 251: imprezachassis
    { {144, 147, 146,   0}, 3, 0xFFFF }, // 252: imprezachassis
    { {147, 144, 145,   0}, 3, 0xFFFF }, // 253: imprezachassis
    { { 92, 148,  91,   0}, 3, 0xFFFF }, // 254: imprezachassis
    { {148,  92, 149,   0}, 3, 0xFFFF }, // 255: imprezachassis
    { {148,  67,  91,   0}, 3, 0xFFFF }, // 256: imprezachassis
    { { 67, 148, 150,   0}, 3, 0xFFFF }, // 257: imprezachassis
    { {150,  68,  67,   0}, 3, 0xFFFF }, // 258: imprezachassis
    { { 68, 150, 151,   0}, 3, 0xFFFF }, // 259: imprezachassis
    { { 91, 152,  92,   0}, 3, 0xFFFF }, // 260: imprezachassis
    { {152,  91, 153,   0}, 3, 0xFFFF }, // 261: imprezachassis
    { { 67, 153,  91,   0}, 3, 0xFFFF }, // 262: imprezachassis
    { {153,  67, 154,   0}, 3, 0xFFFF }, // 263: imprezachassis
    { { 68, 154,  67,   0}, 3, 0xFFFF }, // 264: imprezachassis
    { {154,  68, 155,   0}, 3, 0xFFFF }, // 265: imprezachassis
    { { 69, 157,  70,   0}, 3, 0xFFFF }, // 266: imprezachassis
    { {157,  69, 156,   0}, 3, 0xFFFF }, // 267: imprezachassis
    { {159,  69,  70,   0}, 3, 0xFFFF }, // 268: imprezachassis
    { { 69, 159, 158,   0}, 3, 0xFFFF }, // 269: imprezachassis
    { {131, 109,  86,   0}, 3, 0x0008 }, // 270: imprezachassis
    { {109, 131, 160,   0}, 3, 0xFFFF }, // 271: imprezachassis
    { {164, 173, 172,   0}, 3, 0x0000 }, // 272: impreza_wheel.002
    { {172, 163, 164,   0}, 3, 0x0000 }, // 273: impreza_wheel.002
    { {166, 175, 174,   0}, 3, 0x0000 }, // 274: impreza_wheel.002
    { {174, 165, 166,   0}, 3, 0x0000 }, // 275: impreza_wheel.002
    { {168, 177, 176,   0}, 3, 0x0000 }, // 276: impreza_wheel.002
    { {176, 167, 168,   0}, 3, 0x0000 }, // 277: impreza_wheel.002
    { {169, 178, 177,   0}, 3, 0x0000 }, // 278: impreza_wheel.002
    { {177, 168, 169,   0}, 3, 0x0000 }, // 279: impreza_wheel.002
    { {161, 170, 178,   0}, 3, 0x0000 }, // 280: impreza_wheel.002
    { {178, 169, 161,   0}, 3, 0x0000 }, // 281: impreza_wheel.002
    { {163, 162, 161,   0}, 3, 0x0000 }, // 282: impreza_wheel.002
    { {161, 164, 163,   0}, 3, 0x0000 }, // 283: impreza_wheel.002
    { {164, 161, 169,   0}, 3, 0x0000 }, // 284: impreza_wheel.002
    { {169, 165, 164,   0}, 3, 0x0000 }, // 285: impreza_wheel.002
    { {165, 169, 168,   0}, 3, 0x0000 }, // 286: impreza_wheel.002
    { {168, 166, 165,   0}, 3, 0x0000 }, // 287: impreza_wheel.002
    { {167, 166, 168,   0}, 3, 0x0000 }, // 288: impreza_wheel.002
    { {170, 161, 162,   0}, 3, 0x0000 }, // 289: impreza_wheel.002
    { {162, 171, 170,   0}, 3, 0x0000 }, // 290: impreza_wheel.002
    { {171, 162, 163,   0}, 3, 0x0000 }, // 291: impreza_wheel.002
    { {163, 172, 171,   0}, 3, 0x0000 }, // 292: impreza_wheel.002
    { {173, 164, 165,   0}, 3, 0x0000 }, // 293: impreza_wheel.002
    { {165, 174, 173,   0}, 3, 0x0000 }, // 294: impreza_wheel.002
    { {175, 166, 167,   0}, 3, 0x0000 }, // 295: impreza_wheel.002
    { {167, 176, 175,   0}, 3, 0x0000 }, // 296: impreza_wheel.002
    { {172, 173, 170,   0}, 3, 0x0000 }, // 297: impreza_wheel.002
    { {170, 171, 172,   0}, 3, 0x0000 }, // 298: impreza_wheel.002
    { {173, 174, 178,   0}, 3, 0x0000 }, // 299: impreza_wheel.002
    { {178, 170, 173,   0}, 3, 0x0000 }, // 300: impreza_wheel.002
    { {174, 175, 177,   0}, 3, 0x0000 }, // 301: impreza_wheel.002
    { {177, 178, 174,   0}, 3, 0x0000 }, // 302: impreza_wheel.002
    { {176, 177, 175,   0}, 3, 0x0000 }, // 303: impreza_wheel.002
    { {182, 190, 191,   0}, 3, 0x0000 }, // 304: impreza_wheel.001
    { {190, 182, 181,   0}, 3, 0x0000 }, // 305: impreza_wheel.001
    { {184, 192, 193,   0}, 3, 0x0000 }, // 306: impreza_wheel.001
    { {192, 184, 183,   0}, 3, 0x0000 }, // 307: impreza_wheel.001
    { {186, 194, 195,   0}, 3, 0x0000 }, // 308: impreza_wheel.001
    { {194, 186, 185,   0}, 3, 0x0000 }, // 309: impreza_wheel.001
    { {187, 195, 196,   0}, 3, 0x0000 }, // 310: impreza_wheel.001
    { {195, 187, 186,   0}, 3, 0x0000 }, // 311: impreza_wheel.001
    { {179, 196, 188,   0}, 3, 0x0000 }, // 312: impreza_wheel.001
    { {196, 179, 187,   0}, 3, 0x0000 }, // 313: impreza_wheel.001
    { {181, 179, 180,   0}, 3, 0x0000 }, // 314: impreza_wheel.001
    { {179, 181, 182,   0}, 3, 0x0000 }, // 315: impreza_wheel.001
    { {182, 187, 179,   0}, 3, 0x0000 }, // 316: impreza_wheel.001
    { {187, 182, 183,   0}, 3, 0x0000 }, // 317: impreza_wheel.001
    { {183, 186, 187,   0}, 3, 0x0000 }, // 318: impreza_wheel.001
    { {186, 183, 184,   0}, 3, 0x0000 }, // 319: impreza_wheel.001
    { {185, 186, 184,   0}, 3, 0x0000 }, // 320: impreza_wheel.001
    { {188, 180, 179,   0}, 3, 0x0000 }, // 321: impreza_wheel.001
    { {180, 188, 189,   0}, 3, 0x0000 }, // 322: impreza_wheel.001
    { {189, 181, 180,   0}, 3, 0x0000 }, // 323: impreza_wheel.001
    { {181, 189, 190,   0}, 3, 0x0000 }, // 324: impreza_wheel.001
    { {191, 183, 182,   0}, 3, 0x0000 }, // 325: impreza_wheel.001
    { {183, 191, 192,   0}, 3, 0x0000 }, // 326: impreza_wheel.001
    { {193, 185, 184,   0}, 3, 0x0000 }, // 327: impreza_wheel.001
    { {185, 193, 194,   0}, 3, 0x0000 }, // 328: impreza_wheel.001
    { {190, 188, 191,   0}, 3, 0x0000 }, // 329: impreza_wheel.001
    { {188, 190, 189,   0}, 3, 0x0000 }, // 330: impreza_wheel.001
    { {191, 196, 192,   0}, 3, 0x0000 }, // 331: impreza_wheel.001
    { {196, 191, 188,   0}, 3, 0x0000 }, // 332: impreza_wheel.001
    { {192, 195, 193,   0}, 3, 0x0000 }, // 333: impreza_wheel.001
    { {195, 192, 196,   0}, 3, 0x0000 }, // 334: impreza_wheel.001
    { {194, 193, 195,   0}, 3, 0x0000 }, // 335: impreza_wheel.001
    { {200, 208, 209,   0}, 3, 0x0000 }, // 336: impreza_wheel.003
    { {208, 200, 199,   0}, 3, 0x0000 }, // 337: impreza_wheel.003
    { {202, 210, 211,   0}, 3, 0x0000 }, // 338: impreza_wheel.003
    { {210, 202, 201,   0}, 3, 0x0000 }, // 339: impreza_wheel.003
    { {204, 212, 213,   0}, 3, 0x0000 }, // 340: impreza_wheel.003
    { {212, 204, 203,   0}, 3, 0x0000 }, // 341: impreza_wheel.003
    { {205, 213, 214,   0}, 3, 0x0000 }, // 342: impreza_wheel.003
    { {213, 205, 204,   0}, 3, 0x0000 }, // 343: impreza_wheel.003
    { {197, 214, 206,   0}, 3, 0x0000 }, // 344: impreza_wheel.003
    { {214, 197, 205,   0}, 3, 0x0000 }, // 345: impreza_wheel.003
    { {199, 197, 198,   0}, 3, 0x0000 }, // 346: impreza_wheel.003
    { {197, 199, 200,   0}, 3, 0x0000 }, // 347: impreza_wheel.003
    { {200, 205, 197,   0}, 3, 0x0000 }, // 348: impreza_wheel.003
    { {205, 200, 201,   0}, 3, 0x0000 }, // 349: impreza_wheel.003
    { {201, 204, 205,   0}, 3, 0x0000 }, // 350: impreza_wheel.003
    { {204, 201, 202,   0}, 3, 0x0000 }, // 351: impreza_wheel.003
    { {203, 204, 202,   0}, 3, 0x0000 }, // 352: impreza_wheel.003
    { {206, 198, 197,   0}, 3, 0x0000 }, // 353: impreza_wheel.003
    { {198, 206, 207,   0}, 3, 0x0000 }, // 354: impreza_wheel.003
    { {207, 199, 198,   0}, 3, 0x0000 }, // 355: impreza_wheel.003
    { {199, 207, 208,   0}, 3, 0x0000 }, // 356: impreza_wheel.003
    { {209, 201, 200,   0}, 3, 0x0000 }, // 357: impreza_wheel.003
    { {201, 209, 210,   0}, 3, 0x0000 }, // 358: impreza_wheel.003
    { {211, 203, 202,   0}, 3, 0x0000 }, // 359: impreza_wheel.003
    { {203, 211, 212,   0}, 3, 0x0000 }, // 360: impreza_wheel.003
    { {208, 206, 209,   0}, 3, 0x0000 }, // 361: impreza_wheel.003
    { {206, 208, 207,   0}, 3, 0x0000 }, // 362: impreza_wheel.003
    { {209, 214, 210,   0}, 3, 0x0000 }, // 363: impreza_wheel.003
    { {214, 209, 206,   0}, 3, 0x0000 }, // 364: impreza_wheel.003
    { {210, 213, 211,   0}, 3, 0x0000 }, // 365: impreza_wheel.003
    { {213, 210, 214,   0}, 3, 0x0000 }, // 366: impreza_wheel.003
    { {212, 211, 213,   0}, 3, 0x0000 } // 367: impreza_wheel.003
};

#define LOD_CAR_NUM_VERTICES 32
#define LOD_CAR_NUM_FACES 23

const Point3D lod_car_vertices[LOD_CAR_NUM_VERTICES] = {
    { -0.6f,  0.1f,  1.4f }, { -0.5f,  0.4f,  1.2f }, { -0.5f,  0.45f,  0.5f }, { -0.4f,  0.8f,   0.1f },
    { -0.4f,  0.75f, -0.6f }, { -0.5f,  0.5f, -1.2f }, { -0.6f,  0.2f,  -1.3f }, { -0.6f,  0.1f,  -1.3f },
    {  0.6f,  0.1f,  1.4f }, {  0.5f,  0.4f,  1.2f }, {  0.5f,  0.45f,  0.5f }, {  0.4f,  0.8f,   0.1f },
    {  0.4f,  0.75f, -0.6f }, {  0.5f,  0.5f, -1.2f }, {  0.6f,  0.2f,  -1.3f }, {  0.6f,  0.1f,  -1.3f },
    { -0.65f, -0.05f,  1.1f }, { -0.65f,  0.25f,  1.1f }, { -0.65f,  0.25f,  0.5f }, { -0.65f, -0.05f,  0.5f },
    { -0.65f, -0.05f, -0.6f }, { -0.65f,  0.25f, -0.6f }, { -0.65f,  0.25f, -1.2f }, { -0.65f, -0.05f, -1.2f },
    {  0.65f, -0.05f,  1.1f }, {  0.65f,  0.25f,  1.1f }, {  0.65f,  0.25f,  0.5f }, {  0.65f, -0.05f,  0.5f },
    {  0.65f, -0.05f, -0.6f }, {  0.65f,  0.25f, -0.6f }, {  0.65f,  0.25f, -1.2f }, {  0.65f, -0.05f, -1.2f }
};

const Face lod_car_faces[LOD_CAR_NUM_FACES] = {
    { {0, 8, 9, 1}, 4, 0x3186 }, { {1, 9, 10, 2}, 4, 0xFFFF }, { {2, 10, 11, 3}, 4, 0x0008 },
    { {3, 11, 12, 4}, 4, 0xFFFF }, { {4, 12, 13, 5}, 4, 0x0008 }, { {5, 13, 14, 6}, 4, 0xFFFF },
    { {6, 14, 15, 7}, 4, 0x3186 }, { {0, 1, 2, 0}, 3, 0xFFFF }, { {2, 3, 4, 0}, 3, 0xFFFF },
    { {4, 5, 6, 0}, 3, 0xFFFF }, { {0, 2, 4, 0}, 3, 0xFFFF }, { {0, 4, 6, 0}, 3, 0xFFFF },
    { {0, 6, 7, 0}, 3, 0x18C3 }, { {8, 10, 9, 0}, 3, 0xFFFF }, { {10, 12, 11, 0}, 3, 0xFFFF },
    { {12, 14, 13, 0}, 3, 0xFFFF }, { {8, 12, 10, 0}, 3, 0xFFFF }, { {8, 14, 12, 0}, 3, 0xFFFF },
    { {8, 15, 14, 0}, 3, 0x18C3 }, { {16, 17, 18, 19}, 4, 0x0000 }, { {20, 21, 22, 23}, 4, 0x0000 },
    { {24, 25, 26, 27}, 4, 0x0000 }, { {28, 29, 30, 31}, 4, 0x0000 }
};

#define TREE_NUM_VERTICES 13
#define TREE_NUM_FACES 9

const Point3D tree_vertices[TREE_NUM_VERTICES] = {
    // Trunk (0 to 7)
    { -0.12f, 0.0f, -0.12f }, // 0
    {  0.12f, 0.0f, -0.12f }, // 1
    {  0.12f, 0.0f,  0.12f }, // 2
    { -0.12f, 0.0f,  0.12f }, // 3
    { -0.12f, 0.8f, -0.12f }, // 4
    {  0.12f, 0.8f, -0.12f }, // 5
    {  0.12f, 0.8f,  0.12f }, // 6
    { -0.12f, 0.8f,  0.12f }, // 7
    
    // Foliage (8 to 12)
    { -0.7f, 0.8f, -0.7f }, // 8
    {  0.7f, 0.8f, -0.7f }, // 9
    {  0.7f, 0.8f,  0.7f }, // 10
    { -0.7f, 0.8f,  0.7f }, // 11
    {  0.0f, 2.8f,  0.0f }  // 12: Peak
};

const Face tree_faces[TREE_NUM_FACES] = {
    // Trunk sides
    { {0, 1, 5, 4}, 4, 0x51E0 }, // Bark Brown
    { {1, 2, 6, 5}, 4, 0x51E0 },
    { {2, 3, 7, 6}, 4, 0x51E0 },
    { {3, 0, 4, 7}, 4, 0x51E0 },
    
    // Foliage base
    { {8, 11, 10, 9}, 4, 0x04A0 }, // Dark green
    
    // Foliage sides
    { {8, 9, 12}, 3, 0x0640 },   // Mid green
    { {9, 10, 12}, 3, 0x0640 },
    { {10, 11, 12}, 3, 0x0640 },
    { {11, 8, 12}, 3, 0x0640 }
};

#define BILLBOARD_NUM_VERTICES 8
#define BILLBOARD_NUM_FACES 1

const Point3D billboard_vertices[BILLBOARD_NUM_VERTICES] = {
    // Left Post (0 to 1)
    { -1.5f, 0.0f, 0.0f },
    { -1.5f, 1.4f, 0.0f },
    // Right Post (2 to 3)
    {  1.5f, 0.0f, 0.0f },
    {  1.5f, 1.4f, 0.0f },
    // Board (4 to 7)
    { -1.8f, 1.3f, 0.0f }, // 4: Bottom-Left
    {  1.8f, 1.3f, 0.0f }, // 5: Bottom-Right
    {  1.8f, 2.4f, 0.0f }, // 6: Top-Right
    { -1.8f, 2.4f, 0.0f }  // 7: Top-Left
};

const Face billboard_faces[BILLBOARD_NUM_FACES] = {
    { {4, 5, 6, 7}, 4, 0xFFFF } // Board Face (uses base color)
};

#define BRIDGE_NUM_VERTICES 24
#define BRIDGE_NUM_FACES 12

const Point3D bridge_vertices[BRIDGE_NUM_VERTICES] = {
    // Left Pillar (0 to 7)
    { -3.4f, 0.0f, -0.4f }, // 0
    { -2.9f, 0.0f, -0.4f }, // 1
    { -2.9f, 3.5f, -0.4f }, // 2
    { -3.4f, 3.5f, -0.4f }, // 3
    { -3.4f, 0.0f,  0.4f }, // 4
    { -2.9f, 0.0f,  0.4f }, // 5
    { -2.9f, 3.5f,  0.4f }, // 6
    { -3.4f, 3.5f,  0.4f }, // 7
    
    // Right Pillar (8 to 15)
    {  2.9f, 0.0f, -0.4f }, // 8
    {  3.4f, 0.0f, -0.4f }, // 9
    {  3.4f, 3.5f, -0.4f }, // 10
    {  2.9f, 3.5f, -0.4f }, // 11
    {  2.9f, 0.0f,  0.4f }, // 12
    {  3.4f, 0.0f,  0.4f }, // 13
    {  3.4f, 3.5f,  0.4f }, // 14
    {  2.9f, 3.5f,  0.4f }, // 15
    
    // Cross Beam (16 to 23)
    { -3.4f, 3.5f, -0.4f }, // 16 (same as 3)
    {  3.4f, 3.5f, -0.4f }, // 17 (same as 10)
    {  3.4f, 4.3f, -0.4f }, // 18
    { -3.4f, 4.3f, -0.4f }, // 19
    { -3.4f, 3.5f,  0.4f }, // 20 (same as 7)
    {  3.4f, 3.5f,  0.4f }, // 21 (same as 14)
    {  3.4f, 4.3f,  0.4f }, // 22
    { -3.4f, 4.3f,  0.4f }  // 23
};

const Face bridge_faces[BRIDGE_NUM_FACES] = {
    // Left Pillar sides
    { {0, 1, 2, 3}, 4, 0x5AEB }, // Light grey
    { {4, 7, 6, 5}, 4, 0x5AEB },
    { {0, 4, 7, 3}, 4, 0x3186 }, // Dark grey
    { {1, 5, 6, 2}, 4, 0x3186 },
    
    // Right Pillar sides
    { {8, 9, 10, 11}, 4, 0x5AEB },
    { {12, 15, 14, 13}, 4, 0x5AEB },
    { {8, 12, 15, 11}, 4, 0x3186 },
    { {9, 13, 14, 10}, 4, 0x3186 },
    
    // Cross Beam sides
    { {16, 17, 18, 19}, 4, 0xF800 }, // Red front
    { {20, 23, 22, 21}, 4, 0xF800 }, // Red back
    { {19, 18, 22, 23}, 4, 0xFBE0 }, // Orange top
    { {16, 20, 21, 17}, 4, 0x5AEB }  // Grey bottom
};

const Face gantry_faces[BRIDGE_NUM_FACES] = {
    // Left Pillar sides
    { {0, 1, 2, 3}, 4, 0xFFFF },
    { {4, 7, 6, 5}, 4, 0xFFFF },
    { {0, 4, 7, 3}, 4, 0x3186 },
    { {1, 5, 6, 2}, 4, 0x3186 },
    
    // Right Pillar sides
    { {8, 9, 10, 11}, 4, 0xFFFF },
    { {12, 15, 14, 13}, 4, 0xFFFF },
    { {8, 12, 15, 11}, 4, 0x3186 },
    { {9, 13, 14, 10}, 4, 0x3186 },
    
    // Finish beam
    { {16, 17, 18, 19}, 4, 0xFFFF },
    { {20, 23, 22, 21}, 4, 0xFFFF },
    { {19, 18, 22, 23}, 4, 0x0000 },
    { {16, 20, 21, 17}, 4, 0x5AEB }
};

// Player State Variables
float player_segment_float = 0.0f;
float player_w = 0.0f;          // Lateral offset (-2.5 to 2.5 on road)
float player_speed = 0.0f;      // km/h
float player_roll = 0.0f;       // Visual tilt when turning
float player_steer_angle = 0.0f;// Front wheel steer angle
float wheel_spin_angle = 0.0f;  // Spin value
int player_laps = 0;

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
    int lane_change_timer;
    int laps;
};

Opponent opponents[2] = {
    { 2.0f, -0.9f, 0.0f, 0xFFE0, -0.9f, 0, 0 }, // Yellow
    { 3.0f,  0.9f, 0.0f, 0x07FF,  0.9f, 0, 0 }  // Cyan
};

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
void draw3DModel(const Point3D* vertices, int num_vertices, const Face* faces, int num_faces, 
                 float pos_x, float pos_y, float pos_z, 
                 float rot_x, float rot_y, float rot_z, 
                 float scale, uint16_t base_color);
void drawCarRearGlass(float pos_x, float pos_y, float pos_z,
                      float rot_x, float rot_y, float rot_z,
                      float scale);
void drawBillboard(float pos_x, float pos_y, float pos_z, float rot_y, float scale, uint16_t color);
void drawTreeImpostor(float pos_x, float pos_y, float pos_z, float scale);
void drawOpponentBrakeLights(float pos_x, float pos_y, float pos_z, float yaw, bool braking);
void drawStartFinishGantry(int seg);
void drawTracksideLake(int seg);
void drawSegmentScenery(int seg);
void drawSegmentOpponents(int seg);
bool projectPoint(float wx, float wy, float wz, float& sx, float& sy, float& sz);
void getTrackPosition(float seg_float, float lateral_pos, float& out_x, float& out_y, float& out_z, float& out_rx, float& out_rz);
void cameraRelativeToWorld(float local_x, float local_y, float local_z, float& out_x, float& out_y, float& out_z);
void updateCameraTrig();
float stableJitter(int seg, int side, int index, int salt);
float approachFloat(float current, float target, float response, float dt);
uint16_t shadeColor(uint16_t color, float intensity);

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
    
    // Initialize Double Buffer Sprite
    sprite.setColorDepth(16);
    sprite.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);
    sprite.setSwapBytes(true);
    
    // Procedurally Generate Track
    generateTrack();
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
    
    // Push the frame buffer to the physical screen
    unsigned long before_push_us = micros();
    sprite.pushSprite(0, 0);
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
    
    SegmentDef track_layout[] = {
        { 40,  0.0f,      0.0f },    // Straight
        { 40,  0.03927f,  0.0f },    // 90 Right
        { 30,  0.0f,      0.0f },    // Straight
        { 40, -0.03927f,  0.0f },    // 90 Left
        { 50,  0.0f,      0.0f },    // Straight
        { 40,  0.03927f,  0.0f },    // 90 Right
        { 30,  0.0f,      0.0f },    // Straight
        { 40,  0.07854f,  0.0f },    // 180 Right
        { 50,  0.0f,      0.0f },    // Straight
        { 40,  0.03927f,  0.0f }     // 90 Right
    };
    
    int num_defs = sizeof(track_layout) / sizeof(SegmentDef);
    
    for (int l = 0; l < num_defs; l++) {
        for (int s = 0; s < track_layout[l].length; s++) {
            if (seg_count >= NUM_SEGMENTS) break;
            
            heading += track_layout[l].curve;
            elevation += track_layout[l].pitch;
            
            x += sin(heading) * SEGMENT_LENGTH;
            z += cos(heading) * SEGMENT_LENGTH;
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
    float err_x = track[seg_count-1].x + sin(heading)*SEGMENT_LENGTH - track[0].x;
    float err_y = track[seg_count-1].y + elevation*SEGMENT_LENGTH - track[0].y;
    float err_z = track[seg_count-1].z + cos(heading)*SEGMENT_LENGTH - track[0].z;
    
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
        float len = sqrt(fx*fx + fy*fy + fz*fz);
        
        track[i].fx = fx / len;
        track[i].fy = fy / len;
        track[i].fz = fz / len;
        
        // Right vector is perpendicular in the horizontal plane
        float rx = fz;
        float rz = -fx;
        float len_r = sqrt(rx*rx + rz*rz);
        
        track[i].rx = rx / len_r;
        track[i].rz = rz / len_r;
    }
}

void resetRaceState() {
    player_segment_float = 0.0f;
    player_w = 0.0f;
    player_speed = 0.0f;
    player_roll = 0.0f;
    player_steer_angle = 0.0f;
    wheel_spin_angle = 0.0f;
    player_laps = 0;
    race_start_ms = 0;
    race_finish_ms = 0;
    current_lap_ms = 0;
    screen_shake_timer = 0;
    
    opponents[0] = Opponent{ 2.0f, -0.9f, 0.0f, 0xFFE0, -0.9f, 0, 0 };
    opponents[1] = Opponent{ 3.0f,  0.9f, 0.0f, 0x07FF,  0.9f, 0, 0 };
}

// Physics Loop
void updatePhysics(float dt) {
    if (current_state == PLAYING) {
        // Read active-low buttons
        bool steer_left = (digitalRead(0) == LOW);
        bool steer_right = (digitalRead(14) == LOW);
        
        // Speed Auto-Acceleration and Friction
        bool off_road = (fabsf(player_w) > 2.5f);
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
        
        // Centrifugal curve physics
        int p_seg = (int)player_segment_float % NUM_SEGMENTS;
        float curve = track[p_seg].curve;
        float force = curve * (player_speed / 50.0f) * (player_speed / 50.0f) * 0.14f;
        player_w -= force;
        
        // Crash boundary checking
        if (player_w > 3.8f) {
            player_w = 3.8f;
            player_speed *= 0.5f; // lose speed
            screen_shake_timer = 12; // shake camera
        } else if (player_w < -3.8f) {
            player_w = -3.8f;
            player_speed *= 0.5f;
            screen_shake_timer = 12;
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
        
        if (millis() - countdown_start_ms > 4000) {
            current_state = PLAYING;
            lap_start_ms = millis();
            race_start_ms = lap_start_ms;
            race_finish_ms = 0;
        }
    }
    
    // Update Opponent AI
    if (current_state == PLAYING) {
        for (int i = 0; i < 2; i++) {
            Opponent& opp = opponents[i];
        
            opp.segment += (opp.speed / 3.6f) * dt / SEGMENT_LENGTH;
            if (opp.segment >= NUM_SEGMENTS) {
                opp.segment -= NUM_SEGMENTS;
                opp.laps++;
            }
            
            // AI lane changing timer
            if (opp.lane_change_timer <= 0) {
                opp.target_lateral = random(-14, 15) / 10.0f; // lane position [-1.4, 1.4]
                opp.lane_change_timer = random(100, 300);
            } else {
                opp.lane_change_timer--;
            }
            opp.lateral_pos += (opp.target_lateral - opp.lateral_pos) * 0.02f;
            
            // AI deceleration on curves
            int opp_seg = (int)opp.segment % NUM_SEGMENTS;
            float curve = fabsf(track[opp_seg].curve);
            float base_speed = (i == 0) ? 175.0f : 168.0f;
            if (curve > 0.015f) {
                opp.speed += (base_speed - 38.0f - opp.speed) * 0.065f; // slow down
            } else {
                opp.speed += (base_speed - opp.speed) * 0.065f; // speed up
            }
            
            // Collision: Player vs Opponent
            float dist_seg = fabsf(player_segment_float - opp.segment);
            if (dist_seg > NUM_SEGMENTS / 2.0f) dist_seg = NUM_SEGMENTS - dist_seg;
            
            if (dist_seg < 0.6f) { // close in track segment
                float dist_w = fabsf(player_w - opp.lateral_pos);
                if (dist_w < 0.7f) { // close laterally
                    player_speed *= 0.75f; // player loses speed
                    opp.speed *= 0.8f;     // opponent loses speed
                    screen_shake_timer = 8;
                    
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
    
    cam_yaw = atan2(dx, dz);
    cam_pitch = atan2(dy, sqrt(dx*dx + dz*dz));
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
    
    // Rotate world light vector to camera space
    float lx1 = light_dir_x * cam_cos_yaw - light_dir_z * cam_sin_yaw;
    float lz1 = light_dir_x * cam_sin_yaw + light_dir_z * cam_cos_yaw;
    float ly1 = light_dir_y;
    cam_light_x = lx1;
    cam_light_y = ly1 * cam_cos_pitch - lz1 * cam_sin_pitch;
    cam_light_z = ly1 * cam_sin_pitch + lz1 * cam_cos_pitch;
    
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
    }
    
    for (int i = ROAD_DRAW_SEGMENTS; i >= 0; i--) {
        int seg = (start_i + i) % NUM_SEGMENTS;
        RoadProjection& near_p = road_proj[i];
        RoadProjection& far_p = road_proj[i + 1];
        
        // Colors
        uint16_t road_color = track[seg].is_alternate ? 0x39E7 : 0x4208; // alternating greys
        uint16_t curb_color = track[seg].is_alternate ? 0xF800 : 0xFFFF; // alternating red/white
        
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
                     far_p.line_r_x, far_p.line_r_y, far_p.line_l_x, far_p.line_l_y, 0xFFFF);
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
                drawQuad(n0x, n0y, n1x, n1y, f1x, f1y, f0x, f0y, (c & 1) ? 0x0000 : 0xFFFF);
            }
        }
        
        if (i <= SCENERY_DRAW_SEGMENTS) {
            // Keep object draw distance unchanged while testing a longer road horizon.
            drawSegmentScenery(seg);
            drawSegmentOpponents(seg);
        }
    }
    
    // Render Player Car in the foreground last (forces it on top)
    if (current_state == PLAYING || current_state == COUNTDOWN || current_state == FINISHED) {
        float p_yaw = cam_yaw; // Lock to camera for solid 3D perspective
        
        // Bouncing hover/engine vibration
        float bounce_y = 0.0f;
        if (player_speed > 0.0f && current_state == PLAYING) {
            bounce_y = 0.018f * sin(millis() * 0.065f * (player_speed / MAX_SPEED + 0.3f));
        }
        
        float p_x, p_y, p_z;
        cameraRelativeToWorld(0.0f, PLAYER_CAR_CAMERA_Y + bounce_y, PLAYER_CAR_CAMERA_Z, p_x, p_y, p_z);
        
        // Draw body
        draw3DModel(car_vertices, CAR_NUM_VERTICES, car_faces, CAR_NUM_FACES,
                    p_x, p_y, p_z,
                    0.0f, p_yaw + player_steer_angle, player_roll,
                    1.0f, 0x021F); // Subaru blue
        drawCarRearGlass(p_x, p_y, p_z,
                         0.0f, p_yaw + player_steer_angle, player_roll,
                         1.0f);
    }
}

// Side of Road Scenery Placement
void drawSegmentScenery(int seg) {
    float side_offset = 3.8f;
    float bill_offset = 4.2f;
    bool forest_zone = (seg >= 68 && seg < 112) ||
                       (seg >= 186 && seg < 232) ||
                       (seg >= 304 && seg < 344);
    
    if (seg == 0) {
        drawStartFinishGantry(seg);
    }
    if (seg == 252) {
        drawTracksideLake(seg);
    }
    
    // Draw Bridge Arch (every 40 segments)
    if (seg % 40 == 20) {
        draw3DModel(bridge_vertices, BRIDGE_NUM_VERTICES, bridge_faces, BRIDGE_NUM_FACES,
                    track[seg].x, track[seg].y, track[seg].z,
                    0.0f, atan2(track[seg].fx, track[seg].fz), 0.0f,
                    1.0f, 0x5AEB);
    }
    
    // Draw 3D Grass
    if (seg % 2 == 0) {
        static const float grass_offsets[] = { 1.2f, 2.5f, 4.0f };
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
                        sprite.drawLine((int16_t)sx, (int16_t)sy, (int16_t)sx - h/2, (int16_t)sy - h, 0x2D84);
                        sprite.drawLine((int16_t)sx, (int16_t)sy, (int16_t)sx + h/2, (int16_t)sy - h, 0x2D84);
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
        
        draw3DModel(tree_vertices, TREE_NUM_VERTICES, tree_faces, TREE_NUM_FACES,
                    tx, ty, tz,
                    0.0f, 0.0f, 0.0f,
                    0.9f + 0.15f * sin(seg), 0x04C0);
    }
    
    // Draw Tree Right (every 8 segments, offset by 4)
    if (seg % 8 == 4) {
        float tx = track[seg].x + track[seg].rx * side_offset;
        float ty = track[seg].y;
        float tz = track[seg].z + track[seg].rz * side_offset;
        
        draw3DModel(tree_vertices, TREE_NUM_VERTICES, tree_faces, TREE_NUM_FACES,
                    tx, ty, tz,
                    0.0f, 0.0f, 0.0f,
                    0.9f + 0.15f * cos(seg), 0x04C0);
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
                
                drawTreeImpostor(tx, ty, tz, scale);
            }
        }
    }
    
    // Draw Billboard Left (every 16 segments)
    if (seg % 16 == 5) {
        float bx = track[seg].x - track[seg].rx * bill_offset;
        float by = track[seg].y;
        float bz = track[seg].z - track[seg].rz * bill_offset;
        float rot_y = atan2(track[seg].fx, track[seg].fz) + 1.5707f; // face road
        
        drawBillboard(bx, by, bz, rot_y, 0.8f, 0x001F); // Blue
    }
    
    // Draw Billboard Right (every 16 segments)
    if (seg % 16 == 13) {
        float bx = track[seg].x + track[seg].rx * bill_offset;
        float by = track[seg].y;
        float bz = track[seg].z + track[seg].rz * bill_offset;
        float rot_y = atan2(track[seg].fx, track[seg].fz) - 1.5707f; // face road
        
        drawBillboard(bx, by, bz, rot_y, 0.8f, 0xF800); // Red
    }
}

void drawStartFinishGantry(int seg) {
    float yaw = atan2(track[seg].fx, track[seg].fz);
    draw3DModel(bridge_vertices, BRIDGE_NUM_VERTICES, gantry_faces, BRIDGE_NUM_FACES,
                track[seg].x, track[seg].y, track[seg].z,
                0.0f, yaw, 0.0f,
                1.05f, 0xFFFF);
}

void drawTracksideLake(int seg) {
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
    if (center_sz <= 2.0f || center_sz > 52.0f) return;
    
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
    
    for (int i = 0; i < 8; i++) {
        int next = (i + 1) & 7;
        sprite.fillTriangle((int16_t)center_sx, (int16_t)center_sy,
                            (int16_t)shore[i].x, (int16_t)shore[i].y,
                            (int16_t)shore[next].x, (int16_t)shore[next].y,
                            0xA5A6);
    }
    for (int i = 0; i < 8; i++) {
        int next = (i + 1) & 7;
        uint16_t water_color = (i & 1) ? 0x04BF : 0x039F;
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
void drawSegmentOpponents(int seg) {
    for (int i = 0; i < 2; i++) {
        int opp_seg = (int)opponents[i].segment % NUM_SEGMENTS;
        if (opp_seg == seg) {
            float ox, oy, oz, orx, orz;
            getTrackPosition(opponents[i].segment, opponents[i].lateral_pos, ox, oy, oz, orx, orz);
            
            float opp_yaw = atan2(track[opp_seg].fx, track[opp_seg].fz);
            
            draw3DModel(lod_car_vertices, LOD_CAR_NUM_VERTICES, lod_car_faces, LOD_CAR_NUM_FACES,
                        ox, oy, oz,
                        0.0f, opp_yaw, 0.0f,
                        1.0f, opponents[i].color);
            
            bool braking = fabsf(track[opp_seg].curve) > 0.015f;
            drawOpponentBrakeLights(ox, oy, oz, opp_yaw, braking);
        }
    }
}

// 3D Object Renderer (Rotation + Scaling + Camera view matrix + Projection)
void draw3DModel(const Point3D* vertices, int num_vertices, const Face* faces, int num_faces, 
                 float pos_x, float pos_y, float pos_z, 
                 float rot_x, float rot_y, float rot_z, 
                 float scale, uint16_t base_color) {
    static Point2D projected[MODEL_MAX_VERTICES];
    static Point3D cam_space[MODEL_MAX_VERTICES];
    static float camera_z[MODEL_MAX_VERTICES];
    
    if (num_vertices > MODEL_MAX_VERTICES) return;
    bool fast_complex_shading = (num_faces > 96);
    
    float cx = cos(rot_x), sx = sin(rot_x);
    float cy = cos(rot_y), sy = sin(rot_y);
    float cz = cos(rot_z), sz = sin(rot_z);
    
    float c_cam_yaw = cam_cos_yaw, s_cam_yaw = cam_sin_yaw;
    float c_cam_pitch = cam_cos_pitch, s_cam_pitch = cam_sin_pitch;
    
    for (int i = 0; i < num_vertices; i++) {
        float lx = vertices[i].x * scale;
        float ly = vertices[i].y * scale;
        float lz = vertices[i].z * scale;
        
        // Z-axis Roll
        float x1 = lx * cz - ly * sz;
        float y1 = lx * sz + ly * cz;
        float z1 = lz;
        
        // X-axis Pitch
        float x2 = x1;
        float y2 = y1 * cx - z1 * sx;
        float z2 = y1 * sx + z1 * cx;
        
        // Y-axis Yaw
        float rx_world = x2 * cy + z2 * sy;
        float ry_world = y2;
        float rz_world = -x2 * sy + z2 * cy;
        
        // Translate to World Space
        float wx = rx_world + pos_x;
        float wy = ry_world + pos_y;
        float wz = rz_world + pos_z;
        
        // Camera Space Offset
        float cx_cam = wx - cam_x;
        float cy_cam = wy - cam_y;
        float cz_cam = wz - cam_z;
        
        // Camera Yaw
        float rx_cam1 = cx_cam * c_cam_yaw - cz_cam * s_cam_yaw;
        float rz_cam1 = cx_cam * s_cam_yaw + cz_cam * c_cam_yaw;
        float ry_cam1 = cy_cam;
        
        // Camera Pitch
        float rx_cam = rx_cam1;
        float ry_cam = ry_cam1 * c_cam_pitch - rz_cam1 * s_cam_pitch;
        float rz_cam = ry_cam1 * s_cam_pitch + rz_cam1 * c_cam_pitch;
        
        camera_z[i] = rz_cam;
        cam_space[i] = { rx_cam, ry_cam, rz_cam };
        
        if (rz_cam > 0.1f) {
            projected[i].x = center_x + (rx_cam * fov / rz_cam);
            projected[i].y = center_y - (ry_cam * fov / rz_cam);
        } else {
            projected[i].x = -9999.0f;
            projected[i].y = -9999.0f;
        }
    }
    
    struct FaceRenderData {
        int index;
        float avg_z;
        bool behind;
    };
    static FaceRenderData face_data[MODEL_MAX_FACES];
    if (num_faces > MODEL_MAX_FACES) return;
    
    for (int i = 0; i < num_faces; i++) {
        const Face& f = faces[i];
        bool behind = false;
        float sum_z = 0.0f;
        for (int v = 0; v < f.num_vertices; v++) {
            float z = camera_z[f.indices[v]];
            if (z <= 0.1f) behind = true;
            sum_z += z;
        }
        face_data[i] = { i, sum_z / f.num_vertices, behind };
    }
    
    // Painter's sort: furthest faces first.
    for (int gap = num_faces / 2; gap > 0; gap /= 2) {
        for (int i = gap; i < num_faces; i++) {
            FaceRenderData temp = face_data[i];
            int j = i;
            while (j >= gap && face_data[j - gap].avg_z < temp.avg_z) {
                face_data[j] = face_data[j - gap];
                j -= gap;
            }
            face_data[j] = temp;
        }
    }
    
    // Render Faces
    for (int fi = 0; fi < num_faces; fi++) {
        if (face_data[fi].behind) continue;
        int i = face_data[fi].index;
        const Face& f = faces[i];
        
        int i0 = f.indices[0];
        int i1 = f.indices[1];
        int i2 = f.indices[2];
        uint16_t color_to_use = (f.color == 0xFFFF) ? base_color : f.color;
        
        // Normal computation in camera space
        float ax = cam_space[i1].x - cam_space[i0].x;
        float ay = cam_space[i1].y - cam_space[i0].y;
        float az = cam_space[i1].z - cam_space[i0].z;
        
        float bx = cam_space[i2].x - cam_space[i0].x;
        float by = cam_space[i2].y - cam_space[i0].y;
        float bz = cam_space[i2].z - cam_space[i0].z;
        
        float nx = ay * bz - az * by;
        float ny = az * bx - ax * bz;
        float nz = ax * by - ay * bx;
        
        if (nz > 0.0f) { nx = -nx; ny = -ny; nz = -nz; } // Point towards camera
        
        float intensity;
        if (fast_complex_shading) {
            float facing = -nz / (fabsf(nx) + fabsf(ny) + fabsf(nz) + 0.001f);
            if (facing > 1.0f) facing = 1.0f;
            if (facing < 0.0f) facing = 0.0f;
            intensity = 0.70f + 0.30f * facing;
        } else {
            float len = sqrt(nx*nx + ny*ny + nz*nz);
            if (len > 0.0f) {
                nx /= len; ny /= len; nz /= len;
            }
            
            // Dot product flat lighting
            float dot = nx * cam_light_x + ny * cam_light_y + nz * cam_light_z;
            intensity = 0.66f + 0.34f * dot; // high noon ambient + diffuse
            if (intensity > 1.0f) intensity = 1.0f;
            if (intensity < 0.45f) intensity = 0.45f;
        }
        uint16_t shaded = shadeColor(color_to_use, intensity);
        
        if (f.num_vertices == 3) {
            sprite.fillTriangle((int16_t)projected[i0].x, (int16_t)projected[i0].y,
                                (int16_t)projected[i1].x, (int16_t)projected[i1].y,
                                (int16_t)projected[i2].x, (int16_t)projected[i2].y,
                                shaded);
        } else if (f.num_vertices == 4) {
            int i3 = f.indices[3];
            sprite.fillTriangle((int16_t)projected[i0].x, (int16_t)projected[i0].y,
                                (int16_t)projected[i1].x, (int16_t)projected[i1].y,
                                (int16_t)projected[i2].x, (int16_t)projected[i2].y,
                                shaded);
            sprite.fillTriangle((int16_t)projected[i0].x, (int16_t)projected[i0].y,
                                (int16_t)projected[i2].x, (int16_t)projected[i2].y,
                                (int16_t)projected[i3].x, (int16_t)projected[i3].y,
                                shaded);
        }
    }
}

void drawCarRearGlass(float pos_x, float pos_y, float pos_z,
                      float rot_x, float rot_y, float rot_z,
                      float scale) {
    static const Point3D rear_glass[4] = {
        { -0.42f, 1.025f, -0.79f },
        {  0.42f, 1.025f, -0.79f },
        {  0.24f, 0.742f, -1.275f },
        { -0.24f, 0.742f, -1.275f }
    };
    
    Point2D projected[4];
    
    float cx = cos(rot_x), sx = sin(rot_x);
    float cy = cos(rot_y), sy = sin(rot_y);
    float cz = cos(rot_z), sz = sin(rot_z);
    
    for (int i = 0; i < 4; i++) {
        float lx = rear_glass[i].x * scale;
        float ly = rear_glass[i].y * scale;
        float lz = rear_glass[i].z * scale;
        
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
}

void drawTreeImpostor(float pos_x, float pos_y, float pos_z, float scale) {
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
    
    sprite.fillRect(x - trunk_w / 2, leaf_base_y, trunk_w, trunk_h, 0x3920);
    sprite.fillTriangle(x - half_w, leaf_base_y,
                        x + half_w, leaf_base_y,
                        x, top_y,
                        0x0340);
    sprite.fillTriangle(x - half_w * 3 / 4, leaf_base_y - h / 4,
                        x + half_w * 3 / 4, leaf_base_y - h / 4,
                        x, top_y + h / 6,
                        0x04C0);
}

void drawOpponentBrakeLights(float pos_x, float pos_y, float pos_z, float yaw, bool braking) {
    float cy = cos(yaw);
    float sy = sin(yaw);
    
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
void drawBillboard(float pos_x, float pos_y, float pos_z, float rot_y, float scale, uint16_t color) {
    // 1. Draw the board face
    draw3DModel(billboard_vertices, 8, billboard_faces, BILLBOARD_NUM_FACES,
                pos_x, pos_y, pos_z,
                0.0f, rot_y, 0.0f,
                scale, color);
                
    // 2. Draw support posts as lines (simpler, faster)
    float c_y = cos(rot_y), s_y = sin(rot_y);
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
    if (projectPoint(lp0_x, lp0_y, lp0_z, sx0, sy0, sz0) && projectPoint(lp1_x, lp1_y, lp1_z, sx1, sy1, sz1)) {
        sprite.drawLine((int16_t)sx0, (int16_t)sy0, (int16_t)sx1, (int16_t)sy1, 0x4A69);
    }
    if (projectPoint(rp0_x, rp0_y, rp0_z, sx0, sy0, sz0) && projectPoint(rp1_x, rp1_y, rp1_z, sx1, sy1, sz1)) {
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
    float len = sqrt(out_rx*out_rx + out_rz*out_rz);
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
    cam_cos_yaw = cos(cam_yaw);
    cam_sin_yaw = sin(cam_yaw);
    cam_cos_pitch = cos(cam_pitch);
    cam_sin_pitch = sin(cam_pitch);
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

// Render background (Gradient sky, sun, scrolling mountains, green grass)
void drawBackground(int horizon_y, float yaw) {
    int sky_height = horizon_y;
    if (sky_height < 0) sky_height = 0;
    if (sky_height > SCREEN_HEIGHT) sky_height = SCREEN_HEIGHT;
    
    // Sky gradient bands
    if (sky_height > 0) {
        int bands = 8;
        int band_h = sky_height / bands;
        if (band_h < 1) band_h = 1;
        
        for (int b = 0; b < bands; b++) {
            float ratio = (float)b / (float)(bands - 1);
            uint8_t r = (uint8_t)(4 + ratio * 10);
            uint8_t g = (uint8_t)(28 + ratio * 24);
            uint8_t b_val = (uint8_t)(31 - ratio * 2);
            uint16_t sky_color = (r << 11) | (g << 5) | b_val;
            
            int y0 = b * band_h;
            int h = (b == bands - 1) ? (sky_height - y0) : band_h;
            if (h > 0) {
                sprite.fillRect(0, y0, SCREEN_WIDTH, h, sky_color);
            }
        }
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
    
    // Grass
    int grass_y = horizon_y;
    if (grass_y < 0) grass_y = 0;
    if (grass_y < SCREEN_HEIGHT) {
        sprite.fillRect(0, grass_y, SCREEN_WIDTH, SCREEN_HEIGHT - grass_y, 0x55E8);
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
    
    const float south_yaw = PI;
    const float visible_half_angle = 0.92f;
    float wind = fmodf(millis() * 0.000035f, 0.28f);
    
    for (int i = 0; i < 3; i++) {
        float cloud_yaw = south_yaw + clouds[i].yaw_offset + wind;
        float delta = cloud_yaw - yaw;
        while (delta > PI) delta -= 2.0f * PI;
        while (delta < -PI) delta += 2.0f * PI;
        if (fabsf(delta) > visible_half_angle) continue;
        
        int cx = 160 + (int)(delta * 185.0f);
        int cy = horizon_y - clouds[i].y_offset;
        int cw = clouds[i].w;
        int ch = clouds[i].h;
        
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
                sprite.fillTriangle(x0, y0, x1, y1, x2, y2, mountains[i].color);
                if (i & 1) {
                    sprite.fillTriangle(x0, y0, x1, y1, x1, y0, mountains[i].shadow);
                } else {
                    sprite.fillTriangle(x1, y1, x2, y2, x1, y0, mountains[i].shadow);
                }
                
                // Snow caps
                if (mountains[i].snow_style != 0) {
                    int sx0 = x1 - (x1 - x0) * 0.35f;
                    int sy0 = y1 + mountains[i].h * 0.35f;
                    int sx2 = x1 + (x2 - x1) * 0.35f;
                    if (mountains[i].snow_style == 1) {
                        sprite.fillTriangle(sx0, sy0, x1, y1, x1, sy0, 0xFFFF);
                    } else if (mountains[i].snow_style == 2) {
                        sprite.fillTriangle(x1, sy0, x1, y1, sx2, sy0, 0xFFFF);
                    } else {
                        sprite.fillTriangle(sx0, sy0, x1, y1, sx2, sy0, 0xFFFF);
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

// Color Shading (0.0 to 1.0 multiplier)
uint16_t shadeColor(uint16_t color, float intensity) {
    if (intensity >= 1.0f) return color;
    if (intensity <= 0.0f) return 0;
    
    uint16_t r = (color >> 11) & 0x1F;
    uint16_t g = (color >> 5) & 0x3F;
    uint16_t b = color & 0x1F;
    
    r = (uint16_t)(r * intensity);
    g = (uint16_t)(g * intensity);
    b = (uint16_t)(b * intensity);
    
    return (r << 11) | (g << 5) | b;
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
    float opp1_dist = opponents[0].segment + opponents[0].laps * NUM_SEGMENTS;
    float opp2_dist = opponents[1].segment + opponents[1].laps * NUM_SEGMENTS;
    
    int rank = 1;
    if (opp1_dist > player_dist) rank++;
    if (opp2_dist > player_dist) rank++;
    
    sprite.setTextDatum(TR_DATUM);
    sprite.setTextSize(2);
    sprite.setTextColor(0xFCE0); // Gold
    const char* pos_suffix = "th";
    if (rank == 1) pos_suffix = "st";
    else if (rank == 2) pos_suffix = "nd";
    else if (rank == 3) pos_suffix = "rd";
    char rank_str[5];
    snprintf(rank_str, sizeof(rank_str), "%d%s", rank, pos_suffix);
    sprite.drawString(rank_str, 310, 8);
    
    sprite.setTextSize(1);
    sprite.setTextColor(0xFFFF);
    sprite.drawString("POS", 310, 26);
    
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
    
    // Track Minimap Progress Bar
    int map_x = 90;
    int map_y = 8;
    int map_w = 140;
    sprite.drawFastHLine(map_x, map_y, map_w, 0x5AEB);
    sprite.drawFastVLine(map_x, map_y - 2, 5, 0xFFFF);
    sprite.drawFastVLine(map_x + map_w, map_y - 2, 5, 0xFFFF);
    
    // Draw opponent dots
    for (int i = 0; i < 2; i++) {
        float opp_ratio = opponents[i].segment / NUM_SEGMENTS;
        int opp_dot_x = map_x + (int)(opp_ratio * map_w);
        sprite.fillCircle(opp_dot_x, map_y, 2, opponents[i].color);
    }
    // Draw player dot
    float p_ratio = player_segment_float / NUM_SEGMENTS;
    int p_dot_x = map_x + (int)(p_ratio * map_w);
    sprite.fillCircle(p_dot_x, map_y, 3, 0xF800);
    
    sprite.setTextDatum(BL_DATUM);
    sprite.setTextSize(1);
    sprite.setTextColor(0x07E0);
    char fps_str[14];
    int fps10 = (int)(measured_fps * 10.0f + 0.5f);
    snprintf(fps_str, sizeof(fps_str), "FPS:%d.%d", fps10 / 10, fps10 % 10);
    sprite.drawString(fps_str, 10, 166);
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
    sprite.drawString("FINISHED!", 160, 50);
    
    sprite.setTextSize(1);
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
    float car_y_val = 0.16f + 0.025f * sin(t * 2.7f);
    float car_z_val = 4.15f;
    
    cam_light_x = 0.18f + 0.18f * sin(t * 0.8f);
    cam_light_y = 0.98f;
    cam_light_z = -0.18f;
    
    draw3DModel(car_vertices, CAR_NUM_VERTICES, car_faces, CAR_NUM_FACES,
                car_x_val, car_y_val, car_z_val,
                0.0f, rot_y, 0.026f * sin(t * 1.55f),
                1.22f, 0x021F); // Subaru blue
    drawCarRearGlass(car_x_val, car_y_val, car_z_val,
                     0.0f, rot_y, 0.026f * sin(t * 1.55f),
                     1.22f);
                
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
    
    int sweep = (int)(sin(t * 1.55f) * 42.0f);
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
