#pragma once
extern "C" {
#include <cstdint>
}

enum class DeployMode : uint8_t {
	DroguePrimary = 0,
	DrogueBackup  = 1,
	MainPrimary   = 2,
	MainBackup    = 3,
	Unused        = 7,
};

enum class DeviceState : uint8_t {
	Receive = 0,
	Config
};

enum class UnitSystem : uint8_t {
    Metric = 0,
    English
};

enum class SensorHealth : uint8_t {
    Off = 0,
    Initializing,
    Ok,
    Warning,
    Error,
    Stale
};

enum class FlightStates : uint8_t {
	WaitingLaunch = 0,
	Launched = 1,
	Burnout = 2,
	Noseover = 3,
	DroguePrimaryEvent = 4,
	DrogueBackupEvent = 5,
	MainPrimaryEvent = 6,
	MainBackupEvent = 7,
	Landed = 8
};

struct Vec3f {
    float x;
    float y;
    float z;

    Vec3f() : x(0), y(0), z(0) {}
    Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3f operator+(const Vec3f& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3f operator-(const Vec3f& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3f operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3f operator/(float s) const { return {x / s, y / s, z / s}; }

    Vec3f& operator+=(const Vec3f& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3f& operator-=(const Vec3f& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3f& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};

struct Quaternionf {
    float w;
    float x;
    float y;
    float z;

    Quaternionf() : w(1), x(0), y(0), z(0) {}
    Quaternionf(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
};
