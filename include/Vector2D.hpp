#pragma once

// Planar state uses X/Z so altitude stays on Z and future lateral dynamics can use X without renaming.

struct Vector2D {
    double x = 0.0;
    double z = 0.0;
};

inline Vector2D operator+(Vector2D a, Vector2D b) {
    return Vector2D{a.x + b.x, a.z + b.z};
}

inline Vector2D operator-(Vector2D a, Vector2D b) {
    return Vector2D{a.x - b.x, a.z - b.z};
}

inline Vector2D operator*(Vector2D v, double s) {
    return Vector2D{v.x * s, v.z * s};
}

inline Vector2D operator*(double s, Vector2D v) {
    return v * s;
}

inline Vector2D operator/(Vector2D v, double s) {
    return Vector2D{v.x / s, v.z / s};
}
