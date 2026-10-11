#ifndef SURFACE_CONTROLLER_MATH_VECTOR3_HPP
#define SURFACE_CONTROLLER_MATH_VECTOR3_HPP

#include <cmath>

namespace surface_controller {
namespace math {

/**
 * @brief Lightweight 3D vector class for embedded flight dynamics.
 */
template <typename T>
struct Vector3 {
    T x{0};
    T y{0};
    T z{0};

    constexpr Vector3() = default;
    constexpr Vector3(T x_val, T y_val, T z_val) : x(x_val), y(y_val), z(z_val) {}

    constexpr Vector3 operator+(const Vector3& rhs) const noexcept {
        return Vector3{x + rhs.x, y + rhs.y, z + rhs.z};
    }

    constexpr Vector3 operator-(const Vector3& rhs) const noexcept {
        return Vector3{x - rhs.x, y - rhs.y, z - rhs.z};
    }

    constexpr Vector3 operator*(T scalar) const noexcept {
        return Vector3{x * scalar, y * scalar, z * scalar};
    }

    constexpr Vector3 operator/(T scalar) const noexcept {
        return Vector3{x / scalar, y / scalar, z / scalar};
    }

    Vector3& operator+=(const Vector3& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vector3& operator*=(T scalar) noexcept {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    constexpr T dot(const Vector3& rhs) const noexcept {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    constexpr Vector3 cross(const Vector3& rhs) const noexcept {
        return Vector3{
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        };
    }

    T norm_sq() const noexcept {
        return x * x + y * y + z * z;
    }

    T norm() const noexcept {
        return std::sqrt(norm_sq());
    }

    Vector3 normalized() const noexcept {
        const T n = norm();
        if (n > static_cast<T>(1e-6)) {
            return (*this) / n;
        }
        return Vector3{0, 0, 0};
    }

    constexpr void reset() noexcept {
        x = static_cast<T>(0);
        y = static_cast<T>(0);
        z = static_cast<T>(0);
    }
};

using Vector3f = Vector3<float>;

} // namespace math
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_MATH_VECTOR3_HPP
