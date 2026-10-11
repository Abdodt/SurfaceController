#ifndef SURFACE_CONTROLLER_COMMON_RESULT_HPP
#define SURFACE_CONTROLLER_COMMON_RESULT_HPP

#include <utility>
#include <cassert>

namespace surface_controller {

/**
 * @brief Zero-overhead status representation for embedded C++17 without exceptions.
 */
enum class Status {
    OK = 0,
    ERROR,
    BUSY,
    TIMEOUT,
    INVALID_ARGUMENT,
    NOT_INITIALIZED,
    BUFFER_FULL,
    BUFFER_EMPTY,
    HARDWARE_FAULT
};

inline const char* statusToString(Status s) {
    switch (s) {
        case Status::OK: return "OK";
        case Status::ERROR: return "ERROR";
        case Status::BUSY: return "BUSY";
        case Status::TIMEOUT: return "TIMEOUT";
        case Status::INVALID_ARGUMENT: return "INVALID_ARGUMENT";
        case Status::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case Status::BUFFER_FULL: return "BUFFER_FULL";
        case Status::BUFFER_EMPTY: return "BUFFER_EMPTY";
        case Status::HARDWARE_FAULT: return "HARDWARE_FAULT";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Lightweight Result type containing either a Value (T) or an Error (E).
 * Strictly zero heap allocation, suitable for -fno-exceptions.
 */
template <typename T, typename E = Status>
class Result {
public:
    constexpr Result(const T& val) : is_ok_(true) {
        new (&storage_.val) T(val);
    }

    constexpr Result(T&& val) : is_ok_(true) {
        new (&storage_.val) T(std::move(val));
    }

    constexpr Result(E err) : is_ok_(false) {
        new (&storage_.err) E(err);
    }

    ~Result() {
        if (is_ok_) {
            storage_.val.~T();
        } else {
            storage_.err.~E();
        }
    }

    Result(const Result& other) : is_ok_(other.is_ok_) {
        if (is_ok_) {
            new (&storage_.val) T(other.storage_.val);
        } else {
            new (&storage_.err) E(other.storage_.err);
        }
    }

    Result& operator=(const Result& other) {
        if (this != &other) {
            this->~Result();
            is_ok_ = other.is_ok_;
            if (is_ok_) {
                new (&storage_.val) T(other.storage_.val);
            } else {
                new (&storage_.err) E(other.storage_.err);
            }
        }
        return *this;
    }

    Result(Result&& other) noexcept : is_ok_(other.is_ok_) {
        if (is_ok_) {
            new (&storage_.val) T(std::move(other.storage_.val));
        } else {
            new (&storage_.err) E(std::move(other.storage_.err));
        }
    }

    Result& operator=(Result&& other) noexcept {
        if (this != &other) {
            this->~Result();
            is_ok_ = other.is_ok_;
            if (is_ok_) {
                new (&storage_.val) T(std::move(other.storage_.val));
            } else {
                new (&storage_.err) E(std::move(other.storage_.err));
            }
        }
        return *this;
    }

    constexpr bool is_ok() const noexcept { return is_ok_; }
    constexpr bool is_err() const noexcept { return !is_ok_; }
    constexpr explicit operator bool() const noexcept { return is_ok_; }

    constexpr const T& value() const noexcept {
        assert(is_ok_);
        return storage_.val;
    }

    constexpr T& value() noexcept {
        assert(is_ok_);
        return storage_.val;
    }

    constexpr E error() const noexcept {
        assert(!is_ok_);
        return storage_.err;
    }

    constexpr T value_or(T default_val) const noexcept {
        return is_ok_ ? storage_.val : default_val;
    }

private:
    union Storage {
        T val;
        E err;
        Storage() {}
        ~Storage() {}
    } storage_;
    bool is_ok_{false};
};

} // namespace surface_controller

#endif // SURFACE_CONTROLLER_COMMON_RESULT_HPP
