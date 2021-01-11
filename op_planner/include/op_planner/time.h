#pragma once
/**
 * @file
 *
 * @brief This library provides the utilities to deal with timestamps.
 * currently our assumption is that every timestamp will be of a
 * precision at 1us.
 */
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <type_traits>

#include "op_planner/macro.h"
#include "op_planner/log.h"
#include "ros/ros.h"

namespace PlannerHNS
{

    /**
 * @class Duration
 * @brief the default Duration is of precision nanoseconds (1e-9 seconds).
 */
    using Duration = std::chrono::nanoseconds;

    /**
 * @class Timestamp
 * @brief the default timestamp uses std::chrono::system_clock. The
 * system_clock is a system-wide realtime clock.
 */
    using Timestamp = std::chrono::time_point<std::chrono::system_clock, Duration>;

    static_assert(
        sizeof(std::chrono::nanoseconds) >= sizeof(int64_t),
        "The underlying type of the nanoseconds should be at least 64 bits.");

    using nanos = std::chrono::nanoseconds;
    using micros = std::chrono::microseconds;
    using millis = std::chrono::milliseconds;
    using seconds = std::chrono::seconds;
    using minutes = std::chrono::minutes;
    using hours = std::chrono::hours;

    /**
 * @brief converts the input duration (nanos) to a 64 bit integer, with
 * the unit specified by PrecisionDuration.
 * @param duration the input duration that needs to be converted to integer.
 * @return an integer representing the duration in the specified unit.
 */
    template <typename PrecisionDuration>
    int64_t AsInt64(const Duration &duration)
    {
        return std::chrono::duration_cast<PrecisionDuration>(duration).count();
    }

    /**
 * @brief converts the input timestamp (nanos) to a 64 bit integer, with
 * the unit specified by PrecisionDuration.
 * @param timestamp the input timestamp that needs to be converted to integer.
 * @return an integer representing the timestamp in the specified unit.
 */
    template <typename PrecisionDuration>
    int64_t AsInt64(const Timestamp &timestamp)
    {
        return AsInt64<PrecisionDuration>(timestamp.time_since_epoch());
    }

    /**
 * @brief converts the input duration (nanos) to a double in seconds.
 * The original precision will be preserved.
 * @param duration the input duration that needs to be converted.
 * @return a double in seconds.
 */
    inline double ToSecond(const Duration &duration)
    {
        return static_cast<double>(AsInt64<nanos>(duration)) * 1e-9;
    }

    /**
 * @brief converts the input timestamp (nanos) to a double in seconds.
 * The original precision will be preserved.
 * @param timestamp the input timestamp that needs to be converted.
 * @return a double representing the same timestamp in seconds.
 */
    inline double ToSecond(const Timestamp &timestamp)
    {
        return static_cast<double>(AsInt64<nanos>(timestamp.time_since_epoch())) *
               1e-9;
    }

    /**
 * @brief converts the integer-represented timestamp to \class Timestamp.
 * @return a Timestamp object.
 */
    template <typename PrecisionDuration>
    inline Timestamp FromInt64(int64_t timestamp_value)
    {
        return Timestamp(
            std::chrono::duration_cast<nanos>(PrecisionDuration(timestamp_value)));
    }

    /**
 * @brief converts the double to \class Timestamp. The input double has
 * a unit of seconds.
 * @return a Timestamp object.
*/
    inline Timestamp From(double timestamp_value)
    {
        int64_t nanos_value = static_cast<int64_t>(timestamp_value * 1e9);
        return FromInt64<nanos>(nanos_value);
    }

    /**
 * @class Clock
 * @brief a singleton clock that can be used to get the current
 * timestamp. The source can be either system clock or a mock clock.
 * Mock clock is for testing purpose mainly. The mock clock related
 * methods are not thread-safe.
 */
    class Clock
    {
    public:
        static constexpr int64_t PRECISION =
            std::chrono::system_clock::duration::period::den /
            std::chrono::system_clock::duration::period::num;

        /// PRECISION >= 1000000 means the precision is at least 1us.
        static_assert(PRECISION >= 1000000,
                      "The precision of the system clock should be at least 1 "
                      "microsecond.");

        // The clock mode can either be a system clock time, a user mocked time (for
        // test only) or read from ROS.
        enum ClockMode
        {
            SYSTEM = 0,
            MOCK = 1,
            ROS = 2,
        };

        /**
   * @brief gets the current timestamp.
   * @return a Timestamp object representing the current time.
   */
        static Timestamp Now()
        {
            switch (mode())
            {
            case ClockMode::SYSTEM:
                return SystemNow();
            case ClockMode::MOCK:
                return instance()->mock_now_;
            case ClockMode::ROS:
                return From(ros::Time::now().toSec());
            default:
                AFATAL << "Unsupported clock mode: " << mode();
            }
            return From(ros::Time::now().toSec());
        }

        /**
   * @brief gets the current time in second.
   * @return the current time in second.
   */
        static double NowInSeconds() { return ToSecond(Clock::Now()); }

        /**
   * @brief Set the behavior of the \class Clock.
   * @param The new clock mode to be set.
   */
        static void SetMode(ClockMode mode) { instance()->mode_ = mode; }

        /**
   * @brief Gets the current clock mode.
   * @return The current clock mode.
   */
        static ClockMode mode() { return instance()->mode_; }

        /**
   * @brief This is for mock clock mode only. It will set the timestamp
   * for the mock clock.
   */
        static void SetNow(const Duration &duration)
        {
            Clock *clock = instance();
            if (clock->mode_ != ClockMode::MOCK)
            {
                AFATAL << "Cannot set now when clock mode is not MOCK!";
            }
            clock->mock_now_ = Timestamp(duration);
        }

    private:
        /**
   * @brief constructs the \class Clock instance
   * @param mode the desired clock mode
   */
        explicit Clock(ClockMode mode) : mode_(mode), mock_now_(Timestamp())
        {
            ros::Time::init();
        }

        /**
   * @brief Returns the current timestamp based on the system clock.
   * @return the current timestamp based on the system clock.
   */
        static Timestamp SystemNow()
        {
            return std::chrono::time_point_cast<Duration>(
                std::chrono::system_clock::now());
        }

        /// NOTE: Unless mode_ and mock_now_ are guarded by a
        /// lock or become atomic, having multiple threads calling mock
        /// clock related functions are STRICTLY PROHIBITED.

        /// Indicates whether it is in the system clock mode or the mock
        /// clock mode or the ROS time mode.
        ClockMode mode_;

        /// Stores the currently set timestamp, which serves mock clock
        /// queries.
        Timestamp mock_now_;

        /// Explicitly disable default and move/copy constructors.
        DECLARE_SINGLETON(Clock);
    };
inline Clock::Clock()
    : Clock(false ? ClockMode::ROS : ClockMode::SYSTEM) {}
} // namespace PlannerHNS
