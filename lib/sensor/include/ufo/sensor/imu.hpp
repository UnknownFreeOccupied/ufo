/**
 * @author Daniel Duberg (danielduberg@gmail.com)
 * @see https://github.com/UnknownFreeOccupied/ufo
 * @version 1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) 2020-2026, Daniel Duberg
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2020-2026, Daniel Duberg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_SENSOR_IMU_HPP
#define UFO_SENSOR_IMU_HPP

// UFO
#include <ufo/math/mat.hpp>
#include <ufo/math/quat.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <concepts>

namespace ufo
{
/**
 * @struct IMU
 * @brief Represents an Inertial Measurement Unit (IMU) sensor reading.
 *
 * @details
 * This structure holds common IMU measurements, including orientation, angular
 * velocity, and linear acceleration, along with their associated covariance matrices.
 *
 * @tparam T The numeric type (e.g., float, double) used for the measurements.
 */
template <std::floating_point T>
struct IMU {
	/**
	 * @brief Orientation of the sensor in the world frame.
	 *
	 * @details
	 * Represented as a quaternion.
	 */
	Quat<T> orientation;

	/**
	 * @brief Angular velocity of the sensor.
	 *
	 * @details
	 * Measured in radians per second (rad/s).
	 */
	Vec<3, T> angular_velocity;

	/**
	 * @brief Linear acceleration of the sensor.
	 *
	 * @details
	 * Measured in meters per second squared (m/s²). Includes gravity.
	 */
	Vec<3, T> linear_acceleration;

	/**
	 * @brief Covariance matrix for the orientation measurement.
	 *
	 * @details
	 * A 3x3 matrix representing the uncertainty in the orientation.
	 */
	Mat<3, 3, T> orientation_covariance;

	/**
	 * @brief Covariance matrix for the angular velocity measurement.
	 *
	 * @details
	 * A 3x3 matrix representing the uncertainty in the angular velocity.
	 */
	Mat<3, 3, T> angular_velocity_covariance;

	/**
	 * @brief Covariance matrix for the linear acceleration measurement.
	 *
	 * @details
	 * A 3x3 matrix representing the uncertainty in the linear acceleration.
	 */
	Mat<3, 3, T> linear_acceleration_covariance;
};
}  // namespace ufo

#endif  // UFO_SENSOR_IMU_HPP