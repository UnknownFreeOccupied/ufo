/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the
 * Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of
 * Technology
 * All rights reserved.
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
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

#ifndef UFO_VISION_CAMERA_PERSPECTIVE_HPP
#define UFO_VISION_CAMERA_PERSPECTIVE_HPP

// UFO
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/geometry/ray.hpp>
#include <ufo/math/mat.hpp>
#include <ufo/math/transform.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/vision/camera/perspective_intrinsics.hpp>
#include <ufo/vision/image.hpp>

// STL
#include <cmath>
#include <cstddef>
#include <format>
#include <numbers>
#include <ostream>
#include <random>
#include <utility>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief Perspective camera model for ray-generation and projection.
 * @details
 * `PerspectiveCamera` stores the intrinsic parameters and the extrinsic pose
 * (camera-to-world transform) needed to:
 * - build a 4×4 view or projection matrix (`view()`, `projection()`),
 * - cast a dense `Image<Ray3f>` of world-space rays through every pixel
 *   (`rays()`, `rays(policy)`).
 *
 * The pose convention is **camera-to-world**: `pose` is the transform that
 * maps a point from camera space to world space.
 *
 * ### Coordinate conventions
 * - **Camera space**: Right-handed OpenCV convention (+X Right, +Y Down, +Z Forward).
 * - **NDC space**: x ∈ [-1, 1], y ∈ [-1, 1], z ∈ [0, 1].
 * - **Image space**: (row, col) with (0,0) at the top-left corner.
 */
struct PerspectiveCamera {
	/**
	 * @brief Camera-to-world pose transform.
	 */
	Transform3f pose;
	/**
	 * @brief The intrinsic parameters of the camera.
	 */
	PerspectiveIntrinsics intrinsics;
	/**
	 * @brief Distance to the near clipping plane.
	 */
	float near_clip;
	/**
	 * @brief Distance to the far clipping plane.
	 */
	float far_clip;

	/**
	 * @brief Orients the camera so it looks from `center` toward `target`.
	 * @param [in] center The new camera position in world space.
	 * @param [in] target The point in world space to look at.
	 * @param [in] up The world-space up vector.
	 */
	void lookAt(Vec3f center, Vec3f const& target, Vec3f const& up)
	{
		pose = Transform3f{inverse(ufo::lookAt(center, target, up))};
	}

	[[nodiscard]] Mat4f projection() const
	{
		Mat4f m = perspective(fx(), fy(), intrinsics.cx, intrinsics.cy,
		                      static_cast<float>(intrinsics.cols),
		                      static_cast<float>(intrinsics.rows), near_clip, far_clip);
		m[0][0] *= intrinsics.zoom;
		m[1][1] *= intrinsics.zoom;
		return m;
	}

	[[nodiscard]] Mat4f view() const { return Mat4f(pose); }

	[[nodiscard]] Image<Ray3f> rays() const { return rays(ufo::execution::seq); }

	template <class ExecutionPolicy>
	[[nodiscard]] Image<Ray3f> rays(ExecutionPolicy&& policy) const
	{
		auto const   view_inv = view();
		Image<Ray3f> rays(intrinsics.rows, intrinsics.cols);

		ufo::for_each(
		    std::forward<ExecutionPolicy>(policy), std::size_t{}, intrinsics.rows,
		    [this, &rays, &view_inv, fx = fx(), fy = fy()](std::size_t y) {
			    if (0.0f < intrinsics.lens_radius && 0.0f < intrinsics.focus_distance) {
				    bool const   deterministic = intrinsics.deterministic;
				    std::mt19937 gen_local;
				    // Use a static thread_local generator for performance in non-deterministic
				    // mode
				    static thread_local std::mt19937 gen_tl(std::random_device{}());

				    std::mt19937& gen = deterministic ? gen_local : gen_tl;
				    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

				    if (deterministic) {
					    gen.seed(static_cast<unsigned>(y * intrinsics.cols));
				    }

				    for (std::size_t x{}; rays.cols() > x; ++x) {
					    float const dx = (x + 0.5f - intrinsics.cx) / fx;
					    float const dy = (y + 0.5f - intrinsics.cy) / fy;

					    Vec3f dir_cam = normalize(Vec3f(dx, dy, 1.0f));

					    float const r     = intrinsics.lens_radius * std::sqrt(std::abs(dist(gen)));
					    float const theta = 2.0f * std::numbers::pi_v<float> * dist(gen);
					    Vec3f const origin_cam(r * std::cos(theta), r * std::sin(theta), 0.0f);

					    float const ft      = intrinsics.focus_distance / dir_cam.z();
					    Vec3f const p_focus = dir_cam * ft;
					    dir_cam             = normalize(p_focus - origin_cam);

					    rays[y, x].origin = ufo::convert<Vec3f>(
					        view_inv * Vec4f(origin_cam.x(), origin_cam.y(), origin_cam.z(), 1.0f));
					    rays[y, x].direction = ufo::convert<Vec3f>(
					        view_inv * Vec4f(dir_cam.x(), dir_cam.y(), dir_cam.z(), 0.0f));
				    }
			    } else {
				    for (std::size_t x{}; rays.cols() > x; ++x) {
					    float const dx = (x + 0.5f - intrinsics.cx) / fx;
					    float const dy = (y + 0.5f - intrinsics.cy) / fy;

					    Vec3f const dir_cam = normalize(Vec3f(dx, dy, 1.0f));

					    rays[y, x].origin =
					        ufo::convert<Vec3f>(view_inv * Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
					    rays[y, x].direction = ufo::convert<Vec3f>(
					        view_inv * Vec4f(dir_cam.x(), dir_cam.y(), dir_cam.z(), 0.0f));
				    }
			    }
		    });
		return rays;
	}

	[[nodiscard]] float fx() const noexcept
	{
		return 0.0f < intrinsics.fx
		           ? intrinsics.fx
		           : intrinsics.rows / (2.0f * std::tan(intrinsics.vertical_fov / 2.0f));
	}

	[[nodiscard]] float fy() const noexcept
	{
		return 0.0f < intrinsics.fy
		           ? intrinsics.fy
		           : intrinsics.rows / (2.0f * std::tan(intrinsics.vertical_fov / 2.0f));
	}

	[[nodiscard]] friend bool operator==(PerspectiveCamera const&,
	                                     PerspectiveCamera const&) noexcept = default;
};

/**
 * @brief Writes a human-readable summary of `camera` to `os`.
 */
inline std::ostream& operator<<(std::ostream& os, PerspectiveCamera const& camera)
{
	os << "Pose: " << camera.pose << '\n'
	   << camera.intrinsics << "\nNear clip: " << camera.near_clip
	   << ", Far clip: " << camera.far_clip << '\n';
	return os;
}

/**
 * @}
 */
}  // namespace ufo

template <>
struct std::formatter<ufo::PerspectiveCamera> {
	constexpr auto parse(std::format_parse_context& ctx) const { return ctx.begin(); }

	auto format(ufo::PerspectiveCamera const& camera, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Pose: {}\n{}\nNear clip: {}, Far clip: {}",
		                      camera.pose, camera.intrinsics, camera.near_clip,
		                      camera.far_clip);
	}
};

#endif  // UFO_VISION_CAMERA_PERSPECTIVE_HPP
