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

#ifndef UFO_VISION_CAMERA_ORTHOGONAL_HPP
#define UFO_VISION_CAMERA_ORTHOGONAL_HPP

// UFO
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/geometry/ray.hpp>
#include <ufo/numeric/mat.hpp>
#include <ufo/numeric/transform.hpp>
#include <ufo/numeric/vec.hpp>
#include <ufo/vision/camera/orthogonal_intrinsics.hpp>
#include <ufo/vision/image.hpp>

// STL
#include <cmath>
#include <cstddef>
#include <format>
#include <ostream>
#include <utility>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief Orthogonal camera model for ray-generation and projection.
 * @details
 * `OrthogonalCamera` stores the intrinsic parameters and the extrinsic pose
 * (camera-to-world transform) needed to:
 * - build a 4×4 view or projection matrix (`view()`, `projection()`),
 * - cast a dense `Image<Ray3f>` of world-space rays through every pixel
 *   (`rays()`, `rays(policy)`).
 */
struct OrthogonalCamera {
	/**
	 * @brief Camera-to-world pose transform.
	 */
	Transform3f pose;
	/**
	 * @brief The intrinsic parameters of the camera.
	 */
	OrthogonalIntrinsics intrinsics;
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
		float const half_w = intrinsics.width / 2.0f;
		float const half_h = intrinsics.height / 2.0f;
		Mat4f       m = orthogonal(-half_w, half_w, -half_h, half_h, near_clip, far_clip);
		m[0][0] *= intrinsics.zoom;
		m[1][1] *= intrinsics.zoom;
		return m;
	}

	[[nodiscard]] Mat4f view() const { return Mat4f(pose); }

	[[nodiscard]] Image<Ray3f> rays() const { return rays(ufo::execution::seq); }

	template <class ExecutionPolicy>
	[[nodiscard]] Image<Ray3f> rays(ExecutionPolicy&& policy) const
	{
		auto const   proj_inv = inverse(projection());
		auto const   view_inv = view();
		Image<Ray3f> rays(intrinsics.rows, intrinsics.cols);

		ufo::for_each(std::forward<ExecutionPolicy>(policy), std::size_t{}, intrinsics.rows,
		              [&rays, &proj_inv, &view_inv](std::size_t y) {
			              float const vy = ((y + 0.5f) / rays.rows()) * 2.0f - 1.0f;
			              for (std::size_t x{}; rays.cols() > x; ++x) {
				              float const vx = ((x + 0.5f) / rays.cols()) * 2.0f - 1.0f;
				              Vec3f       origin(ufo::convert<Vec3f>(view_inv * proj_inv *
				                                                     Vec4f(vx, vy, 0.0f, 1.0f)));
				              Vec3f       direction(
                          ufo::convert<Vec3f>(view_inv * Vec4f(0.0f, 0.0f, 1.0f, 0.0f)));
				              rays[y, x] = {origin, direction};
			              }
		              });
		return rays;
	}

	[[nodiscard]] friend bool operator==(OrthogonalCamera const&,
	                                     OrthogonalCamera const&) noexcept = default;
};

/**
 * @brief Writes a human-readable summary of `camera` to `os`.
 */
inline std::ostream& operator<<(std::ostream& os, OrthogonalCamera const& camera)
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
struct std::formatter<ufo::OrthogonalCamera> {
	constexpr auto parse(std::format_parse_context& ctx) const { return ctx.begin(); }

	auto format(ufo::OrthogonalCamera const& camera, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Pose: {}\n{}\nNear clip: {}, Far clip: {}",
		                      camera.pose, camera.intrinsics, camera.near_clip,
		                      camera.far_clip);
	}
};

#endif  // UFO_VISION_CAMERA_ORTHOGONAL_HPP
