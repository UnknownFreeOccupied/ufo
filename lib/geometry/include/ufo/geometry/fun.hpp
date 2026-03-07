/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_GEOMETRY_FUN_HPP
#define UFO_GEOMETRY_FUN_HPP

// UFO
#include <ufo/geometry/detail/fun.hpp>

namespace ufo
{
/**
 * @brief Returns the minimum coordinate of the minimum spanning axis-aligned bounding box
 * of a geometry.
 * @tparam Geometry The type of the geometry.
 * @param [in] g The geometry.
 * @return The minimum coordinate of the minimum spanning axis-aligned bounding box.
 */
template <class Geometry>
[[nodiscard]] constexpr Vec<Geometry::dimension(), typename Geometry::value_type> min(
    Geometry const& g)
{
	return detail::min(g);
}

/**
 * @brief Returns the maximum coordinate of the minimum spanning axis-aligned bounding box
 * of a geometry.
 * @tparam Geometry The type of the geometry.
 * @param [in] g The geometry.
 * @return The maximum coordinate of the minimum spanning axis-aligned bounding box.
 */
template <class Geometry>
[[nodiscard]] constexpr Vec<Geometry::dimension(), typename Geometry::value_type> max(
    Geometry const& g)
{
	return detail::max(g);
}

/**
 * @brief Returns the corners of the minimum spanning axis-aligned bounding box of a
 * geometry.
 * @tparam Geometry The type of the geometry.
 * @param [in] g The geometry.
 * @return The corners of the minimum spanning axis-aligned bounding box.
 */
template <class Geometry>
[[nodiscard]] constexpr auto corners(Geometry const& g)
{
	return detail::corners(g);
}
}  // namespace ufo

#endif  // UFO_GEOMETRY_FUN_HPP