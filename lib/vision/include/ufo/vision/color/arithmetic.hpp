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

#ifndef UFO_VISION_COLOR_ARITHMETIC_HPP
#define UFO_VISION_COLOR_ARITHMETIC_HPP

// UFO
#include <ufo/vision/color/concepts.hpp>
#include <ufo/vision/color/detail/arithmetic.hpp>

// STL
#include <functional>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**************************************************************************************
|                                                                                     |
|                                      Addition                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Adds two colors component-wise.
 * @param [in,out] lhs The left-hand side color.
 * @param [in] rhs The right-hand side color.
 * @return The resulting color.
 */
template <Color C>
constexpr C& operator+=(C& lhs, C const& rhs)
{
	lhs = detail::reduce(lhs, rhs, std::plus{});
	return lhs;
}

/**
 * @brief Adds two colors component-wise.
 * @param [in] lhs The left-hand side color.
 * @param [in] rhs The right-hand side color.
 * @return The resulting color.
 */
template <Color C>
constexpr C operator+(C lhs, C const& rhs)
{
	lhs += rhs;
	return lhs;
}

/**************************************************************************************
|                                                                                     |
|                                     Subtraction                                     |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Subtracts two colors component-wise.
 * @param [in,out] lhs The left-hand side color.
 * @param [in] rhs The right-hand side color.
 * @return The resulting color.
 */
template <Color C>
constexpr C& operator-=(C& lhs, C const& rhs)
{
	lhs = detail::reduce(lhs, rhs, std::minus{});
	return lhs;
}

/**
 * @brief Subtracts two colors component-wise.
 * @param [in] lhs The left-hand side color.
 * @param [in] rhs The right-hand side color.
 * @return The resulting color.
 */
template <Color C>
constexpr C operator-(C lhs, C const& rhs)
{
	lhs -= rhs;
	return lhs;
}

/**************************************************************************************
|                                                                                     |
|                                   Multiplication                                    |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Multiplies a color by a scalar.
 * @param [in,out] lhs The left-hand side color.
 * @param [in] rhs The right-hand side scalar.
 * @return The resulting color.
 */
template <Color C>
constexpr C& operator*=(C& lhs, float rhs)
{
	lhs = detail::reduce(lhs, rhs, std::multiplies{});
	return lhs;
}

/**
 * @brief Multiplies two colors component-wise.
 * @param [in,out] lhs The left-hand side color.
 * @param [in] rhs The right-hand side color.
 * @return The resulting color.
 */
template <Color C>
constexpr C& operator*=(C& lhs, C const& rhs)
{
	lhs = detail::reduce(lhs, rhs, std::multiplies{});
	return lhs;
}

/**
 * @brief Multiplies two colors component-wise.
 * @param [in] lhs The left-hand side color.
 * @param [in] rhs The right-hand side color.
 * @return The resulting color.
 */
template <Color C>
constexpr C operator*(C lhs, C const& rhs)
{
	lhs *= rhs;
	return lhs;
}

/**
 * @brief Multiplies a color by a scalar.
 * @param [in] lhs The left-hand side color.
 * @param [in] rhs The right-hand side scalar.
 * @return The resulting color.
 */
template <Color C>
constexpr C operator*(C lhs, float rhs)
{
	lhs *= rhs;
	return lhs;
}

/**
 * @brief Multiplies a scalar by a color.
 * @param [in] lhs The left-hand side scalar.
 * @param [in] rhs The right-hand side color.
 * @return The resulting color.
 */
template <Color C>
constexpr C operator*(float lhs, C rhs)
{
	return detail::reduce(lhs, rhs, std::multiplies{});
}

/**************************************************************************************
|                                                                                     |
|                                      Division                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Divides a color by another color component-wise.
 * @param [in,out] lhs The left-hand side color.
 * @param [in] rhs The right-hand side color.
 * @return The resulting color.
 */
template <Color C>
constexpr C& operator/=(C& lhs, C const& rhs)
{
	lhs = detail::reduce(lhs, rhs, std::divides{});
	return lhs;
}

/**
 * @brief Divides a color by a scalar.
 * @param [in,out] lhs The left-hand side color.
 * @param [in] rhs The right-hand side scalar.
 * @return The resulting color.
 */
template <Color C>
constexpr C& operator/=(C& lhs, float rhs)
{
	lhs = detail::reduce(lhs, rhs, std::divides{});
	return lhs;
}

/**
 * @brief Divides two colors component-wise.
 * @param [in] lhs The left-hand side color.
 * @param [in] rhs The right-hand side color.
 * @return The resulting color.
 */
template <Color C>
constexpr C operator/(C lhs, C const& rhs)
{
	lhs /= rhs;
	return lhs;
}

/**
 * @brief Divides a color by a scalar.
 * @param [in] lhs The left-hand side color.
 * @param [in] rhs The right-hand side scalar.
 * @return The resulting color.
 */
template <Color C>
constexpr C operator/(C lhs, float rhs)
{
	lhs /= rhs;
	return lhs;
}

/**
 * @brief Divides a scalar by a color.
 * @param [in] lhs The left-hand side scalar.
 * @param [in] rhs The right-hand side color.
 * @return The resulting color.
 */
template <Color C>
constexpr C operator/(float lhs, C rhs)
{
	return detail::reduce(lhs, rhs, std::divides{});
}

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_COLOR_ARITHMETIC_HPP