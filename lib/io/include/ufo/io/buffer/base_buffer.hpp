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

#ifndef UFO_UTILITY_BASE_BUFFER_HPP
#define UFO_UTILITY_BASE_BUFFER_HPP

// UFO
#include <ufo/utility/io/dir.hpp>

// STL
#include <cstddef>
#include <cstdint>

namespace ufo
{
class BaseBuffer
{
 public:
	using size_type   = std::size_t;
	using pos_type    = std::uintmax_t;
	using offset_type = std::intmax_t;

	[[nodiscard]] constexpr std::byte const* data() const noexcept { return data_; }

	[[nodiscard]] constexpr bool empty() const noexcept { return 0 == size_; }

	[[nodiscard]] constexpr size_type size() const noexcept { return size_; }

 protected:
	constexpr BaseBuffer() = default;

	constexpr BaseBuffer(std::byte const* data, size_type size) : data_(data), size_(size)
	{
	}

	[[nodiscard]] constexpr pos_type pos(pos_type pos, offset_type off,
	                                     IODir dir) const noexcept
	{
		switch (dir) {
			case IODir::Beg: return off;
			case IODir::End: return size_ + off;
			case IODir::Cur: return pos + off;
		}

		return 0;
	}

 protected:
	std::byte const* data_ = nullptr;
	size_type        size_{};
};
}  // namespace ufo
#endif  // UFO_UTILITY_BASE_BUFFER_HPP