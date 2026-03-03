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

#ifndef UFO_UTILITY_READ_BUFFER_HPP
#define UFO_UTILITY_READ_BUFFER_HPP

// UFO
#include <ufo/utility/io/base_buffer.hpp>
#include <ufo/utility/io/dir.hpp>

// STL
#include <cstddef>
#include <ostream>

namespace ufo
{
class ReadBuffer : virtual public BaseBuffer
{
 public:
	using size_type   = BaseBuffer::size_type;
	using pos_type    = BaseBuffer::pos_type;
	using offset_type = BaseBuffer::offset_type;

	ReadBuffer() = default;

	ReadBuffer(std::byte const* data, size_type count);

	template <class T>
	ReadBuffer& read(T& t)
	{
		return read(&t, sizeof(t));
	}

	ReadBuffer& read(void* dest, size_type count);

	ReadBuffer& read(std::ostream& out, size_type count);

	template <class T>
	void readAt(pos_type pos, T& t) const
	{
		readAt(pos, &t, sizeof(t));
	}

	void readAt(pos_type pos, void* dest, size_type count) const;

	void readAt(pos_type pos, std::ostream& out, size_type count) const;

	template <class T>
	void readAt(offset_type off, IODir dir, T& t) const
	{
		readAt(off, dir, &t, sizeof(t));
	}

	void readAt(offset_type off, IODir dir, void* dest, size_type count) const;

	void readAt(offset_type off, IODir dir, std::ostream& out, size_type count) const;

	bool readLine(std::string& line);

	[[nodiscard]] pos_type readPos() const noexcept;

	ReadBuffer& readSeek(pos_type pos) noexcept;

	ReadBuffer& readSeek(offset_type off, IODir dir) noexcept;

	template <class T>
	ReadBuffer& readSeek(T const& t, pos_type count) noexcept
	{
		return readSeek(sizeof(t) * count);
	}

	template <class T>
	ReadBuffer& readSeek(T const& t, offset_type count_signed, IODir dir) noexcept
	{
		return readSeek(sizeof(t) * count_signed, dir);
	}

	[[nodiscard]] size_type readLeft() const noexcept;

 protected:
	pos_type pos_{};
};
}  // namespace ufo
#endif  // UFO_UTILITY_READ_BUFFER_HPP