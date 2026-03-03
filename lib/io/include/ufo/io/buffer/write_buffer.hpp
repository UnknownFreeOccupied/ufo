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

#ifndef UFO_UTILITY_WRITE_BUFFER_HPP
#define UFO_UTILITY_WRITE_BUFFER_HPP

// UFO
#include <ufo/utility/io/base_buffer.hpp>
#include <ufo/utility/io/dir.hpp>

// STL
#include <cstddef>
#include <istream>
#include <memory>

namespace ufo
{
class WriteBuffer : virtual public BaseBuffer
{
 public:
	using size_type   = BaseBuffer::size_type;
	using pos_type    = BaseBuffer::pos_type;
	using offset_type = BaseBuffer::offset_type;

	WriteBuffer() = default;
	WriteBuffer(WriteBuffer const& other);
	WriteBuffer(WriteBuffer&&) = default;

	WriteBuffer& operator=(WriteBuffer const& rhs);
	WriteBuffer& operator=(WriteBuffer&&) = default;

	virtual ~WriteBuffer() = default;

	template <class T>
	WriteBuffer& write(T const& t)
	{
		return write(&t, sizeof(t));
	}

	WriteBuffer& write(void const* src, size_type count);

	WriteBuffer& write(std::istream& in, size_type count);

	template <class T>
	void writeAt(pos_type pos, T const& t)
	{
		writeAt(pos, &t, sizeof(t));
	}

	void writeAt(pos_type pos, void const* src, size_type count);

	void writeAt(pos_type pos, std::istream& in, size_type count);

	template <class T>
	void writeAt(offset_type off, IODir dir, T const& t)
	{
		writeAt(off, dir, &t, sizeof(t));
	}

	void writeAt(offset_type off, IODir dir, void const* src, size_type count);

	void writeAt(offset_type off, IODir dir, std::istream& in, size_type count);

	void reserve(size_type new_cap);

	virtual void clear();

	[[nodiscard]] std::byte* data();

	[[nodiscard]] size_type capacity() const noexcept;

	[[nodiscard]] pos_type writePos() const noexcept;

	WriteBuffer& writeSeek(pos_type pos) noexcept;

	WriteBuffer& writeSeek(offset_type off, IODir dir) noexcept;

	template <class T>
	WriteBuffer& writeSeek(T const& t, pos_type count) noexcept
	{
		return writeSeek(sizeof(t) * count);
	}

	template <class T>
	WriteBuffer& writeSeek(T const& t, offset_type count_signed, IODir dir) noexcept
	{
		return writeSeek(sizeof(t) * count_signed, dir);
	}

	[[nodiscard]] size_type writeLeft() const noexcept;

 protected:
	struct FreeDeleter {
		void operator()(void* p) const noexcept { std::free(p); }
	};

	std::unique_ptr<std::byte, FreeDeleter> data_;
	size_type                               size_{};
	size_type                               cap_{};
	pos_type                                pos_{};
};
}  // namespace ufo
#endif  // UFO_UTILITY_WRITE_BUFFER_HPP