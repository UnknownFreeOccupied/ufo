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

// UFO
#include <ufo/utility/io/read_buffer.hpp>

// STL
#include <algorithm>
#include <cstring>

namespace ufo
{
ReadBuffer::ReadBuffer(std::byte const* data, size_type count) : BaseBuffer(data, count)
{
}

ReadBuffer& ReadBuffer::read(void* dest, size_type count)
{
	readAt(pos_, dest, count);
	pos_ += count;
	return *this;
}

ReadBuffer& ReadBuffer::read(std::ostream& out, size_type count)
{
	readAt(pos_, out, count);
	pos_ += count;
	return *this;
}

void ReadBuffer::readAt(pos_type pos, void* dest, size_type count) const
{
	if (size() < pos) [[unlikely]] {
		// TODO: Fill in exception message
		throw std::out_of_range("");
	}

	std::memmove(dest, data_ + pos, count);
}

void ReadBuffer::readAt(pos_type pos, std::ostream& out, size_type count) const
{
	if (size() < pos) [[unlikely]] {
		// TODO: Fill in exception message
		throw std::out_of_range("");
	}

	out.write(reinterpret_cast<char const*>(data_ + pos),
	          static_cast<std::streamsize>(count));
}

void ReadBuffer::readAt(offset_type off, IODir dir, void* dest, size_type count) const
{
	readAt(pos(pos_, off, dir), dest, count);
}

void ReadBuffer::readAt(offset_type off, IODir dir, std::ostream& out,
                        size_type count) const
{
	readAt(pos(pos_, off, dir), out, count);
}

bool ReadBuffer::readLine(std::string& line)
{
	// FIXME: Implement correct
	std::byte const* it = std::find_if(data_ + pos_, data_ + size_, [](std::byte b) {
		return '\n' == static_cast<char>(b);
	});
	line.assign(reinterpret_cast<char const*>(data_ + pos_),
	            reinterpret_cast<char const*>(it));
	pos_ = std::min(size_, pos_ + line.size() + 1);
	return pos_ != size_;
}

ReadBuffer::pos_type ReadBuffer::readPos() const noexcept { return pos_; }

ReadBuffer& ReadBuffer::readSeek(pos_type pos) noexcept
{
	pos_ = pos;
	return *this;
}

ReadBuffer& ReadBuffer::readSeek(offset_type off, IODir dir) noexcept
{
	return readSeek(pos(pos_, off, dir));
}

ReadBuffer::size_type ReadBuffer::readLeft() const noexcept
{
	return size_ < pos_ ? 0 : pos_ - size_;
}
}  // namespace ufo