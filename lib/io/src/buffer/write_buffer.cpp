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
#include <ufo/utility/io/write_buffer.hpp>

// STL
#include <cstring>

namespace ufo
{

WriteBuffer::WriteBuffer(WriteBuffer const& other)
{
	if (other.data_) {
		write(other.data_.get(), other.size_);
	}
}

WriteBuffer& WriteBuffer::operator=(WriteBuffer const& rhs)
{
	size_ = {};
	pos_  = {};
	if (rhs.data_) {
		write(rhs.data_.get(), rhs.size_);
	}
	return *this;
}

WriteBuffer& WriteBuffer::write(void const* src, size_type count)
{
	writeAt(pos_, src, count);
	pos_ += count;
	return *this;
}

WriteBuffer& WriteBuffer::write(std::istream& in, size_type count)
{
	writeAt(pos_, in, count);
	pos_ += count;
	return *this;
}

void WriteBuffer::writeAt(pos_type pos, void const* src, size_type count)
{
	reserve(pos + count);

	std::memmove(data_.get() + pos, src, count);

	size_ = std::max(size_, pos + count);
}

void WriteBuffer::writeAt(pos_type pos, std::istream& in, size_type count)
{
	reserve(pos + count);

	in.read(reinterpret_cast<char*>(data_.get() + pos),
	        static_cast<std::streamsize>(count));

	size_ = std::max(size_, pos + count);
}

void WriteBuffer::writeAt(offset_type off, IODir dir, void const* src, size_type count)
{
	return writeAt(pos(pos_, off, dir), src, count);
}

void WriteBuffer::writeAt(offset_type off, IODir dir, std::istream& in, size_type count)
{
	return writeAt(pos(pos_, off, dir), in, count);
}

void WriteBuffer::reserve(size_type new_cap)
{
	if (cap_ >= new_cap) [[likely]] {
		return;
	}

	std::byte* prev = data_.get();
	void*      next = realloc(prev, new_cap * sizeof(std::byte));

	if (nullptr != next) [[likely]] {
		[[maybe_unused]] auto prev = data_.release();
		data_.reset(static_cast<std::byte*>(next));
		BaseBuffer::data_ = static_cast<std::byte const*>(next);
		cap_              = new_cap;
	} else [[unlikely]] {
		free(prev);
		BaseBuffer::data_ = nullptr;
		size_             = {};
		cap_              = {};
		pos_              = {};
		// TODO: Add message
		throw std::bad_alloc();
	}
}

void WriteBuffer::clear()
{
	size_ = 0;
	pos_  = 0;
}

std::byte* WriteBuffer::data() { return data_.get(); }

WriteBuffer::size_type WriteBuffer::capacity() const noexcept { return cap_; }

WriteBuffer::pos_type WriteBuffer::writePos() const noexcept { return pos_; }

WriteBuffer& WriteBuffer::writeSeek(pos_type pos) noexcept
{
	pos_ = pos;
	return *this;
}

WriteBuffer& WriteBuffer::writeSeek(offset_type off, IODir dir) noexcept
{
	return writeSeek(pos(pos_, off, dir));
}

WriteBuffer::size_type WriteBuffer::writeLeft() const noexcept
{
	return size_ < pos_ ? 0 : pos_ - size_;
}
}  // namespace ufo