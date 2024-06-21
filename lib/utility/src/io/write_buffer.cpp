// UFO
#include <ufo/utility/io/write_buffer.hpp>

// STL
#include <cstring>

namespace ufo
{
WriteBuffer::~WriteBuffer()
{
	if (data_) {
		free(data_);
	}
}

WriteBuffer& WriteBuffer::write(void const* src, size_type count)
{
	if (capacity() < pos_ + count) {
		reserve(pos_ + count);
	}

	std::memmove(data_ + pos_, src, count);

	pos_ += count;
	size_ = std::max(size_, pos_);

	return *this;
}

WriteBuffer& WriteBuffer::write(std::istream& in, size_type count)
{
	if (capacity() < pos_ + count) {
		reserve(pos_ + count);
	}

	in.read(reinterpret_cast<char*>(data_), static_cast<std::streamsize>(count));

	pos_ += count;
	size_ = std::max(size_, pos_);

	return *this;
}

void WriteBuffer::reserve(size_type new_cap)
{
	if (cap_ >= new_cap) {
		return;
	}

	if (auto p_new = static_cast<std::byte*>(realloc(data_, new_cap * sizeof(std::byte)))) {
		data_ = p_new;
		cap_  = new_cap;
	} else {
		throw std::bad_alloc();
	}
}

void WriteBuffer::resize(size_type new_size)
{
	reserve(new_size);
	size_ = new_size;
}

void WriteBuffer::clear()
{
	size_ = 0;
	pos_  = 0;
}

std::byte* WriteBuffer::data() { return data_; }

std::byte const* WriteBuffer::data() const { return data_; }

bool WriteBuffer::empty() const { return 0 == size(); }

WriteBuffer::size_type WriteBuffer::size() const { return size_; }

WriteBuffer::size_type WriteBuffer::capacity() const noexcept { return cap_; }

WriteBuffer::size_type WriteBuffer::writePos() const noexcept { return pos_; }

void WriteBuffer::skipWrite(size_type count) noexcept { pos_ += count; }

void WriteBuffer::setWritePos(size_type pos) noexcept { pos_ = pos; }

WriteBuffer::size_type WriteBuffer::writeLeft() const noexcept
{
	return size_ < pos_ ? 0 : pos_ - size_;
}
}  // namespace ufo