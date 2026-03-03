/**
 * @brief All vision-related classes and functions.
 * @author Daniel Duberg (danielduberg@gmail.com)
 * @see https://github.com/UnknownFreeOccupied/ufo
 * @version 1.0
 * @date 2026-02-22
 *
 * @copyright
 * BSD 3-Clause License
 *
 * Copyright (c) 2020-2026, Daniel Duberg
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

#ifndef UFO_VISION_IMAGE_HPP
#define UFO_VISION_IMAGE_HPP

// UFO
#include <ufo/vision/color.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <format>
#include <iterator>
#include <memory>
#include <ranges>
#include <span>
#include <stdexcept>
#include <version>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief Image class for storing and manipulating 2D pixel data.
 * @tparam T The pixel type. Usually a `ufo::Color` or scalar.
 * @details
 * `Image` owns its pixel data and provides both flat (single-index) and
 * two-dimensional (row, col) element access. The storage layout is contiguous and
 * row-major, meaning that pixels in the same row are adjacent in memory.
 *
 * The class satisfies `std::ranges::contiguous_range`, so it can be used directly
 * with range algorithms and range-based for loops.
 */
template <class T>
class Image
{
 public:
	/**
	 * @brief The type of the elements stored in the image.
	 */
	using value_type = T;
	/**
	 * @brief The type of the size of the image.
	 */
	using size_type = std::size_t;
	/**
	 * @brief The type of the difference between two sizes.
	 */
	using difference_type = std::ptrdiff_t;
	/**
	 * @brief The type of a reference to an element.
	 */
	using reference = T&;
	/**
	 * @brief The type of a const reference to an element.
	 */
	using const_reference = T const&;
	/**
	 * @brief The type of a pointer to an element.
	 */
	using pointer = T*;
	/**
	 * @brief The type of a const pointer to an element.
	 */
	using const_pointer = T const*;
	/**
	 * @brief The type of an iterator to an element.
	 */
	using iterator = T*;
	/**
	 * @brief The type of a const iterator to an element.
	 */
	using const_iterator = T const*;
	/**
	 * @brief The type of a reverse iterator to an element.
	 */
	using reverse_iterator = std::reverse_iterator<iterator>;
	/**
	 * @brief The type of a const reverse iterator to an element.
	 */
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	/**
	 * @brief Constructs an empty image with no pixels (rows = cols = 0).
	 */
	Image() = default;

	/**
	 * @brief Copy-constructs an image, performing a deep copy of all pixel data.
	 * @param [in] other The image to copy from.
	 */
	Image(Image const& other)
	    : data_(std::make_unique_for_overwrite<T[]>(other.size()))
	    , rows_(other.rows_)
	    , cols_(other.cols_)
	{
		std::ranges::copy(other, begin());
	}

	/**
	 * @brief Move-constructs an image, transferring ownership of pixel data.
	 * @details
	 * The moved-from image is left in a valid but empty state.
	 */
	Image(Image&&) = default;

	/**
	 * @brief Constructs an image with the given dimensions, filling every pixel with
	 * `value`.
	 * @param [in] rows Number of rows (height).
	 * @param [in] cols Number of columns (width).
	 * @param [in] value Initial value for every pixel. Defaults to a value-initialized `T`.
	 */
	Image(size_type rows, size_type cols, T const& value = T{})
	    : data_(std::make_unique_for_overwrite<T[]>(rows * cols)), rows_(rows), cols_(cols)
	{
		std::ranges::fill(*this, value);
	}

	/**
	 * @brief Copy-assigns from another image, performing a deep copy of all pixel data.
	 * @param [in] rhs The image to copy from.
	 * @return Reference to this image.
	 * @details
	 * Self-assignment is a no-op.
	 */
	Image& operator=(Image const& rhs)
	{
		if (this != &rhs) {
			resize(rhs.rows_, rhs.cols_);
			std::ranges::copy(rhs, begin());
		}
		return *this;
	}

	/**
	 * @brief Move-assigns from another image, transferring ownership of pixel data.
	 * @return Reference to this image.
	 * @details
	 * The moved-from image is left in a valid but empty state.
	 */
	Image& operator=(Image&&) = default;

	/**
	 * @brief Returns a (const) reference to the pixel at the given flat index.
	 * @param [in] self The image.
	 * @param [in] index Flat index in `[0, size())`.
	 * @retval reference if `self` is a mutable reference.
	 * @retval const_reference if `self` is a const reference.
	 * @note No bounds checking is performed.
	 * @details
	 * The flat index corresponds to row-major storage: `index = row * cols() + col`.
	 */
	[[nodiscard]] auto& operator[](this auto& self, size_type index)
	{
		return self.data_[index];
	}

	/**
	 * @brief Returns a (const) reference to the pixel at (row, col).
	 * @param [in] self The image.
	 * @param [in] row Row index in `[0, rows())`.
	 * @param [in] col Column index in `[0, cols())`.
	 * @retval reference if `self` is a mutable reference.
	 * @retval const_reference if `self` is a const reference.
	 * @note No bounds checking is performed.
	 */
	[[nodiscard]] auto& operator[](this auto& self, size_type row, size_type col)
	{
		return self.data_[self.index(row, col)];
	}

	/**
	 * @brief Returns a (const) reference to the pixel at (row, col) with bounds checking.
	 * @param [in] self The image.
	 * @param [in] row Row index.
	 * @param [in] col Column index.
	 * @retval reference if `self` is a mutable reference.
	 * @retval const_reference if `self` is a const reference.
	 * @throws std::out_of_range if `row >= rows()` or `col >= cols()`.
	 */
	[[nodiscard]] auto& at(this auto& self, size_type row, size_type col)
	{
		if (self.rows() <= row || self.cols() <= col) {
			throw std::out_of_range(
			    std::format("Image::at({}, {}): index out of range [0, {}) x [0, {})", row, col,
			                self.rows(), self.cols()));
		}
		return self.data_[self.index(row, col)];
	}

	/**
	 * @brief Returns a span over all pixels in the given row.
	 * @param [in] r Row index in `[0, rows())`. Passing an out-of-range index is undefined
	 *          behavior.
	 * @return A `std::span<T>` covering the `cols()` pixels in row `r`.
	 * @details
	 * The span refers directly into the image's storage; no copy is made.
	 */
	[[nodiscard]] std::span<T> row(size_type r) noexcept
	{
		return {data_.get() + r * cols_, cols_};
	}

	/**
	 * @brief Returns a const span over all pixels in the given row.
	 * @param r Row index in `[0, rows())`.
	 * @return A `std::span<T const>` covering the `cols()` pixels in row `r`.
	 */
	[[nodiscard]] std::span<T const> row(size_type r) const noexcept
	{
		return {data_.get() + r * cols_, cols_};
	}

	/**
	 * @brief Returns a lazy view over all pixels in the given column.
	 * @param [in] self The image.
	 * @param [in] c Column index in `[0, cols())`. Passing an out-of-range index is
	 *          undefined behavior.
	 * @return A range of `rows()` `T` (const) references, one per row, in top-to-bottom
	 * order.
	 * @details
	 * Because storage is row-major, column pixels are not contiguous in memory
	 * (they are `cols()` elements apart). A view is returned rather than a span,
	 * yielding references directly into the image's storage with no copying.
	 */
	[[nodiscard]] auto column(this auto& self, size_type c) noexcept
	{
		return std::ranges::iota_view(size_type{}, self.rows_) |
		       std::views::transform(
		           [&self, c](size_type r) -> decltype(auto) { return self[r, c]; });
	}

	/**
	 * @brief Returns an iterator to the first pixel.
	 * @param [in] self The image.
	 * @retval iterator if `self` is a mutable reference.
	 * @retval const_iterator if `self` is a const reference
	 */
	[[nodiscard]] auto begin(this auto& self) noexcept { return self.data_.get(); }

	/**
	 * @brief Returns a const iterator to the first pixel.
	 * @return A const iterator to the first pixel.
	 */
	[[nodiscard]] const_iterator cbegin() const noexcept { return begin(); }

	/**
	 * @brief Returns a reverse iterator to the last pixel.
	 * @param [in] self The image.
	 * @retval reverse_iterator if `self` is a mutable reference.
	 * @retval const_reverse_iterator if `self` is a const reference
	 */
	[[nodiscard]] auto rbegin(this auto& self) noexcept
	{
		return std::make_reverse_iterator(self.end());
	}

	/**
	 * @brief Returns a const reverse iterator to the last pixel.
	 * @return A const reverse iterator to the last pixel.
	 */
	[[nodiscard]] const_reverse_iterator crbegin() const noexcept { return rbegin(); }

	/**
	 * @brief Returns an iterator one past the last pixel.
	 * @param [in] self The image.
	 * @retval iterator if `self` is a mutable reference.
	 * @retval const_iterator if `self` is a const reference
	 */
	[[nodiscard]] auto end(this auto& self) noexcept
	{
		return self.data_.get() + self.size();
	}

	/**
	 * @brief Returns a const iterator one past the last pixel.
	 * @return A const iterator one past the last pixel.
	 */
	[[nodiscard]] const_iterator cend() const noexcept { return end(); }

	/**
	 * @brief Returns a reverse iterator to one before the first pixel.
	 * @param [in] self The image.
	 * @retval reverse_iterator if `self` is a mutable reference.
	 * @retval const_reverse_iterator if `self` is a const reference
	 */
	[[nodiscard]] auto rend(this auto& self) noexcept
	{
		return std::make_reverse_iterator(self.begin());
	}

	/**
	 * @brief Returns a const reverse iterator to one before the first pixel.
	 * @return A const reverse iterator to one before the first pixel.
	 */
	[[nodiscard]] const_reverse_iterator crend() const noexcept { return rend(); }

	/**
	 * @brief Returns a pointer to the underlying contiguous pixel array.
	 * @param [in] self The image.
	 * @retval pointer to the underlying contiguous pixel array if `self` is a mutable
	 * reference.
	 * @retval const_pointer to the underlying contiguous pixel array if `self` is a
	 * const reference
	 */
	[[nodiscard]] auto data(this auto& self) noexcept { return self.data_.get(); }

	/**
	 * @brief Returns `true` if the image has no pixels (i.e., `size() == 0`).
	 * @retval true if the image has no pixels.
	 * @retval false if the image has pixels.
	 */
	[[nodiscard]] bool empty() const noexcept { return 0 == size(); }

	/**
	 * @brief Returns the number of rows (height) in the image.
	 * @return The number of rows.
	 */
	[[nodiscard]] constexpr size_type rows() const noexcept { return rows_; }

	/**
	 * @brief Returns the number of columns (width) in the image.
	 * @return The number of columns.
	 */
	[[nodiscard]] constexpr size_type cols() const noexcept { return cols_; }

	/**
	 * @brief Returns the total number of pixels (`rows() * cols()`).
	 * @return The total number of pixels.
	 */
	[[nodiscard]] constexpr size_type size() const noexcept { return rows_ * cols_; }

	/**
	 * @brief Computes the flat row-major index for position (row, col).
	 * @param [in] row Row index.
	 * @param [in] col Column index.
	 * @return Flat index into the pixel array.
	 * @details
	 * Equivalent to `row * cols() + col`. No bounds checking is performed.
	 */
	[[nodiscard]] constexpr size_type index(size_type row, size_type col) const noexcept
	{
		return row * cols_ + col;
	}

	/**
	 * @brief Swaps the contents of this image with `other`.
	 * @param [in,out] other The image to swap with.
	 * @details
	 * Runs in O(1); no pixel data is copied.
	 */
	void swap(Image& other) noexcept
	{
		using std::swap;
		swap(data_, other.data_);
		swap(rows_, other.rows_);
		swap(cols_, other.cols_);
	}

	/**
	 * @brief Resizes the image to the given dimensions.
	 * @param [in] rows New number of rows.
	 * @param [in] cols New number of columns.
	 * @details
	 * Reallocates memory only if the total number of pixels changes.
	 * The contents of the image are undefined after a resize.
	 */
	void resize(size_type rows, size_type cols)
	{
		if (rows * cols != size()) {
			data_ = std::make_unique_for_overwrite<T[]>(rows * cols);
		}
		rows_ = rows;
		cols_ = cols;
	}

	/**
	 * @brief Fills the image with the given value.
	 * @param [in] value The value to fill the image with.
	 */
	void fill(T const& value) { std::ranges::fill(*this, value); }

	/**
	 * @brief Clears the image, releasing all memory.
	 */
	void clear() noexcept
	{
		data_.reset();
		rows_ = 0;
		cols_ = 0;
	}

	/**
	 * @brief Returns a transposed copy of the image.
	 * @return A new image with rows and columns swapped.
	 */
	[[nodiscard]] Image transposed() const
	{
		Image ret(cols_, rows_);
		for (size_type r = 0; r < rows(); ++r) {
			for (size_type c = 0; c < cols(); ++c) {
				ret[c, r] = (*this)[r, c];
			}
		}
		return ret;
	}

	/**
	 * @brief Flips the image horizontally in-place.
	 */
	void flipHorizontal() noexcept
	{
		for (size_type r = 0; r < rows(); ++r) {
			auto r_span = row(r);
			std::ranges::reverse(r_span);
		}
	}

	/**
	 * @brief Flips the image vertically in-place.
	 */
	void flipVertical() noexcept
	{
		for (size_type r = 0; r < rows() / 2; ++r) {
			auto r1 = row(r);
			auto r2 = row(rows() - 1 - r);
			std::ranges::swap_ranges(r1, r2);
		}
	}

	/**
	 * @brief Returns a rotated copy of the image.
	 * @param [in] clockwise If true, rotate 90 degrees clockwise. Otherwise,
	 * counter-clockwise.
	 * @return A new rotated image.
	 */
	[[nodiscard]] Image rotate90(bool clockwise = true) const
	{
		Image ret(cols_, rows_);
		if (clockwise) {
			for (size_type r = 0; r < rows(); ++r) {
				for (size_type c = 0; c < cols(); ++c) {
					ret[c, rows() - 1 - r] = (*this)[r, c];
				}
			}
		} else {
			for (size_type r = 0; r < rows(); ++r) {
				for (size_type c = 0; c < cols(); ++c) {
					ret[cols() - 1 - c, r] = (*this)[r, c];
				}
			}
		}
		return ret;
	}

	/**
	 * @brief Samples the image at (row, col) using bilinear interpolation.
	 * @param [in] r Normalized row index in [0, 1].
	 * @param [in] c Normalized column index in [0, 1].
	 * @return The sampled value.
	 * @details
	 * Coordinates are clamped to [0, 1]. The normalized coordinates are mapped
	 * to pixel coordinates by multiplying by (rows - 1) and (cols - 1).
	 */
	[[nodiscard]] T sample(float r, float c) const
	{
		if (empty()) {
			return T{};
		}

		r = std::clamp(r, 0.0f, 1.0f) * static_cast<float>(rows() - 1);
		c = std::clamp(c, 0.0f, 1.0f) * static_cast<float>(cols() - 1);

		size_type r0 = static_cast<size_type>(std::floor(r));
		size_type c0 = static_cast<size_type>(std::floor(c));
		size_type r1 = std::min(r0 + 1, rows() - 1);
		size_type c1 = std::min(c0 + 1, cols() - 1);

		float dr = r - static_cast<float>(r0);
		float dc = c - static_cast<float>(c0);

		auto s00 = (*this)[r0, c0];
		auto s01 = (*this)[r0, c1];
		auto s10 = (*this)[r1, c0];
		auto s11 = (*this)[r1, c1];

		if constexpr (requires { s00 * 1.0f; }) {
			return static_cast<T>(s00 * ((1.0f - dr) * (1.0f - dc)) + s01 * ((1.0f - dr) * dc) +
			                      s10 * (dr * (1.0f - dc)) + s11 * (dr * dc));
		} else {
			return (dr < 0.5f) ? ((dc < 0.5f) ? s00 : s01) : ((dc < 0.5f) ? s10 : s11);
		}
	}

	/**
	 * @brief Samples the image at pixel (row, col) using bilinear interpolation.
	 * @param [in] r Floating-point row index.
	 * @param [in] c Floating-point column index.
	 * @return The sampled value.
	 * @details
	 * Coordinates are clamped to the image boundaries.
	 */
	[[nodiscard]] T samplePixel(float r, float c) const
	{
		if (empty()) {
			return T{};
		}
		float r_norm = (rows() > 1) ? r / static_cast<float>(rows() - 1) : 0.0f;
		float c_norm = (cols() > 1) ? c / static_cast<float>(cols() - 1) : 0.0f;
		return sample(r_norm, c_norm);
	}

	/**
	 * @brief Rescales the image to the given dimensions using bilinear interpolation.
	 * @param [in] rows New number of rows.
	 * @param [in] cols New number of columns.
	 */
	void rescale(size_type rows, size_type cols)
	{
		if (rows == rows_ && cols == cols_) {
			return;
		}
		*this = rescaled(rows, cols);
	}

	/**
	 * @brief Returns a rescaled copy of the image using bilinear interpolation.
	 * @param [in] rows New number of rows.
	 * @param [in] cols New number of columns.
	 * @return A new rescaled image.
	 */
	[[nodiscard]] Image rescaled(size_type rows, size_type cols) const
	{
		Image ret(rows, cols);
		for (size_type r = 0; r < rows; ++r) {
			float r_norm = (rows > 1) ? static_cast<float>(r) / (rows - 1) : 0.0f;
			for (size_type c = 0; c < cols; ++c) {
				float c_norm = (cols > 1) ? static_cast<float>(c) / (cols - 1) : 0.0f;
				ret[r, c]    = sample(r_norm, c_norm);
			}
		}
		return ret;
	}

	/**
	 * @brief Upscales the image by the given factor.
	 * @param [in] factor Upscale factor (> 1).
	 */
	void upscale(float factor)
	{
		rescale(static_cast<size_type>(rows() * factor),
		        static_cast<size_type>(cols() * factor));
	}

	/**
	 * @brief Downscales the image by the given factor.
	 * @param [in] factor Downscale factor (> 1).
	 */
	void downscale(float factor)
	{
		rescale(static_cast<size_type>(rows() / factor),
		        static_cast<size_type>(cols() / factor));
	}

 private:
	std::unique_ptr<T[]> data_;
	size_type            rows_{};
	size_type            cols_{};
};

/**
 * @brief Swaps two images in O(1).
 * @param lhs First image.
 * @param rhs Second image.
 */
template <class T>
void swap(Image<T>& lhs, Image<T>& rhs) noexcept
{
	lhs.swap(rhs);
}

/**
 * @brief An image with 8-bit per channel gray pixels.
 */
using ImageSmallGray = Image<SmallGray>;
/**
 * @brief An image with 32-bit per channel gray pixels.
 */
using ImageFineGray = Image<FineGray>;
/**
 * @brief An image with 8-bit per channel RGB pixels.
 */
using ImageSmallRgb = Image<SmallRgb>;
/**
 * @brief An image with 8-bit per channel RGBA pixels.
 */
using ImageSmallRgbA = Image<SmallRgbA>;
/**
 * @brief An image with 32-bit per channel RGB pixels.
 */
using ImageFineRgb = Image<FineRgb>;
/**
 * @brief An image with 32-bit per channel RGBA pixels.
 */
using ImageFineRgbA = Image<FineRgbA>;

//
// Convert
//

/**
 * @brief Converts an image from one pixel type to another (in-place output).
 * @tparam From Source pixel type.
 * @tparam To   Destination pixel type.
 * @param [in] src  The source image.
 * @param [out] dest The destination image. Resized to match `src` dimensions.
 * @details
 * If `From` and `To` are the same type, the image is copied directly. Otherwise,
 * each pixel is converted element-wise using `convert<To>(pixel)`.
 */
template <class From, class To>
void convert(Image<From> const& src, Image<To>& dest)
{
	if constexpr (std::is_same_v<To, From>) {
		dest = src;
	} else {
		dest.resize(src.rows(), src.cols());
		std::ranges::transform(src, dest.begin(),
		                       [](auto const& x) { return convert<To>(x); });
	}
}

/**
 * @brief Converts an image from one pixel type to another, returning the result.
 * @tparam To   Destination pixel type.
 * @tparam From Source pixel type (deduced).
 * @param [in] image The source image.
 * @return A new `Image<To>` with the converted pixel data.
 * @details
 * If `From` and `To` are the same type, the image is returned by copy directly.
 * Otherwise, each pixel is converted element-wise using `convert<To>(pixel)`.
 */
template <class To, class From>
[[nodiscard]] Image<To> convert(Image<From> const& image)
{
	if constexpr (std::is_same_v<To, From>) {
		return image;
	} else {
		Image<To> ret(image.rows(), image.cols());
		std::ranges::transform(image, ret.begin(),
		                       [](auto const& x) { return convert<To>(x); });
		return ret;
	}
}

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_IMAGE_HPP
