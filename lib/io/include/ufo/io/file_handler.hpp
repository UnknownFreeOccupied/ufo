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

#ifndef UFO_IO_FILE_HANDLER_HPP
#define UFO_IO_FILE_HANDLER_HPP

// STL
#include <array>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <string_view>

namespace ufo
{
/**
 * @brief RAII wrapper around a C `std::FILE*` handle.
 *
 * `FileHandler` manages the lifetime of a C file pointer obtained via `std::fopen`,
 * ensuring the file is properly closed when the handler goes out of scope or is
 * reassigned. It is move-only — copying a `FILE*` has no meaningful semantics.
 *
 * A fixed-size internal buffer supports line-by-line reading via `readline()`.
 *
 * ### Example
 * ```cpp
 * ufo::FileHandler fh("data.txt", "r");
 * if (!fh) throw std::runtime_error("failed to open");
 * while (char* line = fh.readline()) {
 *     // process line
 * }
 * ```
 */
class FileHandler
{
 public:
	//! Maximum number of characters (including the null terminator) read by `readline()`.
	static constexpr std::size_t buffer_size = 1024;

	//! @brief Constructs an empty (closed) file handler.
	FileHandler() = default;

	/**
	 * @brief Opens `file` in the given `modes`.
	 *
	 * @param file   Path to the file to open.
	 * @param modes  Null-terminated `fopen` mode string (e.g. `"r"`, `"wb"`).
	 */
	FileHandler(std::filesystem::path const& file, std::string_view modes);

	//! Copying a `FILE*` handle has no meaningful semantics.
	FileHandler(FileHandler const&)            = delete;
	FileHandler& operator=(FileHandler const&) = delete;

	/**
	 * @brief Move-constructs by transferring ownership of the file handle.
	 *
	 * @post `other` is left in the closed (null) state.
	 */
	FileHandler(FileHandler&& other) noexcept;

	/**
	 * @brief Move-assigns by closing any currently open file, then taking ownership.
	 *
	 * Self-assignment is handled safely.
	 *
	 * @post `other` is left in the closed (null) state.
	 */
	FileHandler& operator=(FileHandler&& other) noexcept;

	//! @brief Closes the managed file handle (if open).
	~FileHandler();

	/**
	 * @brief Closes any currently open file, then opens `file` in `modes`.
	 *
	 * @param file   Path to the file to open.
	 * @param modes  Null-terminated `fopen` mode string (e.g. `"r"`, `"wb"`).
	 */
	void open(std::filesystem::path const& file, std::string_view modes);

	/**
	 * @brief Closes the managed file handle and resets the pointer to null.
	 *
	 * No-op if the handle is already closed.
	 */
	void close() noexcept;

	/**
	 * @brief Returns the underlying `FILE*`.
	 *
	 * @return The managed file pointer, or `nullptr` if not open.
	 */
	[[nodiscard]] std::FILE* get() const noexcept;

	/**
	 * @brief Reads one line from the file into the internal buffer.
	 *
	 * Wraps `std::fgets`. Reads at most `buffer_size - 1` characters (plus a null
	 * terminator). The newline character, if present, is retained in the buffer.
	 *
	 * @return Pointer to the internal buffer on success, or `nullptr` at EOF or on
	 *         error. The returned pointer is valid until the next call to `readline()`
	 *         or until the handler is destroyed / closed.
	 */
	[[nodiscard]] char* readline();

	/**
	 * @brief Returns `true` if the file is currently open (handle is non-null).
	 */
	[[nodiscard]] explicit operator bool() const noexcept;

 private:
	std::FILE*                    fp_ = nullptr;
	std::array<char, buffer_size> buffer_;
};
}  // namespace ufo

#endif  // UFO_IO_FILE_HANDLER_HPP
