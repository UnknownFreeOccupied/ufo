/**
 * @author Daniel Duberg (danielduberg@gmail.com)
 * @see https://github.com/UnknownFreeOccupied/ufo
 * @version 1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) 2020-2026, Daniel Duberg
 *
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

#ifndef UFO_UTILITY_STRING_HPP
#define UFO_UTILITY_STRING_HPP

// STL
#include <cctype>
#include <string>
#include <vector>

namespace ufo
{
/**
 * @brief Splits a string into a vector of substrings using a delimiter.
 * @param s Input string
 * @param delimiter Delimiter character
 * @return Vector of substrings
 */
[[nodiscard]] std::vector<std::string> split(std::string const& s, char delimiter);

/**
 * @brief Joins a vector of strings into a single string with a delimiter.
 * @param strings Vector of strings
 * @param delimiter Delimiter character
 * @return Joined string
 */
[[nodiscard]] std::string join(std::vector<std::string> const& strings, char delimiter);

/**
 * @brief Checks if a string starts with a given prefix.
 * @param s Input string
 * @param prefix Prefix string
 * @return true if s starts with prefix
 */
[[nodiscard]] bool startsWith(std::string const& s, std::string const& prefix);

/**
 * @brief Checks if a string ends with a given suffix.
 * @param s Input string
 * @param suffix Suffix string
 * @return true if s ends with suffix
 */
[[nodiscard]] bool endsWith(std::string const& s, std::string const& suffix);

/**
 * @brief Checks if a string contains a substring.
 * @param s Input string
 * @param sub Substring
 * @return true if s contains sub
 */
[[nodiscard]] bool contains(std::string const& s, std::string const& sub);

/**
 * @brief Converts all characters in the string to lowercase.
 * @param s Input string (copy)
 * @return Lowercase version of the input string
 */
[[nodiscard]] std::string tolower(std::string s);

/**
 * @brief Converts all characters in the string to uppercase.
 * @param s Input string (copy)
 * @return Uppercase version of the input string
 */
[[nodiscard]] std::string toupper(std::string s);

/**
 * @brief Removes leading whitespace from the string (in-place).
 * @param s Input string (modified)
 */
void ltrim(std::string& s);

/**
 * @brief Removes trailing whitespace from the string (in-place).
 * @param s Input string (modified)
 */
void rtrim(std::string& s);

/**
 * @brief Removes leading and trailing whitespace from the string (in-place).
 * @param s Input string (modified)
 */
void trim(std::string& s);

/**
 * @brief Returns a copy of the string with leading whitespace removed.
 * @param s Input string (copy)
 * @return String with leading whitespace removed
 */
[[nodiscard]] std::string ltrimCopy(std::string s);

/**
 * @brief Returns a copy of the string with trailing whitespace removed.
 * @param s Input string (copy)
 * @return String with trailing whitespace removed
 */
[[nodiscard]] std::string rtrimCopy(std::string s);

/**
 * @brief Returns a copy of the string with leading and trailing whitespace removed.
 * @param s Input string (copy)
 * @return String with leading and trailing whitespace removed
 */
[[nodiscard]] std::string trimCopy(std::string s);
}  // namespace ufo

#endif  // UFO_UTILITY_STRING_HPP