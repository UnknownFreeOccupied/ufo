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

// UFO
#include <ufo/utility/string.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

std::vector<std::string> ufo::split(std::string const& s, char delimiter)
{
	std::vector<std::string> result;
	std::string              token;
	std::istringstream       stream(s);
	while (std::getline(stream, token, delimiter)) {
		result.push_back(token);
	}
	return result;
}

std::string ufo::join(std::vector<std::string> const& strings, char delimiter)
{
	if (strings.empty()) {
		return "";
	}

	std::string result = strings.front();
	for (std::size_t i{1}; strings.size() > i; ++i) {
		result += delimiter;
		result += strings[i];
	}
	return result;
}

bool ufo::startsWith(std::string const& s, std::string const& prefix)
{
	return s.size() >= prefix.size() && std::equal(prefix.begin(), prefix.end(), s.begin());
}

bool ufo::endsWith(std::string const& s, std::string const& suffix)
{
	return s.size() >= suffix.size() &&
	       std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

bool ufo::contains(std::string const& s, std::string const& sub)
{
	return s.find(sub) != std::string::npos;
}

std::string ufo::tolower(std::string s)
{
	std::transform(s.begin(), s.end(), s.begin(),
	               [](unsigned char c) { return std::tolower(c); });
	return s;
}

std::string ufo::toupper(std::string s)
{
	std::transform(s.begin(), s.end(), s.begin(),
	               [](unsigned char c) { return std::toupper(c); });
	return s;
}

void ufo::ltrim(std::string& s)
{
	s.erase(s.begin(), std::find_if(s.begin(), s.end(),
	                                [](unsigned char ch) { return !std::isspace(ch); }));
}

void ufo::rtrim(std::string& s)
{
	s.erase(std::find_if(s.rbegin(), s.rend(),
	                     [](unsigned char ch) { return !std::isspace(ch); })
	            .base(),
	        s.end());
}

void ufo::trim(std::string& s)
{
	rtrim(s);
	ltrim(s);
}

std::string ufo::ltrimCopy(std::string s)
{
	ltrim(s);
	return s;
}

std::string ufo::rtrimCopy(std::string s)
{
	rtrim(s);
	return s;
}

std::string ufo::trimCopy(std::string s)
{
	trim(s);
	return s;
}