/*!
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

#ifndef UFO_UTILITY_TIMING_HPP
#define UFO_UTILITY_TIMING_HPP

// UFO
#include <ufo/utility/timer.hpp>

// STL
#include <array>
#include <cstdlib>
#include <initializer_list>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace ufo
{
class Timing : public Timer
{
 public:
	Timing() = default;

	Timing(std::string const& tag);

	Timing(std::string const&                                          tag,
	       std::initializer_list<std::pair<std::size_t const, Timing>> init);

	using Timer::start;

	void start(std::string const& tag);

	void start(std::string const& tag, std::string const& color);

	using Timer::reset;

	Timing const& operator[](std::size_t num) const;

	Timing& operator[](std::size_t num);

	std::string const& tag() const;

	void setTag(std::string const& tag);

	std::string const& color() const;

	void setColor(std::string const& color);

	static constexpr char const* resetColor() { return "\033[0m"; }
	static constexpr char const* blackColor() { return "\033[30m"; }
	static constexpr char const* redColor() { return "\033[31m"; }
	static constexpr char const* greenColor() { return "\033[32m"; }
	static constexpr char const* yellowColor() { return "\033[33m"; }
	static constexpr char const* blueColor() { return "\033[34m"; }
	static constexpr char const* magentaColor() { return "\033[35m"; }
	static constexpr char const* cyanColor() { return "\033[36m"; }
	static constexpr char const* whiteColor() { return "\033[37m"; }
	static constexpr char const* boldBlackColor() { return "\033[1m\033[30m"; }
	static constexpr char const* boldRedColor() { return "\033[1m\033[31m"; }
	static constexpr char const* boldGreenColor() { return "\033[1m\033[32m"; }
	static constexpr char const* boldYellowColor() { return "\033[1m\033[33m"; }
	static constexpr char const* boldBlueColor() { return "\033[1m\033[34m"; }
	static constexpr char const* boldMagentaColor() { return "\033[1m\033[35m"; }
	static constexpr char const* boldCyanColor() { return "\033[1m\033[36m"; }
	static constexpr char const* boldWhiteColor() { return "\033[1m\033[37m"; }

	template <class Period = std::chrono::seconds::period>
	void print(bool first_as_tag = false, bool random_colors = false, bool bold = false,
	           std::size_t start_numbering_level = 1,
	           std::size_t stop_numbering_level  = std::numeric_limits<std::size_t>::max(),
	           std::size_t group_colors_level    = std::numeric_limits<std::size_t>::max(),
	           int         precision             = 4) const
	{
		if (first_as_tag) {
			printf("%s timings", tag().c_str());
		} else {
			printf("Timings");
		}

		if constexpr (std::is_same_v<Period, std::chrono::nanoseconds::period>) {
			printf(" in nanoseconds (ns)\n");
		} else if constexpr (std::is_same_v<Period, std::chrono::microseconds::period>) {
			printf(" in microseconds (µs)\n");
		} else if constexpr (std::is_same_v<Period, std::chrono::milliseconds::period>) {
			printf(" in milliseconds (ms)\n");
		} else if constexpr (std::is_same_v<Period, std::chrono::seconds::period>) {
			printf(" in seconds (s)\n");
		} else if constexpr (std::is_same_v<Period, std::chrono::minutes::period>) {
			printf(" in minutes (min)\n");
		} else if constexpr (std::is_same_v<Period, std::chrono::hours::period>) {
			printf(" in hours (h)\n");
		} else {
			printf(" [PERIOD NOT SUPPORTED]\n");
		}

		using namespace std::string_literals;

		static constexpr std::array const RC{redColor(),  greenColor(),   yellowColor(),
		                                     blueColor(), magentaColor(), cyanColor(),
		                                     whiteColor()};

		std::array<std::vector<std::string>, 8> data{
		    std::vector<std::string>{"Component"s}, std::vector<std::string>{"Total"s},
		    std::vector<std::string>{"Last"s},      std::vector<std::string>{"Mean"s},
		    std::vector<std::string>{"StDev"s},     std::vector<std::string>{"Min"s},
		    std::vector<std::string>{"Max"s},       std::vector<std::string>{"Samples"s}};

		auto timers = timings(first_as_tag);

		addTags(data[0], timers, start_numbering_level, stop_numbering_level);
		addTotal(data[1], timers, precision);
		addLast(data[2], timers, precision);
		addMean(data[3], timers, precision);
		addStd(data[4], timers, precision);
		addMin(data[5], timers, precision);
		addMax(data[6], timers, precision);
		addNumSamples(data[7], timers);

		std::array<int, data.size()> width;
		for (std::size_t i{}; data.size() > i; ++i) {
			width[i] = maxLength(data[i]);
		}

		std::size_t rows = data[0].size();

		// First row special
		{
			auto [left_pad, right_pad] = centeringPadding(data[0][0], width[0]);
			printf("%*s%s%*s", left_pad, "", data[0][0].c_str(), right_pad, "");
			for (std::size_t i{1}; data.size() != i; ++i) {
				auto [left_pad, right_pad] = centeringPadding(data[i][0], width[i]);
				printf("\t%*s%s%*s", left_pad, "", data[i][0].c_str(), right_pad, "");
			}
			printf("\n");
		}

		int rng_color = 0;
		for (std::size_t i{1}; rows > i; ++i) {
			// Set color
			rng_color += timers[i - 1].level <= group_colors_level;
			printf("%s%s", bold ? "\033[1m" : "",
			       random_colors ? RC[rng_color % RC.size()]
			                     : timers[i - 1].timing->color().c_str());

			// Print tag
			printf("%-*s", width[0], data[0][i].c_str());

			// Print rest
			for (std::size_t j{1}; data.size() > j; ++j) {
				auto [left_pad, right_pad] = centeringPadding(data[j][i], width[j]);
				if ("nan" == data[j][i]) {
					// Center aligned
					printf("\t%*s%s%*s", left_pad, "", data[j][i].c_str(), right_pad, "");
				} else {
					// Left aligned
					printf("\t%*s%s%*s", 0, "", data[j][i].c_str(), left_pad + right_pad, "");
				}
			}

			// Reset color
			printf("%s\n", resetColor());
		}
	}

	void printSeconds(
	    bool first_as_tag = false, bool random_colors = false, bool bold = false,
	    std::size_t start_numbering_level = 1,
	    std::size_t stop_numbering_level  = std::numeric_limits<std::size_t>::max(),
	    std::size_t group_colors_level    = std::numeric_limits<std::size_t>::max(),
	    int         precision             = 4) const;

	void printMilliseconds(
	    bool first_as_tag = false, bool random_colors = false, bool bold = false,
	    std::size_t start_numbering_level = 1,
	    std::size_t stop_numbering_level  = std::numeric_limits<std::size_t>::max(),
	    std::size_t group_colors_level    = std::numeric_limits<std::size_t>::max(),
	    int         precision             = 4) const;

	void printMicroseconds(
	    bool first_as_tag = false, bool random_colors = false, bool bold = false,
	    std::size_t start_numbering_level = 1,
	    std::size_t stop_numbering_level  = std::numeric_limits<std::size_t>::max(),
	    std::size_t group_colors_level    = std::numeric_limits<std::size_t>::max(),
	    int         precision             = 4) const;

	void printNanoseconds(
	    bool first_as_tag = false, bool random_colors = false, bool bold = false,
	    std::size_t start_numbering_level = 1,
	    std::size_t stop_numbering_level  = std::numeric_limits<std::size_t>::max(),
	    std::size_t group_colors_level    = std::numeric_limits<std::size_t>::max(),
	    int         precision             = 4) const;

 private:
	struct TimingNL {
		Timing const* timing;
		std::size_t   num;
		int           level;

		TimingNL(Timing const* timing, std::size_t num, int level)
		    : timing(timing), num(num), level(level)
		{
		}
	};

	std::vector<TimingNL> timings(bool skip_first) const;

	void timingsRecurs(std::vector<TimingNL>& data, std::size_t num, int level) const;

	void addTags(std::vector<std::string>& data, std::vector<TimingNL> const& timers,
	             std::size_t start_numbering_level, std::size_t stop_numbering_level) const;

	template <class Period = std::chrono::seconds::period>
	void addTotal(std::vector<std::string>& data, std::vector<TimingNL> const& timers,
	              int precision) const
	{
		std::stringstream ss;
		for (auto const& t : timers) {
			ss << std::fixed << std::setprecision(precision) << t.timing->total<Period>();
			data.push_back(ss.str());
			ss = {};
		}
	}

	template <class Period = std::chrono::seconds::period>
	void addLast(std::vector<std::string>& data, std::vector<TimingNL> const& timers,
	             int precision) const
	{
		std::stringstream ss;
		for (auto const& t : timers) {
			ss << std::fixed << std::setprecision(precision) << t.timing->last<Period>();
			data.push_back(ss.str());
			ss = {};
		}
	}

	template <class Period = std::chrono::seconds::period>
	void addMean(std::vector<std::string>& data, std::vector<TimingNL> const& timers,
	             int precision) const
	{
		std::stringstream ss;
		for (auto const& t : timers) {
			ss << std::fixed << std::setprecision(precision) << t.timing->mean<Period>();
			data.push_back(ss.str());
			ss = {};
		}
	}

	template <class Period = std::chrono::seconds::period>
	void addStd(std::vector<std::string>& data, std::vector<TimingNL> const& timers,
	            int precision) const
	{
		std::stringstream ss;
		for (auto const& t : timers) {
			ss << std::fixed << std::setprecision(precision) << t.timing->std<Period>();
			data.push_back(ss.str());
			ss = {};
		}
	}

	template <class Period = std::chrono::seconds::period>
	void addMin(std::vector<std::string>& data, std::vector<TimingNL> const& timers,
	            int precision) const
	{
		std::stringstream ss;
		for (auto const& t : timers) {
			ss << std::fixed << std::setprecision(precision) << t.timing->min<Period>();
			data.push_back(ss.str());
			ss = {};
		}
	}

	template <class Period = std::chrono::seconds::period>
	void addMax(std::vector<std::string>& data, std::vector<TimingNL> const& timers,
	            int precision) const
	{
		std::stringstream ss;
		for (auto const& t : timers) {
			ss << std::fixed << std::setprecision(precision) << t.timing->max<Period>();
			data.push_back(ss.str());
			ss = {};
		}
	}

	void addNumSamples(std::vector<std::string>&    data,
	                   std::vector<TimingNL> const& timers) const;

	int maxLength(std::vector<std::string> const& data) const;

	std::pair<int, int> centeringPadding(std::string const& str, int max_width) const;

 private:
	std::map<std::size_t, Timing> timer_;
	std::string                   tag_;
	std::string                   color_;
};
}  // namespace ufo

#endif  // UFO_UTILITY_TIMING_HPP