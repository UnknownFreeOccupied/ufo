// UFO
#include <ufo/utility/timing.hpp>

// STL
#include <array>
#include <cmath>

namespace ufo
{
//
// Public functions
//

Timing::Timing(std::string const& tag) : tag_(tag) {}

Timing::Timing(std::string const&                                          tag,
               std::initializer_list<std::pair<std::size_t const, Timing>> init)
    : tag_(tag), timer_(init)
{
}

void Timing::start(std::string const& tag)
{
	Timer::start();
	tag_ = tag;
}

void Timing::start(std::string const& tag, std::string const& color)
{
	start(tag);
	color_ = color;
}

Timing const& Timing::operator[](std::size_t num) const { return timer_.at(num); }

Timing& Timing::operator[](std::size_t num) { return timer_[num]; }

std::string const& Timing::tag() const { return tag_; }

void Timing::setTag(std::string const& tag) { tag_ = tag; }

std::string const& Timing::color() const { return color_; }

void Timing::setColor(std::string const& color) { color_ = color; }

void Timing::printSeconds(bool first_as_tag, bool random_colors, bool bold,
                          std::size_t start_numbering_level,
                          std::size_t stop_numbering_level,
                          std::size_t group_colors_level, int precision) const
{
	print<std::chrono::seconds::period>(first_as_tag, random_colors, bold,
	                                    start_numbering_level, stop_numbering_level,
	                                    group_colors_level, precision);
}

void Timing::printMilliseconds(bool first_as_tag, bool random_colors, bool bold,
                               std::size_t start_numbering_level,
                               std::size_t stop_numbering_level,
                               std::size_t group_colors_level, int precision) const
{
	print<std::chrono::milliseconds::period>(first_as_tag, random_colors, bold,
	                                         start_numbering_level, stop_numbering_level,
	                                         group_colors_level, precision);
}

void Timing::printMicroseconds(bool first_as_tag, bool random_colors, bool bold,
                               std::size_t start_numbering_level,
                               std::size_t stop_numbering_level,
                               std::size_t group_colors_level, int precision) const
{
	print<std::chrono::microseconds::period>(first_as_tag, random_colors, bold,
	                                         start_numbering_level, stop_numbering_level,
	                                         group_colors_level, precision);
}

void Timing::printNanoseconds(bool first_as_tag, bool random_colors, bool bold,
                              std::size_t start_numbering_level,
                              std::size_t stop_numbering_level,
                              std::size_t group_colors_level, int precision) const
{
	print<std::chrono::nanoseconds::period>(first_as_tag, random_colors, bold,
	                                        start_numbering_level, stop_numbering_level,
	                                        group_colors_level, precision);
}

//
// Private functions
//

std::vector<Timing::TimingNL> Timing::timings(bool skip_first) const
{
	std::vector<TimingNL> data;

	if (!skip_first) {
		data.emplace_back(this, 0, 0);
	}

	int level = skip_first ? 0 : 1;

	for (auto const& [n, t] : timer_) {
		t.timingsRecurs(data, n, level);
	}

	return data;
}

void Timing::timingsRecurs(std::vector<TimingNL>& data, std::size_t num, int level) const
{
	data.emplace_back(this, num, level);
	for (auto const& [n, t] : timer_) {
		t.timingsRecurs(data, n, level + 1);
	}
}

void Timing::addTags(std::vector<std::string>& data, std::vector<TimingNL> const& timers,
                     std::size_t start_numbering_level,
                     std::size_t stop_numbering_level) const
{
	for (auto const& t : timers) {
		if (start_numbering_level <= t.level && t.level <= stop_numbering_level) {
			data.push_back(std::string(2 * t.level, ' ') + std::to_string(t.num) + ". " +
			               t.timing->tag());
		} else {
			data.push_back(std::string(2 * t.level, ' ') + t.timing->tag());
		}
	}
}

void Timing::addNumSamples(std::vector<std::string>&    data,
                           std::vector<TimingNL> const& timers) const
{
	for (auto const& t : timers) {
		data.push_back(std::to_string(t.timing->numSamples()));
	}
}

int Timing::maxLength(std::vector<std::string> const& data) const
{
	int l = 0;
	for (std::string const& s : data) {
		l = std::max(l, static_cast<int>(s.length()));
	}
	return l;
}

std::pair<int, int> Timing::centeringPadding(std::string const& str, int max_width) const
{
	int left_pad  = std::ceil((max_width - static_cast<int>(str.length())) / 2.0);
	int right_pad = max_width - (left_pad + static_cast<int>(str.length()));
	return {left_pad, right_pad};
}
}  // namespace ufo