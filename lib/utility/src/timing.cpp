// UFO
#include <ufo/utility/timing.hpp>

// STL
#include <cmath>

namespace ufo
{
//
// Public functions
//

Timing::Timing(std::string const& tag) : tag_(tag) {}

Timing::Timing(std::string const& tag, std::initializer_list<Timing> init)
    : timings_(init), tag_(tag)
{
}

Timing::Timing(Timing const& other) : Timing(other, ReadLock(other.mutex_)) {}

Timing::Timing(Timing&& other) : Timing(std::move(other), WriteLock(other.mutex_)) {}

Timing& Timing::operator=(Timing const& rhs)
{
	if (this != &rhs) {
		WriteLock lhs_lk(mutex_, std::defer_lock);
		ReadLock  rhs_lk(rhs.mutex_, std::defer_lock);
		std::lock(lhs_lk, rhs_lk);
		timings_    = rhs.timings_;
		tag_        = rhs.tag_;
		color_      = rhs.color_;
		started_id_ = rhs.started_id_;
	}
	return *this;
}

Timing& Timing::operator=(Timing&& rhs)
{
	if (this != &rhs) {
		WriteLock lhs_lk(mutex_, std::defer_lock);
		WriteLock rhs_lk(rhs.mutex_, std::defer_lock);
		std::lock(lhs_lk, rhs_lk);
		timings_    = std::move(rhs.timings_);
		tag_        = std::move(rhs.tag_);
		color_      = std::move(rhs.color_);
		started_id_ = std::move(rhs.started_id_);
	}
	return *this;
}

Timing& Timing::start()
{
	Timer::start();
	started_id_ = std::this_thread::get_id();
	return *this;
}

Timing& Timing::start(std::string const& tag)
{
	WriteLock lock(mutex_);

	for (Timing& t : timings_) {
		if (std::this_thread::get_id() == t.started_id_) {
			return t.start(tag);
		}
	}

	for (Timing& t : timings_) {
		if (tag == t.tag()) {
			static_cast<Timer&>(t).start();
			t.started_id_ = std::this_thread::get_id();
			return t;
		}
	}

	Timing& t = timings_.emplace_back(tag);
	static_cast<Timer&>(t).start();
	t.started_id_ = std::this_thread::get_id();
	return t;
}

Timing& Timing::start(std::string const& tag, std::string const& color)
{
	WriteLock lock(mutex_);

	for (Timing& t : timings_) {
		if (std::this_thread::get_id() == t.started_id_) {
			return t.start(tag, color);
		}
	}

	for (Timing& t : timings_) {
		if (tag == t.tag()) {
			static_cast<Timer&>(t).start();
			t.started_id_ = std::this_thread::get_id();
			t.color_      = color;
			return t;
		}
	}

	Timing& t = timings_.emplace_back(tag);
	static_cast<Timer&>(t).start();
	t.started_id_ = std::this_thread::get_id();
	t.color_      = color;
	return t;
}

void Timing::reset()
{
	WriteLock lock(mutex_);

	Timer::reset();
	timings_.clear();
}

bool Timing::stop()
{
	WriteLock lock(mutex_);

	for (Timing& t : timings_) {
		if (std::this_thread::get_id() == t.started_id_) {
			if (t.stop()) {
				return true;
			}
		}
	}

	if (std::this_thread::get_id() == started_id_) {
		Timer::stop();
		started_id_ = {};
		return true;
	}

	return false;
}

std::size_t Timing::stop(std::size_t levels)
{
	if (0 == levels) {
		return 0;
	}

	WriteLock lock(mutex_);

	std::size_t stopped_levels{};
	for (Timing& t : timings_) {
		if (std::this_thread::get_id() == t.started_id_) {
			stopped_levels = t.stop(levels);
			if (levels == stopped_levels) {
				return stopped_levels;
			}
		}
	}

	if (std::this_thread::get_id() == started_id_ && stopped_levels < levels) {
		++stopped_levels;
		Timer::stop();
		started_id_ = {};
	}

	return stopped_levels;
}

void Timing::stopAll()
{
	WriteLock lock(mutex_);

	for (Timing& t : timings_) {
		if (std::this_thread::get_id() == t.started_id_) {
			t.stopAll();
			break;
		}
	}

	if (std::this_thread::get_id() == started_id_) {
		Timer::stop();
		started_id_ = {};
	}
}

Timing const& Timing::operator[](std::string const& tag) const
{
	ReadLock lock(mutex_);
	for (Timing const& t : timings_) {
		if (tag == t.tag()) {
			return t;
		}
	}
	throw std::out_of_range("Tag '" + tag + "' does not exist");
}

Timing& Timing::operator[](std::string const& tag)
{
	WriteLock lock(mutex_);
	for (Timing& t : timings_) {
		if (tag == t.tag()) {
			return t;
		}
	}
	return timings_.emplace_back(tag);
}

std::string const& Timing::tag() const { return tag_; }

void Timing::setTag(std::string const& tag) { tag_ = tag; }

std::string const& Timing::color() const { return color_; }

void Timing::setColor(std::string const& color) { color_ = color; }

void Timing::printSeconds(bool first_as_tag, bool random_colors, bool bold,
                          int start_numbering_level, int stop_numbering_level,
                          int group_colors_level, int precision) const
{
	print<std::chrono::seconds::period>(first_as_tag, random_colors, bold,
	                                    start_numbering_level, stop_numbering_level,
	                                    group_colors_level, precision);
}

void Timing::printMilliseconds(bool first_as_tag, bool random_colors, bool bold,
                               int start_numbering_level, int stop_numbering_level,
                               int group_colors_level, int precision) const
{
	print<std::chrono::milliseconds::period>(first_as_tag, random_colors, bold,
	                                         start_numbering_level, stop_numbering_level,
	                                         group_colors_level, precision);
}

void Timing::printMicroseconds(bool first_as_tag, bool random_colors, bool bold,
                               int start_numbering_level, int stop_numbering_level,
                               int group_colors_level, int precision) const
{
	print<std::chrono::microseconds::period>(first_as_tag, random_colors, bold,
	                                         start_numbering_level, stop_numbering_level,
	                                         group_colors_level, precision);
}

void Timing::printNanoseconds(bool first_as_tag, bool random_colors, bool bold,
                              int start_numbering_level, int stop_numbering_level,
                              int group_colors_level, int precision) const
{
	print<std::chrono::nanoseconds::period>(first_as_tag, random_colors, bold,
	                                        start_numbering_level, stop_numbering_level,
	                                        group_colors_level, precision);
}

//
// Private functions
//

Timing::Timing(Timing const& other, ReadLock /* rhs_lk */)
    : timings_(other.timings_)
    , tag_(other.tag_)
    , color_(other.color_)
    , started_id_(other.started_id_)
{
}

Timing::Timing(Timing&& other, WriteLock /* rhs_lk */)
    : timings_(std::move(other.timings_))
    , tag_(std::move(other.tag_))
    , color_(std::move(other.color_))
    , started_id_(std::move(other.started_id_))
{
}

std::vector<Timing::TimingNL> Timing::timings() const
{
	ReadLock lock(mutex_);

	std::vector<TimingNL> data;

	int level;
	if (0 != Timer::numSamples() || Timer::active()) {
		level = 1;
		data.emplace_back(this, 0, 0);
	} else {
		level = 0;
	}

	std::size_t i{1};
	for (auto& t : timings_) {
		t.timingsRecurs(data, i, level);
		++i;
	}

	return data;
}

void Timing::timingsRecurs(std::vector<TimingNL>& data, std::size_t num, int level) const
{
	ReadLock lock(mutex_);

	data.emplace_back(this, num, level);
	std::size_t i{1};
	for (auto& t : timings_) {
		t.timingsRecurs(data, i, level + 1);
		++i;
	}
}

void Timing::addTags(std::vector<std::string>& data, std::vector<TimingNL> const& timers,
                     int start_numbering_level, int stop_numbering_level) const
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