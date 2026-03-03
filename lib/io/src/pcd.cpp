// UFO
#include <ufo/io/file_handler.hpp>
#include <ufo/io/pcd.hpp>

// STL
#include <filesystem>
#include <format>
#include <stdexcept>

namespace ufo
{
namespace detail
{
void pcdSplit(std::vector<std::string>& result, std::string const& in,
              char const* const delimiters)
{
	// Taken from:
	// https://github.com/PointCloudLibrary/pcl/blob/e5ce18fab8d4ff175eb44cc09ae24ae1a0a2e8eb/io/include/pcl/io/split.h#L25
	auto const  len         = in.length();
	std::size_t token_start = 0;

	result.clear();
	while (token_start < len) {
		// eat leading whitespace
		token_start = in.find_first_not_of(delimiters, token_start);
		if (token_start == std::string::npos) {
			return;  // nothing left but white space
		}

		// find the end of the token
		auto const token_end = in.find_first_of(delimiters, token_start);

		// push token
		if (token_end == std::string::npos) {
			result.emplace_back(in.data() + token_start, len - token_start);
			return;
		} else {
			result.emplace_back(in.data() + token_start, token_end - token_start);
		}

		// set up for next loop
		token_start = token_end + 1;
	}
}
}  // namespace detail

CloudProperties cloudPropertiesPCD(std::filesystem::path const& file)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		throw std::runtime_error(std::format(
		    "[UFO | Cloud Properties PCD] Failed to open file: {}", file.string()));
	}

	CloudProperties prop;

	std::vector<std::string> st;
	for (char* buf = fp.readline(); nullptr != buf; buf = fp.readline()) {
		std::string line(buf);

		if (line.empty()) {
			continue;
		}

		detail::pcdSplit(st, line, "\t\r ");

		std::stringstream sstream(line);
		sstream.imbue(std::locale::classic());

		std::string line_type;
		sstream >> line_type;

		if ("#" == line_type.substr(0, 1)) {
			continue;
		}

		if (("FIELDS" == line_type.substr(0, 6)) || ("COLUMNS" == line_type.substr(0, 7))) {
			int specified_channel_count = static_cast<int>(st.size() - 1);

			for (int i = 0; i < specified_channel_count; ++i) {
				if ("rgb" == st.at(i + 1)) {
					prop.color = true;
				} else if ("rgba" == st.at(i + 1)) {
					prop.color = true;
					prop.alpha = true;
				} else if ("intensity" == st.at(i + 1)) {
					prop.intensity = true;
				} else if ("normal_x" == st.at(i + 1) || "normal_y" == st.at(i + 1) ||
				           "normal_z" == st.at(i + 1)) {
					prop.normal = true;
				}
			}
		} else if ("DATA" == line_type.substr(0, 4)) {
			break;
		}
	}

	return prop;
}
}  // namespace ufo