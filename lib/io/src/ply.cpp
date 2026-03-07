// UFO
#include <ufo/io/file_handler.hpp>
#include <ufo/io/ply.hpp>

// STL
#include <filesystem>
#include <format>
#include <print>
#include <stdexcept>

// RPLY
#include <rply/rply.h>

namespace ufo
{
namespace detail
{
int pointCallback(p_ply_argument argument)
{
	std::vector<Vec3f>* points;
	long                index;
	ply_get_argument_user_data(argument, reinterpret_cast<void**>(&points), &index);

	auto value = static_cast<float>(ply_get_argument_value(argument));
	if (0 == index) {
		points->emplace_back(value, 0.0f, 0.0f);
	} else {
		points->back()[index] = value;
	}

	return 1;
}

template <class Color>
int colorCallback(p_ply_argument argument)
{
	std::vector<Color>* colors;
	long                index;
	ply_get_argument_user_data(argument, reinterpret_cast<void**>(&colors), &index);

	auto value = static_cast<std::uint8_t>(ply_get_argument_value(argument));
	if (0 == index) {
		colors->emplace_back(value, 0u, 0u);
	} else if (1 == index) {
		colors->back().green = value;
	} else if (2 == index) {
		colors->back().blue = value;
	} else if (3 == index) {
		if constexpr (has_alpha_v<Color>) {
			colors->back().alpha = value;
		}
	}

	return 1;
}

int normalCallback(p_ply_argument argument)
{
	std::vector<Normal>* normals;
	long                 index;
	ply_get_argument_user_data(argument, reinterpret_cast<void**>(&normals), &index);

	auto value = static_cast<float>(ply_get_argument_value(argument));
	if (0 == index) {
		normals->emplace_back(value, 0.0f, 0.0f);
	} else if (1 == index) {
		normals->back().y = value;
	} else if (2 == index) {
		normals->back().z = value;
	}

	return 1;
}

int intensityCallback(p_ply_argument argument)
{
	std::vector<Intensity>* intensities;
	long                    index;
	ply_get_argument_user_data(argument, reinterpret_cast<void**>(&intensities), &index);

	auto value = static_cast<float>(ply_get_argument_value(argument));
	intensities->emplace_back(value);

	return 1;
}

bool readPLY(std::filesystem::path const& file, std::vector<Vec3f>& points,
             std::vector<SmallRGB>* colors, std::vector<SmallRGBA>* colors_with_alpha,
             std::vector<Normal>* normals, std::vector<Intensity>* intensities)
{
	p_ply fp = ply_open(file.c_str(), nullptr, 0, nullptr);

	if (!fp) {
		std::println(stderr, "[UFO | Read PLY] Failed to open file: {}", file.string());
		return false;
	}

	if (!ply_read_header(fp)) {
		std::println(stderr, "[UFO | Read PLY] Failed to read header: {}", file.string());
		ply_close(fp);
		return false;
	}

	long num_points      = 0;
	long num_colors      = 0;
	long num_normals     = 0;
	long num_intensities = 0;

	num_points = ply_set_read_cb(fp, "vertex", "x", pointCallback, &points, 0);
	ply_set_read_cb(fp, "vertex", "y", pointCallback, &points, 1);
	ply_set_read_cb(fp, "vertex", "z", pointCallback, &points, 2);

	if (nullptr != colors) {
		num_colors = ply_set_read_cb(fp, "vertex", "red", colorCallback<SmallRGB>, colors, 0);
		ply_set_read_cb(fp, "vertex", "green", colorCallback<SmallRGB>, colors, 1);
		ply_set_read_cb(fp, "vertex", "blue", colorCallback<SmallRGB>, colors, 2);
	} else if (nullptr != colors_with_alpha) {
		num_colors = ply_set_read_cb(fp, "vertex", "red", colorCallback<SmallRGBA>,
		                             colors_with_alpha, 0);
		ply_set_read_cb(fp, "vertex", "green", colorCallback<SmallRGBA>, colors_with_alpha,
		                1);
		ply_set_read_cb(fp, "vertex", "blue", colorCallback<SmallRGBA>, colors_with_alpha, 2);
		ply_set_read_cb(fp, "vertex", "alpha", colorCallback<SmallRGBA>, colors_with_alpha,
		                2);
	}

	if (nullptr != normals) {
		num_normals = ply_set_read_cb(fp, "vertex", "nx", normalCallback, normals, 0);
		ply_set_read_cb(fp, "vertex", "ny", normalCallback, normals, 1);
		ply_set_read_cb(fp, "vertex", "nz", normalCallback, normals, 2);
	}

	if (nullptr != intensities) {
		num_intensities =
		    ply_set_read_cb(fp, "vertex", "intensity", intensityCallback, intensities, 0);
	}

	if (0 >= num_points) {
		std::println(stderr, "[UFO | Read PLY] Number of vertices <= 0");
		ply_close(fp);
		return false;
	}

	long total = std::max({num_points, num_colors, num_normals, num_intensities});

	points.reserve(total);
	if (nullptr != colors) {
		colors->reserve(total);
	} else if (nullptr != colors_with_alpha) {
		colors_with_alpha->reserve(total);
	}
	if (nullptr != normals) {
		normals->reserve(total);
	}
	if (nullptr != intensities) {
		intensities->reserve(total);
	}

	if (!ply_read(fp)) {
		std::println(stderr, "[UFO | Read PLY] Failed to read file: {}", file.string());
		ply_close(fp);
		return false;
	}

	auto actual_size =
	    std::max({points.size(), nullptr != colors ? colors->size() : 0,
	              nullptr != colors_with_alpha ? colors_with_alpha->size() : 0,
	              nullptr != normals ? normals->size() : 0,
	              nullptr != intensities ? intensities->size() : 0});

	if (actual_size != static_cast<std::size_t>(num_points)) {
		points.clear();
		if (nullptr != colors) {
			colors->clear();
		} else if (nullptr != colors_with_alpha) {
			colors_with_alpha->clear();
		}
		if (nullptr != normals) {
			normals->clear();
		}
		if (nullptr != intensities) {
			intensities->clear();
		}
		std::println(stderr,
		             "[UFO | Read PLY] Mismatch in number of points: expected {}, got {}",
		             num_points, actual_size);
		ply_close(fp);
		return false;
	}

	points.resize(total);
	if (nullptr != colors) {
		colors->resize(total);
	} else if (nullptr != colors_with_alpha) {
		colors_with_alpha->resize(total);
	}
	if (nullptr != normals) {
		normals->resize(total);
	}
	if (nullptr != intensities) {
		intensities->resize(total);
	}

	ply_close(fp);

	return true;
}

bool writePLY(std::filesystem::path const& file, std::span<Vec3f const> points,
              std::span<SmallRGB const>  colors,
              std::span<SmallRGBA const> colors_with_alpha,
              std::span<Normal const> normals, std::span<Intensity const> intensities,
              bool ascii)
{
	p_ply fp = ply_create(file.c_str(), ascii ? PLY_ASCII : PLY_LITTLE_ENDIAN, nullptr, 0,
	                      nullptr);

	if (!fp) {
		std::println(stderr, "[UFO | Write PLY] Failed to create file: {}", file.string());
		return false;
	}

	ply_add_comment(fp, "Created by UFO");

	ply_add_element(fp, "vertex", static_cast<long>(points.size()));
	ply_add_property(fp, "x", PLY_FLOAT32, PLY_FLOAT32, PLY_FLOAT32);
	ply_add_property(fp, "y", PLY_FLOAT32, PLY_FLOAT32, PLY_FLOAT32);
	ply_add_property(fp, "z", PLY_FLOAT32, PLY_FLOAT32, PLY_FLOAT32);

	if (!colors.empty()) {
		ply_add_property(fp, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		ply_add_property(fp, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		ply_add_property(fp, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
	} else if (!colors_with_alpha.empty()) {
		ply_add_property(fp, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		ply_add_property(fp, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		ply_add_property(fp, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		ply_add_property(fp, "alpha", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
	}

	if (!normals.empty()) {
		ply_add_property(fp, "nx", PLY_FLOAT32, PLY_FLOAT32, PLY_FLOAT32);
		ply_add_property(fp, "ny", PLY_FLOAT32, PLY_FLOAT32, PLY_FLOAT32);
		ply_add_property(fp, "nz", PLY_FLOAT32, PLY_FLOAT32, PLY_FLOAT32);
	}

	if (!intensities.empty()) {
		ply_add_property(fp, "intensity", PLY_FLOAT32, PLY_FLOAT32, PLY_FLOAT32);
	}

	if (!ply_write_header(fp)) {
		std::println(stderr, "[UFO | Write PLY] Failed to write header");
		ply_close(fp);
		return false;
	}

	for (std::size_t i{}; points.size() > i; ++i) {
		ply_write(fp, points[i].x);
		ply_write(fp, points[i].y);
		ply_write(fp, points[i].z);

		if (!colors.empty()) {
			ply_write(fp, static_cast<unsigned char>(colors[i].red));
			ply_write(fp, static_cast<unsigned char>(colors[i].green));
			ply_write(fp, static_cast<unsigned char>(colors[i].blue));
		} else if (!colors_with_alpha.empty()) {
			ply_write(fp, static_cast<unsigned char>(colors_with_alpha[i].red));
			ply_write(fp, static_cast<unsigned char>(colors_with_alpha[i].green));
			ply_write(fp, static_cast<unsigned char>(colors_with_alpha[i].blue));
			ply_write(fp, static_cast<unsigned char>(colors_with_alpha[i].alpha));
		}

		if (!normals.empty()) {
			ply_write(fp, static_cast<float>(normals[i].x));
			ply_write(fp, static_cast<float>(normals[i].y));
			ply_write(fp, static_cast<float>(normals[i].z));
		}

		if (!intensities.empty()) {
			ply_write(fp, static_cast<float>(intensities[i].intensity));
		}
	}

	ply_close(fp);

	return true;
}
}  // namespace detail

CloudProperties cloudPropertiesPLY(std::filesystem::path const& file)
{
	p_ply fp = ply_open(file.c_str(), nullptr, 0, nullptr);

	if (!fp) {
		throw std::runtime_error(std::format(
		    "[UFO | Cloud Properties PLY] Failed to open file: {}", file.string()));
	}

	if (!ply_read_header(fp)) {
		ply_close(fp);
		throw std::runtime_error(
		    std::format("[UFO | Cloud Properties PLY] Failed to read header"));
	}

	CloudProperties prop;

	prop.color     = 0 < ply_set_read_cb(fp, "vertex", "red", nullptr, nullptr, 0);
	prop.alpha     = 0 < ply_set_read_cb(fp, "vertex", "alpha", nullptr, nullptr, 0);
	prop.normal    = 0 < ply_set_read_cb(fp, "vertex", "nx", nullptr, nullptr, 0);
	prop.intensity = 0 < ply_set_read_cb(fp, "vertex", "intensity", nullptr, nullptr, 0);

	ply_close(fp);

	return prop;
}
}  // namespace ufo