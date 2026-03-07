// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/map/integrator/angular_integrator.hpp>
#include <ufo/map/ufomap.hpp>
#include <ufo/numeric/transform3.hpp>
#include <ufo/vision/camera.hpp>

// STL
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <future>
#include <iostream>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// C STL
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// Orbbec
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>

// TOML
#include <toml++/toml.hpp>

// STB
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// PCL
#include <pcl/filters/crop_box.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/icp.h>

#define RESET       "\033[0m"
#define BLACK       "\033[30m"        /* Black */
#define RED         "\033[31m"        /* Red */
#define GREEN       "\033[32m"        /* Green */
#define YELLOW      "\033[33m"        /* Yellow */
#define BLUE        "\033[34m"        /* Blue */
#define MAGENTA     "\033[35m"        /* Magenta */
#define CYAN        "\033[36m"        /* Cyan */
#define WHITE       "\033[37m"        /* White */
#define BOLDBLACK   "\033[1m\033[30m" /* Bold Black */
#define BOLDRED     "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m" /* Bold White */

using Map   = ufo::Map3D<ufo::OccupancyMap, ufo::ColorMap>;
using Cloud = ufo::PointCloud<3, float, ufo::FineLab>;

class Camera
{
 public:
	Camera(std::string_view name, ufo::Transform3f pose, char const* serial_number,
	       std::uint32_t width = 640, std::uint32_t height = 400, std::uint32_t fps = 15,
	       OBFormat color_format = OB_FORMAT_RGB, OBFormat depth_format = OB_FORMAT_Y16)
	    : name_(name), pose_(pose)
	{
		ob::Context ctx;
		auto        devices = ctx.queryDeviceList();
		device_             = devices->getDeviceBySN(serial_number);
		if (nullptr == device_) {
			std::cerr << BOLDRED
			          << "Could not find a device with serial number: " << serial_number << '\n'
			          << RESET;
			exit(EXIT_FAILURE);
		}

		config_ = std::make_shared<ob::Config>();
		config_->enableVideoStream(OB_STREAM_COLOR, width, height, fps, color_format);
		config_->enableVideoStream(OB_STREAM_DEPTH, width, height, fps, depth_format);
		config_->setFrameAggregateOutputMode(
		    // OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE
		    OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
		config_->setAlignMode(ALIGN_D2C_HW_MODE);

		pipeline_ = std::make_shared<ob::Pipeline>(device_);
		pipeline_->enableFrameSync();
		pipeline_->start(config_);

		point_cloud_ = std::make_shared<ob::PointCloudFilter>();
	}

	~Camera() { pipeline_->stop(); }

	bool update()
	{
		auto frame_set = pipeline_->waitForFrameset(100);
		// auto frame_set = pipeline_->waitForFrames(100);
		if (nullptr == frame_set) {
			return false;
		}

		point_cloud_->setCreatePointFormat(OB_FORMAT_RGB_POINT);

		frame_ = point_cloud_->process(frame_set);

		return true;
	}

	[[nodiscard]] Cloud cloud() const
	{
		std::uint32_t       num_points = frame_->dataSize() / sizeof(OBColorPoint);
		OBColorPoint const* point      = static_cast<OBColorPoint const*>(frame_->data());

		Cloud cloud;
		cloud.reserve(num_points);

		for (std::uint32_t i{}; num_points > i; ++i) {
			cloud.emplace_back(
			    ufo::Vec3f(point->x, point->y, point->z),
			    ufo::convert<ufo::FineLab>(ufo::FineRGB{point->r, point->g, point->b}));
		}

		return cloud;
	}

	[[nodiscard]] ufo::Transform3f pose() const { return pose_; }

 private:
	std::string                           name_;
	ufo::Transform3f                      pose_;
	std::shared_ptr<ob::Device>           device_;
	std::shared_ptr<ob::Config>           config_;
	std::shared_ptr<ob::Pipeline>         pipeline_;
	std::shared_ptr<ob::PointCloudFilter> point_cloud_;
	std::shared_ptr<ob::Frame>            frame_;
};

struct Renderer {
	ufo::Image<ufo::Ray3f>          rays;
	ufo::Image<ufo::TraceResult<3>> nodes;
	ufo::Image<ufo::SmallRGB>       raw_rgb_image;
	ufo::Image<std::uint8_t>        raw_depth_image;
	std::vector<char>               rgb_image;
	std::vector<char>               depth_image;
	std::filesystem::path           save_dir;
	bool                            named_pipe;
	std::size_t                     image_index{};
	int                             fd;
	ufo::SmallRGB                   background_color;
	float                           min_dist = 0.01;
	float                           max_dist = 4.6;

	Renderer(ufo::Image<ufo::Ray3f> rays, std::filesystem::path save_dir, bool named_pipe,
	         ufo::SmallRGB background_color)
	    : rays(rays)
	    , nodes(rays.rows(), rays.cols())
	    , raw_rgb_image(rays.rows(), rays.cols())
	    , raw_depth_image(rays.rows(), rays.cols())
	    , save_dir(save_dir)
	    , named_pipe(named_pipe)
	    , background_color(background_color)
	{
		if (named_pipe) {
			mkfifo("ufo_manipulation", 0666);
			fd = open("ufo_manipulation", O_WRONLY);
		}
	}

	~Renderer() { close(fd); }

	void render(Map const& map)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
		    new pcl::PointCloud<pcl::PointXYZRGB>);
		for (auto node : map.query(ufo::pred::Leaf{} && ufo::pred::Occupied{})) {
			auto coord = map.center(node.index);
			auto color = map.colorRGB(node.index);
			pcl_cloud->push_back(pcl::PointXYZRGB(coord.x, coord.y, coord.z, color.red,
			                                      color.green, color.blue));
		}
		// for (auto node : map) {
		// 	if (map.isLeaf(node.index) && map.occupancyOccupied(node.index)) {
		// 		auto coord = map.center(node.index);
		// 		auto color = map.color<ufo::ColorType::RGB8U>(node.index);
		// 		pcl_cloud->push_back(pcl::PointXYZRGB(coord.x, coord.y, coord.z, color.red,
		// 		                                      color.green, color.blue));
		// 	}
		// }

		pcl::io::savePCDFile(
		    "/home/dduberg/Downloads/manipulation/src/ufo/lib/map/apps/manipulation/renders/"
		    "map.pcd",
		    *pcl_cloud, true);

		map.trace(ufo::execution::par, rays.begin(), rays.end(), nodes.begin(),
		          ufo::pred::Leaf{} && ufo::pred::Occupied{}, min_dist, max_dist);

		ufo::transform(ufo::execution::par, nodes.begin(), nodes.end(), raw_rgb_image.begin(),
		               [this, &map](auto hit) {
			               return map.valid(hit.node) ? map.colorRGB(hit.node)
			                                          : background_color;
		               });

		double max_depth = 0;
		for (auto const& n : nodes) {
			double depth = map.valid(n.node) ? n.distance : max_dist;
			max_depth    = std::max(max_depth, depth);
		}

		ufo::transform(ufo::execution::par, nodes.begin(), nodes.end(),
		               raw_depth_image.begin(), [this, &map, max_depth](auto hit) {
			               double depth = map.valid(hit.node) ? hit.distance : max_depth;
			               depth        = std::round(depth * 255 / max_depth);
			               //  depth        = std::clamp(depth, min_dist, max_dist);
			               //  float ranged = (depth - min_dist) /
			               //                 (max_dist - min_dist);  // dmin->0.0, dmax->1.0
			               //  float out = 1.0 - ranged;              // 0 -> white, 1 -> black
			               //  //  output* *= 1 / 2.2;  // most picture data is gamma-compressed

			               //  return static_cast<std::uint8_t>(
			               //      out * std::numeric_limits<std::uint8_t>::max());
			               return static_cast<std::uint8_t>(depth);
		               });

		ufo::Image<int> seen(raw_rgb_image.rows(), raw_rgb_image.cols(), 0);
		std::cout << raw_rgb_image(92, 95) << "\n";
		std::cout << raw_rgb_image(95, 92) << "\n";
		std::set<std::pair<int, int>> open;
		for (int iter{}; 1 > iter; ++iter) {
			for (int r{}; raw_rgb_image.rows() > r; ++r) {
				for (int c{}; raw_rgb_image.cols() > c; ++c) {
					if (255 == raw_depth_image(r, c)) {
						continue;
					}

					seen(r, c) = 1;
					open.insert(std::pair(c, r));

					while (!open.empty()) {
						auto cur = *open.begin();
						open.erase(cur);
						for (int y = -1; 1 >= y; ++y) {
							for (int x = -1; 1 >= x; ++x) {
								int ry = cur.second + y;
								int cx = cur.first + x;
								if (0 > ry || raw_rgb_image.rows() <= ry || 0 > cx ||
								    raw_rgb_image.cols() <= cx || 0 < seen(ry, cx)) {
									continue;
								}

								if (255 == raw_depth_image(ry, cx)) {
									seen(ry, cx) = 2;
									continue;
								}

								seen(ry, cx) = 1;
								open.insert(std::pair(cx, ry));
							}
						}
					}

					std::size_t num_white{};
					std::size_t num_green{};

					for (int r{}; raw_rgb_image.rows() > r; ++r) {
						for (int c{}; raw_rgb_image.cols() > c; ++c) {
							if (1 != seen(r, c)) {
								continue;
							}

							auto color = raw_rgb_image(r, c);

							if (130 < color.red && 160 > color.red && 190 < color.green &&
							    210 > color.green && 130 < color.blue && 150 > color.blue) {
								++num_green;
							} else if (220 < color.red && 220 < color.green && 220 < color.blue) {
								++num_white;
							}
						}
					}

					if (5 > num_green && 30 > num_white) {
						for (int r{}; raw_rgb_image.rows() > r; ++r) {
							for (int c{}; raw_rgb_image.cols() > c; ++c) {
								if (0 == seen(r, c)) {
									continue;
								}

								raw_rgb_image(r, c)   = background_color;
								raw_depth_image(r, c) = 255;
							}
						}
					}

					std::fill(seen.begin(), seen.end(), 0);
					open.clear();
				}
			}
		}

		auto f = [this](std::string const& name, auto raw_image, auto image, int comp) {
			if (0 != stbi_write_jpg_to_func(
			             [](void* context, void* data, int size) {
				             std::vector<char>* v = static_cast<std::vector<char>*>(context);
				             std::size_t        s = v->size();
				             v->resize(s + size);
				             std::memcpy(v->data() + s, data, size);
			             },
			             &image, raw_image.cols(), raw_image.rows(),
			             sizeof(typename decltype(raw_image)::value_type) / sizeof(char),
			             raw_image.data(), 95)) {
				std::cerr << BOLDRED << "Error writing jpg image\n" << RESET;
			}

			// if (named_pipe) {
			// 	if (-1 == write(fd, rgb_image.data(), rgb_image.size())) {
			// 		std::cerr << BOLDRED << "Error while writing to pipe\n" << RESET;
			// 	}
			// }

			std::filesystem::path path =
			    save_dir / (name + "_" + std::to_string(image_index) + ".jpg");
			std::cout << "Saving file " << path << '\n';
			std::ofstream file(path, std::ios::binary | std::ios::out);
			file.write(image.data(), image.size());
			file.close();

			image.clear();
		};

		f("rgb", raw_rgb_image, rgb_image, 3);
		f("depth", raw_depth_image, depth_image, 1);

		// TODO: Make the index nicer

		++image_index;
	}
};

[[nodiscard]] std::vector<Camera> createCameras(toml::table const& config)
{
	std::cout << "Creating cameras\n";

	std::uint32_t default_width  = 640;
	std::uint32_t default_height = 400;
	std::uint32_t default_fps    = 15;

	if (auto v = config["width"].value<std::uint32_t>(); v) {
		default_width = *v;
	} else {
		std::cout << YELLOW << "\tMissing default camera width, using '" << default_width
		          << "'.\n";
	}

	if (auto v = config["height"].value<std::uint32_t>(); v) {
		default_height = *v;
	} else {
		std::cout << YELLOW << "\tMissing default camera height, using '" << default_height
		          << "'.\n";
	}

	if (auto v = config["fps"].value<std::uint32_t>(); v) {
		default_fps = *v;
	} else {
		std::cout << YELLOW << "\tMissing default camera fps, using '" << default_fps
		          << "'.\n";
	}

	std::vector<Camera> cameras;

	for (auto const& [name, value] : config) {
		if (!value.is_table()) {
			continue;
		}

		std::cout << "\tCamera '" << name << "'\n";

		std::string_view serial_number;
		ufo::Transform3f pose;
		std::uint32_t    width  = default_width;
		std::uint32_t    height = default_height;
		std::uint32_t    fps    = default_fps;

		auto params = *value.as_table();

		if (auto v = params["serial_number"].value<std::string_view>(); v) {
			serial_number = *v;
		} else {
			std::cerr << BOLDRED << "\t - Missing serial_number.\n" << RESET;
			exit(EXIT_FAILURE);
		}

		if ("" == serial_number) {
			std::cerr << BOLDRED << "\t - Empty serial_number.\n" << RESET;
			exit(EXIT_FAILURE);
		}

		if (auto v = params["pose"].as_array(); v) {
			std::vector<float> components;
			v->for_each([&components](auto&& e) {
				if constexpr (toml::is_number<decltype(e)>) {
					components.push_back(*e);
				} else {
					std::cerr << BOLDRED
					          << "\t - 'pose' should consist of 7 floats in the format [x, y, z, "
					             "qw, qx, qy, qz] "
					             "(e.g., [0, 0, 0, 1, 0, 0, 0])\n"
					          << RESET;
					exit(EXIT_FAILURE);
				}
			});
			if (7 != components.size()) {
				std::cerr << BOLDRED
				          << "\t - 'pose' should be in the format [x, y, z, qw, qx, qy, qz] "
				             "(e.g., [0, 0, 0, 1, 0, 0, 0])\n"
				          << RESET;
				exit(EXIT_FAILURE);
			}

			pose = ufo::Transform3f(normalize(ufo::Quatf(components[3], components[4],
			                                             components[5], components[6])),
			                        ufo::Vec3f(components[0], components[1], components[2]));
		} else {
			std::cerr << BOLDRED << "\t - Missing pose.\n" << RESET;
			exit(EXIT_FAILURE);
		}

		if (auto v = params["width"].value<std::uint32_t>(); v) {
			width = *v;
		} else {
			std::cout << YELLOW << "\t - Missing width, using default width of '" << width
			          << "' instead.\n"
			          << RESET;
		}

		if (auto v = params["height"].value<std::uint32_t>(); v) {
			height = *v;
		} else {
			std::cout << YELLOW << "\t - Missing height, using default height of '" << height
			          << "' instead.\n"
			          << RESET;
		}

		if (auto v = params["fps"].value<std::uint32_t>(); v) {
			fps = *v;
		} else {
			std::cout << YELLOW << "\t - Missing fps, using default fps of '" << fps
			          << "' instead.\n"
			          << RESET;
		}

		std::cout << "\t - Serial number: " << serial_number << '\n';
		std::cout << "\t - Pose: " << pose << '\n';
		std::cout << "\t - Width: " << width << '\n';
		std::cout << "\t - Height: " << height << '\n';
		std::cout << "\t - FPS: " << fps << '\n';

		cameras.emplace_back(name.str(), pose, serial_number.data(), width, height, fps);
	}

	std::cout << "Created " << cameras.size() << " cameras\n";

	return cameras;
}

[[nodiscard]] Map createMap(toml::table const& config)
{
	std::cout << "Creating UFOMap\n";

	float resolution = 0.1f;
	if (auto v = config["resolution"].value<float>(); v) {
		resolution = *v;
	} else {
		std::cout << YELLOW << " - Missing UFOMap resolution, using '" << resolution
		          << "'.\n";
	}

	std::cout << " - Resolution: " << resolution << " m\n";

	return Map(resolution);
}

[[nodiscard]] ufo::AngularIntegrator<3> createIntegrator(toml::table const& config)
{
	std::cout << "Creating UFOMap integrator\n";

	ufo::AngularIntegrator<3> integrator;

	if (auto v = config["angular_resolution"].value<float>(); v) {
		integrator.angularResolution(ufo::radians(*v));
	} else {
		std::cout << YELLOW << " - Missing 'angular_resolution', using '"
		          << ufo::degrees(integrator.angularResolution()) << "'.\n";
	}

	if (auto v = config["min_distance"].value<float>(); v) {
		integrator.min_distance = *v;
	} else {
		std::cout << YELLOW << " - Missing 'min_distance', using '" << integrator.min_distance
		          << "'.\n";
	}

	if (auto v = config["max_distance"].value<float>(); v) {
		integrator.max_distance = *v;
	} else {
		std::cout << YELLOW << " - Missing 'max_distance', using '" << integrator.max_distance
		          << "'.\n";
	}

	if (auto v = config["sensor_angular_resolution"].value<float>(); v) {
		integrator.sensor_angular_resolution = ufo::radians(*v);
	} else {
		std::cout << YELLOW << " - Missing 'sensor_angular_resolution', using '"
		          << ufo::degrees(integrator.sensor_angular_resolution) << "'.\n";
	}

	if (auto v = config["translation_error"].value<float>(); v) {
		integrator.translation_error = *v;
	} else {
		std::cout << YELLOW << " - Missing 'translation_error', using '"
		          << integrator.translation_error << "'.\n";
	}

	if (auto v = config["orientation_error"].value<float>(); v) {
		integrator.orientation_error = *v;
	} else {
		std::cout << YELLOW << " - Missing 'orientation_error', using '"
		          << integrator.orientation_error << "'.\n";
	}

	if (auto v = config["occupancy_hit"].value<float>(); v) {
		integrator.occupancy_hit = *v;
	} else {
		std::cout << YELLOW << " - Missing 'occupancy_hit', using '"
		          << integrator.occupancy_hit << "'.\n";
	}

	if (auto v = config["occupancy_miss"].value<float>(); v) {
		integrator.occupancy_miss = *v;
	} else {
		std::cout << YELLOW << " - Missing 'occupancy_miss', using '"
		          << integrator.occupancy_miss << "'.\n";
	}

	std::cout << " - Angular resolution: " << ufo::degrees(integrator.angularResolution())
	          << "°\n";
	std::cout << " - Min. distance: " << integrator.min_distance << " m\n";
	std::cout << " - Max. distance: " << integrator.max_distance << " m\n";
	std::cout << " - Sensor angular resolution: "
	          << ufo::degrees(integrator.sensor_angular_resolution) << "°\n";
	std::cout << " - Translation error: " << integrator.translation_error << " m\n";
	std::cout << " - Orientation error: " << ufo::degrees(integrator.orientation_error)
	          << "°\n";
	std::cout << " - Occupancy hit: " << (100 * integrator.occupancy_hit) << "%\n";
	std::cout << " - Occupancy miss: " << (100 * integrator.occupancy_miss) << "%\n";

	return integrator;
}

[[nodiscard]] Renderer createRenderer(toml::table const& config)
{
	std::cout << "Creating renderer\n";

	bool named_pipe = false;
	if (auto v = config["named_pipe"].value<bool>(); v) {
		named_pipe = *v;
	} else {
		std::cerr << BOLDRED << " - Missing named_pipe, using default value of '"
		          << std::boolalpha << named_pipe << "'\n"
		          << RESET;
	}

	std::filesystem::path save_dir;

	if (auto v = config["output_dir"].value<std::string_view>(); v) {
		save_dir = *v;
	} else {
		std::cerr << BOLDRED << " - Missing output_dir.\n" << RESET;
		exit(EXIT_FAILURE);
	}

	if ("" == save_dir) {
		std::cerr << BOLDRED << " - Empty output_dir.\n" << RESET;
		exit(EXIT_FAILURE);
	}

	unsigned width  = 160;
	unsigned height = 160;

	if (auto v = config["width"].value<std::uint32_t>(); v) {
		width = *v;
	} else {
		std::cout << YELLOW << " - Missing width, using default width of '" << width
		          << "' instead.\n"
		          << RESET;
	}

	if (auto v = config["height"].value<std::uint32_t>(); v) {
		height = *v;
	} else {
		std::cout << YELLOW << " - Missing height, using default height of '" << height
		          << "' instead.\n"
		          << RESET;
	}

	float elevation = 5.0f;

	if (auto v = config["elevation"].value<std::uint32_t>(); v) {
		elevation = *v;
	} else {
		std::cout << YELLOW << " - Missing elevation, using default elevation of '"
		          << elevation << "' instead.\n"
		          << RESET;
	}

	auto coord_f = [&config, elevation](std::string const& name,
	                                    ufo::Vec2f const&  default_val = ufo::Vec2f{}) {
		if (auto v = config[name].as_array(); v) {
			std::vector<float> components;
			v->for_each([&name, &components](auto&& e) {
				if constexpr (toml::is_number<decltype(e)>) {
					components.push_back(*e);
				} else {
					std::cerr << BOLDRED << " - coordinate '" + name
					          << "' should consist of 2 floats in the format [x, y] (e.g., [0, "
					             "0])\n"
					          << RESET;
					exit(EXIT_FAILURE);
				}
			});
			if (2 != components.size()) {
				std::cerr << BOLDRED << " - coordinate '" + name
				          << "' should consist of 2 floats in the format [x, y] (e.g., [0, 0])\n"
				          << RESET;
				exit(EXIT_FAILURE);
			}
			return ufo::Vec3f(components[0], components[1], elevation);
		} else {
			std::cerr << BOLDRED << " - Missing coordinate '" + name
			          << "', using default coordinate '" << default_val << "' instead.\n"
			          << RESET;
			return ufo::Vec3f(default_val, elevation);
		}
	};

	ufo::Vec3f top_left     = coord_f("top_left", ufo::Vec2f(-2.0f, 2.0f));
	ufo::Vec3f top_right    = coord_f("top_right", ufo::Vec2f(2.0f, 2.0f));
	ufo::Vec3f bottom_right = coord_f("bottom_right", ufo::Vec2f(2.0f, -2.0f));
	ufo::Vec3f bottom_left  = coord_f("bottom_left", ufo::Vec2f(-2.0f, -2.0f));

	ufo::Image<ufo::Ray3f> rays(width, height);
	for (std::size_t r{}; rays.rows() > r; ++r) {
		for (std::size_t c{}; rays.cols() > c; ++c) {
			auto top             = ufo::lerp(top_left, top_right, float(c) / rays.cols());
			auto bottom          = ufo::lerp(bottom_left, bottom_right, float(c) / rays.cols());
			rays(r, c).origin    = ufo::lerp(top, bottom, float(r) / rays.rows());
			rays(r, c).direction = ufo::Vec3f(0, 0, -1);
		}
	}

	ufo::SmallRGB background_color;
	if (auto v = config["background_color"].as_array(); v) {
		std::vector<std::uint8_t> components;
		v->for_each([&components](auto&& e) {
			if constexpr (toml::is_number<decltype(e)>) {
				components.push_back(*e);
			} else {
				std::cerr << BOLDRED
				          << " - background_color should consist of 3 8-bit unsigned integers in "
				             "the format [r, g, b] (e.g., [0, 32, 255])\n"
				          << RESET;
				exit(EXIT_FAILURE);
			}
		});
		if (3 != components.size()) {
			std::cerr
			    << BOLDRED
			    << " - background_color should consist of 3 8-bit unsigned integers in the "
			       "format [r, g, b] (e.g., [0, 32, 255])\n"
			    << RESET;
			exit(EXIT_FAILURE);
		}
		background_color = ufo::SmallRGB{components[0], components[1], components[2]};
	} else {
		std::cerr << BOLDRED << " - Missing background_color, using default color '"
		          << background_color << "' instead.\n"
		          << RESET;
	}

	return Renderer(rays, save_dir, named_pipe, background_color);

	// TODO: Old below

	// Image<Ray3> rays(rows, cols);

	// ufo::Camera camera;

	// if (auto v = config["pose"].as_array(); v) {
	// 	std::vector<float> components;
	// 	v->for_each([&components](auto&& e) {
	// 		if constexpr (toml::is_number<decltype(e)>) {
	// 			components.push_back(*e);
	// 		} else {
	// 			std::cerr << BOLDRED
	// 			          << " - 'pose' should consist of 7 floats in the format [x, y, z, "
	// 			             "qw, qx, qy, qz] "
	// 			             "(e.g., [0, 0, 0, 1, 0, 0, 0])\n"
	// 			          << RESET;
	// 			exit(EXIT_FAILURE);
	// 		}
	// 	});
	// 	if (7 != components.size()) {
	// 		std::cerr << BOLDRED
	// 		          << " - 'pose' should be in the format [x, y, z, qw, qx, qy, qz] "
	// 		             "(e.g., [0, 0, 0, 1, 0, 0, 0])\n"
	// 		          << RESET;
	// 		exit(EXIT_FAILURE);
	// 	}

	// 	camera.pose = ufo::Transform3f(
	// 	    ufo::Quatf(components[3], components[4], components[5], components[6]),
	// 	    ufo::Vec3f(components[0], components[1], components[2]));
	// } else {
	// 	std::cerr << BOLDRED << " - Missing pose.\n" << RESET;
	// 	exit(EXIT_FAILURE);
	// }

	// if (auto v = config["width"].value<std::uint32_t>(); v) {
	// 	camera.cols = *v;
	// } else {
	// 	std::cout << YELLOW << " - Missing width, using default width of '" << camera.cols
	// 	          << "' instead.\n"
	// 	          << RESET;
	// }

	// if (auto v = config["height"].value<std::uint32_t>(); v) {
	// 	camera.rows = *v;
	// } else {
	// 	std::cout << YELLOW << " - Missing height, using default height of '" << camera.rows
	// 	          << "' instead.\n"
	// 	          << RESET;
	// }

	// if (auto v = config["near_clip"].value<float>(); v) {
	// 	camera.near_clip = *v;
	// } else {
	// 	std::cout << YELLOW << " - Missing 'near_clip', using default near clip of '"
	// 	          << camera.near_clip << "' instead.\n"
	// 	          << RESET;
	// }

	// if (auto v = config["far_clip"].value<float>(); v) {
	// 	camera.far_clip = *v;
	// } else {
	// 	std::cout << YELLOW << " - Missing 'far_clip', using default far clip of '"
	// 	          << camera.far_clip << "' instead.\n"
	// 	          << RESET;
	// }

	// if (auto v = config["zoom"].value<float>(); v) {
	// 	camera.zoom = *v;
	// } else {
	// 	std::cout << YELLOW << " - Missing zoom, using default zoom of '" << camera.zoom
	// 	          << "' instead.\n"
	// 	          << RESET;
	// }

	// if (auto v = config["vertical_fov"].value<float>(); v) {
	// 	camera.vertical_fov = *v;
	// } else {
	// 	std::cout << YELLOW << " - Missing 'vertical_fov', using default vertical FOV of '"
	// 	          << camera.vertical_fov << "' instead.\n"
	// 	          << RESET;
	// }

	// camera.projection_type = ufo::ProjectionType::ORTHOGONAL;

	// std::filesystem::path save_dir;

	// if (auto v = config["output_dir"].value<std::string_view>(); v) {
	// 	save_dir = *v;
	// } else {
	// 	std::cerr << BOLDRED << " - Missing output_dir.\n" << RESET;
	// 	exit(EXIT_FAILURE);
	// }

	// if ("" == save_dir) {
	// 	std::cerr << BOLDRED << " - Empty output_dir.\n" << RESET;
	// 	exit(EXIT_FAILURE);
	// }

	// return Renderer(camera.rays(), save_dir, false);
}

[[nodiscard]] std::vector<std::pair<std::filesystem::path, ufo::Transform3f>> loadDataset(
    toml::table const& config)
{
	std::cout << "Loading dataset\n";

	if (auto v = config["use_dataset"].value<bool>(); v) {
		if (!(*v)) {
			std::cout << " - dataset disabled.\n";
			return std::vector<std::pair<std::filesystem::path, ufo::Transform3f>>();
		}
	} else {
		std::cerr << BOLDRED << " - Missing use_dataset.\n" << RESET;
		exit(EXIT_FAILURE);
	}

	std::filesystem::path dir;
	if (auto v = config["dataset_path"].value<std::string_view>(); v) {
		dir = *v;
	} else {
		std::cerr << BOLDRED << " - Missing dataset_path.\n" << RESET;
		exit(EXIT_FAILURE);
	}

	if ("" == dir) {
		std::cerr << BOLDRED << " - Empty dataset_path.\n" << RESET;
		exit(EXIT_FAILURE);
	}

	std::vector<std::filesystem::path> clouds;
	std::vector<ufo::Transform3f>      poses;
	for (auto const& entry : std::filesystem::directory_iterator(dir)) {
		if (!entry.is_regular_file()) {
			continue;
		}

		auto const& path = entry.path();

		if (".pcd" == path.extension()) {
			clouds.push_back(path);
		} else if (".tsv" == path.extension()) {
			if (!poses.empty()) {
				std::cerr << BOLDRED << " - Multiple poses files (i.e., '.tsv' files).\n"
				          << RESET;
				exit(EXIT_FAILURE);
			}

			std::ifstream data(path, std::ios::in | std::ios::binary);
			std::string   s;
			while (std::getline(data, s)) {
				std::stringstream  ss(s);
				std::vector<float> elements;

				std::string tmp;
				while (getline(ss, tmp, '\t')) {
					elements.push_back(std::stof(tmp));
				}

				poses.emplace_back(
				    normalize(ufo::Quatf(elements[6], elements[3], elements[4], elements[5])),
				    ufo::Vec3f(elements[0], elements[1], elements[2]));
				// poses.emplace_back(
				//     normalize(ufo::Quatf(elements[3], elements[4], elements[5], elements[6])),
				//     ufo::Vec3f(elements[0], elements[1], elements[2]));
			}
		}
	}

	std::cout << " - Found " + std::to_string(clouds.size()) + " clouds and " +
	                 std::to_string(poses.size()) + " poses.\n";

	if (clouds.size() != poses.size()) {
		std::cerr << BOLDRED << " - Number of clouds is not the same as number of poses\n"
		          << RESET;
		exit(EXIT_FAILURE);
	}

	assert(clouds.size() == poses.size());

	std::sort(clouds.begin(), clouds.end());

	std::vector<std::pair<std::filesystem::path, ufo::Transform3f>> ret;
	ret.reserve(clouds.size());

	std::transform(clouds.begin(), clouds.end(), poses.begin(), std::back_inserter(ret),
	               [](auto const& a, auto const& b) { return std::make_pair(a, b); });

	return ret;
}

int main(int argc, char* argv[])
{
	if (2 != argc) {
		std::cerr << BOLDRED << "Run the program like: `./UFOManipulation config.toml`\n"
		          << RESET;
		return EXIT_FAILURE;
	}

	toml::table tbl;
	try {
		tbl = toml::parse_file(argv[1]);
	} catch (toml::parse_error const& err) {
		std::cerr << BOLDRED << "Error parsing file '" << *err.source().path << "':\n"
		          << err.description() << "\n (" << err.source().begin << ")\n"
		          << RESET;
		return EXIT_FAILURE;
	}

	Map map = createMap(*tbl["map"].as_table());
	// Map  map(0.1);
	auto integrator = createIntegrator(*tbl["integrator"].as_table());
	auto cameras    = createCameras(*tbl["cameras"].as_table());
	auto renderer   = createRenderer(*tbl["renderer"].as_table());

	auto dataset = loadDataset(*tbl["dataset"].as_table());
	if (!dataset.empty()) {
		std::size_t                             i{};
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_first(
		    new pcl::PointCloud<pcl::PointXYZRGBA>);
		Eigen::Matrix4f camera_to_world{
		    // clang-format off
					{0, 0, 1, 0},
					{1, 0, 0, 0},
					{0, 1, 0, 0},
					{0, 0, 0, 1},
		    // clang-format on
		};
		for (auto const& [cloud_file, pose] : dataset) {
			std::cout << "\rIntegrating dataset [" << ++i << "/" << dataset.size() << "]"
			          << std::flush;

			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud(
			    new pcl::PointCloud<pcl::PointXYZRGBA>);
			if (-1 ==
			    pcl::io::loadPCDFile<pcl::PointXYZRGBA>(cloud_file.string(), *pcl_cloud)) {
				std::cerr << BOLDRED << "Could not read file " + cloud_file.string() + ".\n"
				          << RESET;
				exit(EXIT_FAILURE);
				continue;
			}

			for (auto& p : *pcl_cloud) {
				p.x *= 0.001f;
				p.y *= 0.001f;
				p.z *= 0.001f;
			}

			{
				ufo::Mat4x4f    tf = static_cast<ufo::Mat4x4f>(pose);
				Eigen::Matrix4f pcl_tf{
				    // clang-format off
					{tf[0][0], tf[1][0], tf[2][0], tf[3][0]},
					{tf[0][1], tf[1][1], tf[2][1], tf[3][1]},
					{tf[0][2], tf[1][2], tf[2][2], tf[3][2]},
					{tf[0][3], tf[1][3], tf[2][3], tf[3][3]},
				    // clang-format on
				};

				pcl_tf *= camera_to_world;

				pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, pcl_tf);

				pcl::CropBox<pcl::PointXYZRGBA> filter;
				filter.setMin(Eigen::Vector4f(-2.7, -2.7, -1.0, 0.0));
				filter.setMax(Eigen::Vector4f(2.7, 2.7, 10.0, 0.0));
				filter.setInputCloud(pcl_cloud);
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ds(
				    new pcl::PointCloud<pcl::PointXYZRGBA>);
				filter.filter(*ds);
				*pcl_cloud = *ds;

				pcl::UniformSampling<pcl::PointXYZRGBA> sampler;
				// pcl::VoxelGrid<pcl::PointXYZRGBA> sampler;
				sampler.setInputCloud(pcl_cloud);

				double sample_size = 0.02;
				int    nn          = 30;
				double std_ratio   = 1.5;
				sampler.setRadiusSearch(sample_size);
				// sampler.setLeafSize(sample_size, sample_size, sample_size);

				sampler.filter(*ds);

				// std::cout << pcl_cloud->size() << " vs " << ds->size() << '\n';

				if (!pcl_first->empty()) {
					pcl::GeneralizedIterativeClosestPoint6D icp;
					// pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
					icp.setInputSource(ds);
					icp.setInputTarget(pcl_first);

					icp.setMaximumIterations(50);
					icp.setMaxCorrespondenceDistance(0.3);
					icp.setRANSACOutlierRejectionThreshold(sample_size * 6);
					// icp.useBFGS();

					icp.setEuclideanFitnessEpsilon(1e-07);
					icp.setTransformationEpsilon(1e-09);
					icp.setTransformationRotationEpsilon(1e-07);

					// pcl::PointCloud<pcl::PointXYZRGBA> final;
					// icp.align(final);  //, pcl_tf);

					// // *pcl_cloud = final;

					// pcl_tf = icp.getFinalTransformation();
					// pcl_tf(0, 3) = 0;
					// pcl_tf(1, 3) = 0;
					// pcl_tf(2, 3) = 0;
					// pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, pcl_tf);

					std::cout << '\n' << pcl_tf << '\n';
				}

				if (pcl_first->empty()) {
					pcl_first = ds;
					// pcl_first = pcl_cloud;
				}
			}

			Cloud cloud;
			cloud.reserve(pcl_cloud->size());
			for (auto const& point : *pcl_cloud) {
				cloud.emplace_back(
				    ufo::Vec3f(point.x, point.y, point.z),
				    ufo::convert<ufo::FineLab>(ufo::SmallRGB{point.r, point.g, point.b}));
			}

			// ufo::transformInPlace(ufo::execution::par, (pose * camera_to_world), cloud);

			auto nodes = map.create(ufo::execution::par, cloud.view<ufo::Vec3f>());

			for (std::size_t i{}; cloud.size() > i; ++i) {
				// auto const& p = ufo::TreeCoord(cloud.view<ufo::Vec3f>()[i], 0);
				auto const& c = cloud.view<ufo::FineLab>()[i];
				auto        n = nodes[i];
				map.occupancyUpdate(n, 0.6f, false);
				map.colorAdd(n, c, false);
				// map.occupancySet(n, 0.8f, false);
				// map.colorSet(n, c, false);
			}

			// for (float x)

			// integrator(ufo::execution::par, map, cloud, pose);

			// if (2 == i) {
			// 	break;
			// }
		}
		std::cout << "\n";
	}

	map.propagate(ufo::execution::par);
	map.modifiedReset();

	std::vector<std::future<bool>> res;
	std::size_t                    iter{};
	while (true) {
		++iter;

		// res.clear();

		// // Start update for each camera
		// for (auto& camera : cameras) {
		// 	res.emplace_back(std::async(std::launch::async, &Camera::update, &camera));
		// }

		// assert(cameras.size() == res.size());

		// for (std::size_t i{}; cameras.size() > i; ++i) {
		// 	// Wait for camera update to finish
		// 	res[i].wait();

		// 	auto cloud = cameras[i].cloud();
		// 	auto pose  = cameras[i].pose();

		// 	// Update map
		// 	integrator(map, cloud, pose);
		// }

		if (10 == iter) {
			// Render view
			renderer.render(map);

			break;
		}
	}

	return EXIT_SUCCESS;
}