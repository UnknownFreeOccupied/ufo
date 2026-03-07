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

#ifndef UFO_VIZ_VIZ_HPP
#define UFO_VIZ_VIZ_HPP

// UFO
#include <ufo/compute/compute.hpp>
#include <ufo/vision/camera.hpp>
#include <ufo/vision/color.hpp>
#include <ufo/viz/renderable.hpp>
// #include <ufo/viz/renderable/map.hpp>

// STL
#include <array>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Forward declare
struct GLFWwindow;

namespace ufo
{
class Viz
{
 public:
	Viz();

	~Viz();

	[[nodiscard]] bool running() const;

	bool open(int width = 1280, int height = 800, bool resizable = true,
	          std::string const&  window_name      = "UFOViz",
	          WGPUPowerPreference power_preference = WGPUPowerPreference_Undefined,
	          WGPUBackendType     backend_type     = WGPUBackendType_Undefined);

	void close();

	void run();

	void runAsync();

	void update();

	[[nodiscard]] WGPUDevice device() const;

	void addRenderable(std::shared_ptr<Renderable> const& renderable);

	// template <std::size_t Dim, class... Maps>
	// void addRenderable(Map<Dim, Maps...> const& map)
	// {
	// 	renderables_.push_back(std::make_shared<RenderableMap<Map<Dim, Maps...>>>(map));
	// }

	// TODO: Implement other

	void eraseRenderable(std::shared_ptr<Renderable> const& renderable);

	// template <std::size_t Dim, class... Maps>
	// bool eraseRenderable(Map<Dim, Maps...> const& map)
	// {
	// 	auto it = std::find_if(
	// 	    renderables_.begin(), renderables_.end(),
	// 	    [&map](std::shared_ptr<Renderable> const& renderable) {
	// 		    auto renderable_map =
	// 		        std::dynamic_pointer_cast<RenderableMap<Map<Dim, Maps...>>>(renderable);
	// 		    if (nullptr == renderable_map) {
	// 			    return false;
	// 		    }
	// 		    return renderable_map->map() == map;
	// 	    });

	// 	if (it != renderables_.end()) {
	// 		renderables_.erase(it);
	// 		return true;
	// 	}

	// 	return false;
	// }

	// TODO: Implement other

	void clearRenderables();

	[[nodiscard]] std::size_t numRenderables() const;

	void loadConfig(std::filesystem::path const& config);

	void saveConfig(std::filesystem::path const& file) const;

 private:
	[[nodiscard]] WGPULimits requiredLimits(WGPUAdapter adapter) const;

	[[nodiscard]] WGPUTextureFormat surfaceFormat(
	    WGPUSurfaceCapabilities capabilities) const;

	void resizeSurface(int width, int height);

	void updateCamera(float dt);

	bool initWindow(int width, int height, bool resizable, std::string const& title);

	bool initWGPU(WGPUPowerPreference power_preference, WGPUBackendType backend_type);

	bool initGUI();

	bool initCamera();

	void updateGui();

	// Party events
	void onMouseMove(double x_pos, double y_pos);

	void onMouseButton(int button, int action, int modifiers);

	void onScroll(double x_offset, double y_offset);

	void onKey(int key, int scancode, int action, int mods);

	void updateViewMatrix();

 private:
	GLFWwindow* window_ = nullptr;

	WGPUInstance             instance_       = nullptr;
	WGPUSurface              surface_        = nullptr;
	WGPUAdapter              adapter_        = nullptr;
	WGPUDevice               device_         = nullptr;
	WGPUQueue                queue_          = nullptr;
	WGPUSurfaceConfiguration surface_config_ = WGPU_SURFACE_CONFIGURATION_INIT;

	WGPUTextureFormat surface_preferred_format_ = WGPUTextureFormat_Undefined;
	int               surface_width_            = 0;
	int               surface_height_           = 0;

	WGPUTextureFormat depth_texture_format_ = WGPUTextureFormat_Depth24Plus;
	WGPUTexture       depth_texture_        = nullptr;
	WGPUTextureView   depth_view_           = nullptr;

	FineRGBA clear_color_ = FineRGBA(0.45f, 0.55f, 0.60f, 1.00f);

	bool show_left_panel_   = true;
	bool show_right_panel_  = true;
	bool show_bottom_panel_ = true;
	int  control_type_      = 0;
	int  projection_type_   = 0;

	std::thread render_thread_;

	float prev_time_{};

	float scale_ = 1.0f;

	std::vector<std::shared_ptr<Renderable>> renderables_;

	Camera     camera_;
	float      translation_speed_ = 2.5f;
	float      rotation_speed_    = 90.0f;  // Degrees per second
	ufo::Vec2f angles_{0.0f, 0.0f};
	ufo::Vec3f center_{4.4f, 0.0f, 1.7f};
	float      zoom_ = 0.0f;

	bool       mouse_drag_         = false;
	ufo::Vec2f start_mouse_pos_    = {0.0f, 0.0f};
	Camera     start_camera_state_ = {};
	float      mouse_sense_        = 1.0f;
	float      scroll_sensitivity_ = 0.1f;

	std::vector<float> frame_times_;
};
}  // namespace ufo

#endif  // UFO_VIZ_VIZ_HPP