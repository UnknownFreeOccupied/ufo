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

// UFO
#include <ufo/glfw_webgpu/glfw_surface.h>

#include <ufo/viz/viz.hpp>

// STL
#include <algorithm>
#include <cassert>
#include <print>
#include <stdexcept>

// GLFW
#include <GLFW/glfw3.h>
#ifndef __EMSCRIPTEN__
#include <GLFW/glfw3native.h>
#endif

// ImGui
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_wgpu.h>
#include <imgui.h>

// EMSCRIPTEN
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/html5.h>
#if defined(IMGUI_IMPL_WEBGPU_BACKEND_WGPU)
#include <emscripten/html5_webgpu.h>
#endif
#include "../libs/emscripten/emscripten_mainloop_stub.h"
#endif

namespace ufo
{
Viz::Viz() {}

Viz::~Viz() { close(); }

bool Viz::running() const
{
	return nullptr != window_ && !glfwWindowShouldClose(window_);
}

void Viz::run()
{
#if defined(__EMSCRIPTEN__)
	// For an Emscripten build we are disabling file-system access, so let's not attempt to
	// do a fopen() of the imgui.ini file. You may manually call LoadIniSettingsFromMemory()
	// to load settings from your own storage.
	io.IniFilename = nullptr;

	auto callback = [](void* arg) {
		Viz* v = static_cast<Viz*>(arg);
		v->update();
	};
	emscripten_set_main_loop_arg(callback, this, 0, true);
#else
	while (running()) {
		update();
	}
#endif
}

void Viz::runAsync() { render_thread_ = std::thread(&Viz::run, this); }

bool Viz::open(int width, int height, bool resizable, std::string const& window_name,
               WGPUPowerPreference power_preference, WGPUBackendType backend_type)
{
	if (running()) {
		return false;
	}

	if (!initWindow(width, height, resizable, window_name)) {
		std::println(stderr, "[UFO | Viz] Failed to initialize window");
		return false;
	}

	if (!initWGPU(power_preference, backend_type)) {
		std::println(stderr, "[UFO | Viz] Failed to initialize WebGPU");
		glfwDestroyWindow(window_);
		glfwTerminate();
		return false;
	}

	glfwShowWindow(window_);

	if (!initGUI()) {
		std::println(stderr, "[UFO | Viz] Failed to initialize GUI");
		// TODO: Clean up
		return false;
	}

	if (!initCamera()) {
		std::println(stderr, "[UFO | Viz] Failed to initialize camera");
		// TODO: Clean up
		return false;
	}

	return true;
}

void Viz::close()
{
	if (!running()) {
		return;
	}

	if (render_thread_.joinable()) {
		render_thread_.join();
	}

	for (auto& renderable : renderables_) {
		renderable->release();
	}

	// ImGUI

	ImGui_ImplWGPU_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	// WGPU & GLFW

	compute::release(depth_view_);
	compute::release(depth_texture_);
	compute::release(queue_);
	compute::release(device_);
	compute::release(adapter_);
	wgpuSurfaceUnconfigure(surface_);  // FIXME: Needed?
	compute::release(surface_);
	glfwDestroyWindow(window_);
	compute::release(instance_);
	glfwTerminate();

	// GLFW

	glfwDestroyWindow(window_);
	glfwTerminate();

	window_ = nullptr;

	instance_      = nullptr;
	surface_       = nullptr;
	adapter_       = nullptr;
	device_        = nullptr;
	queue_         = nullptr;
	depth_texture_ = nullptr;
	depth_view_    = nullptr;
}

void Viz::resizeSurface(int width, int height)
{
	// scale_ = ImGui_ImplGlfw_GetContentScaleForWindow(window_);

	surface_width_  = width * scale_;
	surface_height_ = height * scale_;

	surface_config_.width  = surface_width_;
	surface_config_.height = surface_height_;

	wgpuSurfaceConfigure(surface_, &surface_config_);

	if (nullptr != depth_view_) {
		compute::release(depth_view_);
		depth_view_ = nullptr;
	}
	if (nullptr != depth_texture_) {
		compute::release(depth_texture_);
		depth_texture_ = nullptr;
	}

	if (WGPUTextureFormat_Undefined != depth_texture_format_) {
		WGPUTextureDescriptor depth_texture_desc   = WGPU_TEXTURE_DESCRIPTOR_INIT;
		depth_texture_desc.label                   = {"[UFO | Viz] Z Buffer", WGPU_STRLEN};
		depth_texture_desc.usage                   = WGPUTextureUsage_RenderAttachment;
		depth_texture_desc.size.width              = surface_width_;
		depth_texture_desc.size.height             = surface_height_;
		depth_texture_desc.size.depthOrArrayLayers = 1;
		depth_texture_desc.format                  = depth_texture_format_;

		depth_texture_ = wgpuDeviceCreateTexture(device_, &depth_texture_desc);

		WGPUTextureViewDescriptor depth_texture_view_desc = WGPU_TEXTURE_VIEW_DESCRIPTOR_INIT;
		depth_texture_view_desc.label           = {"[UFO | Viz] Z Buffer View", WGPU_STRLEN};
		depth_texture_view_desc.format          = wgpuTextureGetFormat(depth_texture_);
		depth_texture_view_desc.dimension       = WGPUTextureViewDimension_2D;
		depth_texture_view_desc.baseMipLevel    = 0;
		depth_texture_view_desc.mipLevelCount   = 1;
		depth_texture_view_desc.baseArrayLayer  = 0;
		depth_texture_view_desc.arrayLayerCount = 1;
		depth_texture_view_desc.aspect          = WGPUTextureAspect_All;
		depth_texture_view_desc.usage           = WGPUTextureUsage_RenderAttachment;

		depth_view_ = wgpuTextureCreateView(depth_texture_, &depth_texture_view_desc);
	}
}

void Viz::update()
{
	float cur_time = glfwGetTime();
	float dt       = cur_time - prev_time_;
	prev_time_     = cur_time;

	glfwPollEvents();

	wgpuInstanceProcessEvents(instance_);

	if (0 != glfwGetWindowAttrib(window_, GLFW_ICONIFIED)) {
		ImGui_ImplGlfw_Sleep(10);
		return;
	}

	// React to changes in screen size
	int width, height;
	glfwGetFramebufferSize(window_, &width, &height);
	if (width != surface_width_ || height != surface_height_) {
		// ImGui_ImplWGPU_InvalidateDeviceObjects();
		resizeSurface(width, height);
		// ImGui_ImplWGPU_CreateDeviceObjects();
	}

	WGPUSurfaceTexture surface_texture;
	wgpuSurfaceGetCurrentTexture(surface_, &surface_texture);
	if (ImGui_ImplWGPU_IsSurfaceStatusError(surface_texture.status)) {
		std::println(stderr, "[UFO | Viz] Unrecoverable Surface Texture status");
		// TODO: Implement
		// std::println(stderr, "[UFO | Viz] Unrecoverable Surface Texture status={}",
		//              surface_texture.status);
		abort();  // TODO: What to do here?
	}
	if (ImGui_ImplWGPU_IsSurfaceStatusSubOptimal(surface_texture.status)) {
		if (nullptr != surface_texture.texture) {
			wgpuTextureRelease(surface_texture.texture);
		}

		if (0 < width && 0 < height) {
			// ImGui_ImplWGPU_InvalidateDeviceObjects();
			resizeSurface(width, height);
			// ImGui_ImplWGPU_CreateDeviceObjects();
		}

		return;
	}

	updateGui();

	// TODO: For some reason the surface texture size can be different from the imgui
	// display size

	ImGuiIO& io = ImGui::GetIO();
	// std::println("Framebuffer size:          {}x{}", width, height);
	// std::println("Display size:              {}x{}", io.DisplaySize.x, io.DisplaySize.y);
	// std::println("Display framebuffer scale: {}x{}", io.DisplayFramebufferScale.x,
	//              io.DisplayFramebufferScale.y);
	// std::print("\n");

	if (width != io.DisplaySize.x || height != io.DisplaySize.y) {
		wgpuTextureRelease(surface_texture.texture);
		return;
	}

	updateCamera(dt);

	WGPUTextureViewDescriptor view_desc = WGPU_TEXTURE_VIEW_DESCRIPTOR_INIT;
	view_desc.label                     = {"[UFO | Viz] Surface Texture View", WGPU_STRLEN};
	view_desc.format                    = surface_config_.format;
	view_desc.dimension                 = WGPUTextureViewDimension_2D;
	view_desc.mipLevelCount             = WGPU_MIP_LEVEL_COUNT_UNDEFINED;
	view_desc.arrayLayerCount           = WGPU_ARRAY_LAYER_COUNT_UNDEFINED;
	view_desc.aspect                    = WGPUTextureAspect_All;
	view_desc.usage                     = WGPUTextureUsage_RenderAttachment;

	WGPUTextureView texture_view =
	    wgpuTextureCreateView(surface_texture.texture, &view_desc);
	assert(nullptr != texture_view);

	WGPURenderPassColorAttachment color_attachments =
	    WGPU_RENDER_PASS_COLOR_ATTACHMENT_INIT;
	color_attachments.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
	color_attachments.loadOp     = WGPULoadOp_Clear;
	color_attachments.storeOp    = WGPUStoreOp_Store;
	color_attachments.clearValue = WGPUColor{
	    clear_color_.red * clear_color_.alpha, clear_color_.green * clear_color_.alpha,
	    clear_color_.blue * clear_color_.alpha, clear_color_.alpha};
	color_attachments.view = texture_view;

	WGPURenderPassDepthStencilAttachment depth_attachment =
	    WGPU_RENDER_PASS_DEPTH_STENCIL_ATTACHMENT_INIT;
	depth_attachment.depthLoadOp     = WGPULoadOp_Clear;
	depth_attachment.depthStoreOp    = WGPUStoreOp_Store;
	depth_attachment.depthClearValue = 1.0;
	depth_attachment.depthReadOnly   = false;
	depth_attachment.view            = depth_view_;

	WGPURenderPassDescriptor render_pass_desc = WGPU_RENDER_PASS_DESCRIPTOR_INIT;
	render_pass_desc.label                    = {"[UFO | Viz] Render Pass", WGPU_STRLEN};
	render_pass_desc.colorAttachmentCount     = 1;
	render_pass_desc.colorAttachments         = &color_attachments;
	render_pass_desc.depthStencilAttachment   = &depth_attachment;

	WGPUCommandEncoderDescriptor encoder_desc = WGPU_COMMAND_ENCODER_DESCRIPTOR_INIT;
	encoder_desc.label = {"[UFO | Viz] Command Encoder", WGPU_STRLEN};

	WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device_, &encoder_desc);

	WGPURenderPassEncoder pass =
	    wgpuCommandEncoderBeginRenderPass(encoder, &render_pass_desc);

	ImGui_ImplWGPU_RenderDrawData(ImGui::GetDrawData(), pass);

	for (auto& renderable : renderables_) {
		renderable->update(device_, encoder, pass, camera_);
	}

	wgpuRenderPassEncoderEnd(pass);

	WGPUCommandBufferDescriptor command_buffer_desc = WGPU_COMMAND_BUFFER_DESCRIPTOR_INIT;
	command_buffer_desc.label = {"[UFO | Viz] Command Buffer", WGPU_STRLEN};
	WGPUCommandBuffer command_buffer =
	    wgpuCommandEncoderFinish(encoder, &command_buffer_desc);

	wgpuQueueSubmit(queue_, 1, &command_buffer);

#ifndef __EMSCRIPTEN__
	wgpuSurfacePresent(surface_);
#endif

	compute::release(command_buffer);
	compute::release(pass);
	compute::release(encoder);
	compute::release(texture_view);
	compute::release(surface_texture.texture);
}

WGPUDevice Viz::device() const { return device_; }

void Viz::addRenderable(std::shared_ptr<Renderable> const& renderable)
{
	// NOTE: Not thread safe, call before open() or ensure device_ is valid

	renderables_.push_back(renderable);

	if (nullptr == device_) {
		return;
	}

	renderable->init(device_, surface_preferred_format_);
}

void Viz::eraseRenderable(std::shared_ptr<Renderable> const& renderable)
{
	// NOTE: Not thread safe, call before open() or ensure device_ is valid

	renderables_.erase(std::remove(renderables_.begin(), renderables_.end(), renderable),
	                   renderables_.end());
}

void Viz::clearRenderables()
{
	// NOTE: Not thread safe, call before open() or ensure device_ is valid

	renderables_.clear();
}

void Viz::loadConfig(std::filesystem::path const& config)
{
	// TODO: Implement
}

void Viz::saveConfig(std::filesystem::path const& file) const
{
	// TODO: Implement
}

WGPULimits Viz::requiredLimits(WGPUAdapter adapter) const
{
	WGPULimits required  = WGPU_LIMITS_INIT;
	WGPULimits supported = WGPU_LIMITS_INIT;

	wgpuAdapterGetLimits(adapter, &supported);

	// These two limits are different because they are "minimum" limits,
	// they are the only ones we may forward from the adapter's supported limits.
	required.minUniformBufferOffsetAlignment = supported.minUniformBufferOffsetAlignment;
	required.minStorageBufferOffsetAlignment = supported.minStorageBufferOffsetAlignment;

	// TODO: Update to what is needed

	required.maxBindGroups = 2;

	required.maxBufferSize               = 2'147'483'648;
	required.maxStorageBufferBindingSize = 2'147'483'648;

	required.maxBufferSize = std::min(required.maxBufferSize, supported.maxBufferSize);
	required.maxStorageBufferBindingSize = std::min(required.maxStorageBufferBindingSize,
	                                                supported.maxStorageBufferBindingSize);

	required.maxComputeWorkgroupStorageSize    = 16352;
	required.maxComputeInvocationsPerWorkgroup = 256;
	required.maxComputeWorkgroupSizeX          = 256;
	required.maxComputeWorkgroupSizeY          = 256;
	required.maxComputeWorkgroupSizeZ          = 64;
	required.maxComputeWorkgroupsPerDimension  = 65535;

	required.maxUniformBuffersPerShaderStage = 12;
	required.maxUniformBufferBindingSize     = 65536;

	return required;
}

WGPUTextureFormat Viz::surfaceFormat(WGPUSurfaceCapabilities capabilities) const
{
	assert(0 < capabilities.formatCount);
	return capabilities.formats[0];
}

void Viz::updateCamera(float dt)
{
	// TODO: Implement

	Vec3f speed;
	float speed_multiplier = 2.0f;

	if (GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_W) ||
	    GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_UP)) {
		speed.x += translation_speed_;
	}

	if (GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_S) ||
	    GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_DOWN)) {
		speed.x -= translation_speed_;
	}

	if (GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_A) ||
	    GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_LEFT)) {
		speed.y += translation_speed_;
	}

	if (GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_D) ||
	    GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_RIGHT)) {
		speed.y -= translation_speed_;
	}

	if (GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_SPACE)) {
		speed.z += translation_speed_;
	}

	if (GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_LEFT_CONTROL)) {
		speed.z -= translation_speed_;
	}

	if (GLFW_PRESS == glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT)) {
		speed *= speed_multiplier;
	}

	camera_.pose.translation += camera_.pose.rotation * speed * dt;

	// float      cx     = std::cos(angles_.x);
	// float      cy     = std::cos(angles_.y);
	// float      sx     = std::sin(angles_.x);
	// float      sy     = std::sin(angles_.y);
	// ufo::Vec3f offset = ufo::Vec3f(cx * cy, sx * cy, sy) * std::exp(-zoom_);
	// camera_.pose      = static_cast<ufo::Transform3f>(
	//     ufo::lookAt<float, true>(center_ + offset, center_, camera_.up));
}

bool Viz::initWindow(int width, int height, bool resizable, std::string const& title)
{
	std::println("[UFO | Viz] Initializing GLFW window...");

	glfwSetErrorCallback([](int error, char const* description) {
		std::println(stderr, "[UFO | Viz] GLFW Error ({}): {}", error, description);
	});

	if (!glfwInit()) {
		throw std::runtime_error("[UFO | Viz] Failed to initialize GLFW");
	}

	glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
	glfwWindowHint(GLFW_RESIZABLE, resizable ? GLFW_TRUE : GLFW_FALSE);

	scale_          = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
	surface_width_  = static_cast<std::uint32_t>(width * scale_);
	surface_height_ = static_cast<std::uint32_t>(height * scale_);

	window_ =
	    glfwCreateWindow(surface_width_, surface_height_, title.c_str(), nullptr, nullptr);

	if (nullptr == window_) {
		return false;
	}

	glfwSetWindowUserPointer(window_, static_cast<void*>(this));

	glfwSetKeyCallback(window_,
	                   [](GLFWwindow* window, int key, int scancode, int action, int mods) {
		                   Viz* v = static_cast<Viz*>(glfwGetWindowUserPointer(window));
		                   if (nullptr == v) {
			                   return;
		                   }
		                   v->onKey(key, scancode, action, mods);
	                   });

	glfwSetCursorPosCallback(window_, [](GLFWwindow* window, double x_pos, double y_pos) {
		Viz* v = static_cast<Viz*>(glfwGetWindowUserPointer(window));
		if (nullptr == v) {
			return;
		}
		v->onMouseMove(x_pos, y_pos);
	});

	glfwSetMouseButtonCallback(
	    window_, [](GLFWwindow* window, int button, int action, int mods) {
		    Viz* v = static_cast<Viz*>(glfwGetWindowUserPointer(window));
		    if (nullptr == v) {
			    return;
		    }
		    v->onMouseButton(button, action, mods);
	    });

	glfwSetScrollCallback(window_,
	                      [](GLFWwindow* window, double x_offset, double y_offset) {
		                      Viz* v = static_cast<Viz*>(glfwGetWindowUserPointer(window));
		                      if (nullptr == v) {
			                      return;
		                      }
		                      v->onScroll(x_offset, y_offset);
	                      });

	return true;
}

bool Viz::initWGPU(WGPUPowerPreference power_preference, WGPUBackendType backend_type)
{
	std::println("[UFO | Viz] Initializing WebGPU...");

	std::println("[UFO | Viz] Creating WGPU instance...");
	instance_ = compute::createInstance();

#ifdef __EMSCRIPTEN__
// TODO: Implement
#else
	wgpuSetLogCallback(
	    [](WGPULogLevel level, WGPUStringView msg, void* userdata) {
		    std::println(stderr, "[UFO | Viz] WebGPU Log (level={}): {}",
		                 ImGui_ImplWGPU_GetLogLevelName(level),
		                 std::string(msg.data, msg.length));
	    },
	    nullptr);
	wgpuSetLogLevel(WGPULogLevel_Warn);

	std::println("[UFO | Viz] Creating WGPU surface...");
	surface_ = glfwSurface(instance_, window_);
	if (nullptr == surface_) {
		std::println(stderr, "[UFO | Viz] Failed to create WGPU surface");
		return false;
	}

	std::println("[UFO | Viz] Requesting WGPU adapter...");
	adapter_ = compute::createAdapter(instance_, surface_, power_preference, backend_type);

	ImGui_ImplWGPU_DebugPrintAdapterInfo(adapter_);

	auto limits = requiredLimits(adapter_);
	std::println("[UFO | Viz] Creating WGPU device...");
	device_ = compute::createDevice(adapter_, &limits);

	auto surface_capabilities = compute::surfaceCapabilities(surface_, adapter_);
	surface_preferred_format_ = surfaceFormat(surface_capabilities);
	wgpuSurfaceCapabilitiesFreeMembers(surface_capabilities);
#endif

	surface_config_             = WGPU_SURFACE_CONFIGURATION_INIT;
	surface_config_.presentMode = WGPUPresentMode_Fifo;
	surface_config_.alphaMode   = WGPUCompositeAlphaMode_Auto;
	surface_config_.usage       = WGPUTextureUsage_RenderAttachment;
	surface_config_.device      = device_;
	surface_config_.format      = surface_preferred_format_;

	resizeSurface(surface_width_, surface_height_);

	queue_ = compute::queue(device_);

	return true;
}

bool Viz::initGUI()
{
	std::println("[UFO | Viz] Initializing ImGui...");

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

	ImGui::StyleColorsDark();
	// ImGui::StyleColorsLight();

	ImGuiStyle& style = ImGui::GetStyle();
	style.ScaleAllSizes(scale_);
	style.FontScaleDpi = scale_;

	ImGui_ImplGlfw_InitForOther(window_, true);
#ifdef __EMSCRIPTEN__
	ImGui_ImplGlfw_InstallEmscriptenCallbacks(window_, "#canvas");
#endif

	ImGui_ImplWGPU_InitInfo init_info;
	init_info.Device             = device_;
	init_info.NumFramesInFlight  = 3;
	init_info.RenderTargetFormat = surface_preferred_format_;
	init_info.DepthStencilFormat = depth_texture_format_;
	ImGui_ImplWGPU_Init(&init_info);

	// Load Fonts
	// - If no fonts are loaded, dear imgui will use the default font. You can also load
	// multiple fonts and use ImGui::PushFont()/PopFont() to select them.
	// - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to
	// select the font among multiple.
	// - If the file cannot be loaded, the function will return a nullptr. Please handle
	// those errors in your application (e.g. use an assertion, or display an error and
	// quit).
	// - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for
	// higher quality font rendering.
	// - Read 'docs/FONTS.md' for more instructions and details.
	// - Remember that in C/C++ if you want to include a backslash \ in a string literal you
	// need to write a double backslash \\ !
	// - Emscripten allows preloading a file or folder to be accessible at runtime. See
	// Makefile for details.
	// io.Fonts->AddFontDefault();
	// style.FontSizeBase = 20.0f;
#ifndef IMGUI_DISABLE_FILE_FUNCTIONS
	// io.Fonts->AddFontFromFileTTF("fonts/segoeui.ttf");
	// io.Fonts->AddFontFromFileTTF("fonts/DroidSans.ttf");
	// io.Fonts->AddFontFromFileTTF("fonts/Roboto-Medium.ttf");
	// io.Fonts->AddFontFromFileTTF("fonts/Cousine-Regular.ttf");
	// io.Fonts->AddFontFromFileTTF("fonts/ProggyTiny.ttf");
	// ImFont* font = io.Fonts->AddFontFromFileTTF("fonts/ArialUni.ttf");
	// IM_ASSERT(font != nullptr);
#endif

	return true;
}

bool Viz::initCamera()
{
	std::println("[UFO | Viz] Initializing camera...");

	// TODO: Implement

	// camera_.vertical_fov = ufo::radians(60.0f);
	// camera_.near_clip    = 0.01;
	// camera_.far_clip     = 10000.0;
	// camera_.rows         = surface_config_.height;
	// camera_.cols         = surface_config_.width;

	return true;
}

void Viz::updateGui()
{
	// // Remove resize grips by making them the same color as the window bg
	// ImGui::PushStyleColor(ImGuiCol_ResizeGrip, ImGui::GetColorU32(ImGuiCol_WindowBg));
	// ImGui::PushStyleColor(ImGuiCol_ResizeGripActive,
	// ImGui::GetColorU32(ImGuiCol_WindowBg));
	// ImGui::PushStyleColor(ImGuiCol_ResizeGripHovered,
	//                       ImGui::GetColorU32(ImGuiCol_WindowBg));

	// if (ImGuiMouseCursor_ResizeNWSE == ImGui::GetMouseCursor()) {
	// 	ImGui::SetMouseCursor(ImGuiMouseCursor_Arrow);
	// }

	// // Title bars the same color as the window bg
	// ImGui::PushStyleColor(ImGuiCol_TitleBg, ImGui::GetColorU32(ImGuiCol_WindowBg));
	// ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImGui::GetColorU32(ImGuiCol_WindowBg));
	// ImGui::PushStyleColor(ImGuiCol_TitleBgCollapsed,
	// ImGui::GetColorU32(ImGuiCol_WindowBg));

	// // Separator
	// ImGui::PushStyleColor(ImGuiCol_Separator, ImGui::GetColorU32(ImGuiCol_WindowBg));
	// ImGui::PushStyleColor(ImGuiCol_SeparatorActive,
	// ImGui::GetColorU32(ImGuiCol_WindowBg));
	// ImGui::PushStyleColor(ImGuiCol_SeparatorHovered,
	// ImGui::GetColorU32(ImGuiCol_WindowBg));

	ImGui_ImplWGPU_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	ImGuiIO& io = ImGui::GetIO();

	// TODO: Implement

	ImVec2 top_size    = ImVec2(0, 0);
	ImVec2 bottom_size = ImVec2(0, 0);

	{
		ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always, ImVec2(0.0, 0.0));
		// ImGui::SetNextWindowSize(ImVec2(surface_width_ / 4, surface_height_),
		//                          ImGuiCond_Always);
		ImGui::SetNextWindowSizeConstraints(ImVec2(io.DisplaySize.x, 0),
		                                    ImVec2(io.DisplaySize.x, io.DisplaySize.y));
		ImGui::SetNextWindowBgAlpha(1.0);

		ImGui::Begin("Top", nullptr,
		             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
		                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
		                 ImGuiWindowFlags_AlwaysAutoResize);

		top_size = ImGui::GetWindowSize();

		if (ImGui::Button("[")) {
			show_left_panel_ = !show_left_panel_;
		}
		ImGui::SameLine();
		if (ImGui::Button("_")) {
			show_bottom_panel_ = !show_bottom_panel_;
		}
		ImGui::SameLine();
		if (ImGui::Button("]")) {
			show_right_panel_ = !show_right_panel_;
		}

		ImGui::End();
	}

	if (show_bottom_panel_) {
		ImGui::SetNextWindowPos(ImVec2(0, io.DisplaySize.y), ImGuiCond_Always,
		                        ImVec2(0.0, 1.0));
		// ImGui::SetNextWindowSize(ImVec2(surface_width_ / 4, surface_height_),
		//                          ImGuiCond_Always);
		ImGui::SetNextWindowSizeConstraints(ImVec2(io.DisplaySize.x, 100),
		                                    ImVec2(io.DisplaySize.x, 500));
		ImGui::SetNextWindowBgAlpha(1.0);

		ImGui::Begin("Info", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

		bottom_size = ImGui::GetWindowSize();

		ImGui::Text("Framerate %.1f FPS (%.3f ms/frame)", io.Framerate,
		            1000.0f / io.Framerate);

		frame_times_.push_back(1000.0f / io.Framerate);
		if (frame_times_.size() > 1000) {
			frame_times_.erase(frame_times_.begin());
		}

		float total = 0.0f;
		float min   = std::numeric_limits<float>::max();
		float max   = std::numeric_limits<float>::lowest();
		for (auto const& ft : frame_times_) {
			total += ft;
			min = std::min(min, ft);
			max = std::max(max, ft);
		}
		float average = total / frame_times_.size();

		std::string overlay =
		    std::format("avg {:.3f} ms, min {:.3f} ms, max {:.3f}", average, min, max);
		ImGui::PlotLines(
		    "Frame Times", frame_times_.data(), frame_times_.size(), 0, overlay.c_str(),
		    *std::min_element(frame_times_.begin(), frame_times_.end()),
		    *std::max_element(frame_times_.begin(), frame_times_.end()), ImVec2(0, 100));

		ImGui::End();
	}

	if (show_left_panel_) {
		ImGui::SetNextWindowPos(ImVec2(0, top_size.y), ImGuiCond_Always, ImVec2(0.0, 0.0));
		// ImGui::SetNextWindowSize(ImVec2(surface_width_ / 4, surface_height_),
		//                          ImGuiCond_Always);
		ImGui::SetNextWindowSizeConstraints(
		    ImVec2(100, io.DisplaySize.y - top_size.y - bottom_size.y),
		    ImVec2(io.DisplaySize.x / 2, io.DisplaySize.y - top_size.y - bottom_size.y));
		ImGui::SetNextWindowBgAlpha(1.0);

		ImGui::Begin("Left", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

		ImGui::ColorEdit3("clear color", (float*)&clear_color_);

		ImGui::End();
	}

	if (show_right_panel_) {
		ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x, top_size.y), ImGuiCond_Always,
		                        ImVec2(1.0, 0.0));
		// ImGui::SetNextWindowSize(ImVec2(surface_width_ / 4, surface_height_),
		//                          ImGuiCond_Always);
		ImGui::SetNextWindowSizeConstraints(
		    ImVec2(100, io.DisplaySize.y - top_size.y - bottom_size.y),
		    ImVec2(io.DisplaySize.x / 2, io.DisplaySize.y - top_size.y - bottom_size.y));
		ImGui::SetNextWindowBgAlpha(1.0);

		ImGui::Begin("View", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

		ImGui::SeparatorText("Camera");

		if (ImGui::BeginTable("CameraTable", 2, ImGuiTableFlags_SizingStretchProp)) {
			ImGui::TableNextColumn();
			ImGui::Text("Control");
			ImGui::SetItemTooltip("I am a tooltip");
			ImGui::TableNextColumn();
			ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
			if (ImGui::BeginTabBar("ControlTabs")) {
				if (ImGui::BeginTabItem("FPS")) {
					control_type_ = 0;

					if (ImGui::BeginTable("FPSTable", 2, ImGuiTableFlags_SizingStretchProp)) {
						ImGui::TableNextColumn();
						ImGui::Text("Move speed");
						ImGui::TableNextColumn();
						ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
						ImGui::SliderFloat("##t_speed", &translation_speed_, 0.01f, 10000.0f,
						                   "%.3f (m/s)", ImGuiSliderFlags_Logarithmic);

						ImGui::EndTable();
					}

					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("Orbit")) {
					control_type_ = 1;

					if (ImGui::BeginTable("OrbitTable", 2, ImGuiTableFlags_SizingStretchProp)) {
						ImGui::TableNextColumn();
						ImGui::Text("Fixed axis");
						ImGui::TableNextColumn();
						ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
						char const* items[] = {"None", "X-axis", "Y-axis", "Z-axis"};
						ImGui::Combo("##fixed_axis", &projection_type_, items, IM_ARRAYSIZE(items));

						ImGui::EndTable();
					}

					ImGui::EndTabItem();
				}
				ImGui::EndTabBar();
			}

			ImGui::TableNextColumn();
			ImGui::Text("Mouse sense");
			ImGui::TableNextColumn();
			ImGui::SliderFloat("##mouse_sense", &rotation_speed_, 0.01f, 10.0f, "%.3f");

			ImGui::TableNextColumn();
			ImGui::Text("Projection");
			ImGui::TableNextColumn();
			char const* projection_items[] = {"Perspective", "Orthogonal"};
			ImGui::Combo("##perspective", &projection_type_, projection_items,
			             IM_ARRAYSIZE(projection_items));

			ImGui::TableNextColumn();
			ImGui::Text("Position");
			ImGui::TableNextColumn();
			ImGui::DragFloat3("##position", &camera_.pose.translation.x, 0.01f, 0.0f, 0.0f,
			                  "%.3f");

			ImGui::TableNextColumn();
			ImGui::Text("Orientation");
			ImGui::TableNextColumn();
			if (ImGui::BeginTabBar("OrientationTabs")) {
				Quatf rot(camera_.pose.rotation);
				if (ImGui::BeginTabItem("Quat")) {
					if (ImGui::DragFloat4("##orientation_quat", &rot.w, 0.01f, -1.0f, 1.0f,
					                      "%.3f")) {
						camera_.pose.rotation = static_cast<Mat3x3f>(rot);
					}
					ImGui::SetItemTooltip("W, X, Y, Z");
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("RPY")) {
					Vec3f rpy{degrees(pitch(rot)), degrees(yaw(rot)), degrees(roll(rot))};

					if (ImGui::DragFloat3("##orientation_rpy", &rpy.x, 0.1f, -360.0f, 360.0f,
					                      "%.2f")) {
						rpy.x                 = radians(rpy.x);
						rpy.y                 = radians(rpy.y);
						rpy.z                 = radians(rpy.z);
						camera_.pose.rotation = static_cast<Mat3x3f>(Quatf(rpy));
					}
					ImGui::SetItemTooltip("Roll, Pitch, Yaw (deg)");
					ImGui::EndTabItem();
				}
				ImGui::EndTabBar();
			}

			ImGui::TableNextColumn();
			ImGui::Text("Field of view");
			ImGui::TableNextColumn();
			float fov_deg = degrees(camera_.vertical_fov);
			if (ImGui::SliderFloat("##fov", &fov_deg, 1.0f, 179.0f, "%.1f (deg)")) {
				camera_.vertical_fov = radians(fov_deg);
			}

			ImGui::TableNextColumn();
			ImGui::Text("Zoom");
			ImGui::TableNextColumn();
			ImGui::SliderFloat("##zoom", &zoom_, -100.0f, 100.0f, "%.3f");
			ImGui::EndTable();
		}

		ImGui::End();
	}

	ImGui::Render();
}

void Viz::onMouseMove(double x_pos, double y_pos)
{
	if (mouse_drag_) {
		Vec2f cur_mouse_pos(static_cast<float>(x_pos), static_cast<float>(y_pos));
		Vec2f delta = (cur_mouse_pos - start_mouse_pos_) * mouse_sense_;

		// TODO: Implement

		// m_cameraState.angles = m_drag.startCameraState.angles + delta;
		// // Clamp to avoid going too far when orbitting up/down
		// m_cameraState.angles.y =
		//     glm::clamp(m_cameraState.angles.y, -PI / 2 + 1e-5f, PI / 2 - 1e-5f);

		// Vec3f euler_angles = toEulerAngles(start_camera_state_.pose.rotation);

		// euler_angles.y -= rotation_speed_ * delta.x * 0.01f;
		// euler_angles.x -= rotation_speed_ * delta.y * 0.01f;

		// // Clamp the pitch to avoid gimball lock
		// euler_angles.x = std::clamp(euler_angles.x, -ufo::half_pi<float>() + 0.01f,
		//                             ufo::half_pi<float>() - 0.01f);

		// start_camera_state_.pose.rotation = static_cast<Mat3x3f>(Quatf(euler_angles));
		// camera_.pose.rotation             = start_camera_state_.pose.rotation;

		updateViewMatrix();
	}

	// TODO: Implement
}

void Viz::onMouseButton(int button, int action, int modifiers)
{
	ImGuiIO& io = ImGui::GetIO();
	if (io.WantCaptureMouse) {
		// Don't rotate the camera if the mouse is already captured by an ImGui
		// interaction at this frame.
		return;
	}

	if (GLFW_MOUSE_BUTTON_LEFT == button) {
		switch (action) {
			case GLFW_PRESS:
				mouse_drag_ = true;
				double xpos, ypos;
				glfwGetCursorPos(window_, &xpos, &ypos);
				start_mouse_pos_    = Vec2f(static_cast<float>(xpos), static_cast<float>(ypos));
				start_camera_state_ = camera_;
				break;
			case GLFW_RELEASE: mouse_drag_ = false; break;
		}
	}

	// TODO: Implement
}

void Viz::onScroll(double x_offset, double y_offset)
{
	// TODO: Implement
	zoom_ += scroll_sensitivity_ * static_cast<float>(y_offset);
	updateViewMatrix();
}

void Viz::onKey(int key, int scancode, int action, int mods)
{
	// TODO: Implement
}

void Viz::updateViewMatrix()
{
	// TODO: Implement
	// float cx              = cos(m_cameraState.angles.x);
	// float sx              = sin(m_cameraState.angles.x);
	// float cy              = cos(m_cameraState.angles.y);
	// float sy              = sin(m_cameraState.angles.y);
	// vec3  position        = vec3(cx * cy, sx * cy, sy) * std::exp(-m_cameraState.zoom);
	// m_uniforms.viewMatrix = glm::lookAt(position, vec3(0.0f), vec3(0, 0, 1));
	// m_queue.writeBuffer(m_uniformBuffer, offsetof(MyUniforms, viewMatrix),
	//                     &m_uniforms.viewMatrix, sizeof(MyUniforms::viewMatrix));
}
}  // namespace ufo