// UFO
#include <ufo/compute/compute.hpp>

// STL
#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>

namespace ufo::compute
{
WGPUInstance createInstance() { return wgpuCreateInstance(nullptr); }

WGPUAdapter createAdapter(WGPUInstance instance, WGPUSurface surface,
                          WGPUPowerPreference power_preference,
                          WGPUBackendType     backend_type)
{
	assert(nullptr != instance);

	struct UserData {
		WGPUAdapter adapter       = nullptr;
		bool        request_ended = false;
	} user_data{};

	WGPURequestAdapterOptions options{};
	options.nextInChain       = nullptr;
	options.powerPreference   = power_preference;
	options.compatibleSurface = surface;
	options.backendType       = backend_type;
	// options.forceFallbackAdapter = false;

	auto callback = [](WGPURequestAdapterStatus status, WGPUAdapter adapter,
	                   char const* message, void* user_data) {
		UserData& data = *static_cast<UserData*>(user_data);
		if (WGPURequestAdapterStatus_Success == status) {
			data.adapter = adapter;
		} else {
			std::cerr << "Could not get WebGPU adapter: " << message << std::endl;
		}
		data.request_ended = true;
	};

	wgpuInstanceRequestAdapter(instance, &options, callback,
	                           static_cast<void*>(&user_data));

	// We wait until user_data.request_ended gets true
#ifdef __EMSCRIPTEN__
	while (!user_data.request_ended) {
		emscripten_sleep(100);
	}
#endif  // __EMSCRIPTEN__

	assert(user_data.request_ended);
	assert(nullptr != user_data.adapter);

	return user_data.adapter;
}

namespace detail
{
WGPUDevice createDevice(WGPUAdapter adapter, WGPURequiredLimits const* required_limits)
{
	assert(nullptr != adapter);

	struct UserData {
		WGPUDevice device        = nullptr;
		bool       request_ended = false;
	} user_data{};

	WGPUDeviceDescriptor desc{};
	desc.nextInChain = nullptr;
	// desc.label       = (window_name_ + " Device").c_str();
	// desc.requiredFeatureCount = 0;
	// desc.requiredFeatures     = nullptr;

	desc.requiredLimits = required_limits;

	desc.defaultQueue.nextInChain = nullptr;
	// desc.defaultQueue.label       = (window_name_ + " Default Queue").c_str();

	desc.deviceLostUserdata = nullptr;
	desc.deviceLostCallback = [](WGPUDeviceLostReason reason, char const* message,
	                             void* /* pUserData */) {
		std::cout << "Device lost: reason " << reason;
		if (message) {
			std::cout << " (" << message << ")";
		}
		std::cout << std::endl;
	};

	// desc.uncapturedErrorCallbackInfo.nextInChain = nullptr;
	// desc.uncapturedErrorCallbackInfo.userdata    = nullptr;
	// desc.uncapturedErrorCallbackInfo.callback    = nullptr;

	auto callback = [](WGPURequestDeviceStatus status, WGPUDevice device,
	                   char const* message, void* user_data) {
		UserData& data = *static_cast<UserData*>(user_data);
		if (WGPURequestDeviceStatus_Success == status) {
			data.device = device;
		} else {
			std::cout << "Could not get WebGPU device: " << message << std::endl;
		}
		data.request_ended = true;
	};

	wgpuAdapterRequestDevice(adapter, &desc, callback, static_cast<void*>(&user_data));

	// We wait until user_data.request_ended gets true
#ifdef __EMSCRIPTEN__
	while (!user_data.request_ended) {
		emscripten_sleep(100);
	}
#endif  // __EMSCRIPTEN__

	assert(user_data.request_ended);
	assert(nullptr != user_data.device);

	return user_data.device;
}
}  // namespace detail

WGPUDevice createDevice(WGPUAdapter adapter)
{
	return detail::createDevice(adapter, nullptr);
}

WGPUDevice createDevice(WGPUAdapter adapter, WGPURequiredLimits const& required_limits)
{
	return detail::createDevice(adapter, &required_limits);
}

WGPUQueue queue(WGPUDevice device)
{
	assert(nullptr != device);
	return wgpuDeviceGetQueue(device);
}

std::size_t bufferPaddedSize(std::size_t size)
{
	static constexpr std::size_t const COPY_BUFFER_ALIGNMENT = 4;

	std::size_t align_mask = COPY_BUFFER_ALIGNMENT - 1;
	return std::max((size + align_mask) & ~align_mask, COPY_BUFFER_ALIGNMENT);
}

WGPUBuffer createBuffer(WGPUDevice device, std::size_t size, WGPUBufferUsageFlags usage,
                        bool mapped_at_creation)
{
	std::size_t padded_size = bufferPaddedSize(size);

	WGPUBufferDescriptor desc{};
	desc.label            = "";
	desc.size             = padded_size;
	desc.usage            = usage;
	desc.mappedAtCreation = mapped_at_creation;
	return wgpuDeviceCreateBuffer(device, &desc);
}

WGPUBuffer createBufferInit(WGPUDevice device, WGPUBufferUsageFlags usage, void* content,
                            std::size_t content_size)
{
	if (0 == content_size) {
		return createBuffer(device, 0, usage, false);
	}

	WGPUBuffer buffer = createBuffer(device, content_size, usage, true);

	assert(nullptr != buffer);

	void* buf = wgpuBufferGetMappedRange(buffer, 0, content_size);
	std::memcpy(buf, content, content_size);
	wgpuBufferUnmap(buffer);

	return buffer;
}

WGPUShaderModule loadShaderModule(WGPUDevice device, std::filesystem::path const& path)
{
	std::ifstream file(path);
	if (!file.is_open()) {
		return nullptr;
	}

	file.seekg(0, std::ios::end);
	std::size_t size = file.tellg();
	std::string shader_source(size, ' ');
	file.seekg(0);
	file.read(shader_source.data(), size);

	WGPUShaderModuleWGSLDescriptor shader_code_desc{};
	shader_code_desc.chain.next  = nullptr;
	shader_code_desc.chain.sType = WGPUSType_ShaderModuleWGSLDescriptor;
	shader_code_desc.code        = shader_source.c_str();

	WGPUShaderModuleDescriptor shader_desc{};
	shader_desc.nextInChain = nullptr;
#ifdef WEBGPU_BACKEND_WGPU
	shader_desc.hintCount = 0;
	shader_desc.hints     = nullptr;
#endif
	shader_desc.nextInChain = &shader_code_desc.chain;
	return wgpuDeviceCreateShaderModule(device, &shader_desc);
}

void setDefault(WGPULimits& limits)
{
	limits.maxTextureDimension1D                     = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxTextureDimension2D                     = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxTextureDimension3D                     = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxTextureArrayLayers                     = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxBindGroups                             = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxBindGroupsPlusVertexBuffers            = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxBindingsPerBindGroup                   = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxDynamicUniformBuffersPerPipelineLayout = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxDynamicStorageBuffersPerPipelineLayout = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxSampledTexturesPerShaderStage          = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxSamplersPerShaderStage                 = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxStorageBuffersPerShaderStage           = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxStorageTexturesPerShaderStage          = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxUniformBuffersPerShaderStage           = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxUniformBufferBindingSize               = WGPU_LIMIT_U64_UNDEFINED;
	limits.maxStorageBufferBindingSize               = WGPU_LIMIT_U64_UNDEFINED;
	limits.minUniformBufferOffsetAlignment           = WGPU_LIMIT_U32_UNDEFINED;
	limits.minStorageBufferOffsetAlignment           = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxVertexBuffers                          = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxBufferSize                             = WGPU_LIMIT_U64_UNDEFINED;
	limits.maxVertexAttributes                       = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxVertexBufferArrayStride                = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxInterStageShaderComponents             = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxInterStageShaderVariables              = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxColorAttachments                       = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxColorAttachmentBytesPerSample          = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxComputeWorkgroupStorageSize            = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxComputeInvocationsPerWorkgroup         = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxComputeWorkgroupSizeX                  = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxComputeWorkgroupSizeY                  = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxComputeWorkgroupSizeZ                  = WGPU_LIMIT_U32_UNDEFINED;
	limits.maxComputeWorkgroupsPerDimension          = WGPU_LIMIT_U32_UNDEFINED;
}

void setDefault(WGPUBindGroupLayoutEntry& binding_layout)
{
	binding_layout.buffer.nextInChain      = nullptr;
	binding_layout.buffer.type             = WGPUBufferBindingType_Undefined;
	binding_layout.buffer.hasDynamicOffset = false;

	binding_layout.sampler.nextInChain = nullptr;
	binding_layout.sampler.type        = WGPUSamplerBindingType_Undefined;

	binding_layout.storageTexture.nextInChain   = nullptr;
	binding_layout.storageTexture.access        = WGPUStorageTextureAccess_Undefined;
	binding_layout.storageTexture.format        = WGPUTextureFormat_Undefined;
	binding_layout.storageTexture.viewDimension = WGPUTextureViewDimension_Undefined;

	binding_layout.texture.nextInChain   = nullptr;
	binding_layout.texture.multisampled  = false;
	binding_layout.texture.sampleType    = WGPUTextureSampleType_Undefined;
	binding_layout.texture.viewDimension = WGPUTextureViewDimension_Undefined;
}
}  // namespace ufo::compute