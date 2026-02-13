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

#ifndef UFO_COMPUTE_COMPUTE_HPP
#define UFO_COMPUTE_COMPUTE_HPP

#define UFO_WEBGPU

// STL
#include <filesystem>

// WebGPU
#include <webgpu/webgpu.h>
#include <webgpu/wgpu.h>

// TODO: Remove these when webgpu.h is updated to the latest spec.
// clang-format off
/**
 * 'True' value of @ref WGPUBool.
 *
 * @remark It's not usually necessary to use this, as `true` (from
 * `stdbool.h` or C++) casts to the same value.
 */
#define WGPU_TRUE (UINT32_C(1))
/**
 * 'False' value of @ref WGPUBool.
 *
 * @remark It's not usually necessary to use this, as `false` (from
 * `stdbool.h` or C++) casts to the same value.
 */
#define WGPU_FALSE (UINT32_C(0))

#define _wgpu_ENUM_ZERO_INIT(type) type{}
#define _wgpu_STRUCT_ZERO_INIT {}

/**
 * Initializer for @ref WGPUInstanceDescriptor.
 */
#define WGPU_INSTANCE_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUInstanceDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.requiredFeatureCount=*/0 _wgpu_COMMA \
    /*.requiredFeatures=*/NULL _wgpu_COMMA \
    /*.requiredLimits=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUSurfaceDescriptor.
 */
#define WGPU_SURFACE_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUSurfaceDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUSurfaceSourceMetalLayer.
 */
#define WGPU_SURFACE_SOURCE_METAL_LAYER_INIT _wgpu_MAKE_INIT_STRUCT(WGPUSurfaceSourceMetalLayer, { \
    /*.chain=*/_wgpu_MAKE_INIT_STRUCT(WGPUChainedStruct, { \
        /*.next=*/NULL _wgpu_COMMA \
        /*.sType=*/WGPUSType_SurfaceSourceMetalLayer _wgpu_COMMA \
    }) _wgpu_COMMA \
    /*.layer=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUSurfaceSourceXlibWindow.
 */
#define WGPU_SURFACE_SOURCE_XLIB_WINDOW_INIT _wgpu_MAKE_INIT_STRUCT(WGPUSurfaceSourceXlibWindow, { \
    /*.chain=*/_wgpu_MAKE_INIT_STRUCT(WGPUChainedStruct, { \
        /*.next=*/NULL _wgpu_COMMA \
        /*.sType=*/WGPUSType_SurfaceSourceXlibWindow _wgpu_COMMA \
    }) _wgpu_COMMA \
    /*.display=*/NULL _wgpu_COMMA \
    /*.window=*/0 _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUSurfaceSourceWaylandSurface.
 */
#define WGPU_SURFACE_SOURCE_WAYLAND_SURFACE_INIT _wgpu_MAKE_INIT_STRUCT(WGPUSurfaceSourceWaylandSurface, { \
    /*.chain=*/_wgpu_MAKE_INIT_STRUCT(WGPUChainedStruct, { \
        /*.next=*/NULL _wgpu_COMMA \
        /*.sType=*/WGPUSType_SurfaceSourceWaylandSurface _wgpu_COMMA \
    }) _wgpu_COMMA \
    /*.display=*/NULL _wgpu_COMMA \
    /*.surface=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUSurfaceSourceWindowsHWND.
 */
#define WGPU_SURFACE_SOURCE_WINDOWS_HWND_INIT _wgpu_MAKE_INIT_STRUCT(WGPUSurfaceSourceWindowsHWND, { \
    /*.chain=*/_wgpu_MAKE_INIT_STRUCT(WGPUChainedStruct, { \
        /*.next=*/NULL _wgpu_COMMA \
        /*.sType=*/WGPUSType_SurfaceSourceWindowsHWND _wgpu_COMMA \
    }) _wgpu_COMMA \
    /*.hinstance=*/NULL _wgpu_COMMA \
    /*.hwnd=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPURequestAdapterOptions.
 */
#define WGPU_REQUEST_ADAPTER_OPTIONS_INIT _wgpu_MAKE_INIT_STRUCT(WGPURequestAdapterOptions, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.featureLevel=*/WGPUFeatureLevel_Core _wgpu_COMMA \
    /*.powerPreference=*/WGPUPowerPreference_Undefined _wgpu_COMMA \
    /*.forceFallbackAdapter=*/WGPU_FALSE _wgpu_COMMA \
    /*.backendType=*/WGPUBackendType_Undefined _wgpu_COMMA \
    /*.compatibleSurface=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPURequestAdapterCallbackInfo.
 */
#define WGPU_REQUEST_ADAPTER_CALLBACK_INFO_INIT _wgpu_MAKE_INIT_STRUCT(WGPURequestAdapterCallbackInfo, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.mode=*/_wgpu_ENUM_ZERO_INIT(WGPUCallbackMode) _wgpu_COMMA \
    /*.callback=*/NULL _wgpu_COMMA \
    /*.userdata1=*/NULL _wgpu_COMMA \
    /*.userdata2=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUQueueDescriptor.
 */
#define WGPU_QUEUE_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUQueueDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUDeviceLostCallbackInfo.
 */
#define WGPU_DEVICE_LOST_CALLBACK_INFO_INIT _wgpu_MAKE_INIT_STRUCT(WGPUDeviceLostCallbackInfo, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.mode=*/_wgpu_ENUM_ZERO_INIT(WGPUCallbackMode) _wgpu_COMMA \
    /*.callback=*/NULL _wgpu_COMMA \
    /*.userdata1=*/NULL _wgpu_COMMA \
    /*.userdata2=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUUncapturedErrorCallbackInfo.
 */
#define WGPU_UNCAPTURED_ERROR_CALLBACK_INFO_INIT _wgpu_MAKE_INIT_STRUCT(WGPUUncapturedErrorCallbackInfo, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.callback=*/NULL _wgpu_COMMA \
    /*.userdata1=*/NULL _wgpu_COMMA \
    /*.userdata2=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPURequestDeviceCallbackInfo.
 */
#define WGPU_REQUEST_DEVICE_CALLBACK_INFO_INIT _wgpu_MAKE_INIT_STRUCT(WGPURequestDeviceCallbackInfo, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.mode=*/_wgpu_ENUM_ZERO_INIT(WGPUCallbackMode) _wgpu_COMMA \
    /*.callback=*/NULL _wgpu_COMMA \
    /*.userdata1=*/NULL _wgpu_COMMA \
    /*.userdata2=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUDeviceDescriptor.
 */
#define WGPU_DEVICE_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUDeviceDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.requiredFeatureCount=*/0 _wgpu_COMMA \
    /*.requiredFeatures=*/NULL _wgpu_COMMA \
    /*.requiredLimits=*/NULL _wgpu_COMMA \
    /*.defaultQueue=*/WGPU_QUEUE_DESCRIPTOR_INIT _wgpu_COMMA \
    /*.deviceLostCallbackInfo=*/WGPU_DEVICE_LOST_CALLBACK_INFO_INIT _wgpu_COMMA \
    /*.uncapturedErrorCallbackInfo=*/WGPU_UNCAPTURED_ERROR_CALLBACK_INFO_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPULimits.
 */
#define WGPU_LIMITS_INIT _wgpu_MAKE_INIT_STRUCT(WGPULimits, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.maxTextureDimension1D=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxTextureDimension2D=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxTextureDimension3D=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxTextureArrayLayers=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxBindGroups=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxBindGroupsPlusVertexBuffers=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxBindingsPerBindGroup=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxDynamicUniformBuffersPerPipelineLayout=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxDynamicStorageBuffersPerPipelineLayout=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxSampledTexturesPerShaderStage=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxSamplersPerShaderStage=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxStorageBuffersPerShaderStage=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxStorageTexturesPerShaderStage=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxUniformBuffersPerShaderStage=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxUniformBufferBindingSize=*/WGPU_LIMIT_U64_UNDEFINED _wgpu_COMMA \
    /*.maxStorageBufferBindingSize=*/WGPU_LIMIT_U64_UNDEFINED _wgpu_COMMA \
    /*.minUniformBufferOffsetAlignment=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.minStorageBufferOffsetAlignment=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxVertexBuffers=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxBufferSize=*/WGPU_LIMIT_U64_UNDEFINED _wgpu_COMMA \
    /*.maxVertexAttributes=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxVertexBufferArrayStride=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxInterStageShaderVariables=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxColorAttachments=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxColorAttachmentBytesPerSample=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxComputeWorkgroupStorageSize=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxComputeInvocationsPerWorkgroup=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxComputeWorkgroupSizeX=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxComputeWorkgroupSizeY=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxComputeWorkgroupSizeZ=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
    /*.maxComputeWorkgroupsPerDimension=*/WGPU_LIMIT_U32_UNDEFINED _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUBufferDescriptor.
 */
#define WGPU_BUFFER_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUBufferDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.usage=*/WGPUBufferUsage_None _wgpu_COMMA \
    /*.size=*/0 _wgpu_COMMA \
    /*.mappedAtCreation=*/WGPU_FALSE _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUShaderSourceWGSL.
 */
#define WGPU_SHADER_SOURCE_WGSL_INIT _wgpu_MAKE_INIT_STRUCT(WGPUShaderSourceWGSL, { \
    /*.chain=*/_wgpu_MAKE_INIT_STRUCT(WGPUChainedStruct, { \
        /*.next=*/NULL _wgpu_COMMA \
        /*.sType=*/WGPUSType_ShaderSourceWGSL _wgpu_COMMA \
    }) _wgpu_COMMA \
    /*.code=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUPipelineLayoutDescriptor.
 */
#define WGPU_PIPELINE_LAYOUT_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUPipelineLayoutDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.bindGroupLayoutCount=*/0 _wgpu_COMMA \
    /*.bindGroupLayouts=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUConstantEntry.
 */
#define WGPU_CONSTANT_ENTRY_INIT _wgpu_MAKE_INIT_STRUCT(WGPUConstantEntry, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.key=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.value=*/0. _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUShaderModuleDescriptor.
 */
#define WGPU_SHADER_MODULE_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUShaderModuleDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUBindGroupEntry.
 */
#define WGPU_BIND_GROUP_ENTRY_INIT _wgpu_MAKE_INIT_STRUCT(WGPUBindGroupEntry, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.binding=*/0 _wgpu_COMMA \
    /*.buffer=*/NULL _wgpu_COMMA \
    /*.offset=*/0 _wgpu_COMMA \
    /*.size=*/WGPU_WHOLE_SIZE _wgpu_COMMA \
    /*.sampler=*/NULL _wgpu_COMMA \
    /*.textureView=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUBindGroupLayoutEntry.
 */
#define WGPU_BIND_GROUP_LAYOUT_ENTRY_INIT _wgpu_MAKE_INIT_STRUCT(WGPUBindGroupLayoutEntry, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.binding=*/0 _wgpu_COMMA \
    /*.visibility=*/WGPUShaderStage_None _wgpu_COMMA \
    /*.bindingArraySize=*/0 _wgpu_COMMA \
    /*.buffer=*/_wgpu_STRUCT_ZERO_INIT _wgpu_COMMA \
    /*.sampler=*/_wgpu_STRUCT_ZERO_INIT _wgpu_COMMA \
    /*.texture=*/_wgpu_STRUCT_ZERO_INIT _wgpu_COMMA \
    /*.storageTexture=*/_wgpu_STRUCT_ZERO_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUProgrammableStageDescriptor.
 */
#define WGPU_PROGRAMMABLE_STAGE_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUProgrammableStageDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.module=*/NULL _wgpu_COMMA \
    /*.entryPoint=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.constantCount=*/0 _wgpu_COMMA \
    /*.constants=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUComputePipelineDescriptor.
 */
#define WGPU_COMPUTE_PIPELINE_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUComputePipelineDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.layout=*/NULL _wgpu_COMMA \
    /*.compute=*/WGPU_PROGRAMMABLE_STAGE_DESCRIPTOR_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUCommandEncoderDescriptor.
 */
#define WGPU_COMMAND_ENCODER_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUCommandEncoderDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUComputePassDescriptor.
 */
#define WGPU_COMPUTE_PASS_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUComputePassDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.timestampWrites=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUCommandBufferDescriptor.
 */
#define WGPU_COMMAND_BUFFER_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUCommandBufferDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUTextureDescriptor.
 */
#define WGPU_TEXTURE_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUTextureDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.usage=*/WGPUTextureUsage_None _wgpu_COMMA \
    /*.dimension=*/WGPUTextureDimension_Undefined _wgpu_COMMA \
    /*.size=*/WGPU_EXTENT_3D_INIT _wgpu_COMMA \
    /*.format=*/WGPUTextureFormat_Undefined _wgpu_COMMA \
    /*.mipLevelCount=*/1 _wgpu_COMMA \
    /*.sampleCount=*/1 _wgpu_COMMA \
    /*.viewFormatCount=*/0 _wgpu_COMMA \
    /*.viewFormats=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUTextureViewDescriptor.
 */
#define WGPU_TEXTURE_VIEW_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUTextureViewDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.format=*/WGPUTextureFormat_Undefined _wgpu_COMMA \
    /*.dimension=*/WGPUTextureViewDimension_Undefined _wgpu_COMMA \
    /*.baseMipLevel=*/0 _wgpu_COMMA \
    /*.mipLevelCount=*/WGPU_MIP_LEVEL_COUNT_UNDEFINED _wgpu_COMMA \
    /*.baseArrayLayer=*/0 _wgpu_COMMA \
    /*.arrayLayerCount=*/WGPU_ARRAY_LAYER_COUNT_UNDEFINED _wgpu_COMMA \
    /*.aspect=*/WGPUTextureAspect_Undefined _wgpu_COMMA \
    /*.usage=*/WGPUTextureUsage_None _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUBindGroupLayoutDescriptor.
 */
#define WGPU_BIND_GROUP_LAYOUT_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUBindGroupLayoutDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.entryCount=*/0 _wgpu_COMMA \
    /*.entries=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUBindGroupDescriptor.
 */
#define WGPU_BIND_GROUP_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUBindGroupDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.layout=*/NULL _wgpu_COMMA \
    /*.entryCount=*/0 _wgpu_COMMA \
    /*.entries=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUOrigin3D.
 */
#define WGPU_ORIGIN_3D_INIT _wgpu_MAKE_INIT_STRUCT(WGPUOrigin3D, { \
    /*.x=*/0 _wgpu_COMMA \
    /*.y=*/0 _wgpu_COMMA \
    /*.z=*/0 _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUTexelCopyTextureInfo.
 */
#define WGPU_TEXEL_COPY_TEXTURE_INFO_INIT _wgpu_MAKE_INIT_STRUCT(WGPUTexelCopyTextureInfo, { \
    /*.texture=*/NULL _wgpu_COMMA \
    /*.mipLevel=*/0 _wgpu_COMMA \
    /*.origin=*/WGPU_ORIGIN_3D_INIT _wgpu_COMMA \
    /*.aspect=*/WGPUTextureAspect_Undefined _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUExtent3D.
 */
#define WGPU_EXTENT_3D_INIT _wgpu_MAKE_INIT_STRUCT(WGPUExtent3D, { \
    /*.width=*/0 _wgpu_COMMA \
    /*.height=*/1 _wgpu_COMMA \
    /*.depthOrArrayLayers=*/1 _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUTexelCopyBufferLayout.
 */
#define WGPU_TEXEL_COPY_BUFFER_LAYOUT_INIT _wgpu_MAKE_INIT_STRUCT(WGPUTexelCopyBufferLayout, { \
    /*.offset=*/0 _wgpu_COMMA \
    /*.bytesPerRow=*/WGPU_COPY_STRIDE_UNDEFINED _wgpu_COMMA \
    /*.rowsPerImage=*/WGPU_COPY_STRIDE_UNDEFINED _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUTexelCopyBufferInfo.
 */
#define WGPU_TEXEL_COPY_BUFFER_INFO_INIT _wgpu_MAKE_INIT_STRUCT(WGPUTexelCopyBufferInfo, { \
    /*.layout=*/WGPU_TEXEL_COPY_BUFFER_LAYOUT_INIT _wgpu_COMMA \
    /*.buffer=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUBufferMapCallbackInfo.
 */
#define WGPU_BUFFER_MAP_CALLBACK_INFO_INIT _wgpu_MAKE_INIT_STRUCT(WGPUBufferMapCallbackInfo, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.mode=*/_wgpu_ENUM_ZERO_INIT(WGPUCallbackMode) _wgpu_COMMA \
    /*.callback=*/NULL _wgpu_COMMA \
    /*.userdata1=*/NULL _wgpu_COMMA \
    /*.userdata2=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUSurfaceCapabilities.
 */
#define WGPU_SURFACE_CAPABILITIES_INIT _wgpu_MAKE_INIT_STRUCT(WGPUSurfaceCapabilities, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.usages=*/WGPUTextureUsage_None _wgpu_COMMA \
    /*.formatCount=*/0 _wgpu_COMMA \
    /*.formats=*/NULL _wgpu_COMMA \
    /*.presentModeCount=*/0 _wgpu_COMMA \
    /*.presentModes=*/NULL _wgpu_COMMA \
    /*.alphaModeCount=*/0 _wgpu_COMMA \
    /*.alphaModes=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUSurfaceConfiguration.
 */
#define WGPU_SURFACE_CONFIGURATION_INIT _wgpu_MAKE_INIT_STRUCT(WGPUSurfaceConfiguration, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.device=*/NULL _wgpu_COMMA \
    /*.format=*/WGPUTextureFormat_Undefined _wgpu_COMMA \
    /*.usage=*/WGPUTextureUsage_RenderAttachment _wgpu_COMMA \
    /*.width=*/0 _wgpu_COMMA \
    /*.height=*/0 _wgpu_COMMA \
    /*.viewFormatCount=*/0 _wgpu_COMMA \
    /*.viewFormats=*/NULL _wgpu_COMMA \
    /*.alphaMode=*/WGPUCompositeAlphaMode_Auto _wgpu_COMMA \
    /*.presentMode=*/WGPUPresentMode_Undefined _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUSurfaceTexture.
 */
#define WGPU_SURFACE_TEXTURE_INIT _wgpu_MAKE_INIT_STRUCT(WGPUSurfaceTexture, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.texture=*/NULL _wgpu_COMMA \
    /*.status=*/_wgpu_ENUM_ZERO_INIT(WGPUSurfaceGetCurrentTextureStatus) _wgpu_COMMA \
})

/**
 * Indicates no depth clear value is specified. For more info,
 * see @ref SentinelValues and the places that use this sentinel value.
 */
#define WGPU_DEPTH_CLEAR_VALUE_UNDEFINED (NAN)

/**
 * Initializer for @ref WGPURenderPassDepthStencilAttachment.
 */
#define WGPU_RENDER_PASS_DEPTH_STENCIL_ATTACHMENT_INIT _wgpu_MAKE_INIT_STRUCT(WGPURenderPassDepthStencilAttachment, { \
    /*.view=*/NULL _wgpu_COMMA \
    /*.depthLoadOp=*/WGPULoadOp_Undefined _wgpu_COMMA \
    /*.depthStoreOp=*/WGPUStoreOp_Undefined _wgpu_COMMA \
    /*.depthClearValue=*/WGPU_DEPTH_CLEAR_VALUE_UNDEFINED _wgpu_COMMA \
    /*.depthReadOnly=*/WGPU_FALSE _wgpu_COMMA \
    /*.stencilLoadOp=*/WGPULoadOp_Undefined _wgpu_COMMA \
    /*.stencilStoreOp=*/WGPUStoreOp_Undefined _wgpu_COMMA \
    /*.stencilClearValue=*/0 _wgpu_COMMA \
    /*.stencilReadOnly=*/WGPU_FALSE _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPURenderPassDescriptor.
 */
#define WGPU_RENDER_PASS_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPURenderPassDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.colorAttachmentCount=*/0 _wgpu_COMMA \
    /*.colorAttachments=*/NULL _wgpu_COMMA \
    /*.depthStencilAttachment=*/NULL _wgpu_COMMA \
    /*.occlusionQuerySet=*/NULL _wgpu_COMMA \
    /*.timestampWrites=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUColor.
 */
#define WGPU_COLOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUColor, { \
    /*.r=*/0. _wgpu_COMMA \
    /*.g=*/0. _wgpu_COMMA \
    /*.b=*/0. _wgpu_COMMA \
    /*.a=*/0. _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPURenderPassColorAttachment.
 */
#define WGPU_RENDER_PASS_COLOR_ATTACHMENT_INIT _wgpu_MAKE_INIT_STRUCT(WGPURenderPassColorAttachment, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.view=*/NULL _wgpu_COMMA \
    /*.depthSlice=*/WGPU_DEPTH_SLICE_UNDEFINED _wgpu_COMMA \
    /*.resolveTarget=*/NULL _wgpu_COMMA \
    /*.loadOp=*/WGPULoadOp_Undefined _wgpu_COMMA \
    /*.storeOp=*/WGPUStoreOp_Undefined _wgpu_COMMA \
    /*.clearValue=*/WGPU_COLOR_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUVertexState.
 */
#define WGPU_VERTEX_STATE_INIT _wgpu_MAKE_INIT_STRUCT(WGPUVertexState, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.module=*/NULL _wgpu_COMMA \
    /*.entryPoint=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.constantCount=*/0 _wgpu_COMMA \
    /*.constants=*/NULL _wgpu_COMMA \
    /*.bufferCount=*/0 _wgpu_COMMA \
    /*.buffers=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUPrimitiveState.
 */
#define WGPU_PRIMITIVE_STATE_INIT _wgpu_MAKE_INIT_STRUCT(WGPUPrimitiveState, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.topology=*/WGPUPrimitiveTopology_Undefined _wgpu_COMMA \
    /*.stripIndexFormat=*/WGPUIndexFormat_Undefined _wgpu_COMMA \
    /*.frontFace=*/WGPUFrontFace_Undefined _wgpu_COMMA \
    /*.cullMode=*/WGPUCullMode_Undefined _wgpu_COMMA \
    /*.unclippedDepth=*/WGPU_FALSE _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUMultisampleState.
 */
#define WGPU_MULTISAMPLE_STATE_INIT _wgpu_MAKE_INIT_STRUCT(WGPUMultisampleState, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.count=*/1 _wgpu_COMMA \
    /*.mask=*/0xFFFFFFFF _wgpu_COMMA \
    /*.alphaToCoverageEnabled=*/WGPU_FALSE _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPURenderPipelineDescriptor.
 */
#define WGPU_RENDER_PIPELINE_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPURenderPipelineDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.layout=*/NULL _wgpu_COMMA \
    /*.vertex=*/WGPU_VERTEX_STATE_INIT _wgpu_COMMA \
    /*.primitive=*/WGPU_PRIMITIVE_STATE_INIT _wgpu_COMMA \
    /*.depthStencil=*/NULL _wgpu_COMMA \
    /*.multisample=*/WGPU_MULTISAMPLE_STATE_INIT _wgpu_COMMA \
    /*.fragment=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUVertexBufferLayout.
 */
#define WGPU_VERTEX_BUFFER_LAYOUT_INIT _wgpu_MAKE_INIT_STRUCT(WGPUVertexBufferLayout, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.stepMode=*/WGPUVertexStepMode_Undefined _wgpu_COMMA \
    /*.arrayStride=*/0 _wgpu_COMMA \
    /*.attributeCount=*/0 _wgpu_COMMA \
    /*.attributes=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUFragmentState.
 */
#define WGPU_FRAGMENT_STATE_INIT _wgpu_MAKE_INIT_STRUCT(WGPUFragmentState, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.module=*/NULL _wgpu_COMMA \
    /*.entryPoint=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.constantCount=*/0 _wgpu_COMMA \
    /*.constants=*/NULL _wgpu_COMMA \
    /*.targetCount=*/0 _wgpu_COMMA \
    /*.targets=*/NULL _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUColorTargetState.
 */
#define WGPU_COLOR_TARGET_STATE_INIT _wgpu_MAKE_INIT_STRUCT(WGPUColorTargetState, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.format=*/WGPUTextureFormat_Undefined _wgpu_COMMA \
    /*.blend=*/NULL _wgpu_COMMA \
    /*.writeMask=*/WGPUColorWriteMask_All _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUBlendComponent.
 */
#define WGPU_BLEND_COMPONENT_INIT _wgpu_MAKE_INIT_STRUCT(WGPUBlendComponent, { \
    /*.operation=*/WGPUBlendOperation_Undefined _wgpu_COMMA \
    /*.srcFactor=*/WGPUBlendFactor_Undefined _wgpu_COMMA \
    /*.dstFactor=*/WGPUBlendFactor_Undefined _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUBlendState.
 */
#define WGPU_BLEND_STATE_INIT _wgpu_MAKE_INIT_STRUCT(WGPUBlendState, { \
    /*.color=*/WGPU_BLEND_COMPONENT_INIT _wgpu_COMMA \
    /*.alpha=*/WGPU_BLEND_COMPONENT_INIT _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUStencilFaceState.
 */
#define WGPU_STENCIL_FACE_STATE_INIT _wgpu_MAKE_INIT_STRUCT(WGPUStencilFaceState, { \
    /*.compare=*/WGPUCompareFunction_Undefined _wgpu_COMMA \
    /*.failOp=*/WGPUStencilOperation_Undefined _wgpu_COMMA \
    /*.depthFailOp=*/WGPUStencilOperation_Undefined _wgpu_COMMA \
    /*.passOp=*/WGPUStencilOperation_Undefined _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUDepthStencilState.
 */
#define WGPU_DEPTH_STENCIL_STATE_INIT _wgpu_MAKE_INIT_STRUCT(WGPUDepthStencilState, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.format=*/WGPUTextureFormat_Undefined _wgpu_COMMA \
    /*.depthWriteEnabled=*/WGPUOptionalBool_Undefined _wgpu_COMMA \
    /*.depthCompare=*/WGPUCompareFunction_Undefined _wgpu_COMMA \
    /*.stencilFront=*/WGPU_STENCIL_FACE_STATE_INIT _wgpu_COMMA \
    /*.stencilBack=*/WGPU_STENCIL_FACE_STATE_INIT _wgpu_COMMA \
    /*.stencilReadMask=*/0xFFFFFFFF _wgpu_COMMA \
    /*.stencilWriteMask=*/0xFFFFFFFF _wgpu_COMMA \
    /*.depthBias=*/0 _wgpu_COMMA \
    /*.depthBiasSlopeScale=*/0.f _wgpu_COMMA \
    /*.depthBiasClamp=*/0.f _wgpu_COMMA \
})

/**
 * Initializer for @ref WGPUSamplerDescriptor.
 */
#define WGPU_SAMPLER_DESCRIPTOR_INIT _wgpu_MAKE_INIT_STRUCT(WGPUSamplerDescriptor, { \
    /*.nextInChain=*/NULL _wgpu_COMMA \
    /*.label=*/WGPU_STRING_VIEW_INIT _wgpu_COMMA \
    /*.addressModeU=*/WGPUAddressMode_Undefined _wgpu_COMMA \
    /*.addressModeV=*/WGPUAddressMode_Undefined _wgpu_COMMA \
    /*.addressModeW=*/WGPUAddressMode_Undefined _wgpu_COMMA \
    /*.magFilter=*/WGPUFilterMode_Undefined _wgpu_COMMA \
    /*.minFilter=*/WGPUFilterMode_Undefined _wgpu_COMMA \
    /*.mipmapFilter=*/WGPUMipmapFilterMode_Undefined _wgpu_COMMA \
    /*.lodMinClamp=*/0.f _wgpu_COMMA \
    /*.lodMaxClamp=*/32.f _wgpu_COMMA \
    /*.compare=*/WGPUCompareFunction_Undefined _wgpu_COMMA \
    /*.maxAnisotropy=*/1 _wgpu_COMMA \
})
// clang-format on

namespace ufo::compute
{
[[nodiscard]] WGPUInstance createInstance(
    WGPUInstanceDescriptor const* descriptor = nullptr);

[[nodiscard]] WGPUAdapter createAdapter(
    WGPUInstance instance, WGPUSurface compatible_surface = nullptr,
    WGPUPowerPreference power_preference       = WGPUPowerPreference_Undefined,
    WGPUBackendType     backend_type           = WGPUBackendType_Undefined,
    WGPUBool            force_fallback_adapter = false);

[[nodiscard]] WGPUDevice createDevice(WGPUAdapter       adapter,
                                      WGPULimits const* required_limits = nullptr);

[[nodiscard]] WGPUQueue queue(WGPUDevice device);

[[nodiscard]] std::size_t bufferPaddedSize(std::size_t size);

[[nodiscard]] std::size_t bufferPaddedSize(std::size_t width,
                                           std::size_t bytes_per_pixel);

[[nodiscard]] WGPUBuffer createBuffer(WGPUDevice device, std::string label,
                                      std::size_t size, WGPUBufferUsage usage,
                                      bool mapped_at_creation = false);

[[nodiscard]] WGPUBuffer createBufferInit(WGPUDevice device, WGPUBufferUsage usage,
                                          void* content, std::size_t content_size);

[[nodiscard]] WGPUSurfaceCapabilities surfaceCapabilities(WGPUSurface surface,
                                                          WGPUAdapter adapter);

[[nodiscard]] WGPUShaderModule loadShaderModule(WGPUDevice                   device,
                                                std::filesystem::path const& path);

// Release

void release(WGPUInstance instance);

void release(WGPUSurface surface);

void release(WGPUAdapter adapter);

void release(WGPUDevice device);

void release(WGPUQueue queue);

void release(WGPUShaderModule shader_module);

void release(WGPUBindGroupLayout bind_group_layout);

void release(WGPUPipelineLayout pipeline_layout);

void release(WGPURenderPipeline render_pipeline);

void release(WGPUTexture texture);

void release(WGPUTextureView texture_view);

void release(WGPURenderPassEncoder render_pass_encoder);

void release(WGPUCommandEncoder command_encoder);

void release(WGPUCommandBuffer command_buffer);

void release(WGPUSampler sampler);
}  // namespace ufo::compute

#endif  // UFO_COMPUTE_COMPUTE_HPP