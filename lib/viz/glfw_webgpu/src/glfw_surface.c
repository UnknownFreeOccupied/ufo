// UFO
#include <ufo/glfw_webgpu/glfw_surface.h>

// STL
#include <assert.h>

// WebGPU
#include <webgpu/webgpu.h>

// GLFW
#if defined(GLFW_EXPOSE_NATIVE_COCOA)
#include <Foundation/Foundation.h>
#include <QuartzCore/CAMetalLayer.h>
#endif

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#ifdef __cplusplus
extern "C" {
#endif

WGPUSurface glfwSurface(WGPUInstance instance, GLFWwindow* window)
{
#if defined(GLFW_EXPOSE_NATIVE_COCOA)
	{
		id        metal_layer = NULL;
		NSWindow* ns_window   = glfwGetCocoaWindow(window);
		[ns_window.contentView setWantsLayer:YES];
		metal_layer = [CAMetalLayer layer];
		[ns_window.contentView setLayer:metal_layer];
		return wgpuInstanceCreateSurface(
		    instance, &(WGPUSurfaceDescriptor const){
		                  .nextInChain =
		                      (WGPUChainedStruct const*)&(WGPUSurfaceSourceMetalLayer const){
		                          .chain =
		                              (WGPUChainedStruct const){
		                                  .sType = WGPUSType_SurfaceSourceMetalLayer,
		                              },
		                          .layer = metal_layer,
		                      },
		              });
	}
#elif defined(GLFW_EXPOSE_NATIVE_WAYLAND) && defined(GLFW_EXPOSE_NATIVE_X11)
	if (GLFW_PLATFORM_X11 == glfwGetPlatform()) {
		Display* x11_display = glfwGetX11Display();
		Window   x11_window  = glfwGetX11Window(window);
		return wgpuInstanceCreateSurface(
		    instance, &(WGPUSurfaceDescriptor const){
		                  .nextInChain =
		                      (WGPUChainedStruct const*)&(WGPUSurfaceSourceXlibWindow const){
		                          .chain =
		                              (WGPUChainedStruct const){
		                                  .sType = WGPUSType_SurfaceSourceXlibWindow,
		                              },
		                          .display = x11_display,
		                          .window  = x11_window,
		                      },
		              });
	}
	if (GLFW_PLATFORM_WAYLAND == glfwGetPlatform()) {
		struct wl_display* wayland_display = glfwGetWaylandDisplay();
		struct wl_surface* wayland_surface = glfwGetWaylandWindow(window);
		return wgpuInstanceCreateSurface(
		    instance,
		    &(WGPUSurfaceDescriptor const){
		        .nextInChain =
		            (WGPUChainedStruct const*)&(WGPUSurfaceSourceWaylandSurface const){
		                .chain =
		                    (WGPUChainedStruct const){
		                        .sType = WGPUSType_SurfaceSourceWaylandSurface,
		                    },
		                .display = wayland_display,
		                .surface = wayland_surface,
		            },
		    });
	}
#elif defined(GLFW_EXPOSE_NATIVE_WIN32)
	{
		HWND      hwnd      = glfwGetWin32Window(window);
		HINSTANCE hinstance = GetModuleHandle(NULL);
		return wgpuInstanceCreateSurface(
		    instance, &(WGPUSurfaceDescriptor const){
		                  .nextInChain =
		                      (WGPUChainedStruct const*)&(WGPUSurfaceSourceWindowsHWND const){
		                          .chain =
		                              (WGPUChainedStruct const){
		                                  .sType = WGPUSType_SurfaceSourceWindowsHWND,
		                              },
		                          .hinstance = hinstance,
		                          .hwnd      = hwnd,
		                      },
		              });
	}
#else
#error "Unsupported GLFW native platform"
#endif
}

#ifdef __cplusplus
}
#endif