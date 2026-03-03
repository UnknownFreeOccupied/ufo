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

#ifndef UFO_CONTAINER_TREE_DATA_HPP
#define UFO_CONTAINER_TREE_DATA_HPP

// UFO
#include <ufo/compute/compute.hpp>
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/math/math.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <cstring>

namespace ufo
{
template <class Derived, std::size_t Dim, class... Ts>
class TreeData
{
	friend Derived;

 public:
	static constexpr TreeIndex::offset_type const BF = ipow(std::size_t(2), Dim);

	using LeafData  = TreeContainer<typename Ts::template LeafBlock<Dim, BF>...>;
	using InnerData = TreeContainer<typename Ts::template InnerBlock<Dim, BF>...>;

	using Index    = TreeIndex;
	using pos_type = Index::pos_type;

	static constexpr std::size_t const NumBuffers = sizeof...(Ts);

 public:
	~TreeData() { gpuRelease(); }

	[[nodiscard]] LeafData& leafData() { return leaf_data_; }

	[[nodiscard]] LeafData const& leafData() const { return leaf_data_; }

	[[nodiscard]] InnerData& innerData() { return inner_data_; }

	[[nodiscard]] InnerData const& innerData() const { return inner_data_; }

	/**
	 * @brief Checks if a block exists.
	 *
	 * @param block the block to check
	 * @return `true` if the block exists, `false` otherwise.
	 */
	[[nodiscard]] bool exists(pos_type block) const
	{
		return leaf(block) ? leafExists(block) : innerExists(block);
	}

	[[nodiscard]] bool leafExists(pos_type block) const
	{
		assert(leaf(block));
		return leaf_data_.capacity() > removeLeafType(block);
	}

	[[nodiscard]] bool innerExists(pos_type block) const
	{
		assert(inner(block));
		return inner_data_.capacity() > removeInnerType(block);
	}

	bool gpuInit(WGPUPowerPreference power_preference = WGPUPowerPreference_HighPerformance,
	             WGPUBackendType     backend_type     = WGPUBackendType_Undefined)
	{
		if (nullptr != device_) {
			return false;
		}

		instance_ = compute::createInstance();
		adapter_ = compute::createAdapter(instance_, nullptr, power_preference, backend_type);
		auto required_limits = requiredLimits(adapter_);
		device_              = compute::createDevice(adapter_, &required_limits);
		queue_               = compute::queue(device_);

		return true;
	}

	bool gpuInit(WGPULimits const&   required_limits,
	             WGPUSurface         compatible_surface = nullptr,
	             WGPUPowerPreference power_preference = WGPUPowerPreference_HighPerformance,
	             WGPUBackendType     backend_type     = WGPUBackendType_Undefined)
	{
		if (nullptr != device_) {
			return false;
		}

		instance_ = compute::createInstance();
		adapter_  = compute::createAdapter(instance_, compatible_surface, power_preference,
		                                   backend_type);
		device_   = compute::createDevice(adapter_, &required_limits);
		queue_    = compute::queue(device_);

		return true;
	}

	bool gpuInit(WGPUAdapter adapter)
	{
		assert(nullptr != adapter);

		return gpuInit(adapter, requiredLimits(adapter));
	}

	bool gpuInit(WGPUAdapter adapter, WGPULimits const& required_limits)
	{
		if (nullptr != device_) {
			return false;
		}

		assert(nullptr != adapter);

		// Increase reference count
		wgpuAdapterAddRef(adapter);

		adapter_ = adapter;
		device_  = compute::createDevice(adapter, &required_limits);
		queue_   = compute::queue(device_);

		return true;
	}

	bool gpuInit(WGPUDevice device)
	{
		if (nullptr != device_) {
			return false;
		}

		assert(nullptr != device);

		// Increase reference count
		wgpuDeviceAddRef(device);

		device_ = device;
		queue_  = compute::queue(device);

		return true;
	}

	void gpuRelease()
	{
		for (auto& buffers : leaf_buffers_) {
			for (WGPUBuffer& buf : buffers) {
				if (nullptr != buf) {
					wgpuBufferRelease(buf);
				}
			}
			buffers.clear();
		}

		for (auto& buffers : inner_buffers_) {
			for (WGPUBuffer& buf : buffers) {
				if (nullptr != buf) {
					wgpuBufferRelease(buf);
				}
			}
			buffers.clear();
		}

		if (nullptr != queue_) {
			wgpuQueueRelease(queue_);
			queue_ = nullptr;
		}

		if (nullptr != device_) {
			wgpuDeviceRelease(device_);
			device_ = nullptr;
		}

		if (nullptr != adapter_) {
			wgpuAdapterRelease(adapter_);
			adapter_ = nullptr;
		}

		if (nullptr != instance_) {
			wgpuInstanceRelease(instance_);
			instance_ = nullptr;
		}
	}

	[[nodiscard]] WGPUDevice gpuDevice() const { return device_; }

	[[nodiscard]] WGPUQueue gpuQueue() const { return queue_; }

	template <class T>
	[[nodiscard]] std::size_t gpuNumBuffers() const
	{
		return gpuNumLeafBuffers<T>() + gpuNumInnerBuffers<T>();
	}

	template <class T>
	[[nodiscard]] std::size_t gpuNumLeafBuffers() const
	{
		return leaf_buffers_[index_v<T, Ts...>].size();
	}

	template <class T>
	[[nodiscard]] std::size_t gpuNumInnerBuffers() const
	{
		return inner_buffers_[index_v<T, Ts...>].size();
	}

	template <class T>
	[[nodiscard]] WGPUBuffer gpuLeafBuffer(std::size_t index) const
	{
		return leaf_buffers_[index_v<T, Ts...>][index];
	}

	template <class T>
	[[nodiscard]] WGPUBuffer gpuInnerBuffer(std::size_t index) const
	{
		return inner_buffers_[index_v<T, Ts...>][index];
	}

	template <class T>
	[[nodiscard]] std::size_t gpuLeafBufferSize(std::size_t index) const
	{
		return wgpuBufferGetSize(gpuLeafBuffer<T>(index));
	}

	template <class T>
	[[nodiscard]] std::size_t gpuInnerBufferSize(std::size_t index) const
	{
		return wgpuBufferGetSize(gpuInnerBuffer<T>(index));
	}

	void gpuRead()
	{
		gpuReadLeaf();
		gpuReadInner();
	}

	void gpuReadLeaf() { (gpuReadLeaf<Ts>(), ...); }

	void gpuReadInner() { (gpuReadInner<Ts>(), ...); }

	template <class T>
	void gpuRead()
	{
		gpuReadLeaf<T>();
		gpuReadInner<T>();
	}

	template <class T>
	void gpuReadLeaf()
	{
		// TODO: Implement
	}

	template <class T>
	void gpuReadInner()
	{
		// TODO: Implement
	}

	bool gpuWrite()
	{
		bool a = gpuWriteLeaf();
		bool b = gpuWriteInner();
		return a || b;
	}

	bool gpuWriteLeaf() { return (gpuWriteLeaf<Ts>() | ...); }

	bool gpuWriteInner() { return (gpuWriteInner<Ts>() | ...); }

	template <class T>
	bool gpuWrite()
	{
		bool a = gpuWriteLeaf<T>();
		bool b = gpuWriteInner<T>();
		return a || b;
	}

	template <class T>
	bool gpuWriteLeaf()
	{
		// TODO: Make gpuWriteLeaf and gpuWriteInner with a single implementation.

		assert(nullptr != device_);
		assert(nullptr != queue_);

		using Block = typename T::template LeafBlock<Dim, BF>;

		std::size_t const size = leaf_data_.template serializedSize<Block>();

		auto& buffers = leaf_buffers_[index_v<T, Ts...>];

		if (0 == size) {
			for (auto& buf : buffers) {
				wgpuBufferRelease(buf);
			}
			bool empty = buffers.empty();
			buffers.clear();
			return !empty;
		}

		constexpr std::size_t const bucket_size =
		    LeafData::template serializedBucketSize<Block>();

		std::size_t const buckets_per_buffer = max_buffer_size_ / bucket_size;
		std::size_t const buffer_size        = bucket_size * buckets_per_buffer;

		std::size_t const num_buffers = 1 + (size - 1) / buffer_size;

		buffers.reserve(num_buffers);

		bool updated = false;

		auto it   = leaf_data_.template beginBucket<Block>();
		auto last = leaf_data_.template endBucket<Block>();

		for (std::size_t i{}; num_buffers > i; ++i) {
			if (buffers.size() <= i) {
				updated = true;

				auto& buffer = buffers.emplace_back(compute::createBuffer(
				    device_, "", buffer_size, WGPUBufferUsage_Storage | WGPUBufferUsage_CopyDst,
				    true));

				assert(nullptr != buffer);

				void* buf = wgpuBufferGetMappedRange(buffer, 0, buffer_size);

				for (std::size_t i{}; buckets_per_buffer > i && it != last; ++i, ++it) {
					auto& [data, modified] = *it;

					std::memcpy(buf, data.data(), bucket_size);
					buf      = static_cast<void*>(static_cast<unsigned char*>(buf) + bucket_size);
					modified = false;
				}

				wgpuBufferUnmap(buffer);
			} else {
				WGPUBuffer& buffer = buffers[i];

				std::size_t offset = 0;
				for (std::size_t i{}; buckets_per_buffer > i && it != last; ++i, ++it) {
					auto& [data, modified] = *it;

					if (modified) {
						wgpuQueueWriteBuffer(queue_, buffer, offset, data.data(), bucket_size);
						modified = false;
					}
					offset += bucket_size;
				}
			}
		}

		// FIXME: Probably do not want to release them but instead reuse later when needed,
		// like how std::vector works. Need a function similar to `shrink_to_fit`. Also, need
		// to keep track of which ones are "active" and not.
		for (std::size_t i = num_buffers; buffers.size() > i; ++i) {
			updated = true;
			wgpuBufferRelease(buffers[i]);
		}
		buffers.resize(num_buffers);

		return updated;
	}

	template <class T>
	bool gpuWriteInner()
	{
		// TODO: Make gpuWriteLeaf and gpuWriteInner with a single implementation.

		assert(nullptr != device_);
		assert(nullptr != queue_);

		using Block = typename T::template InnerBlock<Dim, BF>;

		std::size_t const size = inner_data_.template serializedSize<Block>();

		auto& buffers = inner_buffers_[index_v<T, Ts...>];

		if (0 == size) {
			for (auto& buf : buffers) {
				wgpuBufferRelease(buf);
			}
			bool empty = buffers.empty();
			buffers.clear();
			return !empty;
		}

		constexpr std::size_t const bucket_size =
		    InnerData::template serializedBucketSize<Block>();

		std::size_t const buckets_per_buffer = max_buffer_size_ / bucket_size;
		std::size_t const buffer_size        = bucket_size * buckets_per_buffer;

		std::size_t const num_buffers = 1 + (size - 1) / buffer_size;

		buffers.reserve(num_buffers);

		bool updated = false;

		auto it   = inner_data_.template beginBucket<Block>();
		auto last = inner_data_.template endBucket<Block>();

		for (std::size_t i{}; num_buffers > i; ++i) {
			if (buffers.size() <= i) {
				updated = true;

				auto& buffer = buffers.emplace_back(compute::createBuffer(
				    device_, "", buffer_size, WGPUBufferUsage_Storage | WGPUBufferUsage_CopyDst,
				    true));

				assert(nullptr != buffer);

				void* buf = wgpuBufferGetMappedRange(buffer, 0, buffer_size);

				for (std::size_t i{}; buckets_per_buffer > i && it != last; ++i, ++it) {
					auto& [data, modified] = *it;

					std::memcpy(buf, data.data(), bucket_size);
					buf      = static_cast<void*>(static_cast<unsigned char*>(buf) + bucket_size);
					modified = false;
				}

				wgpuBufferUnmap(buffer);
			} else {
				WGPUBuffer& buffer = buffers[i];

				std::size_t offset = 0;
				for (std::size_t i{}; buckets_per_buffer > i && it != last; ++i, ++it) {
					auto& [data, modified] = *it;

					if (modified) {
						wgpuQueueWriteBuffer(queue_, buffer, offset, data.data(), bucket_size);
						modified = false;
					}
					offset += bucket_size;
				}
			}
		}

		// FIXME: Probably do not want to release them but instead reuse later when needed,
		// like how std::vector works. Need a function similar to `shrink_to_fit`. Also, need
		// to keep track of which ones are "active" and not.
		for (std::size_t i = num_buffers; buffers.size() > i; ++i) {
			updated = true;
			wgpuBufferRelease(buffers[i]);
		}
		buffers.resize(num_buffers);

		return updated;
	}

 protected:
	[[nodiscard]] static constexpr bool leaf(pos_type block) noexcept
	{
		return Index::TYPE_BIT != (Index::TYPE_BIT & block);
	}

	[[nodiscard]] static constexpr bool inner(pos_type block) noexcept
	{
		return !leaf(block);
	}

	[[nodiscard]] static constexpr pos_type addLeafType(pos_type block) noexcept
	{
		return block;
	}

	[[nodiscard]] static constexpr pos_type removeLeafType(pos_type block) noexcept
	{
		return block;
	}

	[[nodiscard]] static constexpr pos_type addInnerType(pos_type block) noexcept
	{
		return Index::TYPE_BIT | block;
	}

	[[nodiscard]] static constexpr pos_type removeInnerType(pos_type block) noexcept
	{
		return ~Index::TYPE_BIT & block;
	}

	[[nodiscard]] std::size_t size() const { return leafSize() + innerSize(); }

	[[nodiscard]] std::size_t leafSize() const { return leaf_data_.size(); }

	[[nodiscard]] std::size_t innerSize() const { return inner_data_.size(); }

	void reserve(std::size_t cap)
	{
		leafReserve((cap + 1) / 2);
		innerReserve(cap / 2);
	}

	void leafReserve(std::size_t cap) { leaf_data_.reserve(cap); }

	void innerReserve(std::size_t cap) { inner_data_.reserve(cap); }

	void clear()
	{
		leafClear();
		innerClear();
	}

	void leafClear() { leaf_data_.clear(); }

	void innerClear() { inner_data_.clear(); }

	[[nodiscard]] pos_type create(bool leaf) { return leaf ? leafCreate() : innerCreate(); }

	[[nodiscard]] pos_type leafCreate() { return addLeafType(leaf_data_.create()); }

	[[nodiscard]] pos_type innerCreate() { return addInnerType(inner_data_.create()); }

	[[nodiscard]] pos_type createThreadSafe(bool leaf)
	{
		return leaf ? leafCreateThreadSafe() : innerCreateThreadSafe();
	}

	[[nodiscard]] pos_type leafCreateThreadSafe()
	{
		return addLeafType(leaf_data_.createThreadSafe());
	}

	[[nodiscard]] pos_type innerCreateThreadSafe()
	{
		return addInnerType(inner_data_.createThreadSafe());
	}

	void erase(pos_type block) { leaf(block) ? leafErase(block) : innerErase(block); }

	void leafErase(pos_type block) { leaf_data_.eraseBlock(removeLeafType(block)); }

	void innerErase(pos_type block) { inner_data_.eraseBlock(removeInnerType(block)); }

	template <class T>
	[[nodiscard]] T& leafBlock(pos_type block)
	{
		assert(leafExists(block));
		return leaf_data_.template get<T>(removeLeafType(block));
	}

	template <class T>
	[[nodiscard]] T const& leafBlock(pos_type block) const
	{
		return leaf_data_.template get<T>(removeLeafType(block));
	}

	template <class T>
	[[nodiscard]] T& innerBlock(pos_type block)
	{
		assert(innerExists(block));
		return inner_data_.template get<T>(removeInnerType(block));
	}

	template <class T>
	[[nodiscard]] T const& innerBlock(pos_type block) const
	{
		return inner_data_.template get<T>(removeInnerType(block));
	}

 private:
	[[nodiscard]] WGPULimits requiredLimits(WGPUAdapter adapter)
	{
		WGPULimits required  = WGPU_LIMITS_INIT;
		WGPULimits supported = WGPU_LIMITS_INIT;

		wgpuAdapterGetLimits(adapter, &supported);

		// These two limits are different because they are "minimum" limits,
		// they are the only ones we may forward from the adapter's supported limits.
		required.minUniformBufferOffsetAlignment = supported.minUniformBufferOffsetAlignment;
		required.minStorageBufferOffsetAlignment = supported.minStorageBufferOffsetAlignment;

		max_buffer_size_ =
		    std::min(max_buffer_size_, static_cast<std::size_t>(supported.maxBufferSize));
		max_buffer_size_ =
		    std::min(max_buffer_size_,
		             static_cast<std::size_t>(supported.maxStorageBufferBindingSize));

		required.maxBufferSize               = max_buffer_size_;
		required.maxStorageBufferBindingSize = max_buffer_size_;

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

 protected:
	LeafData  leaf_data_;
	InnerData inner_data_;

	WGPUInstance                                    instance_ = nullptr;
	WGPUAdapter                                     adapter_  = nullptr;
	WGPUDevice                                      device_   = nullptr;
	WGPUQueue                                       queue_    = nullptr;
	std::array<std::vector<WGPUBuffer>, NumBuffers> leaf_buffers_{};
	std::array<std::vector<WGPUBuffer>, NumBuffers> inner_buffers_{};

	std::size_t max_buffer_size_ = 1'073'741'824 / 2;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_DATA_HPP