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

#ifndef UFO_CONTAINER_TREE_INNER_BLOCK_HPP
#define UFO_CONTAINER_TREE_INNER_BLOCK_HPP

// UFO
// #include <ufo/container/tree/code.hpp>
#include <ufo/container/tree/index.hpp>

// STL
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ufo
{
template <std::size_t Dim, std::size_t BF>
struct TreeInnerBlock {
	using modified_type = std::uint16_t;

	static_assert(sizeof(std::atomic<modified_type>) == sizeof(modified_type));
	static_assert(sizeof(std::atomic<TreeIndex::pos_type>) == sizeof(TreeIndex::pos_type));

	alignas(std::atomic_ref<TreeIndex::pos_type>::required_alignment)
	    std::array<TreeIndex::pos_type, BF> children;

	// Code to the first node of the block
	// std::array<typename TreeCode<Dim>::code_type, 3> code;

	TreeIndex::pos_type parent_block;
	std::uint8_t        parent_offset;

	std::uint8_t depth;

	// Bit set saying if the node corresponding to the bit has been modified
	alignas(std::atomic_ref<modified_type>::required_alignment) modified_type modified;
};

static_assert(24 == sizeof(TreeInnerBlock<2, 4>));
static_assert(40 == sizeof(TreeInnerBlock<3, 8>));
static_assert(72 == sizeof(TreeInnerBlock<4, 16>));

static_assert(std::is_standard_layout_v<TreeInnerBlock<2, 4>>);
static_assert(std::is_standard_layout_v<TreeInnerBlock<3, 8>>);
static_assert(std::is_standard_layout_v<TreeInnerBlock<4, 16>>);
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_INNER_BLOCK_HPP