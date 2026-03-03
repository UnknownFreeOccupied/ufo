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

#ifndef UFO_MAP_VOID_REGION_DETAIL_MAP_HPP
#define UFO_MAP_VOID_REGION_DETAIL_MAP_HPP

// UFO
#include <ufo/execution/execution.hpp>
#include <ufo/map/serialized_block.hpp>
#include <ufo/map/type.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <cstddef>
#include <ostream>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo::detail
{
template <class Derived, class Tree, class Data>
class VoidRegionMap
{
 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::VOID_REGION;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

	using Block      = Data;
	using LeafBlock  = typename Block::template LeafBlock<Dim, BF>;
	using InnerBlock = typename Block::template InnerBlock<Dim, BF>;

	static_assert(std::is_standard_layout_v<LeafBlock>,
	              "VoidRegionMap's LeafBlock needs to be standard layout");
	static_assert(std::is_standard_layout_v<InnerBlock>,
	              "VoidRegionMap's InnerBlock needs to be standard layout");

	template <class T>
	static constexpr inline bool const is_node_type_v = Tree::template is_node_type_v<T>;

	using Index       = typename Tree::Index;
	using Node        = typename Tree::Node;
	using Code        = typename Tree::Code;
	using Key         = typename Tree::Key;
	using Point       = typename Tree::Point;
	using Coord       = typename Tree::Coord;
	using coord_type  = typename Tree::coord_type;
	using depth_type  = typename Tree::depth_type;
	using pos_type    = typename Tree::pos_type;
	using offset_type = typename Tree::offset_type;
	using length_type = typename Tree::length_type;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool voidRegion(NodeType const& node) const
	{
		Index n = derived().index(node);
		return voidRegion(n.pos, n.offset);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool voidRegionContains(NodeType const& node) const
	{
		Index n = derived().index(node);
		return voidRegionContains(n.pos, n.offset);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Modifiers                                       |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void voidRegionSet(NodeType const& node, bool value, bool propagate = true)
	{
		auto node_f = [this, value](Index node) {
			voidRegionSet(node.pos, node.offset, value);
		};

		auto block_f = [this, value](pos_type block) { voidRegionSet(block, value); };

		auto update_f = [this](Index node, pos_type children) -> bool {
			return onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>                    = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryOp, Index>, bool> = true>
	void voidRegionUpdate(NodeType const& node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			voidRegionSet(node.pos, node.offset, unary_op(node));
		};

		auto block_f = [this, unary_op](pos_type block) {
			if (derived().isPureLeaf(block)) {
				LeafBlock& lb = vrLeafBlock(block);
				for (offset_type i{}; BF > i; ++i) {
					voidRegionSet(lb, i, unary_op(Index(block, i)));
				}
			} else {
				InnerBlock& ib = vrInnerBlock(block);
				for (offset_type i{}; BF > i; ++i) {
					voidRegionSet(ib, i, unary_op(Index(block, i)));
				}
			}
		};

		auto update_f = [this](Index node, pos_type children) -> bool {
			return onPropagateChildren(node, children);
		};

		derived().recursLeaves(node, node_f, block_f, update_f, propagate);
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr Derived& derived() noexcept
	{
		return *static_cast<Derived*>(this);
	}

	[[nodiscard]] constexpr Derived const& derived() const noexcept
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Block                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] LeafBlock& vrLeafBlock(pos_type pos)
	{
		return derived().template leafBlock<LeafBlock>(pos);
	}

	[[nodiscard]] LeafBlock const& vrLeafBlock(pos_type pos) const
	{
		return derived().template leafBlock<LeafBlock>(pos);
	}

	[[nodiscard]] LeafBlock const& vrLeafBlockConst(pos_type pos) const
	{
		return vrLeafBlock(pos);
	}

	[[nodiscard]] InnerBlock& vrInnerBlock(pos_type pos)
	{
		return derived().template innerBlock<InnerBlock>(pos);
	}

	[[nodiscard]] InnerBlock const& vrInnerBlock(pos_type pos) const
	{
		return derived().template innerBlock<InnerBlock>(pos);
	}

	[[nodiscard]] InnerBlock const& vrInnerBlockConst(pos_type pos) const
	{
		return vrInnerBlock(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expects                              |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot(pos_type block) { voidRegionSet(vrInnerBlock(block), false); }

	void onInitLeafChildren(Index node, pos_type children)
	{
		voidRegionSet(vrLeafBlock(children),
		              voidRegion(vrInnerBlockConst(node.pos), node.offset));
	}

	void onInitInnerChildren(Index node, pos_type children)
	{
		voidRegionSet(vrInnerBlock(children),
		              voidRegion(vrInnerBlockConst(node.pos), node.offset));
	}

	bool onPropagateChildren(Index node, pos_type children)
	{
		auto&      block         = vrInnerBlock(node.pos);
		bool const prev_value    = voidRegion(block, node.offset);
		bool const prev_contains = voidRegionContains(block, node.offset);

		bool const value    = voidRegion(children).all();
		bool const contains = voidRegionContains(children).any();

		voidRegionSet(block, node.offset, value, contains);

		return (value != prev_value) || (contains != prev_contains);
	}

	[[nodiscard]] bool onIsPrunable(pos_type block) const
	{
		return !voidRegion(block).some();
	}

	void onPruneLeafChildren(Index node, [[maybe_unused]] pos_type children)
	{
		auto& block = vrInnerBlock(node.pos);
		voidRegionSet(block, node.offset, voidRegion(block, node.offset));
	}

	void onPruneInnerChildren(Index node, [[maybe_unused]] pos_type children)
	{
		auto& block = vrInnerBlock(node.pos);
		voidRegionSet(block, node.offset, voidRegion(block, node.offset));
	}

	[[nodiscard]] std::size_t onSerializedSize(SerializedBlocks<BF> const&  blocks,
	                                           [[maybe_unused]] std::size_t num_nodes) const
	{
		return blocks.size() * sizeof(typename BitSet<BF>::value_type);
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onRead(ExecutionPolicy&& policy, ReadBuffer& in,
	            SerializedBlocks<BF> const& blocks)
	{
		for_each(std::forward<ExecutionPolicy>(policy), blocks.begin(), blocks.end(),
		         [this, &in](SerializedBlock<BF> const& block) {
			         if (derived().isPureLeaf(block.block)) {
				         read(in, block, vrLeafBlock(block.block));
			         } else {
				         read(in, block, vrInnerBlock(block.block));
			         }
		         });
	}

	template <class BlockT,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	void read(ReadBuffer const& in, SerializedBlock<BF> const& sb, BlockT& block)
	{
		typename BitSet<BF>::value_type data;
		std::size_t                     pos = in.readPos() + sizeof(data) * sb.data_start;

		in.readAt(pos, data);
		BitSet<BF> const value(data);
		for (offset_type i{}; BF > i; ++i) {
			if (!sb.offsets[i]) {
				continue;
			}

			voidRegionSet(block, i, value[i]);
		}
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onWrite(ExecutionPolicy&& policy, WriteBuffer& out,
	             SerializedBlocks<BF> const& blocks) const
	{
		for_each(std::forward<ExecutionPolicy>(policy), blocks.begin(), blocks.end(),
		         [this, &out](SerializedBlock<BF> const& block) {
			         auto        data = voidRegion(block.block).data();
			         std::size_t pos  = out.writePos() + sizeof(data) * block.data_start;
			         out.writeAt(pos, data);
		         });
	}

	void onDotFile(std::ostream& out, Index node) const
	{
		if (voidRegion(node)) {
			out << "Void region: <font color='green'><b>true</b></font>";
		} else {
			out << "Void region: <font color='red'>false</font>";
		}
	}

	void onGpuRelease() {}

	void onGpuRead() {}

	void onGpuReadLeaf() {}

	void onGpuReadInner() {}

	void onGpuWrite() {}

	void onGpuWriteLeaf() {}

	void onGpuWriteInner() {}

	/**************************************************************************************
	|                                                                                     |
	|                                  Helper functions                                   |
	|                                                                                     |
	**************************************************************************************/

	// Void region

	[[nodiscard]] static bool voidRegion(LeafBlock const& block, offset_type offset)
	{
		return block.voidRegion(offset);
	}

	[[nodiscard]] static bool voidRegion(InnerBlock const& block, offset_type offset)
	{
		return block.voidRegion(offset);
	}

	[[nodiscard]] bool voidRegion(pos_type block, offset_type offset) const
	{
		return derived().isPureLeaf(block) ? voidRegion(vrLeafBlock(block), offset)
		                                   : voidRegion(vrInnerBlock(block), offset);
	}

	[[nodiscard]] static BitSet<BF> voidRegion(LeafBlock const& block)
	{
		return block.voidRegion();
	}

	[[nodiscard]] static BitSet<BF> voidRegion(InnerBlock const& block)
	{
		return block.voidRegion();
	}

	[[nodiscard]] BitSet<BF> voidRegion(pos_type block) const
	{
		return derived().isPureLeaf(block) ? voidRegion(vrLeafBlock(block))
		                                   : voidRegion(vrInnerBlock(block));
	}

	// Void region contains

	[[nodiscard]] static bool voidRegionContains(LeafBlock const& block, offset_type offset)
	{
		return voidRegion(block, offset);
	}

	[[nodiscard]] static bool voidRegionContains(InnerBlock const& block,
	                                             offset_type       offset)
	{
		return block.voidRegionContains(offset);
	}

	[[nodiscard]] bool voidRegionContains(pos_type block, offset_type offset) const
	{
		return derived().isPureLeaf(block) ? voidRegionContains(vrLeafBlock(block), offset)
		                                   : voidRegionContains(vrInnerBlock(block), offset);
	}

	[[nodiscard]] static BitSet<BF> voidRegionContains(LeafBlock const& block)
	{
		return voidRegion(block);
	}

	[[nodiscard]] static BitSet<BF> voidRegionContains(InnerBlock const& block)
	{
		return block.voidRegionContains();
	}

	[[nodiscard]] BitSet<BF> voidRegionContains(pos_type block) const
	{
		return derived().isPureLeaf(block) ? voidRegionContains(vrLeafBlock(block))
		                                   : voidRegionContains(vrInnerBlock(block));
	}

	// Void region set

	static void voidRegionSet(LeafBlock& block, offset_type offset, bool value)
	{
		return block.voidRegionSet(offset, value);
	}

	static void voidRegionSet(InnerBlock& block, offset_type offset, bool value,
	                          bool contains)
	{
		return block.voidRegionSet(offset, value, contains);
	}

	static void voidRegionSet(InnerBlock& block, offset_type offset, bool value)
	{
		return voidRegionSet(block, offset, value, value);
	}

	void voidRegionSet(pos_type block, offset_type offset, bool value)
	{
		return derived().isPureLeaf(block)
		           ? voidRegionSet(vrLeafBlock(block), offset, value)
		           : voidRegionSet(vrInnerBlock(block), offset, value);
	}

	static void voidRegionSet(LeafBlock& block, bool value)
	{
		return block.voidRegionSet(value);
	}

	static void voidRegionSet(InnerBlock& block, bool value, bool contains)
	{
		return block.voidRegionSet(value, contains);
	}

	static void voidRegionSet(InnerBlock& block, bool value)
	{
		return voidRegionSet(block, value, value);
	}

	void voidRegionSet(pos_type block, bool value)
	{
		return derived().isPureLeaf(block) ? voidRegionSet(vrLeafBlock(block), value)
		                                   : voidRegionSet(vrInnerBlock(block), value);
	}
};
}  // namespace ufo::detail

#endif  // UFO_MAP_VOID_REGION_DETAIL_MAP_HPP