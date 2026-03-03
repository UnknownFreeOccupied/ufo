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

#ifndef UFO_MAP_OCCUPANCY_DETAIL_MAP_HPP
#define UFO_MAP_OCCUPANCY_DETAIL_MAP_HPP

// UFO
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/map/occupancy/predicate.hpp>
#include <ufo/map/occupancy/propagation_criteria.hpp>
#include <ufo/map/occupancy/state.hpp>
#include <ufo/map/serialized_block.hpp>
#include <ufo/map/type.hpp>
#include <ufo/math/math.hpp>
#include <ufo/math/transform3.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/enum.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/macros.hpp>
#include <ufo/utility/scalar_type.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <cstdint>
#include <iostream>
#include <limits>
#include <type_traits>

namespace ufo::detail
{
template <class Derived, class Tree, class Data>
class OccupancyMap
{
 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::OCCUPANCY;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

	using Block      = Data;
	using LeafBlock  = typename Block::template LeafBlock<Dim, BF>;
	using InnerBlock = typename Block::template InnerBlock<Dim, BF>;

	static_assert(std::is_standard_layout_v<LeafBlock>,
	              "OccupancyMap's LeafBlock needs to be standard layout");
	static_assert(std::is_standard_layout_v<InnerBlock>,
	              "OccupancyMap's InnerBlock needs to be standard layout");

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

	using occupancy_type = float;
	using logit_type     = typename Block::occupancy_type;

	static_assert(std::is_signed_v<logit_type> ||
	                  std::is_same_v<logit_type, OccupancyState>,
	              "OccupancyMap's logit type needs to be signed");

	static constexpr auto const MinLogit = []() {
		if constexpr (std::is_same_v<logit_type, OccupancyState>) {
			return static_cast<int>(to_underlying(OccupancyState::FREE));
		} else if constexpr (std::is_floating_point_v<logit_type>) {
			return std::numeric_limits<logit_type>::lowest();
		} else {
			return static_cast<logit_type>(-std::numeric_limits<logit_type>::max());
		}
	}();

	static constexpr auto const MaxLogit = []() {
		if constexpr (std::is_same_v<logit_type, OccupancyState>) {
			return static_cast<int>(to_underlying(OccupancyState::OCCUPIED));
		} else {
			return std::numeric_limits<logit_type>::max();
		}
	}();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Info                                         |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] static constexpr bool occupancyDirectional() { return false; }

	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] occupancy_type occupancy(NodeType const& node) const
	{
		return occupancy(occupancyLogit(node));
	}

	[[nodiscard]] occupancy_type occupancy(logit_type value) const
	{
		occupancy_type v;
		if constexpr (std::is_same_v<logit_type, OccupancyState>) {
			v = (to_underlying(value) + 1) * occupancy_type(0.5);
		} else if constexpr (std::is_floating_point_v<logit_type>) {
			v = logitToProbability(value);
		} else {
			v = static_cast<occupancy_type>(value) / MaxLogit;
			v = (v + 1) * occupancy_type(0.5);
			v = lerp(min_occupancy_logit_, max_occupancy_logit_, v);
			v = logitToProbability(v);
		}

		return UFO_CLAMP(v, min_occupancy_, max_occupancy_);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] logit_type occupancyLogit(NodeType const& node) const
	{
		Index n = derived().index(node);
		return occupancyLogit(n.pos, n.offset);
	}

	[[nodiscard]] logit_type occupancyLogit(occupancy_type value) const
	{
		assert(0 <= value && 1 >= value);

		if constexpr (std::is_same_v<logit_type, OccupancyState>) {
			return static_cast<OccupancyState>(static_cast<int>(free_thres_ <= value) +
			                                   static_cast<int>(occupied_thres_ < value) - 1);
		} else if constexpr (std::is_floating_point_v<logit_type>) {
			return probabilityToLogit(value);
		} else {
			occupancy_type v = probabilityToLogit(value);
			v = (v - min_occupancy_logit_) / (max_occupancy_logit_ - min_occupancy_logit_);
			return static_cast<logit_type>(lerp(static_cast<occupancy_type>(MinLogit),
			                                    static_cast<occupancy_type>(MaxLogit), v));
		}
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] OccupancyState occupancyState(NodeType const& node) const
	{
		return occupancyLogitState(occupancyLogit(node));
	}

	[[nodiscard]] OccupancyState occupancyState(occupancy_type value) const
	{
		return occupancyLogitState(occupancyLogit(value));
	}

	[[nodiscard]] OccupancyState occupancyLogitState(logit_type value) const
	{
		return static_cast<OccupancyState>(static_cast<int>(free_thres_logit_ <= value) +
		                                   static_cast<int>(occupied_thres_logit_ < value) -
		                                   1);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyUnknown(NodeType const& node) const
	{
		return OccupancyState::UNKNOWN == occupancyState(node);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyFree(NodeType const& node) const
	{
		return OccupancyState::FREE == occupancyState(node);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyOccupied(NodeType const& node) const
	{
		return OccupancyState::OCCUPIED == occupancyState(node);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyContains(NodeType const& node, OccupancyState state) const
	{
		Index n = derived().index(node);
		return occupancyContains(n.pos, n.offset, state);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyContainsUnknown(NodeType const& node) const
	{
		return occupancyContains(node, OccupancyState::UNKNOWN);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyContainsFree(NodeType const& node) const
	{
		return occupancyContains(node, OccupancyState::FREE);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool occupancyContainsOccupied(NodeType const& node) const
	{
		return occupancyContains(node, OccupancyState::OCCUPIED);
	}

	[[nodiscard]] occupancy_type occupancyClamp(occupancy_type value) const
	{
		return UFO_CLAMP(value, min_occupancy_, max_occupancy_);
	}

	[[nodiscard]] logit_type occupancyLogitClamp(logit_type value) const
	{
		if constexpr (std::is_floating_point_v<logit_type>) {
			return UFO_CLAMP(value, min_clamp_thres_, max_clamp_thres_);
		} else {
			return value;
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void occupancyLogitSet(NodeType const& node, logit_type value, bool propagate = true)
	{
		auto node_f = [this, v = occupancyLogitClamp(value)](Index node) {
			occupancyLogitSet(node.pos, node.offset, v);
		};

		auto block_f = [this, v = occupancyLogitClamp(value)](pos_type block) {
			occupancyLogitSet(block, v);
		};

		auto update_f = [this](Index node, pos_type children) -> bool {
			return onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void occupancyLogitUpdate(NodeType const& node, logit_type change,
	                          bool propagate = true)
	{
		auto node_f = [this, change](Index node) {
			occupancyLogitUpdate(node.pos, node.offset, change);
		};

		auto block_f = [this, change](pos_type block) {
			occupancyLogitUpdate(block, change);
		};

		auto update_f = [this](Index node, pos_type children) -> bool {
			return onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	template <
	    class NodeType, class UnaryOp,
	    std::enable_if_t<is_node_type_v<NodeType>, bool>                          = true,
	    std::enable_if_t<std::is_invocable_r_v<logit_type, UnaryOp, Index>, bool> = true>
	void occupancyLogitUpdate(NodeType const& node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			occupancyLogitSet(node.pos, node.offset, occupancyLogitClamp(unary_op(node)));
		};

		auto block_f = [this, unary_op](pos_type block) {
			if (derived().isPureLeaf(block)) {
				LeafBlock& ob = oLeafBlock(block);
				for (offset_type i{}; BF > i; ++i) {
					occupancyLogitSet(ob, i, occupancyLogitClamp(unary_op(Index(block, i))));
				}
			} else {
				InnerBlock& ob = oInnerBlock(block);
				for (offset_type i{}; BF > i; ++i) {
					occupancyLogitSet(ob, i, occupancyLogitClamp(unary_op(Index(block, i))));
				}
			}
		};

		auto update_f = [this](Index node, pos_type children) -> bool {
			return onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void occupancySet(NodeType const& node, occupancy_type value, bool propagate = true)
	{
		occupancyLogitSet(node, occupancyLogit(value), propagate);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void occupancyUpdate(NodeType const& node, occupancy_type change, bool propagate = true)
	{
		occupancyLogitUpdate(node, occupancyLogit(change), propagate);
	}

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<std::is_invocable_r_v<occupancy_type, UnaryOp, Index>,
	                           bool>                           = true>
	void occupancyUpdate(NodeType const& node, UnaryOp unary_op, bool propagate = true)
	{
		occupancyLogitUpdate(
		    node, [this, unary_op](Index node) { return occupancyLogit(unary_op(node)); },
		    propagate);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Sensor model                                     |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr logit_type occupancyLogitFreeThres() const noexcept
	{
		return free_thres_logit_;
	}

	[[nodiscard]] constexpr occupancy_type occupancyFreeThres() const noexcept
	{
		return occupancy(occupancyLogitFreeThres());
	}

	[[nodiscard]] constexpr logit_type occupancyLogitOccupiedThres() const noexcept
	{
		return occupied_thres_logit_;
	}

	[[nodiscard]] constexpr occupancy_type occupancyOccupiedThres() const noexcept
	{
		return occupancy(occupancyLogitOccupiedThres());
	}

	void occupancyLogitSetThres(logit_type free_thres, logit_type occupied_thres,
	                            bool propagate = true)
	{
		// FIXME: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		if (free_thres_logit_ == free_thres && occupied_thres_logit_ == occupied_thres) {
			return;
		}

		free_thres_logit_     = free_thres;
		occupied_thres_logit_ = occupied_thres;

		derived().modifiedSet();

		if (propagate) {
			derived().propagate();
			derived().modifiedReset();
		}
	}

	void occupancySetThres(occupancy_type free_thres, occupancy_type occupied_thres,
	                       bool propagate = true)
	{
		// FIXME: Should add a warning that these are very computational expensive to
		// call since the whole tree has to be updated

		occupancyLogitSetThres(occupancyLogit(free_thres), occupancyLogit(occupied_thres),
		                       propagate);
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

	[[nodiscard]] LeafBlock& oLeafBlock(pos_type pos)
	{
		return derived().template leafBlock<LeafBlock>(pos);
	}

	[[nodiscard]] LeafBlock const& oLeafBlock(pos_type pos) const
	{
		return derived().template leafBlock<LeafBlock>(pos);
	}

	[[nodiscard]] LeafBlock const& oLeafBlockConst(pos_type pos) const
	{
		return oLeafBlock(pos);
	}

	[[nodiscard]] InnerBlock& oInnerBlock(pos_type pos)
	{
		return derived().template innerBlock<InnerBlock>(pos);
	}

	[[nodiscard]] InnerBlock const& oInnerBlock(pos_type pos) const
	{
		return derived().template innerBlock<InnerBlock>(pos);
	}

	[[nodiscard]] InnerBlock const& oInnerBlockConst(pos_type pos) const
	{
		return oInnerBlock(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expects                              |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot(pos_type block)
	{
		InnerBlock& ob = oInnerBlock(block);
		if constexpr (std::is_same_v<logit_type, OccupancyState>) {
			occupancyLogitSet(ob, OccupancyState::UNKNOWN);
		} else {
			occupancyLogitSet(ob, logit_type{});
		}
	}

	void onInitLeafChildren(Index node, pos_type children)
	{
		occupancyLogitSet(oLeafBlock(children),
		                  occupancyLogit(oInnerBlockConst(node.pos), node.offset));
	}

	void onInitInnerChildren(Index node, pos_type children)
	{
		occupancyLogitSet(oInnerBlock(children),
		                  occupancyLogit(oInnerBlockConst(node.pos), node.offset));
	}

	bool onPropagateChildren(Index node, pos_type children)
	{
		InnerBlock& block      = oInnerBlock(node.pos);
		logit_type  prev_value = occupancyLogit(block, node.offset);
		bool        prev_u = occupancyContains(block, node.offset, OccupancyState::UNKNOWN);
		bool        prev_f = occupancyContains(block, node.offset, OccupancyState::FREE);
		bool        prev_o = occupancyContains(block, node.offset, OccupancyState::OCCUPIED);

		logit_type value;
		bool       u = false;
		bool       f = false;
		bool       o = false;

		auto v = MinLogit;
		if (derived().isPureLeaf(children)) {
			LeafBlock const& ob = oLeafBlockConst(children);

			for (offset_type i{}; BF > i; ++i) {
				auto x = occupancyLogit(ob, i);

				OccupancyState const s = occupancyLogitState(x);
				u                      = u || s == OccupancyState::UNKNOWN;
				f                      = f || s == OccupancyState::FREE;
				o                      = o || s == OccupancyState::OCCUPIED;

				if constexpr (std::is_same_v<logit_type, OccupancyState>) {
					decltype(v) const y = to_underlying(x);
					v                   = UFO_MAX(v, y);
				} else {
					v = UFO_MAX(v, x);
				}
			}
		} else {
			InnerBlock const& ob = oInnerBlockConst(children);

			for (offset_type i{}; BF > i; ++i) {
				auto x = occupancyLogit(ob, i);

				u = u || occupancyContains(ob, i, OccupancyState::UNKNOWN);
				f = f || occupancyContains(ob, i, OccupancyState::FREE);
				o = o || occupancyContains(ob, i, OccupancyState::OCCUPIED);

				if constexpr (std::is_same_v<logit_type, OccupancyState>) {
					decltype(v) const y = to_underlying(x);
					v                   = UFO_MAX(v, y);
				} else {
					v = UFO_MAX(v, x);
				}
			}
		}

		if constexpr (std::is_same_v<logit_type, OccupancyState>) {
			value = static_cast<OccupancyState>(v);
		} else {
			value = v;
		}

		occupancyLogitSet(block, node.offset, value, u, f, o);

		return prev_value != value || prev_u != u || prev_f != f || prev_o != o;
	}

	[[nodiscard]] bool onIsPrunable(pos_type block) const
	{
		if (derived().isPureLeaf(block)) {
			LeafBlock const& ob    = oLeafBlock(block);
			logit_type const first = occupancyLogit(ob, 0);
			for (offset_type i = 1; BF > i; ++i) {
				if (first != occupancyLogit(ob, i)) {
					return false;
				}
			}
		} else {
			InnerBlock const& ob    = oInnerBlock(block);
			logit_type const  first = occupancyLogit(ob, 0);
			for (offset_type i = 1; BF > i; ++i) {
				if (first != occupancyLogit(ob, i)) {
					return false;
				}
			}
		}

		return true;
	}

	void onPruneLeafChildren(Index node, [[maybe_unused]] pos_type children)
	{
		InnerBlock& ob = oInnerBlock(node.pos);
		occupancyLogitSet(ob, node.offset, occupancyLogit(ob, node.offset));
	}

	void onPruneInnerChildren(Index node, [[maybe_unused]] pos_type children)
	{
		InnerBlock& ob = oInnerBlock(node.pos);
		occupancyLogitSet(ob, node.offset, occupancyLogit(ob, node.offset));
	}

	[[nodiscard]] std::size_t onSerializedSize(
	    [[maybe_unused]] SerializedBlocks<BF> const& blocks, std::size_t num_nodes) const
	{
		// FIXME: What is OccupancyState?
		// Scalar type + (OccupancyState & Direction) + num_nodes * logit_type
		return sizeof(ScalarType) + sizeof(std::uint8_t) + num_nodes * sizeof(logit_type);
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onRead(ExecutionPolicy&& policy, ReadBuffer& in,
	            SerializedBlocks<BF> const& blocks)
	{
		ScalarType st;
		in.read(st);

		switch (st) {
			case ScalarType::INT8:
				return read<std::int8_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::UINT8:
				return read<std::uint8_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::INT16:
				return read<std::int16_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::UINT16:
				return read<std::uint16_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::INT32:
				return read<std::int32_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::UINT32:
				return read<std::uint32_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::INT64:
				return read<std::int64_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::UINT64:
				return read<std::uint64_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::FLOAT32:
				return read<float>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::FLOAT64:
				return read<double>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::UNKNOWN:
				return read<OccupancyState>(std::forward<ExecutionPolicy>(policy), in, blocks);
		}
	}

	template <
	    class T, class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void read(ExecutionPolicy&& policy, ReadBuffer& in, SerializedBlocks<BF> const& blocks)
	{
		std::uint8_t r;
		in.read(r);

		switch (r) {
			case 0b0u: return read<T, false>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case 0b1u: return read<T, true>(std::forward<ExecutionPolicy>(policy), in, blocks);
		}
	}

	template <
	    class T, bool Directional, class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void read(ExecutionPolicy&& policy, ReadBuffer& in, SerializedBlocks<BF> const& blocks)
	{
		for_each(std::forward<ExecutionPolicy>(policy), blocks.begin(), blocks.end(),
		         [this, in](SerializedBlock<BF> const& block) {
			         if (derived().isPureLeaf(block.block)) {
				         read<T, Directional>(in, block, oLeafBlock(block.block));
			         } else {
				         read<T, Directional>(in, block, oInnerBlock(block.block));
			         }
		         });
	}

	template <class T, bool Directional, class BlockT,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	void read(ReadBuffer const& in, SerializedBlock<BF> const& sb, BlockT& block)
	{
		using LogitType = std::conditional_t<Directional, std::array<T, 2 * Dim>, T>;

		std::size_t pos = in.readPos() + sizeof(LogitType) * sb.data_start;

		LogitType logit;
		for (offset_type i{}; BF > i; ++i) {
			if (!sb.offsets[i]) {
				continue;
			}

			in.readAt(pos, logit);
			pos += sizeof(LogitType);

			if constexpr (std::is_same_v<logit_type, LogitType>) {
				occupancyLogitSet(block, i, logit);
			} else {
				// TODO: Implement conversion
			}
		}
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onWrite(ExecutionPolicy&& policy, WriteBuffer& out,
	             SerializedBlocks<BF> const& blocks) const
	{
		std::uint8_t const r = (static_cast<std::uint8_t>(occupancyDirectional()) << 0);

		out.write(scalar_type_v<logit_type>);
		out.write(r);

		for_each(std::forward<ExecutionPolicy>(policy), blocks.begin(), blocks.end(),
		         [this, &out](SerializedBlock<BF> const& block) {
			         if (derived().isPureLeaf(block.block)) {
				         write(out, block, oLeafBlock(block.block));
			         } else {
				         write(out, block, oInnerBlock(block.block));
			         }
		         });
	}

	template <class BlockT,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	void write(WriteBuffer& out, SerializedBlock<BF> const& sb, BlockT const& block) const
	{
		std::size_t pos = out.writePos() + sizeof(logit_type) * sb.data_start;
		for (offset_type i{}; BF > i; ++i) {
			if (!sb.offsets[i]) {
				continue;
			}

			out.writeAt(pos, occupancyLogit(block, i));
			pos += sizeof(logit_type);
		}
	}

	void onDotFile(std::ostream& out, Index node) const
	{
		// TODO: Implement

		out << "Occupancy: " << occupancy(node);
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

	// Occupancy logit

	[[nodiscard]] bool occupancyLogit(LeafBlock const& block, offset_type offset) const
	{
		return block.occupancy(offset);
	}

	[[nodiscard]] bool occupancyLogit(InnerBlock const& block, offset_type offset) const
	{
		return block.occupancy(offset);
	}

	[[nodiscard]] bool occupancyLogit(pos_type block, offset_type offset) const
	{
		return derived().isPureLeaf(block) ? occupancyLogit(oLeafBlock(block), offset)
		                                   : occupancyLogit(oInnerBlock(block), offset);
	}

	// Occupancy contains

	[[nodiscard]] bool occupancyContains(LeafBlock const& block, offset_type offset,
	                                     OccupancyState state) const
	{
		return occupancyLogitState(occupancyLogit(block, offset)) == state;
	}

	[[nodiscard]] bool occupancyContains(InnerBlock const& block, offset_type offset,
	                                     OccupancyState state) const
	{
		return block.occupancyContains(offset, state);
	}

	[[nodiscard]] bool occupancyContains(pos_type block, offset_type offset,
	                                     OccupancyState state) const
	{
		return derived().isPureLeaf(block)
		           ? occupancyContains(oLeafBlock(block), offset, state)
		           : occupancyContains(oInnerBlock(block), offset, state);
	}

	// Occupancy logit set

	void occupancyLogitSet(LeafBlock& block, offset_type offset, logit_type value)
	{
		return block.occupancySet(offset, value);
	}

	void occupancyLogitSet(InnerBlock& block, offset_type offset, logit_type value, bool u,
	                       bool f, bool o)
	{
		return block.occupancySet(offset, value, u, f, o);
	}

	void occupancyLogitSet(InnerBlock& block, offset_type offset, logit_type value)
	{
		OccupancyState const s = occupancyLogitState(value);
		bool const           u = OccupancyState::UNKNOWN == s;
		bool const           f = OccupancyState::FREE == s;
		bool const           o = OccupancyState::OCCUPIED == s;
		return occupancyLogitSet(block, offset, value, u, f, o);
	}

	void occupancyLogitSet(pos_type block, offset_type offset, logit_type value)
	{
		return derived().isPureLeaf(block)
		           ? occupancyLogitSet(oLeafBlock(block), offset, value)
		           : occupancyLogitSet(oInnerBlock(block), offset, value);
	}

	void occupancyLogitSet(LeafBlock& block, logit_type value)
	{
		return block.occupancySet(value);
	}

	void occupancyLogitSet(InnerBlock& block, logit_type value, bool u, bool f, bool o)
	{
		return block.occupancySet(value, u, f, o);
	}

	void occupancyLogitSet(InnerBlock& block, logit_type value)
	{
		OccupancyState const s = occupancyLogitState(value);
		bool const           u = OccupancyState::UNKNOWN == s;
		bool const           f = OccupancyState::FREE == s;
		bool const           o = OccupancyState::OCCUPIED == s;
		return occupancyLogitSet(block, value, u, f, o);
	}

	void occupancyLogitSet(pos_type block, logit_type value)
	{
		return derived().isPureLeaf(block) ? occupancyLogitSet(oLeafBlock(block), value)
		                                   : occupancyLogitSet(oInnerBlock(block), value);
	}

	// Occupancy logit update

	void occupancyLogitUpdate(LeafBlock& block, offset_type offset, logit_type change)
	{
		logit_type value = occupancyLogit(block, offset);

		if constexpr (std::is_same_v<logit_type, OccupancyState>) {
			auto const v = to_underlying(value) + to_underlying(change);
			value        = static_cast<OccupancyState>(UFO_CLAMP(v, MinLogit, MaxLogit));
		} else if constexpr (std::is_floating_point_v<logit_type>) {
			value += change;
		} else {
			using T   = std::int64_t;
			T const v = static_cast<T>(value) + static_cast<T>(change);
			value     = static_cast<logit_type>(
          UFO_CLAMP(v, static_cast<T>(MinLogit), static_cast<T>(MaxLogit)));
		}

		occupancyLogitSet(block, offset, occupancyLogitClamp(value));
	}

	void occupancyLogitUpdate(InnerBlock& block, offset_type offset, logit_type change)
	{
		logit_type value = occupancyLogit(block, offset);

		if constexpr (std::is_same_v<logit_type, OccupancyState>) {
			auto const v = to_underlying(value) + to_underlying(change);
			value        = static_cast<OccupancyState>(UFO_CLAMP(v, MinLogit, MaxLogit));
		} else if constexpr (std::is_floating_point_v<logit_type>) {
			value += change;
		} else {
			using T   = std::int64_t;
			T const v = static_cast<T>(value) + static_cast<T>(change);
			value     = static_cast<logit_type>(
          UFO_CLAMP(v, static_cast<T>(MinLogit), static_cast<T>(MaxLogit)));
		}

		occupancyLogitSet(block, offset, occupancyLogitClamp(value));
	}

	void occupancyLogitUpdate(pos_type block, offset_type offset, logit_type change)
	{
		return derived().isPureLeaf(block)
		           ? occupancyLogitUpdate(oLeafBlock(block), offset, change)
		           : occupancyLogitUpdate(oInnerBlock(block), offset, change);
	}

	void occupancyLogitUpdate(LeafBlock& block, logit_type change)
	{
		for (offset_type i{}; BF > i; ++i) {
			occupancyLogitUpdate(block, i, change);
		}
	}

	void occupancyLogitUpdate(InnerBlock& block, logit_type change)
	{
		for (offset_type i{}; BF > i; ++i) {
			occupancyLogitUpdate(block, i, change);
		}
	}

	void occupancyLogitUpdate(pos_type block, logit_type change)
	{
		return derived().isPureLeaf(block) ? occupancyLogitUpdate(oLeafBlock(block), change)
		                                   : occupancyLogitUpdate(oInnerBlock(block), change);
	}

 private:
	occupancy_type min_occupancy_        = 0.1192;
	occupancy_type max_occupancy_        = 0.9710;
	occupancy_type min_occupancy_logit_  = probabilityToLogit(min_occupancy_);
	occupancy_type max_occupancy_logit_  = probabilityToLogit(max_occupancy_);
	occupancy_type free_thres_           = 0.5;
	occupancy_type occupied_thres_       = 0.5;
	logit_type     free_thres_logit_     = occupancyLogit(free_thres_);
	logit_type     occupied_thres_logit_ = occupancyLogit(occupied_thres_);
	logit_type     min_clamp_thres_      = occupancyLogit(min_occupancy_);
	logit_type     max_clamp_thres_      = occupancyLogit(max_occupancy_);
};
}  // namespace ufo::detail

#endif  // UFO_MAP_OCCUPANCY_DETAIL_MAP_HPP