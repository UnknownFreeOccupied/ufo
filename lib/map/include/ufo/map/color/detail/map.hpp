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

#ifndef UFO_MAP_COLOR_DETAIL_MAP_HPP
#define UFO_MAP_COLOR_DETAIL_MAP_HPP

// UFO
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/map/serialized_block.hpp>
#include <ufo/map/type.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/enum.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/scalar_type.hpp>
#include <ufo/vision/color.hpp>

// STL
#include <algorithm>
#include <array>
#include <cassert>
#include <functional>
#include <iostream>
#include <type_traits>
#include <vector>

namespace ufo::detail
{
template <class Derived, class Tree, class Data>
class ColorMap
{
 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::COLOR;

	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

	using Block      = Data;
	using LeafBlock  = typename Block::template LeafBlock<Dim, BF>;
	using InnerBlock = typename Block::template InnerBlock<Dim, BF>;

	using ColorBlock      = Block;
	using ColorLeafBlock  = LeafBlock;
	using ColorInnerBlock = InnerBlock;

	static_assert(std::is_standard_layout_v<ColorLeafBlock>,
	              "ColorMap's LeafBlock needs to be standard layout");
	static_assert(std::is_standard_layout_v<ColorInnerBlock>,
	              "ColorMap's InnerBlock needs to be standard layout");

	template <class T>
	static constexpr inline bool const is_node_type_v = Tree::template is_node_type_v<T>;

	// Container
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

	using color_type = typename Block::color_type;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Info                                         |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] static constexpr ColorType colorType()
	{
		return color_type_v<color_type>;
	}

	[[nodiscard]] static constexpr ScalarType colorValueType()
	{
		return scalar_type_v<value_type_t<color_type>>;
	}

	[[nodiscard]] static constexpr bool colorHasAlpha() { return has_alpha_v<color_type>; }

	[[nodiscard]] static constexpr bool colorHasWeight()
	{
		return has_weight_v<color_type>;
	}

	[[nodiscard]] static constexpr bool colorDirectional() { return false; }

	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class To                                         = color_type, class NodeType,
	          std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::enable_if_t<is_color_v<To>, To> color(NodeType node) const
	{
		Index n = derived().index(node);
		return convert<To>(color(n.pos, n.offset));
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] SmallRGB colorRGB(NodeType node) const
	{
		return color<SmallRGB>(node);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] SmallRGBA colorRGBA(NodeType node) const
	{
		return color<SmallRGBA>(node);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] auto colorAlpha(NodeType node) const
	{
		return alpha(color(node));
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] float colorWeight(NodeType node) const
	{
		return weight(color(node));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType, ColorType CT, class T, bool Alpha, bool Weight,
	          std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void colorSet(NodeType node, Color<CT, T, Alpha, Weight> value, bool propagate = true)
	{
		auto const v = convert<color_type>(value);

		auto node_f = [this, v](Index node) { colorSet(node.pos, node.offset, v); };

		auto block_f = [this, v](pos_type block) { colorSet(block, v); };

		auto update_f = [this](Index node, pos_type children) {
			return onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	//
	// Add color
	//

	template <class NodeType, ColorType CT, class T, bool Alpha, bool Weight,
	          std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void colorAdd(NodeType node, Color<CT, T, Alpha, Weight> value, bool propagate = true)
	{
		auto const v = convert<color_type>(value);

		auto node_f = [this, v](Index node) {
			colorUpdate(node.pos, node.offset, v, std::plus<color_type>{});
		};

		auto block_f = [this, v](pos_type block) {
			colorUpdate(block, v, std::plus<color_type>{});
		};

		auto update_f = [this](Index node, pos_type children) {
			return onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	//
	// Subtract color
	//

	template <class NodeType, ColorType CT, class T, bool Alpha, bool Weight,
	          std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void colorSub(NodeType node, Color<CT, T, Alpha, Weight> value, bool propagate = true)
	{
		auto const v = convert<color_type>(value);

		auto node_f = [this, v](Index node) {
			colorUpdate(node.pos, node.offset, v, std::minus<color_type>{});
		};

		auto block_f = [this, v](pos_type block) {
			colorUpdate(block, v, std::minus<color_type>{});
		};

		auto update_f = [this](Index node, pos_type children) {
			return onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	//
	// Update color
	//

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<std::is_invocable_v<UnaryOp, Index>, bool> = true>
	void colorUpdate(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			colorSet(node.pos, node.offset, convert<color_type>(unary_op(node)));
		};

		auto block_f = [this, unary_op](pos_type block) {
			if (derived().isPureLeaf(block)) {
				LeafBlock& cb = colorLeafBlock(block);
				for (offset_type i{}; BF > i; ++i) {
					colorSet(cb, i, convert<color_type>(unary_op(Index(block, i))));
				}
			} else {
				InnerBlock& cb = colorInnerBlock(block);
				for (offset_type i{}; BF > i; ++i) {
					colorSet(cb, i, convert<color_type>(unary_op(Index(block, i))));
				}
			}
		};

		auto update_f = [this](Index node, pos_type children) {
			return onPropagateChildren(node, children);
		};

		derived().recursLeaves(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void colorClear(NodeType node, bool propagate = true)
	{
		colorSet(node, color_type{}, propagate);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Prune                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr bool colorPruneNormalize() const { return prune_normalize_; }

	void colorPruneNormalize(bool value) { prune_normalize_ = value; }

	[[nodiscard]] constexpr float colorPruneMaxDeltaE() const
	{
		return std::sqrt(prune_max_delta_e_sq_);
	}

	void colorPruneMaxDeltaE(float value) { prune_max_delta_e_sq_ = value * value; }

	[[nodiscard]] constexpr ColorDeltaE colorPruneDeltaE() const { return prune_delta_e_; }

	void colorPruneDeltaE(ColorDeltaE value) { prune_delta_e_ = value; }

	[[nodiscard]] constexpr ColorSpace colorPruneSpace() const { return prune_space_; }

	void colorPruneSpace(ColorSpace value) { prune_space_ = value; }

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Block                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] ColorLeafBlock& colorLeafBlock(pos_type pos)
	{
		return derived().template leafBlock<ColorLeafBlock>(pos);
	}

	[[nodiscard]] ColorLeafBlock const& colorLeafBlock(pos_type pos) const
	{
		return derived().template leafBlock<ColorLeafBlock>(pos);
	}

	[[nodiscard]] ColorLeafBlock const& colorLeafBlockConst(pos_type pos) const
	{
		return colorLeafBlock(pos);
	}

	[[nodiscard]] ColorInnerBlock& colorInnerBlock(pos_type pos)
	{
		return derived().template innerBlock<ColorInnerBlock>(pos);
	}

	[[nodiscard]] ColorInnerBlock const& colorInnerBlock(pos_type pos) const
	{
		return derived().template innerBlock<ColorInnerBlock>(pos);
	}

	[[nodiscard]] ColorInnerBlock const& colorInnerBlockConst(pos_type pos) const
	{
		return colorInnerBlock(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expects                              |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot(pos_type block) { colorSet(colorInnerBlock(block), color_type{}); }

	void onInitLeafChildren(Index node, pos_type children)
	{
		colorSet(colorLeafBlock(children),
		         color(colorInnerBlockConst(node.pos), node.offset));
	}

	void onInitInnerChildren(Index node, pos_type children)
	{
		colorSet(colorInnerBlock(children),
		         color(colorInnerBlockConst(node.pos), node.offset));
	}

	bool onPropagateChildren(Index node, pos_type children)
	{
		auto& block = colorInnerBlock(node.pos);

		color_type const prev_value = color(block, node.offset);
		color_type const value      = average(color(children));

		colorSet(block, node.offset, value);

		return value != prev_value;
	}

	[[nodiscard]] bool onIsPrunable(pos_type block) const
	{
		using std::begin;
		using std::end;
		using namespace color;

		auto const& colors = color(block);

		auto const f = *begin(colors);
		auto const m = prune_max_delta_e_sq_;
		auto const d = prune_delta_e_;
		auto const s = prune_space_;

		return prune_normalize_
		           ? std::all_of(begin(colors) + 1, end(colors),
		                         [f = removeWeight(f), m, d, s](auto const& c) {
			                         return m >= deltaESquared(f, removeWeight(c), d, s);
		                         })
		           :
		           // FIXME: How to do this?
		           std::all_of(begin(colors) + 1, end(colors),
		                       [f](auto const& c) { return f == c; });
	}

	void onPruneLeafChildren([[maybe_unused]] Index    node,
	                         [[maybe_unused]] pos_type children)
	{
	}

	void onPruneInnerChildren([[maybe_unused]] Index    node,
	                          [[maybe_unused]] pos_type children)
	{
	}

	[[nodiscard]] std::size_t onSerializedSize(
	    [[maybe_unused]] SerializedBlocks<BF> const& blocks, std::size_t num_nodes) const
	{
		// ColorType, T, (Alpha, Weight, Directional)
		return sizeof(ColorType) + sizeof(ScalarType) + sizeof(std::uint8_t) +
		       num_nodes * sizeof(color_type);
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onRead(ExecutionPolicy&& policy, ReadBuffer& in,
	            SerializedBlocks<BF> const& blocks)
	{
		ColorType ct;
		in.read(ct);

		switch (ct) {
			case ColorType::RGB:
				return read<ColorType::RGB>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ColorType::LRGB:
				return read<ColorType::LRGB>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ColorType::LAB:
				return read<ColorType::LAB>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ColorType::LCH:
				return read<ColorType::LCH>(std::forward<ExecutionPolicy>(policy), in, blocks);
		}
	}

	template <
	    ColorType CT, class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void read(ExecutionPolicy&& policy, ReadBuffer& in, SerializedBlocks<BF> const& blocks)
	{
		ScalarType st;
		in.read(st);

		switch (st) {
			// case ScalarType::INT8:
			// 	return read<CT, std::int8_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::UINT8:
				return read<CT, std::uint8_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			// case ScalarType::INT16:
			// 	return read<CT, std::int16_t>(std::forward<ExecutionPolicy>(policy), in,
			// blocks); case ScalarType::UINT16: 	return read<CT,
			// std::uint16_t>(std::forward<ExecutionPolicy>(policy), in, blocks); case
			// ScalarType::INT32: 	return read<CT,
			// std::int32_t>(std::forward<ExecutionPolicy>(policy), in, blocks); case
			// ScalarType::UINT32: 	return read<CT,
			// std::uint32_t>(std::forward<ExecutionPolicy>(policy), in, blocks); case
			// ScalarType::INT64: 	return read<CT,
			// std::int64_t>(std::forward<ExecutionPolicy>(policy), in, blocks); case
			// ScalarType::UINT64: 	return read<CT,
			// std::uint64_t>(std::forward<ExecutionPolicy>(policy), in, blocks);
			case ScalarType::FLOAT32:
				return read<CT, float>(std::forward<ExecutionPolicy>(policy), in, blocks);
			// case ScalarType::FLOAT64:
			// 	return read<CT, double>(std::forward<ExecutionPolicy>(policy), in, blocks);
			default: assert(false);
		}
	}

	template <
	    ColorType CT, class T, class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void read(ExecutionPolicy&& policy, ReadBuffer& in, SerializedBlocks<BF> const& blocks)
	{
		std::uint8_t r;
		in.read(r);

		switch (r) {
			case 0b000u:
				return read<CT, T, false, false, false>(std::forward<ExecutionPolicy>(policy), in,
				                                        blocks);
			case 0b001u:
				return read<CT, T, true, false, false>(std::forward<ExecutionPolicy>(policy), in,
				                                       blocks);
			case 0b010u:
				return read<CT, T, false, true, false>(std::forward<ExecutionPolicy>(policy), in,
				                                       blocks);
			case 0b011u:
				return read<CT, T, true, true, false>(std::forward<ExecutionPolicy>(policy), in,
				                                      blocks);
			case 0b100u:
				return read<CT, T, false, false, true>(std::forward<ExecutionPolicy>(policy), in,
				                                       blocks);
			case 0b101u:
				return read<CT, T, true, false, true>(std::forward<ExecutionPolicy>(policy), in,
				                                      blocks);
			case 0b110u:
				return read<CT, T, false, true, true>(std::forward<ExecutionPolicy>(policy), in,
				                                      blocks);
			case 0b111u:
				return read<CT, T, true, true, true>(std::forward<ExecutionPolicy>(policy), in,
				                                     blocks);
		}
	}

	template <
	    ColorType CT, class T, bool Alpha, bool Weight, bool Directional,
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void read(ExecutionPolicy&& policy, ReadBuffer const& in,
	          SerializedBlocks<BF> const& blocks)
	{
		for_each(std::forward<ExecutionPolicy>(policy), blocks.begin(), blocks.end(),
		         [this, &in](SerializedBlock<BF> const& block) {
			         if (derived().isPureLeaf(block.block)) {
				         read<CT, T, Alpha, Weight, Directional>(in, block,
				                                                 colorLeafBlock(block.block));
			         } else {
				         read<CT, T, Alpha, Weight, Directional>(in, block,
				                                                 colorInnerBlock(block.block));
			         }
		         });
	}

	template <ColorType CT, class T, bool Alpha, bool Weight, bool Directional,
	          class BlockT,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	void read(ReadBuffer const& in, SerializedBlock<BF> const& sb, BlockT& block)
	{
		using C         = Color<CT, T, Alpha, Weight>;
		using ColorType = std::conditional_t<Directional, std::array<C, 2 * Dim>, C>;

		std::size_t pos = in.readPos() + sizeof(ColorType) * sb.data_start;

		ColorType color;
		for (offset_type i{}; BF > i; ++i) {
			if (!sb.offsets[i]) {
				continue;
			}

			in.readAt(pos, color);
			pos += sizeof(ColorType);

			if constexpr (Directional) {
				colorSet(block, i, convert<color_type>(average(color)));
			} else {
				colorSet(block, i, convert<color_type>(color));
			}
		}
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onWrite(ExecutionPolicy&& policy, WriteBuffer& out,
	             SerializedBlocks<BF> const& blocks) const
	{
		std::uint8_t const r = (static_cast<std::uint8_t>(colorHasAlpha()) << 0) |
		                       (static_cast<std::uint8_t>(colorHasWeight()) << 1) |
		                       (static_cast<std::uint8_t>(colorDirectional()) << 2);

		out.write(colorType());
		out.write(colorValueType());
		out.write(r);

		for_each(std::forward<ExecutionPolicy>(policy), blocks.begin(), blocks.end(),
		         [this, &out](SerializedBlock<BF> const& block) {
			         if (derived().isPureLeaf(block.block)) {
				         write(out, block, colorLeafBlock(block.block));
			         } else {
				         write(out, block, colorInnerBlock(block.block));
			         }
		         });
	}

	template <class BlockT,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	void write(WriteBuffer& out, SerializedBlock<BF> const& sb, BlockT const& block) const
	{
		std::size_t pos = out.writePos() + sizeof(color_type) * sb.data_start;
		for (offset_type i{}; BF > i; ++i) {
			if (!sb.offsets[i]) {
				continue;
			}

			out.writeAt(pos, color(block, i));
			pos += sizeof(color_type);
		}
	}

	void onDotFile(std::ostream& out, Index node) const { out << "Color: " << color(node); }

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

	template <class BlockT,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	[[nodiscard]] static auto const& color(BlockT const& block)
	{
		return block.color();
	}

	[[nodiscard]] auto const& color(pos_type block) const
	{
		return derived().isPureLeaf(block) ? color(colorLeafBlock(block))
		                                   : color(colorInnerBlock(block));
	}

	template <class BlockT,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	[[nodiscard]] static color_type color(BlockT const& block, std::size_t index)
	{
		return block.color(index);
	}

	[[nodiscard]] color_type color(pos_type block, std::size_t index) const
	{
		return derived().isPureLeaf(block) ? color(colorLeafBlock(block), index)
		                                   : color(colorInnerBlock(block), index);
	}

	template <class BlockT,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	static void colorSet(BlockT& block, color_type value)
	{
		return block.colorSet(value);
	}

	void colorSet(pos_type block, color_type value)
	{
		return derived().isPureLeaf(block) ? colorSet(colorLeafBlock(block), value)
		                                   : colorSet(colorInnerBlock(block), value);
	}

	template <class BlockT,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	static void colorSet(BlockT& block, std::size_t index, color_type value)
	{
		return block.colorSet(index, value);
	}

	void colorSet(pos_type block, std::size_t index, color_type value)
	{
		return derived().isPureLeaf(block) ? colorSet(colorLeafBlock(block), index, value)
		                                   : colorSet(colorInnerBlock(block), index, value);
	}

	template <class BlockT, class BinaryOp,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	void colorUpdate(BlockT& block, color_type value, BinaryOp op)
	{
		for (std::size_t i{}; BF > i; ++i) {
			colorUpdate(block, i, value, op);
		}
	}

	template <class BinaryOp>
	void colorUpdate(pos_type block, color_type value, BinaryOp op)
	{
		return derived().isPureLeaf(block) ? colorUpdate(colorLeafBlock(block), value, op)
		                                   : colorUpdate(colorInnerBlock(block), value, op);
	}

	template <class BlockT, class BinaryOp,
	          std::enable_if_t<contains_type_v<BlockT, LeafBlock, InnerBlock>, bool> = true>
	void colorUpdate(BlockT& block, std::size_t index, color_type value, BinaryOp op)
	{
		colorSet(block, index, op(std::move(color(block, index)), value));
	}

	template <class BinaryOp>
	void colorUpdate(pos_type block, std::size_t index, color_type value, BinaryOp op)
	{
		return derived().isPureLeaf(block)
		           ? colorUpdate(colorLeafBlock(block), index, value, op)
		           : colorUpdate(colorInnerBlock(block), index, value, op);
	}

 private:
	bool        prune_normalize_      = false;
	float       prune_max_delta_e_sq_ = 2.3f * 2.3f;
	ColorDeltaE prune_delta_e_        = ColorDeltaE::EUCLIDEAN;
	ColorSpace  prune_space_          = ColorSpace::NATIVE;
};
}  // namespace ufo::detail

#endif  // UFO_MAP_COLOR_DETAIL_MAP_HPP