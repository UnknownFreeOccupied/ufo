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

#ifndef UFO_MAP_LABEL_MAP_HPP
#define UFO_MAP_LABEL_MAP_HPP

// UFO
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/map/block.hpp>
#include <ufo/map/label/block.hpp>
#include <ufo/map/label/predicate.hpp>
#include <ufo/map/type.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <iostream>
#include <type_traits>

namespace ufo
{
template <class Derived, class Tree>
class LabelMap
{
 private:
	template <class Derived2, class Tree2>
	friend class LabelMap;

 public:
	// NOTE: For some reason MSVC requires these to be public
	static constexpr auto const BF  = Tree::branchingFactor();
	static constexpr auto const Dim = Tree::dimensions();

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	static constexpr MapType const Type = MapType::LABEL;

	// Container
	using Index    = typename Tree::Index;
	using Node     = typename Tree::Node;
	using Code     = typename Tree::Code;
	using Key      = typename Tree::Key;
	using Point    = typename Tree::Point;
	using Coord    = typename Tree::Coord;
	using coord_t  = typename Tree::coord_t;
	using depth_t  = typename Tree::depth_t;
	using offset_t = typename Tree::offset_t;
	using length_t = typename Tree::length_t;
	using pos_t    = typename Tree::pos_t;

	// Label
	using label_t = typename LabelBlock<BF>::label_t;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] label_t label(NodeType node) const
	{
		Index n = derived().index(node);
		return labelBlock(n.pos)[n.offset].label;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Modifiers                                        |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief
	 *
	 * @note If `NodeType` is `Index`, only propagates up to `node` and it does not set
	 * modified if propagation is `false`.
	 *
	 * @tparam NodeType Should be of type `Node`, `Code`, `Key`, or `Coord`
	 * @param node
	 * @param value
	 * @param propagate
	 */
	template <class NodeType,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool> = true>
	void labelSet(NodeType node, label_t value, bool propagate = true)
	{
		LabelElement elem(value);

		auto node_f  = [this, elem](Index node) { labelBlock(node.pos)[node.offset] = elem; };
		auto block_f = [this, elem](pos_t pos) { labelBlock(pos).fill(elem); };

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

	template <class NodeType, class UnaryOp,
	          std::enable_if_t<Tree::template is_node_type_v<NodeType>, bool>        = true,
	          std::enable_if_t<std::is_invocable_r_v<label_t, UnaryOp, Index>, bool> = true>
	void labelUpdate(NodeType node, UnaryOp unary_op, bool propagate = true)
	{
		auto node_f = [this, unary_op](Index node) {
			label_t value = unary_op(node);

			auto& lb = LabelBlock(node.pos)[node.offset];
			lb.label = value;
		};

		auto block_f = [this, node_f](pos_t pos) {
			for (std::size_t i{}; BF > i; ++i) {
				node_f(Index(pos, i));
			}
		};

		auto update_f = [this](Index node, pos_t children) {
			onPropagateChildren(node, children);
		};

		derived().recursParentFirst(node, node_f, block_f, update_f, propagate);
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	LabelMap() { onInitRoot(); }

	LabelMap(LabelMap const& other) = default;

	LabelMap(LabelMap&& other) = default;

	template <class Derived2, class Tree2>
	LabelMap(LabelMap<Derived2, Tree2> const& other)
	{
	}

	template <class Derived2, class Tree2>
	LabelMap(LabelMap<Derived2, Tree2>&& other)
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~LabelMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	LabelMap& operator=(LabelMap const& rhs) = default;

	LabelMap& operator=(LabelMap&& rhs) = default;

	template <class Derived2, class Tree2>
	LabelMap& operator=(LabelMap<Derived2, Tree2> const& rhs)
	{
		return *this;
	}

	template <class Derived2, class Tree2>
	LabelMap& operator=(LabelMap<Derived2, Tree2>&& rhs)
	{
		return *this;
	}

	//
	// Swap
	//

	void swap(LabelMap& other) noexcept {}

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

	[[nodiscard]] LabelBlock<BF>& labelBlock(pos_t pos)
	{
		return derived().template data<LabelBlock<BF>>(pos);
	}

	[[nodiscard]] LabelBlock<BF> const& labelBlock(pos_t pos) const
	{
		return derived().template data<LabelBlock<BF>>(pos);
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Functions Derived expect                               |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot()
	{
		auto& block = labelBlock(0);

		label_t value{};
		block[0].label = value;
	}

	void onInitChildren(Index node, pos_t children)
	{
		labelBlock(children).fill(labelBlock(node.pos)[node.offset]);
	}

	void onPropagateChildren(Index node, pos_t children)
	{
		auto const& children_block = labelBlock(children);
		auto&       lb             = labelBlock(node.pos)[node.offset];

		label_t value{};
		for (std::size_t i{}; BF > i; ++i) {
			value |= children_block[i].label;
		}

		lb.label = value;
	}

	[[nodiscard]] bool onIsPrunable(pos_t block) const
	{
		return std::all_of(
		    begin(labelBlock(block).data) + 1, end(labelBlock(block).data),
		    [v = labelBlock(block)[0].label](auto const& e) { return v == e.label; });
	}

	void onPruneChildren(Index node, pos_t children) {}

	//
	// Input/output (read/write)
	//

	[[nodiscard]] constexpr std::size_t serializedSize(
	    std::vector<std::pair<pos_t, BitSet<BF>>> const& /* nodes */,
	    std::size_t num_nodes) const
	{
		return num_nodes * sizeof(label_t);
	}

	void onRead(ReadBuffer& in, std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes)
	{
		for (auto [block, offset] : nodes) {
			auto& lb = labelBlock(block);

			if (offset.all()) {
				std::array<label_t, BF> label;
				in.read(label);
				for (offset_t i{}; BF > i; ++i) {
					lb[i] = LabelElement(label[i]);
				}
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						label_t value;
						in.read(value);
						lb[i] = LabelElement(value);
					}
				}
			}
		}
	}

	void onWrite(WriteBuffer&                                     out,
	             std::vector<std::pair<pos_t, BitSet<BF>>> const& nodes) const
	{
		for (auto [block, offset] : nodes) {
			auto const& lb = LabelBlock(block);

			if (offset.all()) {
				std::array<label_t, BF> label;
				for (offset_t i{}; BF > i; ++i) {
					label[i] = lb[i].label;
				}
				out.write(label);
			} else {
				for (offset_t i{}; BF > i; ++i) {
					if (offset[i]) {
						out.write(lb[i].label);
					}
				}
			}
		}
	}

	//
	// Dot file info
	//

	void onDotFileInfo(std::ostream& out, Index node) const
	{
		out << "Label: " << label(node);
	}
};

template <std::size_t Dim, std::size_t BF>
struct map_block<LabelMap, Dim, BF> {
	using type = LabelBlock<BF>;
};
}  // namespace ufo

#endif