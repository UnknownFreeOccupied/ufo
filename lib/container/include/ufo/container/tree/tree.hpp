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

#ifndef UFO_CONTAINER_TREE_HPP
#define UFO_CONTAINER_TREE_HPP

// UFO
#include <ufo/container/tree/block.hpp>
#include <ufo/container/tree/code.hpp>
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/coord.hpp>
#include <ufo/container/tree/data.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/container/tree/iterator.hpp>
#include <ufo/container/tree/key.hpp>
#include <ufo/container/tree/nearest_iterator.hpp>
#include <ufo/container/tree/node.hpp>
#include <ufo/container/tree/predicate.hpp>
#include <ufo/container/tree/query_iterator.hpp>
#include <ufo/container/tree/query_nearest_iterator.hpp>
#include <ufo/container/tree/trace_result.hpp>
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/geometry/aabb.hpp>
#include <ufo/geometry/ray.hpp>
#include <ufo/numeric/math.hpp>
#include <ufo/numeric/vec.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/iterator_wrapper.hpp>
#include <ufo/utility/macros.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo
{
enum class NearestSearchAlgorithm { DEPTH_FIRST, A_STAR };

/**
 * @brief
 *
 * Utilizing curiously recurring template pattern (CRTP)
 *
 * \tparam Derived ...
 * \tparam Dim ...
 * \tparam Ts ...
 */
template <class Derived, std::size_t Dim, class... Blocks>
class Tree : public TreeData<Derived, Dim, TreeBlock, Blocks...>
{
 protected:
	//
	// Friends
	//

	friend Derived;

	using Data = TreeData<Derived, Dim, TreeBlock, Blocks...>;

	static constexpr TreeIndex::offset_type const BF = Data::BF;

	using LeafBlock  = typename TreeBlock::LeafBlock<Dim, BF>;
	using InnerBlock = typename TreeBlock::InnerBlock<Dim, BF>;

	using modified_type = typename LeafBlock::modified_type;

	static constexpr modified_type const MODIFIED_ALL_SET =
	    static_cast<modified_type>(~(~0u << BF));
	static constexpr modified_type const MODIFIED_NONE_SET = {};

 public:
	//
	// Tags
	//

	using coord_type  = float;
	using length_type = double;
	using depth_type  = std::uint32_t;

	using Index       = TreeIndex;
	using Node        = TreeNode<Dim>;
	using Code        = TreeCode<Dim>;
	using Key         = TreeKey<Dim>;
	using Coord       = TreeCoord<Dim, coord_type>;
	using Point       = Vec<Dim, coord_type>;
	using Length      = Vec<Dim, length_type>;
	using Bounds      = AABB<Dim, coord_type>;
	using Ray         = ufo::Ray<Dim, coord_type>;
	using TraceResult = ufo::TraceResult<Dim>;

	using pos_type    = typename Index::pos_type;
	using offset_type = typename Index::offset_type;
	using code_type   = typename Code::code_type;
	using key_type    = typename Key::key_type;

	// Iterators

	using const_iterator = TreeIterator<Derived>;

	template <class Predicate>
	using const_query_iterator_pred = TreeQueryIterator<Derived, Predicate>;
	using const_query_iterator      = TreeQueryIterator<Derived>;

	template <class Geometry>
	using const_nearest_iterator_geom = TreeNearestIterator<Derived, Geometry>;
	using const_nearest_iterator      = TreeNearestIterator<Derived>;

	template <class Predicate, class Geometry>
	using const_query_nearest_iterator_pred_geom =
	    TreeQueryNearestIterator<Derived, Predicate, Geometry>;
	using const_query_nearest_iterator = TreeQueryNearestIterator<Derived>;

	template <class Predicate>
	using ConstQuery =
	    IteratorWrapper<const_query_iterator_pred<Predicate>, const_query_iterator>;

	template <class Geometry>
	using ConstNearest =
	    IteratorWrapper<const_nearest_iterator_geom<Geometry>, const_nearest_iterator>;

	template <class Predicate, class Geometry>
	using ConstQueryNearest =
	    IteratorWrapper<const_query_nearest_iterator_pred_geom<Predicate, Geometry>,
	                    const_query_nearest_iterator>;

	template <class T>
	struct is_node_type
	    : std::disjunction<
	          contains_convertible_type<remove_cvref_t<T>, Index, Node, Code, Key, Coord>,
	          std::is_constructible<remove_cvref_t<T>, Coord>> {
	};

	template <class T>
	static constexpr inline bool const is_node_type_v = is_node_type<T>::value;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tree                                         |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Returns the branching factor of the tree (i.e., 2 = binary tree, 4 = quadtree,
	 * 8 = octree, 16 = hextree).
	 *
	 * @return The branching factor of the tree.
	 */
	[[nodiscard]] static constexpr offset_type branchingFactor() noexcept { return BF; }

	/**
	 * @brief Returns the number of dimensions of the tree (i.e., 1 = binary tree, 2 =
	 * quadtree, 3 = octree, 4 = hextree).
	 *
	 * @return The number of dimensions of the tree.
	 */
	[[nodiscard]] static constexpr std::size_t dimensions() noexcept { return Dim; }

	/**
	 * @brief Returns the number of nodes in the tree.
	 *
	 * @return The number of nodes in the tree.
	 */
	[[nodiscard]] std::size_t size() const
	{
		// Num. blocks * branching factor - (root block only has one usable node, remove rest)
		return Data::size() * BF - (BF - 1);
	}

	/**
	 * @brief Increase the capacity of the tree to at least hold `num_nodes` nodes.
	 *
	 * @param num_nodes The new capacity.
	 */
	void reserve(std::size_t num_nodes) { Data::reserve((num_nodes + BF - 1) / BF); }

	/**
	 * @brief Erases all nodes from the tree.
	 */
	void clear()
	{
		Data::clear();
		createRoot();
		derived().onInitRoot();
	}

	void clear(Length const& leaf_node_length, depth_type num_depth_levels)
	{
		init(leaf_node_length, num_depth_levels);
		clear();
	}

	void clear(length_type const& leaf_node_length, depth_type num_depth_levels)
	{
		clear(Length(leaf_node_length), num_depth_levels);
	}

	//
	// Depth
	//

	/**
	 * @brief Returns the number of depth levels of the tree, i.e. `depth() + 1`.
	 *
	 * @return The number of depth levels of the tree.
	 */
	[[nodiscard]] constexpr depth_type numDepthLevels() const noexcept
	{
		return num_depth_levels_;
	}

	/**
	 * @brief Returns the minimum number of depth levels a tree must have.
	 *
	 * @return The minimum number of depth levels a tree must have.
	 */
	[[nodiscard]] static constexpr depth_type minNumDepthLevels() noexcept { return 2; }

	/**
	 * @brief Returns the maximum number of depth levels a tree can have.
	 *
	 * @return The maximum number of depth levels a tree can have.
	 */
	[[nodiscard]] static constexpr depth_type maxNumDepthLevels() noexcept
	{
		return Code::maxDepth() + 1;
	}

	/**
	 * @brief Returns the depth of the root node, i.e. `numDepthLevels() - 1`.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @return The depth of the root node.
	 */
	[[nodiscard]] depth_type depth() const { return numDepthLevels() - 1; }

	/**
	 * @brief Returns the depth of the block.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param block the block
	 * @return The depth of the block.
	 */
	[[nodiscard]] depth_type depth(pos_type block) const
	{
		return static_cast<depth_type>(isPureLeaf(block) ? 0u : treeInnerBlock(block).depth);
	}

	/**
	 * @brief Returns the depth of the node.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param node the node
	 * @return The depth of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr depth_type depth(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return depth(node.pos);
		} else if constexpr (std::is_same_v<T, Node>) {
			return depth(node.code);
		} else if constexpr (std::is_same_v<T, Code>) {
			return node.depth();
		} else if constexpr (std::is_same_v<T, Key>) {
			return node.depth();
		} else if constexpr (std::is_same_v<T, Coord>) {
			return node.depth;
		} else {
			return depth(convert(node));
		}
	}

	//
	// Length
	//

	/**
	 * @brief Returns the length of the tree (/ root node), i.e. `leaf_node_length *
	 * 2^depth()`.
	 *
	 * @return The length of the tree (/ root node).
	 */
	[[nodiscard]] Length length() const { return length(depth()); }

	/**
	 * @brief Returns the length of nodes at `depth`, i.e. `leaf_node_length *
	 * 2^depth`.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param depth the depth
	 * @return The length of nodes at `depth`.
	 */
	[[nodiscard]] Length length(depth_type depth) const
	{
		assert(numDepthLevels() > depth);
		return node_half_length_[depth + 1];
	}

	/**
	 * @brief Returns the length of `node`, i.e. `leaf_node_length * 2^depth(node)`.
	 *
	 * @param node the node
	 * @return The length of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Length length(NodeType node) const
	{
		return length(depth(node));
	}

	/**
	 * @brief Returns the half length of the tree (/ root node), i.e. `length() / 2`.
	 *
	 * @note The half length is often used, therefore this function exists for improved
	 * performance and precision.
	 *
	 * @return The half length of the tree (/ root node).
	 */
	[[nodiscard]] Length halfLength() const { return halfLength(depth()); }

	/**
	 * @brief Returns the half length of nodes at `depth`, i.e. `length(depth) / 2`.
	 *
	 * @note The half length is often used, therefore this function exists for improved
	 * performance and precision.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param depth the depth
	 * @return The half length of nodes at `depth`.
	 */
	[[nodiscard]] Length halfLength(depth_type depth) const
	{
		assert(numDepthLevels() > depth);
		return node_half_length_[depth];
	}

	/**
	 * @brief Returns the half length of `node`, i.e. `length(node) / 2`.
	 *
	 * @note The half length is often used, therefore this function exists for improved
	 * performance and precision.
	 *
	 * @param node the node
	 * @return The half length of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr Length halfLength(NodeType node) const
	{
		return halfLength(depth(node));
	}

	/**
	 * @brief Returns the reciprocal of the length of the tree (/ root node), i.e. `1 /
	 * length()`.
	 *
	 * @note The reciprocal of the length is often used, therefore this function exists for
	 * improved performance and precision.
	 *
	 * @return The reciprocal of the length of the tree (/ root node).
	 */
	[[nodiscard]] Length lengthReciprocal() const { return lengthReciprocal(depth()); }

	/**
	 * @brief Returns the reciprocal of the length of nodes at `depth`, i.e. `1 /
	 * length(depth)`.
	 *
	 * @note The reciprocal of the length is often used, therefore this function exists for
	 * improved performance and precision.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param depth the depth
	 * @return The reciprocal of the length of nodes at `depth`.
	 */
	[[nodiscard]] Length lengthReciprocal(depth_type depth) const
	{
		assert(numDepthLevels() > depth + 1);
		return node_half_length_reciprocal_[depth + 1];
	}

	/**
	 * @brief Returns the reciprocal of the length of `node`, i.e. `1 / length(node)`.
	 *
	 * @note The reciprocal of the length is often used, therefore this function exists for
	 * improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the length of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Length lengthReciprocal(NodeType node) const
	{
		return lengthReciprocal(depth(node));
	}

	/**
	 * @brief Returns the reciprocal of the half length of the tree (/ root node), i.e. `1 /
	 * (length() / 2) = 2 / length()`.
	 *
	 * @note The reciprocal of the half length is often used, therefore this function exists
	 * for improved performance and precision.
	 *
	 * @return The reciprocal of the half length of the tree (/ root node).
	 */
	[[nodiscard]] Length halfLengthReciprocal() const
	{
		return halfLengthReciprocal(depth());
	}

	/**
	 * @brief Returns the reciprocal of the half length of nodes at `depth`, i.e. `1 /
	 * (length(depth) / 2) = 2 / length(depth)`.
	 *
	 * @note The reciprocal of the half length is often used, therefore this function exists
	 * for improved performance and precision.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param depth the depth
	 * @return The reciprocal of the half length of nodes at `depth`.
	 */
	[[nodiscard]] Length halfLengthReciprocal(depth_type depth) const
	{
		assert(numDepthLevels() > depth);
		return node_half_length_reciprocal_[depth];
	}

	/**
	 * @brief Returns the reciprocal of the half length of `node`, i.e. `1 / (length(node) /
	 * 2) = 2 / length(node)`.
	 *
	 * @note The reciprocal of the half length is often used, therefore this function exists
	 * for improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the half length of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Length halfLengthReciprocal(NodeType node) const
	{
		return halfLengthReciprocal(depth(node));
	}

	//
	// Min/max/bounds
	//

	/**
	 * @brief Returns the minimum point of the tree (/ root node).
	 *
	 * @return The minimum point of the tree (/ root node).
	 */
	[[nodiscard]] Point min() const { return center() - cast<coord_type>(halfLength()); }

	/**
	 * @brief Returns the minimum point of `node`.
	 *
	 * @param node the node
	 * @return The minimum point of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Point min(NodeType node) const
	{
		return center(node) - cast<coord_type>(halfLength(node));
	}

	/**
	 * @brief Returns the maximum point of the tree (/ root node).
	 *
	 * @return The maximum point of the tree (/ root node).
	 */
	[[nodiscard]] Point max() const
	{
		Point c = center();
		Point p = c + cast<coord_type>(halfLength());
		// NOTE: Decrease with minimum step so `isInside` is consitent with this.
		for (std::size_t i{}; p.size() > i; ++i) {
			p[i] = std::nextafter(p[i], c[i]);
		}
		return c;
	}

	/**
	 * @brief Returns the maximum point of `node`.
	 *
	 * @param node the node
	 * @return The maximum point of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Point max(NodeType node) const
	{
		Point c = center(node);
		Point p = c + cast<coord_type>(halfLength(node));
		// NOTE: Decrease with minimum step so `isInside` is consitent with this.
		for (std::size_t i{}; p.size() > i; ++i) {
			p[i] = std::nextafter(p[i], c[i]);
		}
		return c;
	}

	/**
	 * @brief Returns the bounds of the tree (/ root node).
	 *
	 * @return The bounds of the tree (/ root node).
	 */
	[[nodiscard]] Bounds bounds() const
	{
		Point c   = center();
		auto  hl  = cast<coord_type>(halfLength());
		auto  min = c - hl;
		auto  max = c + hl;
		// NOTE: Decrease max with minimum step so `isInside` is consitent with this.
		for (std::size_t i{}; max.size() > i; ++i) {
			max[i] = std::nextafter(max[i], c[i]);
		}
		return Bounds(min, max);
	}

	/**
	 * @brief Returns the bounds of `node`.
	 *
	 * @param node the node
	 * @return The bounds of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Bounds bounds(NodeType node) const
	{
		Point c   = center(node);
		auto  hl  = cast<coord_type>(halfLength(node));
		auto  min = c - hl;
		auto  max = c + hl;
		// NOTE: Decrease max with minimum step so `isInside` is consitent with this.
		for (std::size_t i{}; max.size() > i; ++i) {
			max[i] = std::nextafter(max[i], c[i]);
		}
		return Bounds(min, max);
	}

	//
	// Inside
	//

	/**
	 * @brief Checks if a coordinate is inside the tree bounds, i.e. inside `bounds()`.
	 *
	 * @param coord the coordinate
	 * @return `true` if the coordinate is inside the bounds, `false` otherwise.
	 */
	[[nodiscard]] bool isInside(Point coord) const
	{
		// NOTE: This assumes that origin is at zero
		auto const hl = halfLength();
		for (std::size_t i{}; coord.size() > i; ++i) {
			if (-hl[i] > coord[i] || hl[i] <= coord[i]) {
				return false;
			}
		}
		return true;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Center
	//

	/**
	 * @brief Returns the center of the tree (/ root node).
	 *
	 * @return The center of the tree (/ root node).
	 */
	[[nodiscard]] Coord center() const { return Coord(Point(), depth()); }

	/**
	 * @brief Returns the center of `node`.
	 *
	 * @param node the node
	 * @return The center of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr Coord center(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Key>) {
			assert(valid(node));

			auto node_depth = depth(node);

			if (depth() == node_depth) {
				return center();
			}

			// LOOKAT: Check performance, might be a lot faster to have float here and in rest
			// of method
			Length            l = length(node_depth);
			std::int_fast64_t hmv =
			    static_cast<std::int_fast64_t>(half_max_value_ >> node_depth);

			Point coord =
			    cast<coord_type>((cast<length_type>(cast<std::int_fast64_t>(node) - hmv) +
			                      static_cast<length_type>(0.5)) *
			                     l);

			return Coord(coord, node_depth);
		} else {
			return center(key(node));
		}
	}

	/**
	 * @brief Returns the center of `node` if the node is valid (i.e., `valid(node)`).
	 *
	 * @param node the node
	 * @return The center of the node if the node is valid, null otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::optional<Coord> centerChecked(NodeType node) const
	{
		return valid(node) ? std::optional<Coord>(center(node)) : std::nullopt;
	}

	//
	// Center axis
	//

	/**
	 * @brief Returns the center of the tree (/ root node) for the `axis` specified.
	 *
	 * @param axis the axis
	 * @return The center of the tree (/ root node) for the `axis` specified.
	 */
	[[nodiscard]] coord_type centerAxis(std::size_t axis) const
	{
		assert(Dim > axis);
		return center()[axis];
	}

	/**
	 * @brief Returns the center of `node` for the `axis` specified.
	 *
	 * @param node the node
	 * @param axis the axis
	 * @return The center of the node for the `axis` specified.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] coord_type centerAxis(NodeType node, std::size_t axis) const
	{
		assert(Dim > axis);
		return center(node)[axis];
	}

	/**
	 * @brief Returns the center of `node` for the `axis` specified, if the node is valid
	 * (i.e., `valid(node)`).
	 *
	 * @param node the node
	 * @param axis the axis
	 * @return The center of the node for the `axis` specified if the node is valid, null
	 * otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::optional<coord_type> centerAxisChecked(NodeType    node,
	                                                          std::size_t axis) const
	{
		assert(Dim > axis);
		return valid(node) ? std::optional<coord_type>(centerAxis(node, axis)) : std::nullopt;
	}

	//
	// Block
	//

	/**
	 * @brief Returns the block position of the root node.
	 *
	 * @return The block position of the root node.
	 */
	[[nodiscard]] constexpr pos_type block() const noexcept
	{
		return Data::addInnerType(0);
	}

	/**
	 * @brief Returns the block position of `node`.
	 *
	 * @param node the node
	 * @return The block position of the node.
	 */

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] pos_type block(NodeType node) const
	{
		return index(node).pos;
	}

	//
	// Offset
	//

	/**
	 * @brief Returns the offset of the root node.
	 *
	 * @return The offset of the root node.
	 */
	[[nodiscard]] constexpr offset_type offset() const noexcept { return 0; }

	/**
	 * @brief Returns the offset of `node`.
	 *
	 * @param node the node
	 * @return The offset of the node.
	 */

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] offset_type offset(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return node.offset;
		} else if constexpr (std::is_same_v<T, Node>) {
			return offset(code(node));
		} else if constexpr (contains_type_v<T, Code, Key>) {
			return node.offset();
		} else {
			return offset(key(node));
		}
	}

	//
	// Index
	//

	/**
	 * @brief Returns the index of the root node.
	 *
	 * @return The index of the root node.
	 */
	[[nodiscard]] constexpr Index index() const noexcept
	{
		return Index(block(), offset());
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr Index index(NodeType node) const
	{
		assert(valid(node));

		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return node;
		} else if constexpr (std::is_same_v<T, Code>) {
			Index            n         = index();
			depth_type const min_depth = depth(node);
			for (depth_type d = depth(); min_depth < d; --d) {
				Index c = child(n, node.offset(d - 1));
				if (!valid(c.pos)) {
					return n;
				}
				n = c;
			}
			return n;
		} else if constexpr (contains_type_v<T, Node, Key, Coord>) {
			return index(code(node));
		} else {
			return index(convert(node));
		}
	}

	//
	// Node
	//

	/**
	 * @brief Returns the root node.
	 *
	 * @return The root node.
	 */
	[[nodiscard]] Node node() const { return Node(code(), index()); }

	/**
	 * @brief Returns the node corresponding to `node`.
	 *
	 * @param node the node
	 * @return The node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Node node(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return Node(code(node), node);
		} else if constexpr (std::is_same_v<T, Node>) {
			return Node(node.code, index(node));
		} else if constexpr (std::is_same_v<T, Code>) {
			return Node(node, index(node));
		} else if constexpr (contains_type_v<T, Key, Coord>) {
			return this->node(code(node));
		} else {
			return this->node(convert(node));
		}
	}

	/**
	 * @brief Get the node corresponding to a code.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param node The node.
	 * @return The node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Node operator[](NodeType node) const
	{
		return this->node(node);
	}

	//
	// Code
	//

	[[nodiscard]] Code code() const { return Code(std::array<code_type, 3>{}, depth()); }

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Code code(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			assert(valid(node));
			depth_type d = depth(node);

			Code ret;
			ret.setDepth(d);

			do {
				ret.setOffset(d, node.offset);
				node = parent(node);
				++d;
			} while (valid(node.pos));
			return ret;
		} else if constexpr (std::is_same_v<T, Node>) {
			return node.code;
		} else if constexpr (std::is_same_v<T, Code>) {
			return node;
		} else if constexpr (std::is_same_v<T, Key>) {
			return Code(node);
		} else if constexpr (std::is_same_v<T, Coord>) {
			return code(key(node));
		} else {
			return code(convert(node));
		}
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::optional<Code> codeChecked(NodeType node) const
	{
		return valid(node) ? std::optional<Code>(Code(node)) : std::nullopt;
	}

	//
	// Key
	//

	[[nodiscard]] Key key() const { return Key(Vec<Dim, key_type>(0), depth()); }

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Key key(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Code>) {
			return Key(node);
		} else if constexpr (std::is_same_v<T, Key>) {
			return node;
		} else if constexpr (std::is_same_v<T, Coord>) {
			assert(valid(node));

			auto  d = depth(node);
			Point p = node;

			// LOOKAT: Check performance, might be a lot faster to have float here
			Length lr = lengthReciprocal(0);

			auto k = cast<key_type>(
			             cast<std::make_signed_t<key_type>>(floor(cast<length_type>(p) * lr))) +
			         half_max_value_;

			return Key(k >> d, d);
		} else if constexpr (contains_type_v<T, Index, Node>) {
			return key(code(node));
		} else {
			return key(convert(node));
		}
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::optional<Key> keyChecked(NodeType node) const
	{
		return valid(node) ? std::optional<Key>(key(node)) : std::nullopt;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Check if the root of the tree is modified.
	 *
	 * @return Whether the root of the tree is in a modified state.
	 */
	[[nodiscard]] bool modified() const { return modified(index()); }

	/**
	 * @brief Check if a node of the tree is in a modified state.
	 *
	 * @param node The node to check.
	 * @return Whether the node is in a modified state.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool modified(NodeType node) const
	{
		Index n = index(node);
		return modified(n.pos, n.offset);
	}

	void modifiedSet(bool value) { return value ? modifiedSet() : modifiedReset(); }

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void modifiedSet(NodeType node, bool value)
	{
		return value ? modifiedSet(node) : modifiedReset(node);
	}

	void modifiedSet() { modifiedSet(index()); }

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void modifiedSet(NodeType node)
	{
		// NOTE: `create` sets ancestors to modified
		auto n = create(node);
		modifiedSet(n.pos, n.offset);
		if (isParent(n)) {
			modifiedSetChildren(n);
		}
	}

	void modifiedReset() { modifiedReset(index()); }

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void modifiedReset(NodeType node)
	{
		if (!exists(node)) {
			return;
		}

		auto n = index(node);
		modifiedReset(n.pos, n.offset);
		if (isParent(n)) {
			modifiedResetChildren(n);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Touch                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add proper guards for the templates

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	Index create(NodeType node)
	{
		assert(valid(node));

		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			modifiedSetParents(node);
			return node;
		} else {
			Code code         = this->code(node);
			auto wanted_depth = depth(code);
			auto cur_node     = index();
			auto cur_depth    = depth();
			while (wanted_depth < cur_depth) {
				cur_node = createChild(cur_node, code.offset(--cur_depth));
			}
			return cur_node;
		}
	}

	template <class InputIt, class OutputIt>
	OutputIt create(InputIt first, InputIt last, OutputIt d_first)
	{
		using T = remove_cvref_t<typename std::iterator_traits<InputIt>::value_type>;

		if constexpr (std::is_same_v<T, Index>) {
			return std::transform(first, last, d_first,
			                      [this](Index node) { return create(node); });
		} else {
			Index node      = this->index();
			Code  node_code = this->code();

			return std::transform(first, last, d_first,
			                      [this, &node, &node_code](auto const& x) {
				                      Code       d_code = code(x);
				                      depth_type d = Code::depthWhereEqual(node_code, d_code);

				                      node      = ancestor(node, d);
				                      node_code = d_code;
				                      for (depth_type d_depth = depth(d_code); d_depth < d; --d) {
					                      node = createChild(node, d_code.offset(d - 1));
				                      }

				                      return node;
			                      });
		}
	}

	template <class InputIt>
	std::vector<Index> create(InputIt first, InputIt last)
	{
		std::vector<Index> nodes;
		create(first, last, std::back_inserter(nodes));
		return nodes;
	}

	template <
	    class Range, class OutputIt,
	    std::enable_if_t<!is_node_type_v<Range> && !execution::is_execution_policy_v<Range>,
	                     bool> = true>
	OutputIt create(Range const& r, OutputIt d_first)
	{
		using std::begin;
		using std::end;
		return create(begin(r), end(r), d_first);
	}

	template <class Range, std::enable_if_t<!is_node_type_v<Range>, bool> = true>
	std::vector<Index> create(Range const& r)
	{
		std::vector<Index> nodes;
		create(r, std::back_inserter(nodes));
		return nodes;
	}

	// TODO: Continue from here

	template <
	    class ExecutionPolicy, class RandomIt1, class RandomIt2,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	RandomIt2 create(ExecutionPolicy&& policy, RandomIt1 first, RandomIt1 last,
	                 RandomIt2 d_first)
	{
		using T = remove_cvref_t<typename std::iterator_traits<RandomIt1>::value_type>;

		if constexpr (std::is_same_v<T, Index>) {
			return ufo::transform(std::forward<ExecutionPolicy>(policy), first, last, d_first,
			                      [this](Index node) {
				                      modifiedSetParentsThreadSafe(node);
				                      return node;
			                      });
		} else {
			// NOTE: Possible (although, highly unlikely) problem. If this function is called
			// more than max value of `std::size_t`, so `create_call_num` overflows, *AND* a
			// thread has persisted but not been used for a multiple of max value of
			// `std::size_t` iterations in the `transform` call below; then `node` and
			// `node_code` would not be reset to the root node as they should. This means that
			// invalid memory is being accessed.
			static std::size_t create_call_num{};
			++create_call_num;

			return transform(std::forward<ExecutionPolicy>(policy), first, last, d_first,
			                 [this, ccn = create_call_num](auto const& x) {
				                 thread_local Index node      = index();
				                 thread_local Code  node_code = code();

				                 thread_local std::size_t thread_create_call_num = 1;
				                 if (ccn != thread_create_call_num) {
					                 thread_create_call_num = ccn;
					                 node                   = index();
					                 node_code              = code();
				                 }

				                 Code       d_code = code(x);
				                 depth_type d      = Code::depthWhereEqual(node_code, d_code);

				                 node      = ancestor(node, d);
				                 node_code = d_code;
				                 for (depth_type d_depth = depth(d_code); d_depth < d; --d) {
					                 node = createChildThreadSafe(node, d_code.offset(d - 1));
				                 }

				                 return node;
			                 });
		}
	}

	template <
	    class ExecutionPolicy, class RandomIt,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	std::vector<Index> create(ExecutionPolicy&& policy, RandomIt first, RandomIt last)
	{
		__block std::vector<Index> nodes(std::distance(first, last));
		create(std::forward<ExecutionPolicy>(policy), first, last, nodes.begin());
		return nodes;
	}

	template <
	    class ExecutionPolicy, class Range, class RandomIt,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<!is_node_type_v<Range>, bool>                            = true>
	RandomIt create(ExecutionPolicy&& policy, Range const& r, RandomIt d_first)
	{
		using std::begin;
		using std::end;
		return create(std::forward<ExecutionPolicy>(policy), begin(r), end(r), d_first);
	}

	template <
	    class ExecutionPolicy, class Range,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<!is_node_type_v<Range>, bool>                            = true>
	std::vector<Index> create(ExecutionPolicy&& policy, Range const& r)
	{
		using std::size;
		__block std::vector<Index> nodes(size(r));
		create(std::forward<ExecutionPolicy>(policy), r, nodes.begin());
		return nodes;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Erase                                        |
	|                                                                                     |
	**************************************************************************************/

	void eraseChildren() { eraseChildren(index()); }

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void eraseChildren(NodeType node)
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			if (isLeaf(node)) {
				return;
			}

			auto c = children(node);
			for (offset_type i{}; BF > i; ++i) {
				eraseChildren(Index(c, i));
			}

			pruneChildren(node, c);
		} else if constexpr (contains_type_v<T, Node, Code>) {
			if (!exists(node)) {
				return;
			}

			eraseChildren(index(node));
		} else if constexpr (contains_type_v<T, Key, Coord>) {
			eraseChildren(code(node));
		} else {
			eraseChildren(convert(node));
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Pure leaf
	//

	/**
	 * @brief Checks if the block is pure leaf (i.e., can never have children).
	 *
	 * @note Only have to check if the depth of the block is 0.
	 *
	 * @param block the block to check
	 * @return `true` if the block is pure leaf, `false` otherwise.
	 */
	[[nodiscard]] bool isPureLeaf(pos_type block) const
	{
		assert(valid(block));
		return Data::leaf(block);
	}

	/**
	 * @brief Checks if the node is a pure leaf (i.e., can never have children).
	 *
	 * @note Only have to check if the depth of the node is 0.
	 *
	 * @param node the node to check
	 * @return `true` if the node is a pure leaf, `false` otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool isPureLeaf(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return isPureLeaf(block(node));
		} else {
			return 0u == depth(node);
		}
	}

	//
	// Leaf
	//

	/**
	 * @brief Checks if the node is a leaf (i.e., has no children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a leaf, `false` otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool isLeaf(NodeType node) const
	{
		return isPureLeaf(node) || !valid(children(index(node)));
	}

	//
	// Parent
	//

	/**
	 * @brief Checks if the node is a parent (i.e., has children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a parent, `false` otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool isParent(NodeType node) const
	{
		return !isLeaf(node);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Root                                         |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr bool isRoot(pos_type block) const
	{
		return this->block() == block;
	}

	/**
	 * @brief Checks if the node is the root of the tree.
	 *
	 * @param node the node to check
	 * @return `true` if the node is the root, `false` otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr bool isRoot(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return index() == node;
		} else if constexpr (std::is_same_v<T, Node>) {
			return isRoot(node.code);
		} else if constexpr (std::is_same_v<T, Code>) {
			return code() == node;
		} else if constexpr (std::is_same_v<T, Key>) {
			return key() == node;
		} else if constexpr (std::is_same_v<T, Coord>) {
			return isRoot(key(node));
		} else {
			return isRoot(convert(node));
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Valid                                        |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Checks if a block is valid.
	 *
	 * @param block the block to check
	 * @return `true` if the block is valid, `false` otherwise.
	 */
	[[nodiscard]] bool valid(pos_type block) const { return Index::MAX_VALID_POS >= block; }

	/**
	 * @brief Checks if an index is valid.
	 *
	 * @param index the index to check
	 * @return `true` if the index is valid, `false` otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool valid(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return valid(node.pos) && branchingFactor() > node.offset;
		} else if constexpr (std::is_same_v<T, Node>) {
			return valid(code(node));
		} else if constexpr (std::is_same_v<T, Code>) {
			return node.valid() && numDepthLevels() > depth(node);
		} else if constexpr (std::is_same_v<T, Key>) {
			auto const mv = (2 * half_max_value_) >> depth(node);
			for (std::size_t i{}; node.size() != i; ++i) {
				if (mv < node[i]) {
					return false;
				}
			}

			return node.valid() && numDepthLevels() > depth(node);
		} else if constexpr (std::is_same_v<T, Coord>) {
			return isInside(static_cast<Point>(node)) && numDepthLevels() > depth(node);
		} else {
			return valid(convert(node));
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Exist                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] bool exists(pos_type block) const
	{
		return Data::exists(block) && Index::INVALID_POS != parent(block).pos;
	}

	/**
	 * @brief Checks if a node exists.
	 *
	 * @param node the node to check
	 * @return `true` if the node exists, `false` otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool exists(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return exists(node.pos);
		} else if constexpr (contains_type_v<T, Node, Code, Key, Coord>) {
			return depth(index(node)) == depth(node);
		} else {
			return exists(convert(node));
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Traverse                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] std::array<pos_type, BF> const& children(pos_type block) const
	{
		assert(!isPureLeaf(block));
		return treeInnerBlock(block).children;
	}

	[[nodiscard]] pos_type children(Index node) const
	{
		return children(node.pos)[node.offset];
	}

	/**
	 * @brief Returns the `i`:th child of `node`.
	 *
	 * @param node the node to return the child of.
	 * @param offset the offset of the child (in range `[0..2^Dim)`).
	 * @return `i`:th child of `node`.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr auto child(NodeType node, offset_type offset) const
	{
		assert(!isPureLeaf(node));
		assert(BF > offset);

		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return Index(children(node), offset);
		} else if constexpr (std::is_same_v<T, Node>) {
			return Node(child(node.code, offset), node.index);
		} else if constexpr (contains_type_v<T, Code, Key>) {
			return node.child(offset);
		} else if constexpr (std::is_same_v<T, Coord>) {
			auto center      = static_cast<Point>(node);
			auto half_length = halfLength(node);
			auto child_depth = node.depth - depth_type(1);
			return Coord(childCenter(center, half_length, offset), child_depth);
		} else {
			return child(convert(node), offset);
		}
	}

	/**
	 * @brief Get a child of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param offset The offset of the child.
	 * @return The child.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] auto childChecked(NodeType node, offset_type offset) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return isParent(node) && BF > offset ? std::optional(child(node, offset))
			                                     : std::nullopt;
		} else if constexpr (contains_type_v<T, Node, Code, Key, Coord>) {
			return !isPureLeaf(node) && BF > offset ? std::optional(child(node, offset))
			                                        : std::nullopt;
		} else {
			return childChecked(convert(node), offset);
		}
	}

	//
	// Sibling
	//

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] auto sibling(NodeType node, offset_type offset) const
	{
		assert(!isRoot(node));
		assert(BF > offset);

		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return Index(node.pos, offset);
		} else if constexpr (std::is_same_v<T, Node>) {
			return Node(sibling(node.code, offset), node.index);
		} else if constexpr (contains_type_v<T, Code, Key>) {
			return node.sibling(offset);
		} else if constexpr (std::is_same_v<T, Coord>) {
			return center(sibling(key(node), offset));
		} else {
			return sibling(convert(node), offset);
		}
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] auto siblingChecked(NodeType node, offset_type offset) const
	{
		return !isRoot(node) && BF > offset ? std::optional(sibling(node, offset))
		                                    : std::nullopt;
	}

	//
	// Parent
	//

	[[nodiscard]] Index parent(pos_type block) const
	{
		assert(!isRoot(block));
		return isPureLeaf(block) ? parent(treeLeafBlock(block))
		                         : parent(treeInnerBlock(block));
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] auto parent(NodeType node) const
	{
		assert(!isRoot(node));

		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return parent(node.pos);
		} else if constexpr (std::is_same_v<T, Node>) {
			return Node(parent(node.code), node.index);
		} else if constexpr (contains_type_v<T, Code, Key>) {
			return node.parent();
		} else if constexpr (std::is_same_v<T, Coord>) {
			return center(parent(key(node)));
		} else {
			return parent(convert(node));
		}
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] auto parentChecked(NodeType node) const
	{
		return !isRoot(node) ? std::optional(parent(node)) : std::nullopt;
	}

	//
	// Ancestor
	//

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] auto ancestor(NodeType node, depth_type depth) const
	{
		assert(!isRoot(node) || this->depth(node) == depth);
		assert(this->depth(node) <= depth);
		assert(this->depth() >= depth);

		depth_type cur_depth = this->depth(node);

		if (cur_depth >= depth) {
			return node;
		}

		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			node = 0 == cur_depth ? parent(treeLeafBlock(node.pos))
			                      : parent(treeInnerBlock(node.pos));

			for (++cur_depth; cur_depth < depth; ++cur_depth) {
				node = parent(treeInnerBlock(node.pos));
			}

			return node;
		} else if constexpr (std::is_same_v<T, Node>) {
			return Node(ancestor(node.code, depth), node.index);
		} else if constexpr (contains_type_v<T, Code, Key>) {
			return node.toDepth(depth);
		} else if constexpr (std::is_same_v<T, Coord>) {
			return center(ancestor(key(node), depth));
		} else {
			return ancestor(convert(node), depth);
		}
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] auto ancestorChecked(NodeType node, depth_type depth) const
	{
		return (!isRoot(node) || this->depth(node) == depth) && this->depth(node) <= depth &&
		               this->depth() >= depth
		           ? std::optional(ancestor(node, depth))
		           : std::nullopt;
	}

	/**
	 * @brief Depth first traversal of the tree, starting at the root node. The function
	 * 'f' will be called for each node traverse. If 'f' returns true then the children of
	 * the node will also be traverse, otherwise they will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Index>, bool> = true>
	void traverse(UnaryFun f) const
	{
		traverse(index(), f);
	}

	/**
	 * @brief Depth first traversal of the tree, starting at `node`. The function 'f'
	 * will be called for each traversed node. If 'f' returns `true` then the children of
	 * the node will also be traverse, otherwise they will not.
	 *
	 * @param node The node to start the traversal from.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class NodeType, class UnaryFun,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>                     = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Index>, bool> = true>
	void traverse(NodeType node, UnaryFun f) const
	{
		assert(valid(node));

		if (!exists(node)) {
			return;
		}

		Index          cur   = index(node);
		pos_type const start = cur.pos;

		while (f(cur) && isParent(cur)) {
			cur = child(cur, 0);
		}

		while (start != cur.pos) {
			if (BF - 1 <= cur.offset) {
				cur = parent(cur.pos);
			} else {
				++cur.offset;
				while (f(cur) && isParent(cur)) {
					cur = child(cur, 0);
				}
			}
		}
	}

	/**
	 * @brief Depth first traversal of the tree, starting at the root node. The function
	 * 'f' will be called for each node traverse. If 'f' returns true then the children of
	 * the node will also be traverse, otherwise they will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 * @param pred Predicates that need to be fulfilled.
	 * @param only_exists Whether only existing nodes should be traversed.
	 */
	template <class UnaryFun, class Predicate,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Node>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool>                  = true>
	void traverse(UnaryFun f, Predicate const& pred, bool only_exists = true) const
	{
		traverse(node(), f, pred, only_exists);
	}

	/**
	 * @brief Depth first traversal of the tree, starting at `node`. The function 'f'
	 * will be called for each traversed node that fulfills the predicates `pred`.
	 *
	 * @param node The node to start the traversal from.
	 * @param f The callback function to be called.
	 * @param pred Predicates that need to be fulfilled.
	 * @param only_exists Whether only existing nodes should be traversed.
	 */
	template <class NodeType, class UnaryFun, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>            = true,
	          std::enable_if_t<std::is_invocable_v<UnaryFun, Node>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool>          = true>
	void traverse(NodeType node, UnaryFun f, Predicate pred, bool only_exists = true) const
	{
		assert(valid(node));

		using Filter = pred::Filter<Predicate>;

		Filter::init(pred, derived());

		if (only_exists) {
			auto fun = [this, f, &pred](Node node) {
				if (Filter::returnable(pred, derived(), node)) {
					f(node);
				}
				return isParent(node.index) && Filter::traversable(pred, derived(), node);
			};

			if (!exists(node)) {
				return;
			}

			Node           cur   = this->node(node);
			pos_type const start = cur.index.pos;

			while (fun(cur)) {
				cur.code  = cur.code.firstBorn();
				cur.index = child(cur.index, 0);
			}

			while (start != cur.index.pos) {
				if (BF - 1 <= cur.index.offset) {
					cur.code  = cur.code.parent();
					cur.index = parent(cur.index.pos);
				} else {
					cur.code = cur.code.nextSibling();
					++cur.index.offset;
					while (fun(cur)) {
						cur.code  = cur.code.firstBorn();
						cur.index = child(cur.index, 0);
					}
				}
			}
		} else {
			auto fun = [this, f, &pred](Node node) {
				if (Filter::returnable(pred, derived(), node)) {
					f(node);
				}
				return !isPureLeaf(node.index) && Filter::traversable(pred, derived(), node);
			};

			Node             cur   = this->node(node);
			depth_type const start = cur.code.depth();

			while (fun(node)) {
				cur.code  = cur.code.firstborn();
				cur.index = isParent(cur.index) ? child(cur.index, 0) : cur.index;
			}

			while (start != cur.code.depth()) {
				if (BF - 1 <= cur.code.offset()) {
					cur.code = cur.code.parent();
					cur.index =
					    cur.code.depth() > depth(cur.index) ? parent(cur.index.pos) : cur.index;
				} else {
					cur.code = cur.code.nextSibling();
					cur.index.offset += cur.code.depth() == depth(cur.index) ? 1 : 0;
					while (fun(node)) {
						cur.code  = cur.code.firstborn();
						cur.index = isParent(cur.index) ? child(cur.index, 0) : cur.index;
					}
				}
			}
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Iterators                                      |
	|                                                                                     |
	**************************************************************************************/

	//
	// Iterator
	//

	[[nodiscard]] const_iterator begin(bool only_leaves = true,
	                                   bool only_exists = true) const
	{
		return begin(node(), only_leaves, only_exists);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] const_iterator begin(NodeType node, bool only_leaves = true,
	                                   bool only_exists = true) const
	{
		return const_iterator(&derived(), this->node(node), only_leaves, only_exists);
	}

	[[nodiscard]] const_iterator end() const { return const_iterator(); }

	//
	// Query iterator
	//

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] const_query_iterator_pred<Predicate> beginQuery(
	    Predicate const& pred, bool only_exists = true, bool early_stopping = false) const
	{
		return beginQuery(node(), pred, only_exists, early_stopping);
	}

	template <class NodeType, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>   = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] const_query_iterator_pred<Predicate> beginQuery(
	    NodeType node, Predicate const& pred, bool only_exists = true,
	    bool early_stopping = false) const
	{
		return const_query_iterator_pred<remove_cvref_t<Predicate>>(
		    &derived(), this->node(node), pred, only_exists, early_stopping);
	}

	[[nodiscard]] const_query_iterator endQuery() const { return const_query_iterator(); }

	//
	// Nearest iterator
	//

	template <class Geometry>
	[[nodiscard]] const_nearest_iterator_geom<Geometry> beginNearest(
	    Geometry const& geometry, double epsilon = 0.0, bool only_leaves = true,
	    bool only_exists = true) const
	{
		return beginNearest(node(), geometry, epsilon, only_leaves, only_exists);
	}

	template <class NodeType, class Geometry,
	          std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] const_nearest_iterator_geom<Geometry> beginNearest(
	    NodeType node, Geometry const& geometry, double epsilon = 0.0,
	    bool only_leaves = true, bool only_exists = true) const
	{
		return const_nearest_iterator_geom<Geometry>(&derived(), this->node(node), geometry,
		                                             epsilon, only_leaves, only_exists);
	}

	[[nodiscard]] const_nearest_iterator endNearest() const
	{
		return const_nearest_iterator();
	}

	//
	// Query nearest iterator
	//

	template <class Predicate, class Geometry,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] const_query_nearest_iterator_pred_geom<Predicate, Geometry>
	beginQueryNearest(Predicate const& pred, Geometry const& geometry, double epsilon = 0.0,
	                  bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(node(), pred, geometry, epsilon, only_exists,
		                         early_stopping);
	}

	template <class NodeType, class Predicate, class Geometry,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>   = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] const_query_nearest_iterator_pred_geom<Predicate, Geometry>
	beginQueryNearest(NodeType node, Predicate const& pred, Geometry const& geometry,
	                  double epsilon = 0.0, bool only_exists = true,
	                  bool early_stopping = false) const
	{
		return const_query_nearest_iterator_pred_geom<Predicate, Geometry>(
		    &derived(), this->node(node), pred, geometry, epsilon, only_exists,
		    early_stopping);
	}

	[[nodiscard]] const_query_nearest_iterator endQueryNearest() const
	{
		return const_query_nearest_iterator();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Query                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Query
	//

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] ConstQuery<Predicate> query(Predicate const& pred,
	                                          bool             only_exists    = true,
	                                          bool             early_stopping = false) const
	{
		return query(node(), pred, only_exists, early_stopping);
	}

	template <class NodeType, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>   = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] ConstQuery<Predicate> query(NodeType node, Predicate const& pred,
	                                          bool only_exists    = true,
	                                          bool early_stopping = false) const
	{
		return ConstQuery<Predicate>(beginQuery(node, pred, only_exists, early_stopping),
		                             endQuery());
	}

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] ConstQuery<Predicate> operator()(Predicate const& pred,
	                                               bool             only_exists = true,
	                                               bool early_stopping = false) const
	{
		return query(pred, only_exists, early_stopping);
	}

	template <class NodeType, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>   = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] ConstQuery<Predicate> operator()(NodeType node, Predicate const& pred,
	                                               bool only_exists    = true,
	                                               bool early_stopping = false) const
	{
		return query(node, pred, only_exists, early_stopping);
	}

	//
	// Nearest
	//

	template <class Geometry>
	[[nodiscard]] ConstNearest<Geometry> nearest(Geometry const& geometry,
	                                             double          epsilon     = 0.0,
	                                             bool            only_leaves = true,
	                                             bool            only_exists = true) const
	{
		return nearest(node(), geometry, epsilon, only_leaves, only_exists);
	}

	template <class NodeType, class Geometry,
	          std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] ConstNearest<Geometry> nearest(NodeType node, Geometry const& geometry,
	                                             double epsilon     = 0.0,
	                                             bool   only_leaves = true,
	                                             bool   only_exists = true) const
	{
		return ConstNearest<Geometry>(
		    beginNearest(node, geometry, epsilon, only_leaves, only_exists), endNearest());
	}

	//
	// Query nearest
	//

	template <class Predicate, class Geometry,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] ConstQueryNearest<Predicate, Geometry> queryNearest(
	    Predicate const& pred, Geometry const& geometry, double epsilon = 0.0,
	    bool only_exists = true, bool early_stopping = false) const
	{
		return queryNearest(node(), pred, geometry, epsilon, only_exists, early_stopping);
	}

	template <class NodeType, class Predicate, class Geometry,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>   = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] ConstQueryNearest<Predicate, Geometry> queryNearest(
	    NodeType node, Predicate const& pred, Geometry const& geometry,
	    double epsilon = 0.0, bool only_exists = true, bool early_stopping = false) const
	{
		return ConstQueryNearest<Predicate, Geometry>(
		    beginQueryNearest(node, pred, geometry, epsilon, only_exists, early_stopping),
		    endQueryNearest());
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Trace                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add `only_exists`

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] TraceResult trace(Ray const& ray, Predicate const& pred,
	                                float min_dist    = 0.0f,
	                                float max_dist    = std::numeric_limits<float>::max(),
	                                bool  only_exists = true) const
	{
		return trace(node(), ray, pred, min_dist, max_dist, only_exists);
	}

	template <class NodeType, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>   = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] TraceResult trace(NodeType node, Ray const& ray, Predicate pred,
	                                float min_dist    = 0.0f,
	                                float max_dist    = std::numeric_limits<float>::max(),
	                                bool  only_exists = true) const
	{
		// TODO: Implement, also look at `only_exists`

		using Filter = pred::Filter<Predicate>;

		Filter::init(pred, derived());

		Node n = node(node);
		if (!exists(n)) {  // What if !only_exists?
			return TraceResult{Node(), -1.0f};
		}

		auto params = traceInit(n, ray);
		return trace(n, params, pred, min_dist, max_dist, only_exists);
	}

	template <class InputIt, class OutputIt, class Predicate,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	OutputIt trace(InputIt first, InputIt last, OutputIt d_first, Predicate const& pred,
	               float min_dist    = 0.0f,
	               float max_dist    = std::numeric_limits<float>::max(),
	               bool  only_exists = true) const
	{
		return trace(node(), first, last, d_first, pred, min_dist, max_dist, only_exists);
	}

	template <class NodeType, class InputIt, class OutputIt, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>   = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	OutputIt trace(NodeType node, InputIt first, InputIt last, OutputIt d_first,
	               Predicate const& pred, float min_dist = 0.0f,
	               float max_dist    = std::numeric_limits<float>::max(),
	               bool  only_exists = true) const
	{
		// TODO: Implement
	}

	template <class InputIt, class Predicate,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] std::vector<TraceResult> trace(
	    InputIt first, InputIt last, Predicate const& pred, float min_dist = 0.0f,
	    float max_dist = std::numeric_limits<float>::max(), bool only_exists = true) const
	{
		return trace(node(), first, last, pred, min_dist, max_dist, only_exists);
	}

	template <class NodeType, class InputIt, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>   = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] std::vector<TraceResult> trace(
	    NodeType node, InputIt first, InputIt last, Predicate const& pred,
	    float min_dist = 0.0f, float max_dist = std::numeric_limits<float>::max(),
	    bool only_exists = true) const
	{
		std::vector<TraceResult> res;
		trace(node, first, last, std::back_inserter(res), pred, min_dist, max_dist,
		      only_exists);
		return res;
	}

	template <
	    class ExecutionPolicy, class RandomIt1, class RandomIt2, class Predicate,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	RandomIt2 trace(ExecutionPolicy&& policy, RandomIt1 first, RandomIt1 last,
	                RandomIt2 d_first, Predicate const& pred, float min_dist = 0.0f,
	                float max_dist    = std::numeric_limits<float>::max(),
	                bool  only_exists = true) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), index(), first, last, d_first,
		             pred, min_dist, max_dist, only_exists);
	}

	template <
	    class ExecutionPolicy, class NodeType, class RandomIt1, class RandomIt2,
	    class Predicate, std::enable_if_t<is_node_type_v<NodeType>, bool> = true,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	RandomIt2 trace(ExecutionPolicy&& policy, NodeType node, RandomIt1 first,
	                RandomIt1 last, RandomIt2 d_first, Predicate pred,
	                float min_dist    = 0.0f,
	                float max_dist    = std::numeric_limits<float>::max(),
	                bool  only_exists = true) const
	{
		// TODO: Look at `only_exists`

		using Filter = pred::Filter<Predicate>;

		Filter::init(pred, derived());

		Node n = this->node(node);
		if (!exists(n)) {
			for (; last != first; ++first, ++d_first) {
				*d_first = TraceResult{Node(), -1.0f};
			}
			return d_first;
		}

		auto center      = this->center(n);
		auto half_length = halfLength(n);

		return transform(std::forward<ExecutionPolicy>(policy), first, last, d_first,
		                 [&](Ray const& ray) {
			                 auto params = traceInit(ray, center, half_length);
			                 return trace(n, params, pred, min_dist, max_dist, only_exists);
		                 });
	}

	template <
	    class ExecutionPolicy, class RandomIt, class Predicate,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	[[nodiscard]] std::vector<TraceResult> trace(
	    ExecutionPolicy&& policy, RandomIt first, RandomIt last, Predicate const& pred,
	    float min_dist = 0.0f, float max_dist = std::numeric_limits<float>::max(),
	    bool only_exists = true) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), index(), first, last, pred,
		             min_dist, max_dist, only_exists);
	}

	template <
	    class ExecutionPolicy, class NodeType, class RandomIt, class Predicate,
	    std::enable_if_t<is_node_type_v<NodeType>, bool>                          = true,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	[[nodiscard]] std::vector<TraceResult> trace(
	    ExecutionPolicy&& policy, NodeType node, RandomIt first, RandomIt last,
	    Predicate const& pred, float min_dist = 0.0f,
	    float max_dist = std::numeric_limits<float>::max(), bool only_exists = true) const
	{
		__block std::vector<TraceResult> res(std::distance(first, last));
		trace(std::forward<ExecutionPolicy>(policy), node, first, last, res.begin(), pred,
		      min_dist, max_dist, only_exists);
		return res;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Comparison                                      |
	|                                                                                     |
	**************************************************************************************/

	template <class Derived2, std::size_t Dim2, class... Blocks2>
	friend bool operator==(Tree<Derived2, Dim2, Blocks2...> const& lhs,
	                       Tree<Derived2, Dim2, Blocks2...> const& rhs);

	template <class Derived2, std::size_t Dim2, class... Blocks2>
	friend bool operator!=(Tree<Derived2, Dim2, Blocks2...> const& lhs,
	                       Tree<Derived2, Dim2, Blocks2...> const& rhs);

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	Tree(Length leaf_node_length, depth_type num_depth_levels)
	{
		init(leaf_node_length, num_depth_levels);
		createRoot();
	}

	Tree(length_type leaf_node_length, depth_type num_depth_levels)
	    : Tree(Length(leaf_node_length), num_depth_levels)
	{
	}

	Tree(Tree const&) = default;

	Tree(Tree&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~Tree() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	Tree& operator=(Tree const&) = default;

	Tree& operator=(Tree&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                        Init                                         |
	|                                                                                     |
	**************************************************************************************/

	void init(Length leaf_node_length, depth_type num_depth_levels)
	{
		if (minNumDepthLevels() > num_depth_levels ||
		    maxNumDepthLevels() < num_depth_levels) {
			throw std::invalid_argument("'num_depth_levels' has to be in range [" +
			                            std::to_string(+minNumDepthLevels()) + ".." +
			                            std::to_string(+maxNumDepthLevels()) + "], '" +
			                            std::to_string(+num_depth_levels) + "' was supplied.");
		}

		if (length_type(0) >= ufo::min(leaf_node_length) || !isfinite(leaf_node_length)) {
			std::stringstream ss;
			ss << leaf_node_length;
			throw std::invalid_argument(
			    "'leaf_node_length' has to be finite and greater than zero, '" + ss.str() +
			    "' was supplied.");
		}
		if (!isnormal(ldexp(leaf_node_length, num_depth_levels - 1))) {
			std::stringstream ss;
			ss << ldexp(leaf_node_length, num_depth_levels - 1);
			throw std::invalid_argument(
			    "'leaf_node_length * 2^(num_depth_levels - 1)' has to be finite and greater "
			    "than zero, '" +
			    ss.str() + "' was supplied.");
		}
		if (length_type(0) >= ufo::min(length_type(1) / ldexp(leaf_node_length, -1))) {
			std::stringstream ss;
			ss << (length_type(1) / ldexp(leaf_node_length, -1));
			throw std::invalid_argument(
			    "The reciprocal of half 'leaf_node_length' (i.e., 1 / (leaf_node_length / "
			    "2)) has to be a greater than zero, '" +
			    ss.str() + "' was supplied.");
		}

		num_depth_levels_ = num_depth_levels;
		half_max_value_   = key_type(1) << (num_depth_levels - 2);

		// For increased precision
		for (int i{}; node_half_length_.size() > i; ++i) {
			node_half_length_[i]            = ldexp(leaf_node_length, i - 1);
			node_half_length_reciprocal_[i] = length_type(1) / node_half_length_[i];
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Returns a reference of the derived class.
	 *
	 * @return A reference of the derived class.
	 */
	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	/**
	 * @brief Returns a reference of the derived class.
	 *
	 * @return A reference of the derived class.
	 */
	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Create root                                     |
	|                                                                                     |
	**************************************************************************************/

	void createRoot()
	{
		pos_type    p     = Data::innerCreate();
		InnerBlock& block = treeInnerBlock(p);
		block.children.fill(Index::NULL_POS);
		block.parent_block  = Index::NULL_POS;
		block.parent_offset = {};
		block.depth         = numDepthLevels() - 1;
		block.modified      = MODIFIED_NONE_SET;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] LeafBlock& treeLeafBlock(pos_type block)
	{
		assert(isPureLeaf(block));
		return Data::template leafBlock<LeafBlock>(block);
	}

	[[nodiscard]] LeafBlock const& treeLeafBlock(pos_type block) const
	{
		assert(isPureLeaf(block));
		return Data::template leafBlock<LeafBlock>(block);
	}

	[[nodiscard]] LeafBlock const& treeLeafBlockConst(pos_type block) const
	{
		assert(isPureLeaf(block));
		return treeLeafBlock(block);
	}

	[[nodiscard]] InnerBlock& treeInnerBlock(pos_type block)
	{
		assert(!isPureLeaf(block));
		return Data::template innerBlock<InnerBlock>(block);
	}

	[[nodiscard]] InnerBlock const& treeInnerBlock(pos_type block) const
	{
		assert(!isPureLeaf(block));
		return Data::template innerBlock<InnerBlock>(block);
	}

	[[nodiscard]] InnerBlock const& treeInnerBlockConst(pos_type block) const
	{
		assert(!isPureLeaf(block));
		return treeInnerBlock(block);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Recurs                                       |
	|                                                                                     |
	**************************************************************************************/

	template <class UpdateFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UpdateFun, Index, pos_type>,
	                           bool> = true>
	void recursUp(Index node, UpdateFun update_f)
	{
		assert(exists(node));

		offset_type children = node.pos;
		node                 = parent(node);
		// BENCH: Is it faster to use `depth` than `valid` here?
		while (valid(node.pos) && update_f(node, children)) {
			children = node.pos;
			node     = parent(treeInnerBlockConst(node.pos));
		}
	}

	template <class UpdateFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UpdateFun, Index, pos_type>,
	                           bool> = true>
	void recursDown(pos_type block, UpdateFun update_f)
	{
		assert(exists(block));

		if (isPureLeaf(block)) {
			return;
		}

		InnerBlock const& b = treeInnerBlockConst(block);
		for (offset_type i{}; BF > i; ++i) {
			if (pos_type c = children(b, i); valid(c) && update_f(Index(block, i), c)) {
				recursDown(c, update_f);
			}
		}
	}

	template <class UpdateFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UpdateFun, Index, pos_type>,
	                           bool> = true>
	void recursDown(Index node, UpdateFun update_f)
	{
		assert(exists(node));

		if (isPureLeaf(node)) {
			return;
		}

		if (pos_type c = children(node); valid(c) && update_f(node, c)) {
			recursDown(c, update_f);
		}
	}

	template <
	    class NodeFun, class BlockFun, class UpdateFun,
	    std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool>     = true,
	    std::enable_if_t<std::is_invocable_r_v<void, BlockFun, pos_type>, bool> = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, UpdateFun, Index, pos_type>, bool> =
	        true>
	void recursLeaves(pos_type block, NodeFun node_f, BlockFun block_f, UpdateFun update_f)
	{
		assert(exists(block));

		if (isPureLeaf(block)) {
			modifiedSet(treeLeafBlock(block));
			block_f(block);
			return;
		}

		InnerBlock& b = treeInnerBlock(block);
		modifiedSet(b);

		if (allLeaf(b)) {
			block_f(block);
			return;
		}

		for (offset_type i{}; BF > i; ++i) {
			Index n(block, i);
			if (pos_type c = children(b, i); valid(c)) {
				recursLeaves(c, node_f, block_f, update_f);
				update_f(n, c);
			} else {
				node_f(n);
			}
		}
	}

	template <
	    class NodeType, class NodeFun, class BlockFun, class UpdateFun,
	    std::enable_if_t<is_node_type_v<NodeType>, bool>                        = true,
	    std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool>     = true,
	    std::enable_if_t<std::is_invocable_r_v<void, BlockFun, pos_type>, bool> = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, UpdateFun, Index, pos_type>, bool> =
	        true>
	void recursLeaves(NodeType node, NodeFun node_f, BlockFun block_f, UpdateFun update_f,
	                  bool propagate)
	{
		Index n = createThreadSafe(node);

		if (isPureLeaf(n)) {
			modifiedSet(treeLeafBlock(n.pos), n.offset);
			node_f(n);
		} else {
			InnerBlock& block = treeInnerBlock(n.pos);
			modifiedSet(block, n.offset);

			if (pos_type c = children(block, n.offset); valid(c)) {
				recursLeaves(c, node_f, block_f, update_f);
				update_f(n, c);
			} else {
				node_f(n);
			}
		}

		if (propagate) {
			recursUp(n, update_f);
		}
	}

	template <
	    class BlockFun,
	    std::enable_if_t<std::is_invocable_r_v<void, BlockFun, pos_type>, bool> = true>
	void recursParentFirst(pos_type block, BlockFun block_f)
	{
		assert(exists(block));

		block_f(block);

		if (isPureLeaf(block)) {
			modifiedSet(treeLeafBlock(block));
			return;
		}

		InnerBlock& b = treeInnerBlock(block);
		modifiedSet(b);

		for (offset_type i{}; BF > i; ++i) {
			if (pos_type c = children(b, i); valid(c)) {
				recursParentFirst(c, block_f);
			}
		}
	}

	template <
	    class NodeType, class NodeFun, class BlockFun, class UpdateFun,
	    std::enable_if_t<is_node_type_v<NodeType>, bool>                        = true,
	    std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool>     = true,
	    std::enable_if_t<std::is_invocable_r_v<void, BlockFun, pos_type>, bool> = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, UpdateFun, Index, pos_type>, bool> =
	        true>
	void recursParentFirst(NodeType node, NodeFun node_f, BlockFun block_f,
	                       UpdateFun update_f, bool propagate)
	{
		Index n = createThreadSafe(node);

		node_f(n);

		if (isPureLeaf(n)) {
			modifiedSet(treeLeafBlock(n.pos), n.offset);
		} else {
			InnerBlock& block = treeInnerBlock(n.pos);
			modifiedSet(block, n.offset);

			if (pos_type c = children(block, n.offset); valid(c)) {
				recursParentFirst(c, block_f);
			}
		}

		if (propagate) {
			recursUp(n, update_f);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] bool allLeaf([[maybe_unused]] LeafBlock const& block) const
	{
		return true;
	}

	[[nodiscard]] bool allLeaf(InnerBlock const& block) const
	{
		return std::all_of(block.children.begin(), block.children.end(),
		                   [](pos_type e) { return Index::MAX_VALID_POS < e; });
	}

	/**
	 * @brief Checks if all nodes of a block are leaves.
	 *
	 * @param block the block to check
	 * @return `true` if all nodes of the block are leaves, `false` otherwise.
	 */
	[[nodiscard]] bool allLeaf(pos_type block) const
	{
		return isPureLeaf(block) || allLeaf(treeInnerBlock(block));
	}

	[[nodiscard]] bool anyLeaf([[maybe_unused]] LeafBlock const& block) const
	{
		return true;
	}

	[[nodiscard]] bool anyLeaf(InnerBlock const& block) const
	{
		return std::any_of(block.children.begin(), block.children.end(),
		                   [](pos_type e) { return Index::MAX_VALID_POS < e; });
	}

	/**
	 * @brief Checks if any node of a block is a leaf.
	 *
	 * @param block the block to check
	 * @return `true` if any node of the block is a leaf, `false` otherwise.
	 */
	[[nodiscard]] bool anyLeaf(pos_type block) const
	{
		return isPureLeaf(block) || anyLeaf(treeInnerBlock(block));
	}

	/**
	 * @brief Checks if no nodes of a block are leaves.
	 *
	 * @param block the block to check
	 * @return `true` if no nodes of the block are leaves, `false` otherwise.
	 */
	[[nodiscard]] bool noneLeaf(pos_type block) const { return !anyLeaf(block); }

	/**
	 * @brief Checks if all nodes of a block are parents.
	 *
	 * @param block the block to check
	 * @return `true` if all nodes of the block are parents, `false` otherwise.
	 */
	[[nodiscard]] bool allParent(pos_type block) const { return noneLeaf(block); }

	/**
	 * @brief Checks if any node of a block is a parent.
	 *
	 * @param block the block to check
	 * @return `true` if any node of the block is a parent, `false` otherwise.
	 */
	[[nodiscard]] bool anyParent(pos_type block) const { return !allLeaf(block); }

	/**
	 * @brief Checks if no nodes of a block are parents.
	 *
	 * @param block the block to check
	 * @return `true` if no nodes of the block are parents, `false` otherwise.
	 */
	[[nodiscard]] bool noneParent(pos_type block) const { return allLeaf(block); }

	/**************************************************************************************
	|                                                                                     |
	|                                       Center                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Child center
	//

	/**
	 * @brief Returns the center of the `child_index`th child.
	 *
	 * @param center the center of the parent
	 * @param half_length the half length of the parent
	 * @param child the offset of the child
	 * @return The center of the `child`th child.
	 */
	[[nodiscard]] static constexpr Point childCenter(Point center, Length half_length,
	                                                 offset_type child)
	{
		assert(BF > child);
		half_length /= length_type(2);
		for (std::size_t i{}; Point::size() > i; ++i) {
			center[i] += (child & offset_type(1u << i)) ? half_length[i] : -half_length[i];
		}
		return center;
	}

	//
	// Parent center
	//

	/**
	 * @brief Returns the center of the parent of the node.
	 *
	 * @param center the center of the child
	 * @param half_length the half length of the child
	 * @param index the index of the child
	 * @return The center of the parent.
	 */
	[[nodiscard]] static constexpr Point parentCenter(Point center, Length half_length,
	                                                  offset_type index)
	{
		assert(BF > index);
		for (std::size_t i{}; Point::size() > i; ++i) {
			center[i] += (index & offset_type(1u << i)) ? -half_length[i] : half_length[i];
		}
		return center;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Create                                        |
	|                                                                                     |
	**************************************************************************************/

	void initLeafChildren(Index node, InnerBlock const& block, pos_type children)
	{
		LeafBlock& cb    = treeLeafBlock(children);
		cb.parent_block  = node.pos;
		cb.parent_offset = static_cast<std::uint8_t>(node.offset);
		modified(cb) = modified(block, node.offset) ? MODIFIED_ALL_SET : MODIFIED_NONE_SET;

		derived().onInitLeafChildren(node, children);
	}

	void initInnerChildren(Index node, InnerBlock const& block, pos_type children)
	{
		InnerBlock& cb = treeInnerBlock(children);
		cb.children.fill(Index::NULL_POS);
		cb.parent_block  = node.pos;
		cb.parent_offset = static_cast<std::uint8_t>(node.offset);
		cb.depth         = static_cast<std::uint8_t>(block.depth - 1);
		modified(cb) = modified(block, node.offset) ? MODIFIED_ALL_SET : MODIFIED_NONE_SET;

		derived().onInitInnerChildren(node, children);
	}

	pos_type createChildren(Index node)
	{
		assert(!isPureLeaf(node));

		InnerBlock& block = treeInnerBlock(node.pos);

		pos_type c = children(block, node.offset);
		assert(Index::PROCESSING_POS != c);

		if (Index::NULL_POS == c) {
			if (1 == block.depth) {
				c = Data::leafCreate();
				initLeafChildren(node, block, c);
			} else {
				c = Data::innerCreate();
				initInnerChildren(node, block, c);
			}
		}

		modifiedSet(block, node.offset);

		block.children[node.offset] = c;

		return c;
	}

	pos_type createChildrenThreadSafe(Index node)
	{
		assert(!isPureLeaf(node));

		InnerBlock& block = treeInnerBlock(node.pos);

		auto cr = std::atomic_ref(block.children[node.offset]);

		pos_type c = Index::NULL_POS;
		if (cr.compare_exchange_strong(c, Index::PROCESSING_POS, std::memory_order_relaxed)) {
			if (1 == block.depth) {
				c = Data::leafCreateThreadSafe();
				initLeafChildren(node, block, c);
			} else {
				c = Data::innerCreateThreadSafe();
				initInnerChildren(node, block, c);
			}

			modifiedSetThreadSafe(block, node.offset);

			cr.store(c, std::memory_order_release);
			cr.notify_all();
		} else if (valid(c)) {
			modifiedSetThreadSafe(block, node.offset);
		} else {
			cr.wait(Index::PROCESSING_POS, std::memory_order_relaxed);
			c = cr.load(std::memory_order_acquire);
		}

		return c;

		// pos_type c  = cr.load(std::memory_order_relaxed);

		// if (valid(c)) {
		// 	modifiedSetThreadSafe(block, node.offset);
		// } else if (Index::NULL_POS == c &&
		//            cr.compare_exchange_strong(c, Index::PROCESSING_POS)) {
		// 	if (1 == block.depth) {
		// 		c = Data::leafCreateThreadSafe();
		// 		initLeafChildren(node, block, c);
		// 	} else {
		// 		c = Data::innerCreateThreadSafe();
		// 		initInnerChildren(node, block, c);
		// 	}

		// 	modifiedSetThreadSafe(block, node.offset);

		// 	cr.store(c, std::memory_order_release);
		// 	cr.notify_all();
		// } else {
		// 	cr.wait(Index::PROCESSING_POS, std::memory_order_relaxed);
		// 	c = cr.load(std::memory_order_acquire);

		// 	// NOTE: We know that someone else will set modified
		// }

		// return c;
	}

	Index createChild(Index node, offset_type child_offset)
	{
		assert(!isPureLeaf(node));
		assert(BF > child_offset);

		return Index(createChildren(node), child_offset);
	}

	Index createChildThreadSafe(Index node, offset_type child_offset)
	{
		assert(!isPureLeaf(node));
		assert(BF > child_offset);

		return Index(createChildrenThreadSafe(node), child_offset);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	Index createThreadSafe(NodeType node)
	{
		assert(valid(node));

		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			assert(exists(node));
			modifiedSetParentsThreadSafe(node);
			return node;
		} else {
			Code code         = this->code(node);
			auto wanted_depth = depth(code);
			auto cur_node     = index();
			auto cur_depth    = depth();
			while (wanted_depth < cur_depth) {
				cur_node = createChildThreadSafe(cur_node, code.offset(--cur_depth));
			}
			return cur_node;
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Erase                                        |
	|                                                                                     |
	**************************************************************************************/

	void pruneChildren(Index node) { pruneChildren(node, children(node)); }

	void pruneChildren(Index node, pos_type children)
	{
		assert(isParent(node) && this->children(node) == children);

		InnerBlock& block           = treeInnerBlock(node.pos);
		block.children[node.offset] = Index::NULL_POS;

		// NOTE: Important that derived is pruned first in case they use parent

		if (isPureLeaf(children)) {
			derived().onPruneLeafChildren(node, children);
			treeLeafBlock(children).parent_block = Index::INVALID_POS;
			Data::leafErase(children);
		} else {
			derived().onPruneInnerChildren(node, children);
			treeInnerBlock(children).parent_block = Index::INVALID_POS;
			Data::innerErase(children);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Nearest                                       |
	|                                                                                     |
	**************************************************************************************/

	// LOOKAT: Benchmark against only returning the distance

	// TODO: Implement below correctly

	template <bool OnlyDistance = false, bool FastAsSonic = false, class ValueFun,
	          class InnerFun>
	[[nodiscard]] std::conditional_t<OnlyDistance, float, std::pair<float, Index>> nearest(
	    Index node, NearestSearchAlgorithm search_alg, ValueFun value_f, InnerFun inner_f,
	    float max_dist, float epsilon) const
	{
		// FIXME: Look at
		// assert(std::isfinite(max_dist));
		// assert(std::isfinite(epsilon));

		std::conditional_t<OnlyDistance, float, std::pair<float, Index>> closest{};
		if constexpr (OnlyDistance) {
			closest = max_dist;
		} else {
			closest.first = max_dist;
		}

		if (isParent(node) && max_dist >= inner_f(node)) {
			auto cb = children(node);
			auto cd = depth(cb);

			// if (0.0f < epsilon) {
			// switch (search_alg) {
			// 	case NearestSearchAlgorithm::DEPTH_FIRST:
			closest = nearestDepthFirst<OnlyDistance, FastAsSonic>(cb, cd, max_dist, epsilon,
			                                                       value_f, inner_f);
			// 		case NearestSearchAlgorithm::A_STAR:
			// 			closest = nearestAStar(cb, cd, max_dist, epsilon, value_f, inner_f);
			// 	}
			// } else {
			// 	switch (search_alg) {
			// 		case NearestSearchAlgorithm::DEPTH_FIRST:
			// 			closest = nearestDepthFirst(cb, cd, max_dist, value_f, inner_f);
			// 		case NearestSearchAlgorithm::A_STAR:
			// 			closest = nearestAStar(cb, cd, max_dist, value_f, inner_f);
			// 	}
			// }
		}

		if constexpr (!FastAsSonic) {
			max_dist = value_f(node);
		}
		assert(!std::isnan(max_dist));
		if constexpr (OnlyDistance) {
			return UFO_MIN(closest, max_dist);
		} else {
			return closest.first < max_dist ? closest : std::pair{max_dist, node};
		}
	}

	template <class Predicate, class ValueFun, class InnerFun,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] std::pair<float, Index> nearest(Index node, Predicate pred,
	                                              NearestSearchAlgorithm search_alg,
	                                              ValueFun value_f, InnerFun inner_f,
	                                              float max_dist, float epsilon) const
	{
		using Filter = pred::Filter<Predicate>;

		Filter::init(pred, derived());

		auto wrapped_value_f = [value_f, &pred](Index node) -> float {
			return Filter::returnable(pred) ? value_f(node)
			                                : std::numeric_limits<float>::infinity();
		};

		auto wrapped_inner_f = [inner_f, &pred](Index node) -> float {
			return Filter::traversable(pred) ? inner_f(node)
			                                 : std::numeric_limits<float>::infinity();
		};

		return nearest(node, search_alg, wrapped_value_f, wrapped_inner_f, max_dist, epsilon);
	}

	template <bool OnlyDistance, bool FastAsSonic, class ValueFun, class InnerFun>
	[[nodiscard]] std::conditional_t<OnlyDistance, float, std::pair<float, Index>>
	nearestDepthFirst(pos_type block, depth_type depth, float c_dist, float epsilon,
	                  ValueFun value_f, InnerFun inner_f) const
	{
		struct StackElement {
			using Container = std::array<std::pair<float, pos_type>, BF>;
			using Iterator  = typename Container::iterator;

			Container container;
			Iterator  it;

			[[nodiscard]] constexpr float& distance() { return it->first; }

			[[nodiscard]] constexpr float const& distance() const { return it->first; }

			[[nodiscard]] constexpr pos_type& block() { return it->second; }

			[[nodiscard]] constexpr pos_type const& block() const { return it->second; }

			constexpr void start() { it = container.begin(); }

			[[nodiscard]] constexpr bool empty() { return container.end() == it; }

			[[nodiscard]] constexpr bool empty() const { return container.end() == it; }

			StackElement& operator++()
			{
				++it;
				return *this;
			}

			constexpr void sort()
			{
				if constexpr (2 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_2(container);
				} else if constexpr (4 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_4(container);
				} else if constexpr (8 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_8(container);
				} else if constexpr (16 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_16(container);
				} else {
					std::sort(container.begin(), container.end(),
					          [](auto a, auto b) { return a.first < b.first; });
				}
			}
		};

		using Stack = std::array<StackElement, maxNumDepthLevels() - 1>;

		Stack stack;
		// Since we only have one block in the beginning we set the index to `BF - 1u` (the
		// last index)
		stack[depth].it = std::prev(stack[depth].container.end());
		// The first block to go through
		stack[depth].block() = block;
		// This distance does not matter as long as it is less than `c_dist - epsilon`, since
		// we only have one block in the beginning
		stack[depth].distance() = 0.0f;

		std::conditional_t<OnlyDistance, bool, Index> c_node;

		std::array<std::conditional_t<OnlyDistance, float, std::pair<float, offset_type>>, BF>
		    d;

		for (depth_type max_depth = depth + 1; max_depth > depth;) {
			StackElement& se = stack[depth];

			if (se.empty() || c_dist - epsilon <= se.distance()) {
				++depth;
				continue;
			}

			block = se.block();
			++se;

			StackElement& cur = stack[depth - 1u];

			cur.start();

			for (std::size_t i{}; BF > i; ++i) {
				Index node(block, i);
				cur.container[i].first = inner_f(node);
				assert(!std::isnan(cur.container[i].first));
				cur.container[i].second = children(node);

				if constexpr (!FastAsSonic) {
					if constexpr (OnlyDistance) {
						d[i] = value_f(node);
						assert(!std::isnan(d[i]));
					} else {
						d[i].first = value_f(node);
						assert(!std::isnan(d[i].first));
						d[i].second = i;
					}
				}
			}

			if constexpr (!FastAsSonic) {
				if constexpr (OnlyDistance) {
					if constexpr (2 == BF) {
						UFO_MIN_2(d);
					} else if constexpr (4 == BF) {
						UFO_MIN_4(d);
					} else if constexpr (8 == BF) {
						UFO_MIN_8(d);
					} else if constexpr (16 == BF) {
						UFO_MIN_16(d);
					} else {
						for (std::size_t i = 1; BF > i; ++i) {
							d[0] = UFO_MIN(d[0], d[i]);
						}
					}

					c_dist = c_dist <= d[0] ? c_dist : d[0];
				} else {
					if constexpr (2 == BF) {
						UFO_MIN_PAIR_FIRST_2(d);
					} else if constexpr (4 == BF) {
						UFO_MIN_PAIR_FIRST_4(d);
					} else if constexpr (8 == BF) {
						UFO_MIN_PAIR_FIRST_8(d);
					} else if constexpr (16 == BF) {
						UFO_MIN_PAIR_FIRST_16(d);
					} else {
						for (std::size_t i = 1; BF > i; ++i) {
							d[0] = UFO_MIN_PAIR_FIRST(d[0], d[i]);
						}
					}

					c_node = c_dist <= d[0].first ? c_node : Index{block, d[0].second};
					c_dist = c_dist <= d[0].first ? c_dist : d[0].first;
				}
			}

			if (1u == depth) {
				for (auto [dist, child_block] : cur.container) {
					if (c_dist <= dist + epsilon) {
						continue;
					}

					if constexpr (OnlyDistance) {
						for (offset_type i{}; BF > i; ++i) {
							d[i] = value_f(Index(child_block, i));
							assert(!std::isnan(d[i]));
						}

						if constexpr (2 == BF) {
							UFO_MIN_2(d);
						} else if constexpr (4 == BF) {
							UFO_MIN_4(d);
						} else if constexpr (8 == BF) {
							UFO_MIN_8(d);
						} else if constexpr (16 == BF) {
							UFO_MIN_16(d);
						} else {
							for (std::size_t i = 1; BF > i; ++i) {
								d[0] = UFO_MIN(d[0], d[i]);
							}
						}

						c_dist = c_dist <= d[0] ? c_dist : d[0];
					} else {
						for (offset_type i{}; BF > i; ++i) {
							d[i].first = value_f(Index(child_block, i));
							assert(!std::isnan(d[i].first));
							d[i].second = i;
						}

						if constexpr (2 == BF) {
							UFO_MIN_PAIR_FIRST_2(d);
						} else if constexpr (4 == BF) {
							UFO_MIN_PAIR_FIRST_4(d);
						} else if constexpr (8 == BF) {
							UFO_MIN_PAIR_FIRST_8(d);
						} else if constexpr (16 == BF) {
							UFO_MIN_PAIR_FIRST_16(d);
						} else {
							for (std::size_t i = 1; BF > i; ++i) {
								d[0] = UFO_MIN_PAIR_FIRST(d[0], d[i]);
							}
						}

						c_node = c_dist <= d[0].first ? c_node : Index{child_block, d[0].second};
						c_dist = c_dist <= d[0].first ? c_dist : d[0].first;
					}
				}
			} else {
				cur.sort();
				--depth;
			}
		}

		if constexpr (OnlyDistance) {
			return c_dist;
		} else {
			return {c_dist, c_node};
		}
	}

	template <class ValueFun, class InnerFun>
	[[nodiscard]] std::pair<float, Index> nearestDepthFirst(pos_type   block,
	                                                        depth_type depth, float c_dist,
	                                                        ValueFun value_f,
	                                                        InnerFun inner_f) const
	{
		using Stack =
		    std::array<std::pair<std::size_t, std::array<std::pair<float, pos_type>, BF>>,
		               maxNumDepthLevels() - 1>;

		Stack stack;
		stack[depth].first                 = BF - 1u;
		stack[depth].second[BF - 1].first  = 0.0f;
		stack[depth].second[BF - 1].second = block;

		Index c_node;

		for (depth_type max_depth = depth + 1; max_depth > depth;) {
			auto& [idx, c] = stack[depth];

			if (BF <= idx || c_dist <= c[idx].first) {
				++depth;
				continue;
			}

			block = c[idx].second;
			++idx;

			stack[depth - 1].first = 0;
			auto& candidates       = stack[depth - 1].second;

			for (std::size_t i{}; BF > i; ++i) {
				Index node(block, i);
				candidates[i].first = inner_f(node);
				assert(!std::isnan(candidates[i].first));
				candidates[i].second = children(node);
			}

			if (1u == depth) {
				std::array<std::pair<float, offset_type>, BF> d;
				for (auto [dist, child_block] : candidates) {
					if (c_dist <= dist) {
						continue;
					}

					for (offset_type i{}; BF > i; ++i) {
						d[i].first = value_f(Index(child_block, i));
						assert(!std::isnan(d[i].first));
						d[i].second = i;
					}

					if constexpr (2 == BF) {
						UFO_MIN_PAIR_FIRST_2(d);
					} else if constexpr (4 == BF) {
						UFO_MIN_PAIR_FIRST_4(d);
					} else if constexpr (8 == BF) {
						UFO_MIN_PAIR_FIRST_8(d);
					} else if constexpr (16 == BF) {
						UFO_MIN_PAIR_FIRST_16(d);
					} else {
						for (std::size_t i = 1; BF > i; ++i) {
							d[0] = UFO_MIN_PAIR_FIRST(d[0], d[i]);
						}
					}

					c_node = c_dist <= d[0].first ? c_node : Index{child_block, d[0].second};
					c_dist = c_dist <= d[0].first ? c_dist : d[0].first;
				}
			} else {
				if constexpr (2 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_2(candidates);
				} else if constexpr (4 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_4(candidates);
				} else if constexpr (8 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_8(candidates);
				} else if constexpr (16 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_16(candidates);
				} else {
					std::sort(candidates.begin(), candidates.end(),
					          [](auto a, auto b) { return a.first < b.first; });
				}
				--depth;
			}
		}

		return {c_dist, c_node};
	}

	// template <class ValueFun, class InnerFun>
	// [[nodiscard]] std::pair<float, Index> nearestAStar(pos_type block, depth_type depth,
	//                                                    float c_dist, float epsilon,
	//                                                    ValueFun value_f,
	//                                                    InnerFun inner_f) const
	// {
	// 	struct S {
	// 		float   dist;
	// 		pos_type   block;
	// 		depth_type depth;

	// 		S(float dist, pos_type block, depth_type depth) noexcept
	// 		    : dist(dist), block(block), depth(depth)
	// 		{
	// 		}

	// 		bool operator>(S rhs) const noexcept
	// 		{
	// 			// return dist > rhs.dist;
	// 			return dist + (depth << 2) > rhs.dist + (rhs.depth << 2);
	// 		}
	// 	};

	// 	using Queue = std::priority_queue<S, std::vector<S>, std::greater<S>>;

	// 	std::vector<S> container;
	// 	container.reserve(1024);
	// 	Queue queue(std::greater<S>{}, std::move(container));
	// 	queue.emplace(0.0f, block, depth);

	// 	auto max_size = depth << 2;

	// 	Index c_node;

	// 	while (!queue.empty()) {
	// 		auto cur = queue.top();

	// 		if (c_dist + max_size - (cur.depth << 2) <= cur.dist + epsilon) {
	// 			return {c_dist, c_node};
	// 		}

	// 		if (c_dist <= cur.dist + epsilon) {
	// 			queue.pop();
	// 			continue;
	// 		}

	// 		queue.pop();

	// 		block = cur.block;
	// 		depth = cur.depth;

	// 		std::array<std::pair<float, pos_type>, BF> candidates;
	// 		for (std::size_t i{}; BF > i; ++i) {
	// 			Index node(block, i);
	// 			candidates[i].first = inner_f(node);
	// 			assert(!std::isnan(candidates[i].first));
	// 			candidates[i].second = children(node);
	// 		}

	// 		if (1u == depth) {
	// 			std::array<std::pair<float, offset_type>, BF> d;
	// 			for (auto [dist, child_block] : candidates) {
	// 				if (c_dist <= dist + epsilon) {
	// 					continue;
	// 				}

	// 				for (offset_type i{}; BF > i; ++i) {
	// 					d[i].first = value_f(Index(child_block, i));
	// 					assert(!std::isnan(d[i].first));
	// 					d[i].second = i;
	// 				}

	// 				if constexpr (2 == BF) {
	// 					UFO_MIN_PAIR_FIRST_2(d);
	// 				} else if constexpr (4 == BF) {
	// 					UFO_MIN_PAIR_FIRST_4(d);
	// 				} else if constexpr (8 == BF) {
	// 					UFO_MIN_PAIR_FIRST_8(d);
	// 				} else if constexpr (16 == BF) {
	// 					UFO_MIN_PAIR_FIRST_16(d);
	// 				} else {
	// 					for (std::size_t i = 1; BF > i; ++i) {
	// 						d[0] = UFO_MIN_PAIR_FIRST(d[0], d[i]);
	// 					}
	// 				}

	// 				c_node = c_dist <= d[0].first ? c_node : Index{child_block, d[0].second};
	// 				c_dist = c_dist <= d[0].first ? c_dist : d[0].first;
	// 			}
	// 		} else {
	// 			for (auto [dist, child_block] : candidates) {
	// 				if (c_dist <= dist + epsilon) {
	// 					continue;
	// 				}
	// 				queue.emplace(dist, child_block, depth - 1);
	// 			}
	// 		}
	// 	}

	// 	return {c_dist, c_node};
	// }

	// template <class ValueFun, class InnerFun>
	// [[nodiscard]] std::pair<float, Index> nearestAStar(pos_type block, depth_type depth,
	//                                                    float c_dist, ValueFun value_f,
	//                                                    InnerFun inner_f) const
	// {
	// 	struct S {
	// 		float   dist;
	// 		pos_type   block;
	// 		depth_type depth;

	// 		S(float dist, pos_type block, depth_type depth) noexcept
	// 		    : dist(dist), block(block), depth(depth)
	// 		{
	// 		}

	// 		bool operator>(S rhs) const noexcept
	// 		{
	// 			// return dist > rhs.dist;
	// 			return dist + (depth << 2) > rhs.dist + (rhs.depth << 2);
	// 		}
	// 	};

	// 	using Queue = std::priority_queue<S, std::vector<S>, std::greater<S>>;

	// 	std::vector<S> container;
	// 	container.reserve(1024);
	// 	Queue queue(std::greater<S>{}, std::move(container));
	// 	queue.emplace(0.0f, block, depth);

	// 	auto max_size = depth << 2;

	// 	Index c_node;

	// 	while (!queue.empty()) {
	// 		auto cur = queue.top();

	// 		if (c_dist + max_size - (cur.depth << 2) <= cur.dist) {
	// 			return {c_dist, c_node};
	// 		}

	// 		if (c_dist <= cur.dist) {
	// 			queue.pop();
	// 			continue;
	// 		}

	// 		queue.pop();

	// 		block = cur.block;
	// 		depth = cur.depth;

	// 		std::array<std::pair<float, pos_type>, BF> candidates;
	// 		for (std::size_t i{}; BF > i; ++i) {
	// 			Index node(block, i);
	// 			candidates[i].first = inner_f(node);
	// 			assert(!std::isnan(candidates[i].first));
	// 			candidates[i].second = children(node);
	// 		}

	// 		if (1u == depth) {
	// 			std::array<std::pair<float, offset_type>, BF> d;
	// 			for (auto [dist, child_block] : candidates) {
	// 				if (c_dist <= dist) {
	// 					continue;
	// 				}

	// 				for (offset_type i{}; BF > i; ++i) {
	// 					d[i].first = value_f(Index(child_block, i));
	// 					assert(!std::isnan(d[i].first));
	// 					d[i].second = i;
	// 				}

	// 				if constexpr (2 == BF) {
	// 					UFO_MIN_PAIR_FIRST_2(d);
	// 				} else if constexpr (4 == BF) {
	// 					UFO_MIN_PAIR_FIRST_4(d);
	// 				} else if constexpr (8 == BF) {
	// 					UFO_MIN_PAIR_FIRST_8(d);
	// 				} else if constexpr (16 == BF) {
	// 					UFO_MIN_PAIR_FIRST_16(d);
	// 				} else {
	// 					for (std::size_t i = 1; BF > i; ++i) {
	// 						d[0] = UFO_MIN_PAIR_FIRST(d[0], d[i]);
	// 					}
	// 				}

	// 				c_node = c_dist <= d[0].first ? c_node : Index{child_block, d[0].second};
	// 				c_dist = c_dist <= d[0].first ? c_dist : d[0].first;
	// 			}
	// 		} else {
	// 			for (auto [dist, child_block] : candidates) {
	// 				if (c_dist <= dist) {
	// 					continue;
	// 				}
	// 				queue.emplace(dist, child_block, depth - 1);
	// 			}
	// 		}
	// 	}

	// 	return {c_dist, c_node};
	// }

	/**************************************************************************************
	|                                                                                     |
	|                                        Trace                                        |
	|                                                                                     |
	**************************************************************************************/

	struct TraceParams {
		Ray      ray;
		Point    t0;
		Point    t1;
		unsigned a{};
	};

	struct TraceStackElement {
		Point    t0;
		Point    t1;
		Point    tm;
		unsigned cur_node;
		Node     node;

		TraceStackElement() = default;

		constexpr TraceStackElement(Node node, unsigned cur_node, Point const& t0,
		                            Point const& t1, Point const& tm)
		    : node(node), cur_node(cur_node), t0(t0), t1(t1), tm(tm)
		{
		}
	};

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] TraceParams traceInit(NodeType node, Ray const& ray) const
	{
		return traceInit(ray, center(node), halfLength(node));
	}

	[[nodiscard]] static constexpr TraceParams traceInit(Ray const&   ray,
	                                                     Point const& center,
	                                                     Length       half_length) noexcept
	{
		TraceParams params;
		params.ray = ray;

		for (std::size_t i{}; Dim > i; ++i) {
			float origin = 0 > ray.direction[i] ? center[i] * 2 - ray.origin[i] : ray.origin[i];

			auto a = center[i] - half_length[i] - origin;
			auto b = center[i] + half_length[i] - origin;

			// FIXME: Look at
			params.t0[i] = 0 == ray.direction[i] ? 1e+25 * a : a / std::abs(ray.direction[i]);
			params.t1[i] = 0 == ray.direction[i] ? 1e+25 * b : b / std::abs(ray.direction[i]);

			params.a |= unsigned(0 > ray.direction[i]) << i;
		}

		return params;
	}

	[[nodiscard]] static constexpr inline unsigned firstNode(Point const& tm,
	                                                         float const  t) noexcept
	{
		unsigned node = static_cast<unsigned>(tm[0] < t);
		for (unsigned i = 1; Dim > i; ++i) {
			node |= static_cast<unsigned>(tm[i] < t) << i;
		}
		return node;
	}

	[[nodiscard]] static constexpr inline unsigned newNode(unsigned cur,
	                                                       unsigned dim) noexcept
	{
		// You are at cur, you want to move along dim in positive direction
		unsigned x = 1u << dim;
		return ((cur & x) << Dim) | cur | x;
	}

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] constexpr TraceResult trace(Node node, TraceParams const& params,
	                                          Predicate const& pred, float const near_clip,
	                                          float const far_clip, bool only_exists) const
	{
		// TODO: Use `only_exists`

		using Filter = pred::Filter<Predicate>;

		auto returnable = [this, near_clip, far_clip, &pred](Node const& node, float min_dist,
		                                                     float max_dist) {
			return near_clip <= max_dist && far_clip >= min_dist &&
			       Filter::returnable(pred, derived(), node);
		};

		auto traversable = [this, near_clip, far_clip, &pred](
		                       Node const& node, float min_dist, float max_dist) {
			return near_clip <= max_dist && far_clip >= min_dist && isParent(node.index) &&
			       Filter::traversable(pred, derived(), node);
		};

		constexpr auto const new_node_lut = []() {
			std::array<std::array<unsigned, Dim>, BF> lut{};
			for (unsigned cur{}; BF != cur; ++cur) {
				for (unsigned dim{}; Dim != dim; ++dim) {
					lut[cur][dim] = newNode(cur, dim);
				}
			}
			return lut;
		}();

		auto t0 = params.t0;
		auto t1 = params.t1;
		auto tm = (t0 + t1) * 0.5f;
		auto a  = params.a;

		auto min_dist = ufo::max(t0);
		auto max_dist = ufo::min(t1);

		if (min_dist >= max_dist || near_clip > max_dist || far_clip < min_dist) {
			return TraceResult{Node(),

			                   std::numeric_limits<float>::infinity()};
		} else if (returnable(node, min_dist, max_dist)) {
			float distance = std::max(near_clip, min_dist);
			return TraceResult{node, distance};
		} else if (!traversable(node, min_dist, max_dist)) {
			return TraceResult{Node(),

			                   std::numeric_limits<float>::infinity()};
		}

		unsigned cur_node = firstNode(tm, min_dist);

		std::array<TraceStackElement, maxNumDepthLevels()> stack;
		stack[0] = TraceStackElement{node, cur_node, t0, t1, tm};

		for (int idx{}; 0 <= idx;) {
			node     = stack[idx].node;
			cur_node = stack[idx].cur_node;
			t0       = stack[idx].t0;
			t1       = stack[idx].t1;
			tm       = stack[idx].tm;

			// We have a need for speed and we don´t need safety here since we know all the
			// nodes exist and are valid
			node.code  = child(node.code, cur_node ^ a);
			node.index = child(node.index, cur_node ^ a);

			for (unsigned i{}; Dim > i; ++i) {
				t0[i] = (cur_node & (1u << i)) ? tm[i] : t0[i];
				t1[i] = (cur_node & (1u << i)) ? t1[i] : tm[i];
			}

			stack[idx].cur_node = new_node_lut[cur_node][minIndex(t1)];
			idx -= BF <= stack[idx].cur_node;

			min_dist = ufo::max(t0);
			max_dist = ufo::min(t1);

			if (returnable(node, min_dist, max_dist)) {
				float distance = std::max(near_clip, min_dist);
				return TraceResult{node, distance};
			} else if (!traversable(node, min_dist, max_dist)) {
				continue;
			}

			tm = (t0 + t1) * 0.5f;

			unsigned cur_node = firstNode(tm, min_dist);

			stack[++idx] = TraceStackElement{node, cur_node, t0, t1, tm};
		}

		return TraceResult{Node(), std::numeric_limits<float>::infinity()};
	}

 private:
	template <class NodeType>
	[[nodiscard]] constexpr auto convert(NodeType node) const
	{
		using T = remove_cvref_t<NodeType>;
		if constexpr (std::is_convertible_v<T, Index>) {
			return static_cast<Index>(node);
		} else if constexpr (std::is_convertible_v<T, Node>) {
			return static_cast<Node>(node);
		} else if constexpr (std::is_convertible_v<T, Code>) {
			return static_cast<Code>(node);
		} else if constexpr (std::is_convertible_v<T, Key>) {
			return static_cast<Key>(node);
		} else if constexpr (std::is_convertible_v<T, Coord>) {
			return static_cast<Coord>(node);
		} else if constexpr (std::is_constructible_v<T, Coord>) {
			return Coord(node);
		} else {
			static_assert(is_node_type_v<NodeType>, "Not one of the supported node types.");
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modified                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] int modifiedCount(LeafBlock const& block) const
	{
		return std::popcount(modified(block));
	}

	[[nodiscard]] int modifiedCount(InnerBlock const& block) const
	{
		return std::popcount(modified(block));
	}

	[[nodiscard]] bool modifiedAll(LeafBlock const& block) const
	{
		return MODIFIED_ALL_SET == modified(block);
	}

	[[nodiscard]] bool modifiedAll(InnerBlock const& block) const
	{
		return MODIFIED_ALL_SET == modified(block);
	}

	[[nodiscard]] bool modifiedNone(LeafBlock const& block) const
	{
		return MODIFIED_NONE_SET == modified(block);
	}

	[[nodiscard]] bool modifiedNone(InnerBlock const& block) const
	{
		return MODIFIED_NONE_SET == modified(block);
	}

	[[nodiscard]] bool modified(LeafBlock const& block, offset_type offset) const
	{
		return modified_type(1) & (modified(block) >> offset);
	}

	[[nodiscard]] bool modified(InnerBlock const& block, offset_type offset) const
	{
		return modified_type(1) & (modified(block) >> offset);
	}

	[[nodiscard]] bool modified(pos_type block, offset_type offset) const
	{
		return isPureLeaf(block) ? modified(treeLeafBlock(block), offset)
		                         : modified(treeInnerBlock(block), offset);
	}

	[[nodiscard]] modified_type& modified(LeafBlock& block) { return block.modified; }

	[[nodiscard]] modified_type modified(LeafBlock const& block) const
	{
		return block.modified;
	}

	[[nodiscard]] modified_type& modified(InnerBlock& block) { return block.modified; }

	[[nodiscard]] modified_type modified(InnerBlock const& block) const
	{
		return block.modified;
	}

	[[nodiscard]] modified_type& modified(pos_type block)
	{
		return isPureLeaf(block) ? modified(treeLeafBlock(block))
		                         : modified(treeInnerBlock(block));
	}

	[[nodiscard]] modified_type modified(pos_type block) const
	{
		return isPureLeaf(block) ? modified(treeLeafBlock(block))
		                         : modified(treeInnerBlock(block));
	}

	void modifiedSet(LeafBlock& block, offset_type offset)
	{
		modified(block) |= static_cast<modified_type>(1u << offset);
	}

	void modifiedSet(InnerBlock& block, offset_type offset)
	{
		modified(block) |= static_cast<modified_type>(1u << offset);
	}

	void modifiedSet(pos_type block, offset_type offset)
	{
		return isPureLeaf(block) ? modifiedSet(treeLeafBlock(block), offset)
		                         : modifiedSet(treeInnerBlock(block), offset);
	}

	void modifiedSet(LeafBlock& block) { modified(block) = MODIFIED_ALL_SET; }

	void modifiedSet(InnerBlock& block) { modified(block) = MODIFIED_ALL_SET; }

	void modifiedSet(pos_type block)
	{
		return isPureLeaf(block) ? modifiedSet(treeLeafBlock(block))
		                         : modifiedSet(treeInnerBlock(block));
	}

	void modifiedSetThreadSafe(LeafBlock& block, offset_type offset)
	{
		auto m = std::atomic_ref(modified(block));
		m.fetch_or(static_cast<modified_type>(1u << offset), std::memory_order_relaxed);
	}

	void modifiedSetThreadSafe(InnerBlock& block, offset_type offset)
	{
		auto m = std::atomic_ref(modified(block));
		m.fetch_or(static_cast<modified_type>(1u << offset), std::memory_order_relaxed);
	}

	void modifiedSetThreadSafe(pos_type block, offset_type offset)
	{
		return isPureLeaf(block) ? modifiedSetThreadSafe(treeLeafBlock(block), offset)
		                         : modifiedSetThreadSafe(treeInnerBlock(block), offset);
	}

	void modifiedSetThreadSafe(LeafBlock& block)
	{
		auto m = std::atomic_ref(modified(block));
		m.store(MODIFIED_ALL_SET, std::memory_order_relaxed);
	}

	void modifiedSetThreadSafe(InnerBlock& block)
	{
		auto m = std::atomic_ref(modified(block));
		m.store(MODIFIED_ALL_SET, std::memory_order_relaxed);
	}

	void modifiedSetThreadSafe(pos_type block)
	{
		return isPureLeaf(block) ? modifiedSetThreadSafe(treeLeafBlock(block))
		                         : modifiedSetThreadSafe(treeInnerBlock(block));
	}

	void modifiedReset(LeafBlock& block, offset_type offset)
	{
		modified(block) &= static_cast<modified_type>(~(1u << offset));
	}

	void modifiedReset(InnerBlock& block, offset_type offset)
	{
		modified(block) &= static_cast<modified_type>(~(1u << offset));
	}

	void modifiedReset(pos_type block, offset_type offset)
	{
		return isPureLeaf(block) ? modifiedSet(treeLeafBlock(block), offset)
		                         : modifiedSet(treeInnerBlock(block), offset);
	}

	void modifiedReset(LeafBlock& block) { modified(block) = MODIFIED_NONE_SET; }

	void modifiedReset(InnerBlock& block) { modified(block) = MODIFIED_NONE_SET; }

	void modifiedReset(pos_type block)
	{
		return isPureLeaf(block) ? modifiedSet(treeLeafBlock(block))
		                         : modifiedSet(treeInnerBlock(block));
	}

	void modifiedSetParents(Index node)
	{
		if (Index p = parent(node);
		    !valid(p) || modified(treeInnerBlockConst(p.pos), p.offset)) {
			return;
		}

		recursUp(node, [this](Index parent, [[maybe_unused]] pos_type children) -> bool {
			modified_type& m    = modified(treeInnerBlock(parent.pos));
			modified_type  prev = m;
			m |= static_cast<modified_type>(1u << parent.offset);
			return m != prev;
		});
	}

	void modifiedSetParentsThreadSafe(Index node)
	{
		if (Index p = parent(node);
		    !valid(p) || modified(treeInnerBlockConst(p.pos), p.offset)) {
			return;
		}

		recursUp(node, [this](Index parent, [[maybe_unused]] pos_type children) -> bool {
			auto          m = std::atomic_ref(modified(treeInnerBlock(parent.pos)));
			modified_type v = static_cast<modified_type>(1u << parent.offset);
			return v != (m.fetch_or(v, std::memory_order_relaxed) & v);
		});
	}

	void modifiedSetChildren(Index node)
	{
		recursDown(node, [this]([[maybe_unused]] Index parent, pos_type children) -> bool {
			if (isPureLeaf(children)) {
				modifiedSet(treeLeafBlock(children));
				return false;
			} else {
				modified_type& m    = modified(treeInnerBlock(children));
				modified_type  prev = m;
				m                   = MODIFIED_ALL_SET;
				return m != prev;
			}
		});
	}

	void modifiedResetChildren(Index node)
	{
		recursDown(node, [this]([[maybe_unused]] Index parent, pos_type children) -> bool {
			if (isPureLeaf(children)) {
				modifiedReset(treeLeafBlock(children));
				return false;
			} else {
				modified_type& m    = modified(treeInnerBlock(children));
				modified_type  prev = m;
				m                   = MODIFIED_NONE_SET;
				return m != prev;
			}
		});
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Children                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] std::array<pos_type, BF> const& children(InnerBlock const& block) const
	{
		return block.children;
	}

	[[nodiscard]] pos_type children(InnerBlock const& block, offset_type offset) const
	{
		return children(block)[offset];
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Parent                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] Index parent(LeafBlock const& block) const
	{
		return Index(block.parent_block, block.parent_offset);
	}

	[[nodiscard]] Index parent(InnerBlock const& block) const
	{
		return Index(block.parent_block, block.parent_offset);
	}

 private:
	// The number of depth levels
	depth_type num_depth_levels_;
	// Half the maximum key value the tree can store
	key_type half_max_value_;

	// Stores the node half length at a given depth, where the index is the depth
	std::array<Length, maxNumDepthLevels() + 1> node_half_length_;
	// Reciprocal of the node half length at a given depth, where the index is the depth
	std::array<Length, maxNumDepthLevels() + 1> node_half_length_reciprocal_;
};

template <class Derived, std::size_t Dim, class... Blocks>
bool operator==(Tree<Derived, Dim, Blocks...> const& lhs,
                Tree<Derived, Dim, Blocks...> const& rhs)
{
	return lhs.num_depth_levels_ == rhs.num_depth_levels_ &&
	       lhs.node_half_length_ == rhs.node_half_length_ &&
	       std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(),
	                  [&lhs, &rhs](TreeNode<Dim> const& a, TreeNode<Dim> const& b) {
		                  return 0 != a.index.offset ||
		                         (a.code == b.code &&
		                          lhs.treeBlock(a.index) == rhs.treeBlock(b.index) &&
		                          ((lhs.template data<Blocks>(a.index.pos) ==
		                            rhs.template data<Blocks>(b.index.pos)) &&
		                           ...));
	                  });
}

template <class Derived, std::size_t Dim, class... Blocks>
bool operator!=(Tree<Derived, Dim, Blocks...> const& lhs,
                Tree<Derived, Dim, Blocks...> const& rhs)
{
	return !(lhs == rhs);
}
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_HPP