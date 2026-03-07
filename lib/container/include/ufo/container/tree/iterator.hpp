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

#ifndef UFO_CONTAINER_TREE_ITERATOR_HPP
#define UFO_CONTAINER_TREE_ITERATOR_HPP

// UFO
#include <ufo/container/tree/index.hpp>

// STL
#include <cstddef>
#include <iterator>

namespace ufo
{
template <class Tree>
class TreeIterator
{
 private:
	//
	// Friends
	//

	template <class>
	friend class TreeIterator;

 private:
	static constexpr std::size_t const BF = Tree::branchingFactor();

	using Node        = typename Tree::Node;
	using offset_type = typename Tree::offset_type;
	using depth_type  = typename Tree::depth_type;

 public:
	//
	// Tags
	//

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = Node;
	using reference         = value_type const&;
	using pointer           = value_type const*;

	constexpr TreeIterator() = default;

	TreeIterator(Tree const* tree, Node node, bool only_leaves, bool only_exists)
	    : tree_(tree)
	    , cur_(node)
	    , start_(tree->depth(node))
	    , nextNode(nextNodeFun(only_leaves, only_exists))
	{
		if (only_exists && !tree_->exists(cur_)) {
			cur_ = {};
		} else if (only_leaves && tree_->isParent(node.index)) {
			cur_ = nextNode(*tree_, cur_, start_);
		}
	}

	TreeIterator& operator++()
	{
		cur_ = nextNode(*tree_, cur_, start_);
		return *this;
	}

	TreeIterator operator++(int)
	{
		TreeIterator tmp(*this);
		++*this;
		return tmp;
	}

	reference operator*() const { return cur_; }

	pointer operator->() const { return &cur_; }

	friend bool operator==(TreeIterator const& lhs, TreeIterator const& rhs)
	{
		return lhs.cur_.code == rhs.cur_.code;
	}

	friend bool operator!=(TreeIterator const& lhs, TreeIterator const& rhs)
	{
		return !(lhs == rhs);
	}

 private:
	[[nodiscard]] static auto nextNodeFun(bool only_leaves, bool only_exists)
	{
		if (only_leaves && only_exists) {
			return &TreeIterator::next<true, true>;
		} else if (only_leaves && !only_exists) {
			return &TreeIterator::next<true, false>;
		} else if (!only_leaves && only_exists) {
			return &TreeIterator::next<false, true>;
		} else {
			return &TreeIterator::next<false, false>;
		}
	}

	template <bool OnlyLeaves, bool OnlyExists>
	[[nodiscard]] static Node next(Tree const& tree, Node node, depth_type start)
	{
		if constexpr (!OnlyLeaves) {
			// Traverse down the branch

			if constexpr (OnlyExists) {
				if (tree.isParent(node.index)) {
					node.code  = node.code.firstborn();
					node.index = tree.child(node.index, 0);
					return node;
				}
			} else {
				if (0 < node.code.depth()) {
					node.code = node.code.firstborn();
					if (tree.isParent(node.index)) {
						node.index = tree.child(node.index, 0);
					}
					return node;
				}
			}
		}

		// Find next branch

		while (BF - 1 <= node.code.offset()) {
			node.code = node.code.parent();
		}

		if (start <= node.code.depth()) {
			return Node{};
		}

		node.code  = node.code.nextSibling();
		node.index = tree.ancestor(node.index, node.code.depth());

		if constexpr (OnlyExists) {
			++node.index.offset;
		} else {
			node.index.offset += node.code.depth() == tree.depth(node.index) ? 1 : 0;
		}

		if constexpr (OnlyLeaves) {
			// Adjust index

			while (tree.isParent(node.index)) {
				node.index = tree.child(node.index, 0);
			}

			if constexpr (OnlyExists) {
				node.code.setDepth(tree.depth(node.index));
			} else {
				node.code.setDepth(0);
			}
		}

		return node;
	}

 private:
	Tree const* tree_;

	Node       cur_{};
	depth_type start_;

	decltype(&TreeIterator::next<true, true>) nextNode;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_ITERATOR_HPP