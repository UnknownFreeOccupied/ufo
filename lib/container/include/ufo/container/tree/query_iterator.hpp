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

#ifndef UFO_CONTAINER_TREE_QUERY_ITERATOR_HPP
#define UFO_CONTAINER_TREE_QUERY_ITERATOR_HPP

// UFO
#include <ufo/container/tree/index.hpp>
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/container/tree/predicate/predicate.hpp>

// STL
#include <cstddef>
#include <iterator>

namespace ufo
{
template <class Tree, class Predicate = pred::Predicate<Tree>>
class TreeQueryIterator
{
 private:
	//
	// Friends
	//

	template <class, class>
	friend class TreeQueryIterator;

 private:
	static constexpr std::size_t const BF = Tree::branchingFactor();

	using Node        = typename Tree::Node;
	using offset_type = typename Tree::offset_type;
	using depth_type  = typename Tree::depth_type;

	using Filter = pred::Filter<Predicate>;

 public:
	//
	// Tags
	//

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = Node;
	using reference         = value_type const&;
	using pointer           = value_type const*;

	constexpr TreeQueryIterator() = default;

	TreeQueryIterator(Tree const* tree, Node node, Predicate const& pred, bool only_exists,
	                  bool early_stopping)
	    : tree_(tree)
	    , pred_(pred)
	    , cur_(node)
	    , start_(tree->depth(node))
	    , nextNode(nextNodeFun(only_exists, early_stopping))
	{
		if (only_exists && !tree_->exists(cur_)) {
			cur_ = {};
			return;
		}

		Filter::init(pred_, *tree_);

		if (!Filter::returnable(pred_, *tree_, cur_)) {
			if (only_exists) {
				cur_ = next<true, false>(*tree_, pred_, cur_, start_);
			} else {
				cur_ = next<false, false>(*tree_, pred_, cur_, start_);
			}
		}
	}

	TreeQueryIterator(TreeQueryIterator const&) = default;

	template <class Predicate2>
	TreeQueryIterator(TreeQueryIterator<Tree, Predicate2> const& other)
	    : tree_(other.tree_)
	    , pred_(other.pred_)
	    , cur_(other.cur_)
	    , start_(other.start_)
	    , nextNode(other.nextNode)
	{
	}

	TreeQueryIterator& operator++()
	{
		cur_ = nextNode(*tree_, pred_, cur_, start_);
		return *this;
	}

	TreeQueryIterator operator++(int)
	{
		TreeQueryIterator tmp(*this);
		++*this;
		return tmp;
	}

	reference operator*() const { return cur_; }

	pointer operator->() const { return &cur_; }

	template <class Predicate2>
	bool operator==(TreeQueryIterator<Tree, Predicate2> const& other)
	{
		return cur_.code == other.cur_.code;
	}

	template <class Predicate2>
	bool operator!=(TreeQueryIterator<Tree, Predicate2> const& other)
	{
		return !(*this == other);
	}

 private:
	[[nodiscard]] static auto nextNodeFun(bool only_exists, bool early_stopping)
	{
		if (only_exists && early_stopping) {
			return &TreeQueryIterator::next<true, true>;
		} else if (only_exists && !early_stopping) {
			return &TreeQueryIterator::next<true, false>;
		} else if (!only_exists && early_stopping) {
			return &TreeQueryIterator::next<false, true>;
		} else {
			return &TreeQueryIterator::next<false, false>;
		}
	}

	template <bool OnlyExists>
	[[nodiscard]] static bool traversable(Tree const& tree, Predicate const& pred,
	                                      Node node)
	{
		if constexpr (OnlyExists) {
			return tree.isParent(node.index) && Filter::traversable(pred, tree, node);
		} else {
			return 0 < node.code.depth() && Filter::traversable(pred, tree, node);
		}
	}

	template <bool OnlyExists>
	[[nodiscard]] static Node firstborn(Tree const& tree, Node node)
	{
		node.code = node.code.firstborn();
		if constexpr (OnlyExists) {
			node.index = tree.child(node.index, 0);
		} else if (tree.isParent(node.index)) {
			node.index = tree.child(node.index, 0);
		}
		return node;
	}

	template <bool OnlyExists, bool EarlyStopping>
	[[nodiscard]] static Node next(Tree const& tree, Predicate const& pred, Node node,
	                               depth_type start)
	{
		if constexpr (!EarlyStopping) {
			// Traverse down the branch

			while (traversable<OnlyExists>(tree, pred, node)) {
				node = firstborn<OnlyExists>(tree, node);

				if (Filter::returnable(pred, tree, node)) {
					return node;
				}
			}
		}

		while (true) {
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

			if (Filter::returnable(pred, tree, node)) {
				return node;
			}

			while (traversable<OnlyExists>(tree, pred, node)) {
				node = firstborn<OnlyExists>(tree, node);

				if (Filter::returnable(pred, tree, node)) {
					return node;
				}
			}
		}
	}

 private:
	Tree const* tree_;

	Predicate pred_{};

	Node       cur_{};
	depth_type start_;

	decltype(&TreeQueryIterator::next<true, true>) nextNode;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_QUERY_ITERATOR_HPP