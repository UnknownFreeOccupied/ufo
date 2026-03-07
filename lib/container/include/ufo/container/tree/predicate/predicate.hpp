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

#ifndef UFO_CONTAINER_TREE_PREDICATE_PREDICATE_HPP
#define UFO_CONTAINER_TREE_PREDICATE_PREDICATE_HPP

// UFO
#include <ufo/container/tree/predicate/filter.hpp>

// STL
#include <memory>
#include <type_traits>

namespace ufo::pred
{
namespace detail
{
template <class Tree, class = void>
class Dynamic
{
 public:
	virtual ~Dynamic() {}

	virtual void init(Tree const&) = 0;

	[[nodiscard]] virtual bool returnable(Tree const&,
	                                      typename Tree::Node const&) const = 0;

	[[nodiscard]] virtual bool traversable(Tree const&,
	                                       typename Tree::Node const&) const = 0;

	[[nodiscard]] virtual bool returnableRay(Tree const&, typename Tree::Node const&,
	                                         typename Tree::Ray const&) const = 0;

	[[nodiscard]] virtual bool traversableRay(Tree const&, typename Tree::Node const&,
	                                          typename Tree::Ray const&) const = 0;

	[[nodiscard]] virtual Dynamic* clone() const = 0;
};

template <class Tree>
class Dynamic<Tree, std::void_t<typename Tree::value_type>>
{
 public:
	virtual ~Dynamic() {}

	virtual void init(Tree const&) = 0;

	[[nodiscard]] virtual bool returnableValue(typename Tree::value_type const&) const = 0;

	[[nodiscard]] virtual bool returnable(Tree const&,
	                                      typename Tree::Node const&) const = 0;

	[[nodiscard]] virtual bool traversable(Tree const&,
	                                       typename Tree::Node const&) const = 0;

	[[nodiscard]] virtual bool returnableRay(Tree const&, typename Tree::Node const&,
	                                         typename Tree::Ray const&) const = 0;

	[[nodiscard]] virtual bool traversableRay(Tree const&, typename Tree::Node const&,
	                                          typename Tree::Ray const&) const = 0;

	[[nodiscard]] virtual Dynamic* clone() const = 0;
};

template <class Tree, class Predicate, class = void>
class DynamicPredicate
    : public Dynamic<Tree>
    , public Predicate
{
 public:
	using Predicate::Predicate;

	DynamicPredicate(Tree const&, Predicate const& pred) : Predicate(pred) {}

	DynamicPredicate(Tree const&, Predicate&& pred) : Predicate(std::move(pred)) {}

	virtual ~DynamicPredicate() {}

	void init(Tree const& t) override
	{
		Filter<Predicate>::init(static_cast<Predicate&>(*this), t);
	}

	[[nodiscard]] bool returnable(Tree const&                t,
	                              typename Tree::Node const& n) const override
	{
		return Filter<Predicate>::returnable(static_cast<Predicate const&>(*this), t, n);
	}

	[[nodiscard]] bool traversable(Tree const&                t,
	                               typename Tree::Node const& n) const override
	{
		return Filter<Predicate>::traversable(static_cast<Predicate const&>(*this), t, n);
	}

	[[nodiscard]] bool returnableRay(Tree const& t, typename Tree::Node const& n,
	                                 typename Tree::Ray const& r) const override
	{
		return Filter<Predicate>::returnableRay(static_cast<Predicate const&>(*this), t, n,
		                                        r);
	}

	[[nodiscard]] bool traversableRay(Tree const& t, typename Tree::Node const& n,
	                                  typename Tree::Ray const& r) const override
	{
		return Filter<Predicate>::traversableRay(static_cast<Predicate const&>(*this), t, n,
		                                         r);
	}

 protected:
	[[nodiscard]] DynamicPredicate* clone() const override
	{
		return new DynamicPredicate(*this);
	}
};

template <class Tree, class Predicate>
class DynamicPredicate<Tree, Predicate, std::void_t<typename Tree::value_type>>
    : public Dynamic<Tree>
    , public Predicate
{
 public:
	using Predicate::Predicate;

	DynamicPredicate(Tree const&, Predicate const& pred) : Predicate(pred) {}

	DynamicPredicate(Tree const&, Predicate&& pred) : Predicate(std::move(pred)) {}

	virtual ~DynamicPredicate() {}

	void init(Tree const& t) override
	{
		Filter<Predicate>::init(static_cast<Predicate&>(*this), t);
	}

	[[nodiscard]] bool returnableValue(typename Tree::value_type const& v) const override
	{
		return Filter<Predicate>::returnableValue(static_cast<Predicate const&>(*this), v);
	}

	[[nodiscard]] bool returnable(Tree const&                t,
	                              typename Tree::Node const& n) const override
	{
		return Filter<Predicate>::returnable(static_cast<Predicate const&>(*this), t, n);
	}

	[[nodiscard]] bool traversable(Tree const&                t,
	                               typename Tree::Node const& n) const override
	{
		return Filter<Predicate>::traversable(static_cast<Predicate const&>(*this), t, n);
	}

	[[nodiscard]] bool returnableRay(Tree const& t, typename Tree::Node const& n,
	                                 typename Tree::Ray const& r) const override
	{
		return Filter<Predicate>::returnableRay(static_cast<Predicate const&>(*this), t, n,
		                                        r);
	}

	[[nodiscard]] bool traversableRay(Tree const& t, typename Tree::Node const& n,
	                                  typename Tree::Ray const& r) const override
	{
		return Filter<Predicate>::traversableRay(static_cast<Predicate const&>(*this), t, n,
		                                         r);
	}

 protected:
	[[nodiscard]] DynamicPredicate* clone() const override
	{
		return new DynamicPredicate(*this);
	}
};
}  // namespace detail

template <class Tree>
class Predicate
{
 public:
	Predicate() = default;

	Predicate(Predicate const& other)
	{
		if (other.hasPredicate()) {
			predicate_.reset(other.predicate_->clone());
		}
	}

	template <class Pred>
	  requires Filterable<std::remove_cvref_t<Pred>>
	Predicate(Pred&& pred)
	    : predicate_(
	          std::make_unique<detail::DynamicPredicate<Tree, std::remove_cvref_t<Pred>>>(
	              std::forward<Pred>(pred)))
	{
	}

	Predicate(Predicate&&) = default;

	Predicate(detail::Dynamic<Tree> const& pred) : predicate_(pred.clone()) {}

	Predicate& operator=(Predicate const& rhs)
	{
		if (rhs.hasPredicate()) {
			predicate_.reset(rhs.predicate_->clone());
		} else {
			predicate_.reset();
		}
		return *this;
	}

	Predicate& operator=(Predicate&&) = default;

	Predicate& operator=(detail::Dynamic<Tree> const& rhs)
	{
		predicate_.reset(rhs.clone());
		return *this;
	}

	template <class Pred>
	  requires Filterable<std::remove_cvref_t<Pred>>
	Predicate& operator=(Pred&& pred)
	{
		predicate_ =
		    std::make_unique<detail::DynamicPredicate<Tree, std::remove_cvref_t<Pred>>>(
		        std::forward<Pred>(pred));
		return *this;
	}

	Predicate& operator&=(Predicate const& rhs)
	{
		if (rhs.hasPredicate()) {
			if (hasPredicate()) {
				*this = (*this && rhs);
			} else {
				predicate_.reset(rhs.predicate_->clone());
			}
		}
		return *this;
	}

	Predicate& operator|=(Predicate const& rhs)
	{
		if (rhs.hasPredicate()) {
			if (hasPredicate()) {
				*this = (*this || rhs);
			} else {
				predicate_.reset(rhs.predicate_->clone());
			}
		}
		return *this;
	}

	[[nodiscard]] bool hasPredicate() const { return !!predicate_; }

	void init(Tree const& t)
	{
		if (hasPredicate()) {
			predicate_->init(t);
		}
	}

	[[nodiscard]] bool returnableValue(typename Tree::value_type const& v) const
	  requires requires { typename Tree::value_type; }
	{
		return !hasPredicate() || predicate_->returnableValue(v);
	}

	[[nodiscard]] bool returnable(Tree const& t, typename Tree::Node const& n) const
	{
		return !hasPredicate() || predicate_->returnable(t, n);
	}

	[[nodiscard]] bool traversable(Tree const& t, typename Tree::Node const& n) const
	{
		return !hasPredicate() || predicate_->traversable(t, n);
	}

	[[nodiscard]] bool returnableRay(Tree const& t, typename Tree::Node const& n,
	                                 typename Tree::Ray const& r) const
	{
		return !hasPredicate() || predicate_->returnableRay(t, n, r);
	}

	[[nodiscard]] bool traversableRay(Tree const& t, typename Tree::Node const& n,
	                                  typename Tree::Ray const& r) const
	{
		return !hasPredicate() || predicate_->traversableRay(t, n, r);
	}

 private:
	std::unique_ptr<detail::Dynamic<Tree>> predicate_;
};

//
// Filter
//

template <class Tree>
struct Filter<Predicate<Tree>> {
	using Pred = Predicate<Tree>;

	static void init(Pred& p, Tree const& t) { p.init(t); }

	[[nodiscard]] static bool returnableValue(Pred const&                      p,
	                                          typename Tree::value_type const& v)
	  requires requires { typename Tree::value_type; }
	{
		return p.returnableValue(v);
	}

	[[nodiscard]] static bool returnable(Pred const& p, Tree const& t,
	                                     typename Tree::Node const& n)
	{
		return p.returnable(t, n);
	}

	[[nodiscard]] static bool traversable(Pred const& p, Tree const& t,
	                                      typename Tree::Node const& n)
	{
		return p.traversable(t, n);
	}

	[[nodiscard]] static bool returnableRay(Pred const& p, Tree const& t,
	                                        typename Tree::Node const& n,
	                                        typename Tree::Ray const&  r)
	{
		return p.returnableRay(t, n, r);
	}

	[[nodiscard]] static bool traversableRay(Pred const& p, Tree const& t,
	                                         typename Tree::Node const& n,
	                                         typename Tree::Ray const&  r)
	{
		return p.traversableRay(t, n, r);
	}
};
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_PREDICATE_HPP