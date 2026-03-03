---
title: Create Your Own Predicate
description: A guide to creating your own predicates.
---

# Create Your Own Predicate

Need to specialize the following class with the mentioned functions:

```cpp
template <bool Negated>
struct MyPredicate {
	// TODO: Add member variables here
	float example;
	int example2;
};

template <bool Negated>
[[nodiscard]] constexpr MyPredicate<!Negated> operator!(MyPredicate<Negated> const& p) noexcept
{
	return MyPredicate<!Negated>{p.example};
}

template <bool Negated>
struct Filter<MyPredicate<Negated>> {
	using Pred = MyPredicate<Negated>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t) noexcept
	{
		// TODO: Initialize member variables here.
		// This functions runs once when the predicate is created. So you can do some heavy calculation here.
		p.example2 = t.heavyCalc(p.example);
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnableValue(Pred const&  p,
	                                                    Value const& v) noexcept
	{
		// TODO: Check if the value is returnable.
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n) noexcept
	{
		// TODO: Check if the node is returnable.
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                typename Tree::Node const& n) noexcept
	{
		// TODO: Check if the node is traversable.
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnableRay(Pred const& p, Tree const& t,
	                                                  typename Tree::Node const& n,
	                                                  typename Tree::Ray const&  r) noexcept
	{
		// TODO: Check if the node is returnable by the ray.
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversableRay(Pred const& p, Tree const& t,
	                                                   typename Tree::Node const& n,
	                                                   typename Tree::Ray const& r) noexcept
	{
		// TODO: Check if the node is traversable by the ray.
	}
};
```
