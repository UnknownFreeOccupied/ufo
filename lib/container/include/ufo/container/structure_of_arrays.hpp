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

#ifndef UFO_CONTAINER_STRUCTURE_OF_ARRAYS_HPP
#define UFO_CONTAINER_STRUCTURE_OF_ARRAYS_HPP

// UFO
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace ufo
{
template <class... Ts>
  requires(0 < sizeof...(Ts))
class SoA;

namespace detail
{
struct access {
	template <class... Ts>
	static constexpr auto const& data(SoA<Ts...> const& soa) noexcept
	{
		return soa.data_;
	}

	template <class... Ts>
	static constexpr auto& data(SoA<Ts...>& soa) noexcept
	{
		return soa.data_;
	}

	template <class... Ts>
	static constexpr auto data(SoA<Ts...>&& soa) noexcept
	{
		return std::move(soa.data_);
	}
};

template <class T>
struct is_soa : std::false_type {
};

template <class... Ts>
struct is_soa<SoA<Ts...>> : std::true_type {
};

template <class T>
struct is_vector : std::false_type {
};

template <class T, class Alloc>
struct is_vector<std::vector<T, Alloc>> : std::true_type {
};

template <class T>
struct is_span : std::false_type {
};

template <class T, std::size_t Extent>
struct is_span<std::span<T, Extent>> : std::true_type {
};

template <class T>
constexpr inline bool is_soa_v = is_soa<T>::value;

template <class T>
constexpr inline bool is_vector_v = is_vector<T>::value;

template <class T>
constexpr inline bool is_span_v = is_span<T>::value;

template <class T>
struct is_mergable : std::disjunction<is_soa<T>, is_vector<T>, is_span<T>> {
};

template <class T>
constexpr inline bool is_mergable_v = is_mergable<T>::value;

template <class T>
  requires is_mergable_v<T>
struct extract_types {
	using type = std::tuple<std::ranges::range_value_t<T>>;
};

template <class... Ts>
struct extract_types<SoA<Ts...>> {
	using type = std::tuple<Ts...>;
};

template <class T, class Alloc>
struct extract_types<std::vector<T, Alloc>> {
	using type = std::tuple<T>;
};

template <class T>
using extract_types_t = typename extract_types<remove_cvref_t<T>>::type;

template <class... Args>
using merged_types_t = decltype(std::tuple_cat(std::declval<extract_types_t<Args>>()...));

template <class T>
struct tuple_to_soa;

template <class... Ts>
struct tuple_to_soa<std::tuple<Ts...>> {
	using type = SoA<Ts...>;
};

template <class... Args>
using merged_soa_t = typename tuple_to_soa<merged_types_t<Args...>>::type;

template <class... Ts>
constexpr auto const& wrap_data(SoA<Ts...> const& soa) noexcept
{
	return access::data(soa);
}

template <class... Ts>
constexpr auto& wrap_data(SoA<Ts...>& soa) noexcept
{
	return access::data(soa);
}

template <class... Ts>
constexpr auto wrap_data(SoA<Ts...>&& soa) noexcept
{
	return access::data(std::move(soa));
}

template <class T, class Alloc>
constexpr auto wrap_data(std::vector<T, Alloc> const& vec)
{
	return std::make_tuple(vec);
}

template <class T, class Alloc>
constexpr auto wrap_data(std::vector<T, Alloc>& vec)
{
	return std::make_tuple(vec);
}

template <class T, class Alloc>
constexpr auto wrap_data(std::vector<T, Alloc>&& vec)
{
	return std::make_tuple(std::move(vec));
}

template <class R>
  requires is_mergable_v<remove_cvref_t<R>> && (!is_soa_v<remove_cvref_t<R>>) &&
           (!is_vector_v<remove_cvref_t<R>>)
constexpr auto wrap_data(R&& r)
{
	using T = std::ranges::range_value_t<R>;
	return std::make_tuple(std::vector<T>(std::ranges::begin(r), std::ranges::end(r)));
}
}  // namespace detail

template <class... Ts>
  requires(0 < sizeof...(Ts))
class SoA
{
 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	using data_type = std::tuple<std::vector<Ts>...>;

	template <class... Us>
	using zip_view_type = std::ranges::zip_view<std::ranges::ref_view<Us>...>;

	using value_type      = std::tuple<Ts...>;
	using size_type       = std::size_t;
	using difference_type = std::ptrdiff_t;
	using reference       = std::tuple<Ts&...>;
	using const_reference = std::tuple<Ts const&...>;
	using iterator        = std::ranges::iterator_t<zip_view_type<std::vector<Ts>...>>;
	using const_iterator = std::ranges::iterator_t<zip_view_type<std::vector<Ts> const...>>;
	using reverse_iterator       = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	template <class... Us>
	  requires(0 < sizeof...(Us))
	friend class SoA;

	friend struct detail::access;

 private:
	using indices = std::index_sequence_for<Ts...>;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	constexpr SoA() noexcept = default;

	constexpr explicit SoA(size_type count) { resize(count); }

	constexpr SoA(size_type count, value_type const& value) { resize(count, value); }

	template <std::input_iterator InputIt>
	constexpr SoA(InputIt first, InputIt last)
	{
		assign(first, last);
	}

	template <std::ranges::input_range R>
	constexpr SoA(std::from_range_t, R&& range)
	{
		assign_range(std::forward<R>(range));
	}

	constexpr SoA(std::vector<Ts>... vecs) : data_(std::move(vecs)...) {}

	constexpr SoA(SoA const& other) = default;

	constexpr SoA(SoA&& other) noexcept = default;

	constexpr SoA(std::initializer_list<value_type> init) : SoA(init.begin(), init.end()) {}

	template <class... Args>
	  requires(0 < sizeof...(Args)) &&
	          (requires { detail::wrap_data(std::declval<Args>()); } && ...)
	constexpr SoA(Args&&... args)
	    : data_(std::tuple_cat(detail::wrap_data(std::forward<Args>(args))...))
	{
		if constexpr (sizeof...(Args) > 1) {
			size_type const s     = size();
			auto const      check = [s](auto const& vec) {
        if (vec.size() != s) {
          throw std::invalid_argument("SoA: merged components must have the same size");
        }
			};
			std::apply([&check](auto const&... vecs) { (check(vecs), ...); }, data_);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	constexpr ~SoA() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	constexpr SoA& operator=(SoA const& rhs) = default;

	constexpr SoA& operator=(SoA&& rhs) noexcept = default;

	constexpr SoA& operator=(std::initializer_list<value_type> ilist)
	{
		assign(ilist);
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Assign                                        |
	|                                                                                     |
	**************************************************************************************/

	constexpr void assign(size_type count, value_type const& value)
	{
		assign_impl(count, value, indices{});
	}

	template <std::input_iterator I, std::sentinel_for<I> S>
	constexpr void assign(I first, S last)
	{
		assign_range(std::ranges::subrange(first, last));
	}

	constexpr void assign(std::initializer_list<value_type> ilist)
	{
		assign(ilist.begin(), ilist.end());
	}

	template <std::ranges::input_range R>
	  requires(std::same_as<std::ranges::range_value_t<R>, value_type>)
	constexpr void assign_range(R&& range)
	{
		assign_range_impl(std::forward<R>(range), std::make_index_sequence<sizeof...(Ts)>{});
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        View                                         |
	|                                                                                     |
	**************************************************************************************/

	template <class T>
	[[nodiscard]] constexpr std::span<T> view()
	{
		return std::span{std::get<std::vector<T>>(data_)};
	}

	template <class T>
	[[nodiscard]] constexpr std::span<T const> view() const
	{
		return std::span{std::get<std::vector<T>>(data_)};
	}

	template <std::size_t I>
	[[nodiscard]] constexpr std::span<std::tuple_element_t<I, value_type>> view()
	{
		return std::span{std::get<I>(data_)};
	}

	template <std::size_t I>
	[[nodiscard]] constexpr std::span<std::tuple_element_t<I, value_type> const> view()
	    const
	{
		return std::span{std::get<I>(data_)};
	}

	/**************************************************************************************
	|                                                                                     |
	|                                   Element access                                    |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr reference operator[](size_type pos)
	{
		return operator[](pos, indices{});
	}

	[[nodiscard]] constexpr const_reference operator[](size_type pos) const
	{
		return operator[](pos, indices{});
	}

	[[nodiscard]] constexpr reference at(size_type pos)
	{
		check_index(pos);
		return operator[](pos);
	}

	[[nodiscard]] constexpr const_reference at(size_type pos) const
	{
		check_index(pos);
		return operator[](pos);
	}

	[[nodiscard]] constexpr reference front() { return operator[](0u); }

	[[nodiscard]] constexpr const_reference front() const { return operator[](0u); }

	[[nodiscard]] constexpr reference back() { return operator[](size() - 1); }

	[[nodiscard]] constexpr const_reference back() const { return operator[](size() - 1); }

	[[nodiscard]] constexpr data_type* data() noexcept { return &data_; }

	[[nodiscard]] constexpr data_type const* data() const noexcept { return &data_; }

	/**************************************************************************************
	|                                                                                     |
	|                                      Iterators                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr iterator begin() noexcept { return get_view().begin(); }

	[[nodiscard]] constexpr const_iterator begin() const noexcept
	{
		return get_view().begin();
	}

	[[nodiscard]] constexpr const_iterator cbegin() const noexcept { return begin(); }

	[[nodiscard]] constexpr iterator end() noexcept { return get_view().end(); }

	[[nodiscard]] constexpr const_iterator end() const noexcept { return get_view().end(); }

	[[nodiscard]] constexpr const_iterator cend() const noexcept { return end(); }

	[[nodiscard]] constexpr reverse_iterator rbegin() noexcept
	{
		return reverse_iterator(end());
	}

	[[nodiscard]] constexpr const_reverse_iterator rbegin() const noexcept
	{
		return const_reverse_iterator(end());
	}

	[[nodiscard]] constexpr const_reverse_iterator crbegin() const noexcept
	{
		return rbegin();
	}

	[[nodiscard]] constexpr reverse_iterator rend() noexcept
	{
		return reverse_iterator(begin());
	}

	[[nodiscard]] constexpr const_reverse_iterator rend() const noexcept
	{
		return const_reverse_iterator(begin());
	}

	[[nodiscard]] constexpr const_reverse_iterator crend() const noexcept { return rend(); }

	/**************************************************************************************
	|                                                                                     |
	|                                      Capacity                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr bool empty() const noexcept
	{
		return std::get<0>(data_).empty();
	}

	[[nodiscard]] constexpr size_type size() const noexcept
	{
		return std::get<0>(data_).size();
	}

	[[nodiscard]] constexpr size_type max_size() const noexcept
	{
		return max_size_impl(indices{});
	}

	constexpr void reserve(size_type new_cap) { reserve(new_cap, indices{}); }

	[[nodiscard]] constexpr size_type capacity() const noexcept
	{
		return capacity_impl(indices{});
	}

	constexpr void shrink_to_fit() { shrink_to_fit_impl(indices{}); }

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	constexpr void clear() noexcept { clear_impl(indices{}); }

	constexpr iterator insert(iterator pos, value_type const& value)
	{
		return insert_impl(pos, value, indices{});
	}

	constexpr iterator insert(iterator pos, value_type&& value)
	{
		return insert_impl(pos, std::move(value), indices{});
	}

	constexpr iterator insert(iterator pos, size_type count, value_type const& value)
	{
		return insert_impl(pos, count, value, indices{});
	}

	template <std::input_iterator I, std::sentinel_for<I> S>
	constexpr iterator insert(iterator pos, I first, S last)
	{
		return insert_impl(pos, first, last, indices{});
	}

	constexpr iterator insert(iterator pos, std::initializer_list<value_type> ilist)
	{
		return insert(pos, ilist.begin(), ilist.end());
	}

	template <std::ranges::input_range R>
	constexpr iterator insert_range(iterator pos, R&& rg)
	{
		return insert_range_impl(pos, std::forward<R>(rg), indices{});
	}

	template <class... Us>
	constexpr iterator emplace(iterator pos, Us&&... us)
	{
		static_assert(sizeof...(Us) == sizeof...(Ts), "SoA: invalid number of arguments");
		std::size_t const idx = std::distance(begin(), pos);
		emplace_impl(pos, std::forward<Us>(us)...);
		return begin() + idx;
	}

	constexpr iterator erase(iterator pos) { return erase_impl(pos, indices{}); }

	constexpr iterator erase(iterator first, iterator last)
	{
		return erase_impl(first, last, indices{});
	}

	constexpr void push_back(value_type const& value) { push_back_impl(value, indices{}); }

	constexpr void push_back(value_type&& value)
	{
		push_back_impl(std::move(value), indices{});
	}

	template <class... Us>
	constexpr void push_back(Us&&... us)
	{
		emplace_back(std::forward<Us>(us)...);
	}

	// TODO: Check so each Us is of type Ts
	template <class... Us>
	constexpr reference emplace_back(Us&&... us)
	{
		static_assert(sizeof...(Us) == sizeof...(Ts), "SoA: invalid number of arguments");
		emplace_back_impl(std::forward<Us>(us)...);
		return back();
	}

	template <std::ranges::input_range R>
	constexpr void append_range(R&& rg)
	{
		append_range_impl(std::forward<R>(rg), indices{});
	}

	constexpr void pop_back() { pop_back_impl(indices{}); }

	constexpr void resize(size_type count) { resize_impl(count, indices{}); }

	constexpr void resize(size_type count, value_type const& value)
	{
		resize_impl(count, value, indices{});
	}

	constexpr void swap(SoA& other) noexcept { swap_impl(other, indices{}); }

	friend constexpr void swap(SoA& lhs, SoA& rhs) noexcept { lhs.swap(rhs); }

	template <std::size_t I>
	[[nodiscard]] constexpr std::tuple_element_t<I, value_type>& get(size_type pos)
	{
		return std::get<I>(data_)[pos];
	}

	template <std::size_t I>
	[[nodiscard]] constexpr std::tuple_element_t<I, value_type> const& get(
	    size_type pos) const
	{
		return std::get<I>(data_)[pos];
	}

	template <class T>
	[[nodiscard]] constexpr T& get(size_type pos)
	{
		static_assert(contains_type_v<T, Ts...>, "SoA: type not found");
		static_assert(is_unique_v<Ts...>, "SoA: type is not unique, use get<I>(pos) instead");
		return std::get<index_v<T, Ts...>>(data_)[pos];
	}

	template <class T>
	[[nodiscard]] constexpr T const& get(size_type pos) const
	{
		static_assert(contains_type_v<T, Ts...>, "SoA: type not found");
		static_assert(is_unique_v<Ts...>, "SoA: type is not unique, use get<I>(pos) instead");
		return std::get<index_v<T, Ts...>>(data_)[pos];
	}

 private:
	constexpr void check_index(size_type pos) const
	{
		if (size() <= pos) {
			throw std::out_of_range("SoA::at: index " + std::to_string(pos) +
			                        " is out of range for size " + std::to_string(size()));
		}
	}

	template <std::size_t... Is>
	constexpr void assign_impl(size_type count, value_type const& value,
	                           std::index_sequence<Is...>)
	{
		(std::get<Is>(data_).assign(count, std::get<Is>(value)), ...);
	}

	template <std::ranges::input_range R, std::size_t... Is>
	constexpr void assign_range_impl(R&& range, std::index_sequence<Is...>)
	{
		(std::get<Is>(data_).assign(std::views::elements<Is>(range).begin(),
		                            std::views::elements<Is>(range).end()),
		 ...);
	}

	template <std::size_t... Is>
	[[nodiscard]] constexpr reference operator[](size_type pos, std::index_sequence<Is...>)
	{
		return std::tie(std::get<Is>(data_)[pos]...);
	}

	template <std::size_t... Is>
	[[nodiscard]] constexpr const_reference operator[](size_type pos,
	                                                   std::index_sequence<Is...>) const
	{
		return std::tie(std::get<Is>(data_)[pos]...);
	}

	template <std::size_t... Is>
	[[nodiscard]] constexpr size_type max_size_impl(
	    std::index_sequence<Is...>) const noexcept
	{
		return std::min({std::get<Is>(data_).max_size()...});
	}

	template <std::size_t... Is>
	constexpr void reserve_impl(size_type new_cap, std::index_sequence<Is...>)
	{
		(std::get<Is>(data_).reserve(new_cap), ...);
	}

	template <std::size_t... Is>
	[[nodiscard]] constexpr size_type capacity_impl(
	    std::index_sequence<Is...>) const noexcept
	{
		return std::min({std::get<Is>(data_).capacity()...});
	}

	template <std::size_t... Is>
	constexpr void shrink_to_fit_impl(std::index_sequence<Is...>)
	{
		(std::get<Is>(data_).shrink_to_fit(), ...);
	}

	template <std::size_t... Is>
	constexpr void clear_impl(std::index_sequence<Is...>) noexcept
	{
		(std::get<Is>(data_).clear(), ...);
	}

	template <std::size_t... Is>
	constexpr iterator insert_impl(iterator pos, value_type const& value,
	                               std::index_sequence<Is...>)
	{
		std::size_t const idx = std::distance(begin(), pos);
		(std::get<Is>(data_).insert(std::get<Is>(data_).begin() + idx, std::get<Is>(value)),
		 ...);
		return begin() + idx;
	}

	template <std::size_t... Is>
	constexpr iterator insert_impl(iterator pos, value_type&& value,
	                               std::index_sequence<Is...>)
	{
		std::size_t const idx = std::distance(begin(), pos);
		(std::get<Is>(data_).insert(std::get<Is>(data_).begin() + idx,
		                            std::move(std::get<Is>(value))),
		 ...);
		return begin() + idx;
	}

	template <std::size_t... Is>
	constexpr iterator insert_impl(iterator pos, size_type count, value_type const& value,
	                               std::index_sequence<Is...>)
	{
		std::size_t const idx = std::distance(begin(), pos);
		(std::get<Is>(data_).insert(std::get<Is>(data_).begin() + idx, count,
		                            std::get<Is>(value)),
		 ...);
		return begin() + idx;
	}

	template <std::input_iterator I, std::sentinel_for<I> S, std::size_t... Is>
	constexpr iterator insert_impl(iterator pos, I first, S last,
	                               std::index_sequence<Is...>)
	{
		std::size_t const idx = std::distance(begin(), pos);
		auto              rg  = std::ranges::subrange(first, last);
		(std::get<Is>(data_).insert(std::get<Is>(data_).begin() + idx,
		                            std::views::elements<Is>(rg).begin(),
		                            std::views::elements<Is>(rg).end()),
		 ...);
		return begin() + idx;
	}

	template <std::ranges::input_range R, std::size_t... Is>
	constexpr iterator insert_range_impl(iterator pos, R&& rg, std::index_sequence<Is...>)
	{
		std::size_t const idx = std::distance(begin(), pos);
		(std::get<Is>(data_).insert(std::get<Is>(data_).begin() + idx,
		                            std::views::elements<Is>(rg).begin(),
		                            std::views::elements<Is>(rg).end()),
		 ...);
		return begin() + idx;
	}

	template <class... Us>
	constexpr void emplace_impl(iterator pos, Us&&... us)
	{
		emplace_impl(pos, indices{}, std::forward<Us>(us)...);
	}

	template <std::size_t... Is, class... Us>
	constexpr void emplace_impl(iterator pos, std::index_sequence<Is...>, Us&&... us)
	{
		std::size_t const idx = std::distance(begin(), pos);
		(std::get<Is>(data_).emplace(std::get<Is>(data_).begin() + idx, std::forward<Us>(us)),
		 ...);
	}

	template <std::size_t... Is>
	constexpr iterator erase_impl(iterator pos, std::index_sequence<Is...>)
	{
		auto const idx = std::distance(begin(), pos);
		(std::get<Is>(data_).erase(std::get<Is>(data_).begin() + idx), ...);
		return begin() + idx;
	}

	template <std::size_t... Is>
	constexpr iterator erase_impl(iterator first, iterator last, std::index_sequence<Is...>)
	{
		auto const f_idx = std::distance(begin(), first);
		auto const l_idx = std::distance(begin(), last);
		(std::get<Is>(data_).erase(std::get<Is>(data_).begin() + f_idx,
		                           std::get<Is>(data_).begin() + l_idx),
		 ...);
		return begin() + f_idx;
	}

	template <std::size_t... Is>
	constexpr void push_back_impl(value_type const& value, std::index_sequence<Is...>)
	{
		(std::get<Is>(data_).push_back(std::get<Is>(value)), ...);
	}

	template <std::size_t... Is>
	constexpr void push_back_impl(value_type&& value, std::index_sequence<Is...>)
	{
		(std::get<Is>(data_).push_back(std::move(std::get<Is>(value))), ...);
	}

	template <class... Us>
	constexpr void emplace_back_impl(Us&&... us)
	{
		emplace_back_impl(indices{}, std::forward<Us>(us)...);
	}

	template <std::size_t... Is, class... Us>
	constexpr void emplace_back_impl(std::index_sequence<Is...>, Us&&... us)
	{
		(std::get<Is>(data_).emplace_back(std::forward<Us>(us)), ...);
	}

	template <std::ranges::input_range R, std::size_t... Is>
	constexpr void append_range_impl(R&& range, std::index_sequence<Is...>)
	{
		(std::get<Is>(data_).append_range(std::views::elements<Is>(range)), ...);
	}

	template <std::size_t... Is>
	constexpr void pop_back_impl(std::index_sequence<Is...>)
	{
		(std::get<Is>(data_).pop_back(), ...);
	}

	template <std::size_t... Is>
	constexpr void resize_impl(size_type count, std::index_sequence<Is...>)
	{
		(std::get<Is>(data_).resize(count), ...);
	}

	template <std::size_t... Is>
	constexpr void resize_impl(size_type count, value_type const& value,
	                           std::index_sequence<Is...>)
	{
		(std::get<Is>(data_).resize(count, std::get<Is>(value)), ...);
	}

	template <std::size_t... Is>
	constexpr void swap_impl(SoA& other, std::index_sequence<Is...>) noexcept
	{
		(std::get<Is>(data_).swap(std::get<Is>(other.data_)), ...);
	}

	[[nodiscard]] constexpr auto get_view() noexcept { return get_view(indices{}); }

	[[nodiscard]] constexpr auto get_view() const noexcept { return get_view(indices{}); }

	template <std::size_t... Is>
	[[nodiscard]] constexpr auto get_view(std::index_sequence<Is...>) noexcept
	{
		return std::views::zip(std::ranges::ref_view(std::get<Is>(data_))...);
	}

	template <std::size_t... Is>
	[[nodiscard]] constexpr auto get_view(std::index_sequence<Is...>) const noexcept
	{
		return std::views::zip(std::ranges::ref_view(std::get<Is>(data_))...);
	}

	data_type data_;
};

template <class... Ts>
SoA(std::initializer_list<std::tuple<Ts...>>) -> SoA<Ts...>;

template <class... Ts>
SoA(std::size_t, std::tuple<Ts...>) -> SoA<Ts...>;

template <class... Ts>
SoA(std::vector<Ts>...) -> SoA<Ts...>;

// 2-argument merging
template <class... Ts, class... Us>
SoA(SoA<Ts...>, SoA<Us...>) -> SoA<Ts..., Us...>;

template <class... Ts, class U, class Alloc>
SoA(SoA<Ts...>, std::vector<U, Alloc>) -> SoA<Ts..., U>;

template <class T, class Alloc, class... Us>
SoA(std::vector<T, Alloc>, SoA<Us...>) -> SoA<T, Us...>;

template <class T1, class Alloc1, class T2, class Alloc2>
SoA(std::vector<T1, Alloc1>, std::vector<T2, Alloc2>) -> SoA<T1, T2>;

// 3-argument merging
template <class... Ts, class... Us, class... Vs>
SoA(SoA<Ts...>, SoA<Us...>, SoA<Vs...>) -> SoA<Ts..., Us..., Vs...>;

template <class... Ts, class... Us, class V, class Alloc>
SoA(SoA<Ts...>, SoA<Us...>, std::vector<V, Alloc>) -> SoA<Ts..., Us..., V>;

template <class... Ts, class U, class Alloc, class... Vs>
SoA(SoA<Ts...>, std::vector<U, Alloc>, SoA<Vs...>) -> SoA<Ts..., U, Vs...>;

template <class T, class Alloc, class... Us, class... Vs>
SoA(std::vector<T, Alloc>, SoA<Us...>, SoA<Vs...>) -> SoA<T, Us..., Vs...>;

template <class... Ts, class U1, class Alloc1, class U2, class Alloc2>
SoA(SoA<Ts...>, std::vector<U1, Alloc1>, std::vector<U2, Alloc2>) -> SoA<Ts..., U1, U2>;

template <class T1, class Alloc1, class... Us, class T2, class Alloc2>
SoA(std::vector<T1, Alloc1>, SoA<Us...>, std::vector<T2, Alloc2>) -> SoA<T1, Us..., T2>;

template <class T1, class Alloc1, class T2, class Alloc2, class... Vs>
SoA(std::vector<T1, Alloc1>, std::vector<T2, Alloc2>, SoA<Vs...>) -> SoA<T1, T2, Vs...>;

template <class T1, class Alloc1, class T2, class Alloc2, class T3, class Alloc3>
SoA(std::vector<T1, Alloc1>, std::vector<T2, Alloc2>, std::vector<T3, Alloc3>)
    -> SoA<T1, T2, T3>;

//
// Non-member functions
//

template <class... Ts>
[[nodiscard]] constexpr auto begin(SoA<Ts...>& c)
{
	return c.begin();
}

template <class... Ts>
[[nodiscard]] constexpr auto begin(SoA<Ts...> const& c)
{
	return c.begin();
}

template <class... Ts>
[[nodiscard]] constexpr auto end(SoA<Ts...>& c)
{
	return c.end();
}

template <class... Ts>
[[nodiscard]] constexpr auto end(SoA<Ts...> const& c)
{
	return c.end();
}

template <class... Ts>
[[nodiscard]] constexpr auto size(SoA<Ts...> const& c)
{
	return c.size();
}

template <class... Ts, class U>
constexpr typename SoA<Ts...>::size_type erase(SoA<Ts...>& c, U const& value)
{
	auto it = std::remove(c.begin(), c.end(), value);
	auto r  = std::distance(it, c.end());
	c.erase(it, c.end());
	return r;
}

template <class... Ts, class Pred>
constexpr typename SoA<Ts...>::size_type erase_if(SoA<Ts...>& c, Pred pred)
{
	auto it = std::remove_if(c.begin(), c.end(), pred);
	auto r  = std::distance(it, c.end());
	c.erase(it, c.end());
	return r;
}

template <class... Ts>
[[nodiscard]] constexpr bool operator==(SoA<Ts...> const& lhs, SoA<Ts...> const& rhs)
{
	return lhs.size() == rhs.size() && std::ranges::equal(lhs, rhs);
}

template <class... Ts>
[[nodiscard]] constexpr auto operator<=>(SoA<Ts...> const& lhs, SoA<Ts...> const& rhs)
{
	return std::lexicographical_compare_three_way(lhs.begin(), lhs.end(), rhs.begin(),
	                                              rhs.end());
}

template <class... Args>
  requires(0 < sizeof...(Args)) && (detail::is_soa_v<remove_cvref_t<Args>> || ...) &&
          (requires { detail::wrap_data(std::declval<Args>()); } && ...)
[[nodiscard]] constexpr auto merge(Args&&... args)
{
	return detail::merged_soa_t<Args...>(std::forward<Args>(args)...);
}

//
// Type traits
//

template <class T, class... Ts>
struct contains_type<T, SoA<Ts...>> : contains_type<T, Ts...> {
};

template <class T, class... Ts>
[[nodiscard]] constexpr auto view(SoA<Ts...>& soa)
{
	return soa.template view<T>();
}

template <class T, class... Ts>
[[nodiscard]] constexpr auto view(SoA<Ts...> const& soa)
{
	return soa.template view<T>();
}

template <std::size_t I, class... Ts>
[[nodiscard]] constexpr auto view(SoA<Ts...>& soa)
{
	return soa.template view<I>();
}

template <std::size_t I, class... Ts>
[[nodiscard]] constexpr auto view(SoA<Ts...> const& soa)
{
	return soa.template view<I>();
}
}  // namespace ufo

#endif  // UFO_CONTAINER_STRUCTURE_OF_ARRAYS_HPP