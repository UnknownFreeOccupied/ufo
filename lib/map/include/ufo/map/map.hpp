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

#ifndef UFO_MAP_HPP
#define UFO_MAP_HPP

// UFO
#include <ufo/compute/compute.hpp>
#include <ufo/container/tree/block.hpp>
#include <ufo/container/tree/predicate.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/execution/algorithm.hpp>
#include <ufo/map/header.hpp>
#include <ufo/map/predicate_map_type.hpp>
#include <ufo/map/serialized_block.hpp>
#include <ufo/map/type.hpp>
#include <ufo/map/utility.hpp>
#include <ufo/utility/enum.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/macros.hpp>

// STL
#include <algorithm>
#include <bitset>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <ios>
#include <numeric>
#include <tuple>
#include <type_traits>
#include <utility>

namespace ufo
{
namespace detail
{
template <class Map>
class MapHelper : public Map
{
};

template <class... Maps>
class MapHelper<std::tuple<Maps...>> : public Maps...
{
};

template <std::size_t BF>
struct Fulfill {
	TreeIndex::pos_type block;
	BitSet<BF>          offsets;
	BitSet<BF>          children;
};
}  // namespace detail

// All your base are belong to us
template <std::size_t Dim, class... Maps>
class Map final
    : public Tree<Map<Dim, Maps...>, Dim, typename Maps::Block...>
    , public detail::MapHelper<typename Maps::template Map<
          Map<Dim, Maps...>, Tree<Map<Dim, Maps...>, Dim, typename Maps::Block...>>>...
{
	using Base = Tree<Map<Dim, Maps...>, Dim, typename Maps::Block...>;

	using MapBases = decltype(std::tuple_cat(
	    std::declval<as_tuple_t<typename Maps::template Map<Map, Base>>>()...));

	using MapBasesISeq = std::make_index_sequence<std::tuple_size_v<MapBases>>;

	static constexpr auto const BF = Base::branchingFactor();

	template <class T>
	static constexpr inline bool const is_node_type_v = Base::template is_node_type_v<T>;

	//
	// Friends
	//

	friend Base;

#define UFO_MAP_FRIEND(F)                                                               \
	friend std::tuple_element_t<std::min(static_cast<std::size_t>(F + 1),                 \
	                                     MapBasesISeq::size()),                           \
	                            decltype(std::tuple_cat(std::declval<std::tuple<void>>(), \
	                                                    std::declval<MapBases>()))>;
	UFO_REPEAT_128(UFO_MAP_FRIEND, 0)

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	using Index       = typename Base::Index;
	using Node        = typename Base::Node;
	using Code        = typename Base::Code;
	using Key         = typename Base::Key;
	using Point       = typename Base::Point;
	using Coord       = typename Base::Coord;
	using Bounds      = typename Base::Bounds;
	using Length      = typename Base::Length;
	using coord_type  = typename Base::coord_type;
	using depth_type  = typename Base::depth_type;
	using pos_type    = typename Base::pos_type;
	using offset_type = typename Base::offset_type;
	using length_type = typename Base::length_type;

	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	Map(Length     leaf_node_length = Length(0.1),
	    depth_type num_depth_levels = std::min(depth_type(17), Base::maxNumDepthLevels()))
	    : Base(leaf_node_length, num_depth_levels)
	{
		onInitRoot();
	}

	Map(length_type leaf_node_length,
	    depth_type  num_depth_levels = std::min(depth_type(17), Base::maxNumDepthLevels()))
	    : Map(Length(leaf_node_length), num_depth_levels)
	{
	}

	Map(std::filesystem::path const& file) : Map() { read(file); }

	Map(std::istream& in) : Map() { read(in); }

	Map(ReadBuffer& in) : Map() { read(in); }

	Map(Map const&) = default;

	Map(Map&&) = default;

	template <class... Maps2>
	Map(Map<Dim, Maps2...> const& other) : Map(other.write(pred::Leaf{}, mapTypes()))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~Map() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	Map& operator=(Map const&) = default;

	Map& operator=(Map&&) = default;

	template <class... Maps2>
	Map& operator=(Map<Dim, Maps2...> const& rhs)
	{
		read(rhs.write(pred::Leaf{}, mapTypes()));
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Map Types                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] static constexpr MapType mapTypes() noexcept
	{
		return mapTypes(MapBasesISeq{});
	}

	[[nodiscard]] static constexpr std::size_t numMapTypes() noexcept
	{
		return MapBasesISeq::size();
	}

	[[nodiscard]] static constexpr bool hasMapTypes(MapType map_types) noexcept
	{
		return (mapTypes() & map_types) == map_types;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Propagate                                      |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Propagate modified information up the tree.
	 */
	void propagate(MapType map_types = MapType::ALL)
	{
		propagate(Base::index(), map_types);
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void propagate(NodeType const& node, MapType map_types = MapType::ALL)
	{
		auto n = Base::index(node);

		if (Base::isPureLeaf(n)) {
			return;
		}

		// Const access more optimized
		auto const& b = Base::treeInnerBlockConst(n.pos);
		auto const  c = Base::children(b, n.offset);

		if (!Base::valid(c) || !Base::modified(b, n.offset)) {
			return;
		}

		propagate(c, map_types);
		onPropagateChildren(n, c, map_types);

		if (onIsPrunable(c)) {
			Base::pruneChildren(n, c);
		}
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void propagate(ExecutionPolicy&& policy, MapType map_types = MapType::ALL)
	{
		propagate(std::forward<ExecutionPolicy>(policy), Base::index(), map_types);
	}

	template <
	    class ExecutionPolicy, class NodeType,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<is_node_type_v<NodeType>, bool>                          = true>
	void propagate(ExecutionPolicy&& policy, NodeType node,
	               MapType map_types = MapType::ALL)
	{
		auto n = Base::index(node);

		if (Base::isPureLeaf(n)) {
			return;
		}

		// Const access more optimized
		auto const& b = Base::treeInnerBlockConst(n.pos);
		auto const  c = Base::children(b, n.offset);

		if (!Base::valid(c) || !Base::modified(b, n.offset)) {
			return;
		}

		propagate(std::forward<ExecutionPolicy>(policy), c, map_types, false);
		onPropagateChildren(n, c, map_types);

		if (onIsPrunable(c)) {
			Base::pruneChildren(n, c);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] MapHeader header(MapType map_types = MapType::ALL) const
	{
		return MapHeader(Base::length(0), Base::numDepthLevels(), mapTypes() & map_types);
	}

	[[nodiscard]] static bool isMap(std::filesystem::path const& file)
	{
		return MapHeader::isMap(file);
	}

	[[nodiscard]] static bool isMap(std::istream& in) { return MapHeader::isMap(in); }

	[[nodiscard]] static bool isMap(ReadBuffer& in) { return MapHeader::isMap(in); }

	void read(std::filesystem::path const& file, MapType map_types = MapType::ALL,
	          bool propagate = true)
	{
		std::ifstream f = MapHeader::openRead(file);
		read(f, map_types, propagate);
	}

	void read(std::istream& in, MapType map_types = MapType::ALL, bool propagate = true)
	{
		readData(in, readHeader(in), map_types, propagate);
	}

	void read(ReadBuffer& in, MapType map_types = MapType::ALL, bool propagate = true)
	{
		readData(in, readHeader(in), map_types, propagate);
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void read(ExecutionPolicy&& policy, std::filesystem::path const& file,
	          MapType map_types = MapType::ALL, bool propagate = true)
	{
		std::ifstream f = MapHeader::openRead(file);
		read(std::forward<ExecutionPolicy>(policy), f, map_types, propagate);
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void read(ExecutionPolicy&& policy, std::istream& in, MapType map_types = MapType::ALL,
	          bool propagate = true)
	{
		readData(std::forward<ExecutionPolicy>(policy), in, readHeader(in), map_types,
		         propagate);
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void read(ExecutionPolicy&& policy, ReadBuffer& in, MapType map_types = MapType::ALL,
	          bool propagate = true)
	{
		readData(std::forward<ExecutionPolicy>(policy), in, readHeader(in), map_types,
		         propagate);
	}

	[[nodiscard]] MapHeader readHeader(std::filesystem::path const& file) const
	{
		return MapHeader{file};
	}

	[[nodiscard]] MapHeader readHeader(std::istream& in) const { return MapHeader{in}; }

	[[nodiscard]] MapHeader readHeader(ReadBuffer& in) const { return MapHeader{in}; }

	void readData(std::istream& in, MapHeader const& header,
	              MapType map_types = MapType::ALL, bool propagate = true)
	{
		readData(execution::seq, in, header, map_types, propagate);
	}

	void readData(ReadBuffer& in, MapHeader const& header, MapType map_types = MapType::ALL,
	              bool propagate = true)
	{
		readData(execution::seq, in, header, map_types, propagate);
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void readData(ExecutionPolicy&& policy, std::istream& in, MapHeader const& header,
	              MapType map_types = MapType::ALL, bool propagate = true)
	{
		readDataImpl(std::forward<ExecutionPolicy>(policy), in, header, map_types, propagate);
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void readData(ExecutionPolicy&& policy, ReadBuffer& in, MapHeader const& header,
	              MapType map_types = MapType::ALL, bool propagate = true)
	{
		readDataImpl(std::forward<ExecutionPolicy>(policy), in, header, map_types, propagate);
	}

	template <class Predicate                                    = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void write(std::filesystem::path const& file, Predicate const& pred = pred::Leaf{},
	           MapType map_types = MapType::ALL) const
	{
		write(execution::seq, file, pred, map_types);
	}

	template <class Predicate                                    = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void write(std::ostream& out, Predicate const& pred = pred::Leaf{},
	           MapType map_types = MapType::ALL) const
	{
		write(execution::seq, out, pred, map_types);
	}

	template <class Predicate                                    = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void write(WriteBuffer& out, Predicate const& pred = pred::Leaf{},
	           MapType map_types = MapType::ALL) const
	{
		write(execution::seq, out, pred, map_types);
	}

	template <class Predicate                                    = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] Buffer write(Predicate const& pred      = pred::Leaf{},
	                           MapType          map_types = MapType::ALL) const
	{
		return write(execution::seq, pred, map_types);
	}

	template <
	    class ExecutionPolicy, class Predicate = pred::Leaf,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true>
	void write(ExecutionPolicy&& policy, std::filesystem::path const& file,
	           Predicate const& pred = pred::Leaf{}, MapType map_types = MapType::ALL) const
	{
		std::ofstream f = MapHeader::openWrite(file);
		return write(std::forward<ExecutionPolicy>(policy), f, pred, map_types);
	}

	template <
	    class ExecutionPolicy, class Predicate = pred::Leaf,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true>
	void write(ExecutionPolicy&& policy, std::ostream& out,
	           Predicate const& pred = pred::Leaf{}, MapType map_types = MapType::ALL) const
	{
		writeImpl(std::forward<ExecutionPolicy>(policy), out, pred, map_types, true);
	}

	template <
	    class ExecutionPolicy, class Predicate = pred::Leaf,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true>
	void write(ExecutionPolicy&& policy, WriteBuffer& out,
	           Predicate const& pred = pred::Leaf{}, MapType map_types = MapType::ALL) const
	{
		writeImpl(std::forward<ExecutionPolicy>(policy), out, pred, map_types, true);
	}

	template <
	    class ExecutionPolicy, class Predicate = pred::Leaf,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true>
	[[nodiscard]] Buffer write(ExecutionPolicy&& policy,
	                           Predicate const&  pred      = pred::Leaf{},
	                           MapType           map_types = MapType::ALL) const
	{
		Buffer buffer;
		write(std::forward<ExecutionPolicy>(policy), buffer, pred, map_types);
		return buffer;
	}

	template <class Predicate                                    = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] MapHeader writeData(std::filesystem::path const& file,
	                                  Predicate const&             pred = pred::Leaf{},
	                                  MapType map_types = MapType::ALL) const
	{
		return writeData(execution::seq, file, pred, map_types);
	}

	template <class Predicate                                    = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] MapHeader writeData(std::ostream&    out,
	                                  Predicate const& pred      = pred::Leaf{},
	                                  MapType          map_types = MapType::ALL) const
	{
		return writeData(execution::seq, out, pred, map_types);
	}

	template <class Predicate                                    = pred::Leaf,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	[[nodiscard]] MapHeader writeData(WriteBuffer&     out,
	                                  Predicate const& pred      = pred::Leaf{},
	                                  MapType          map_types = MapType::ALL) const
	{
		return writeData(execution::seq, out, pred, map_types);
	}

	template <
	    class ExecutionPolicy, class Predicate = pred::Leaf,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true>
	[[nodiscard]] MapHeader writeData(ExecutionPolicy&&            policy,
	                                  std::filesystem::path const& file,
	                                  Predicate const&             pred = pred::Leaf{},
	                                  MapType map_types = MapType::ALL) const
	{
		std::ofstream f = MapHeader::openWrite(file);
		return writeData(std::forward<ExecutionPolicy>(policy), f, pred, map_types);
	}

	template <
	    class ExecutionPolicy, class Predicate = pred::Leaf,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true>
	[[nodiscard]] MapHeader writeData(ExecutionPolicy&& policy, std::ostream& out,
	                                  Predicate const& pred      = pred::Leaf{},
	                                  MapType          map_types = MapType::ALL) const
	{
		return writeImpl(std::forward<ExecutionPolicy>(policy), out, pred, map_types, false);
	}

	template <
	    class ExecutionPolicy, class Predicate = pred::Leaf,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true,
	    std::enable_if_t<pred::is_pred_v<Predicate>, bool>                        = true>
	[[nodiscard]] MapHeader writeData(ExecutionPolicy&& policy, WriteBuffer& out,
	                                  Predicate const& pred      = pred::Leaf{},
	                                  MapType          map_types = MapType::ALL) const
	{
		return writeImpl(std::forward<ExecutionPolicy>(policy), out, pred, map_types, false);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Dot file                                       |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate                                    = pred::True,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void saveDotFile(std::filesystem::path const& file,
	                 Predicate const&             pred      = pred::True{},
	                 MapType                      map_types = MapType::ALL) const
	{
		saveDotFile(Base::node(), file, pred, map_types);
	}

	template <class Predicate                                    = pred::True,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void saveDotFile(std::ostream& out, Predicate const& pred = pred::True{},
	                 MapType map_types = MapType::ALL) const
	{
		saveDotFile(Base::node(), out, pred, map_types);
	}

	template <class NodeType, class Predicate = pred::True,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>   = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void saveDotFile(NodeType node, std::filesystem::path const& file,
	                 Predicate const& pred      = pred::True{},
	                 MapType          map_types = MapType::ALL) const
	{
		std::ofstream f = MapHeader::openWrite(file);
		saveDotFile(node, f, pred, map_types);
	}

	template <class NodeType, class Predicate = pred::True,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>   = true,
	          std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void saveDotFile(NodeType node, std::ostream& out, Predicate const& pred = pred::True{},
	                 MapType map_types = MapType::ALL) const
	{
		using Filter = pred::Filter<Predicate>;

		Node n = Base::node(node);

		std::string const parent_shape = "shape=polygon,sides=" + std::to_string(BF) + ' ';

		out << std::boolalpha << "graph UFOMap {\n";
		out << "fontname=\"Helvetica,Arial,sans-serif\"\n";
		out << "node [fontname=\"Helvetica,Arial,sans-serif\" shape=ellipse color=lightblue2 "
		       "style=filled]\n";
		out << "edge [fontname=\"Helvetica,Arial,sans-serif\"]\n";

		Filter::init(pred, *this);
		bool valid_return = Base::exists(n) && Filter::returnable(pred, *this, n);
		bool valid_inner  = Base::isParent(n) && Filter::traversable(pred, *this, n);

		if (!valid_return && !valid_inner) {
			out << "}";
			return;
		}

		std::string id =
		    std::to_string(node.index.pos) + '.' + std::to_string(node.index.offset) + '.' +
		    std::to_string(node.code.depth()) + '.' + std::to_string(node.code.offset());
		out << id << " [";

		if (valid_inner) {
			out << parent_shape;
		}

		out << "label=<";
		if (valid_return) {
			// We need to write this node regardless, otherwise we would have disconnected nodes
			onDotFile(out, n.index, map_types);
		} else {
			out << ' ';
		}

		out << ">";
		if (valid_inner) {
			out << " color=darkorange1";
		}
		out << "]\n";

		if (valid_inner) {
			saveDotFileRecurs(out, n, id, pred, map_types, parent_shape);
		}

		out << "}";
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         GPU                                         |
	|                                                                                     |
	**************************************************************************************/

	void gpuRelease()
	{
		gpuRelease(MapBasesISeq{});
		Base::gpuRelease();
	}

	[[nodiscard]] std::size_t gpuNumBuffers(MapType map_type) const
	{
		return MapType::TREE == map_type ? Base::template gpuNumBuffers<TreeBlock>()
		                                 : gpuNumBuffers(map_type, MapBasesISeq{});
	}

	[[nodiscard]] std::size_t gpuNumLeafBuffers(MapType map_type) const
	{
		return MapType::TREE == map_type ? Base::template gpuNumLeafBuffers<TreeBlock>()
		                                 : gpuNumLeafBuffers(map_type, MapBasesISeq{});
	}

	[[nodiscard]] std::size_t gpuNumInnerBuffers(MapType map_type) const
	{
		return MapType::TREE == map_type ? Base::template gpuNumInnerBuffers<TreeBlock>()
		                                 : gpuNumInnerBuffers(map_type, MapBasesISeq{});
	}

	[[nodiscard]] WGPUBuffer gpuLeafBuffer(MapType map_type, std::size_t index = 0) const
	{
		return MapType::TREE == map_type ? Base::template gpuLeafBuffer<TreeBlock>(index)
		                                 : gpuLeafBuffer(map_type, index, MapBasesISeq{});
	}

	[[nodiscard]] WGPUBuffer gpuInnerBuffer(MapType map_type, std::size_t index = 0) const
	{
		return MapType::TREE == map_type ? Base::template gpuInnerBuffer<TreeBlock>(index)
		                                 : gpuInnerBuffer(map_type, index, MapBasesISeq{});
	}

	[[nodiscard]] std::size_t gpuLeafBufferSize(MapType     map_type,
	                                            std::size_t index = 0) const
	{
		return MapType::TREE == map_type ? Base::template gpuLeafBufferSize<TreeBlock>(index)
		                                 : gpuLeafBufferSize(map_type, index, MapBasesISeq{});
	}

	[[nodiscard]] std::size_t gpuInnerBufferSize(MapType     map_type,
	                                             std::size_t index = 0) const
	{
		return MapType::TREE == map_type
		           ? Base::template gpuInnerBufferSize<TreeBlock>(index)
		           : gpuInnerBufferSize(map_type, index, MapBasesISeq{});
	}

	void gpuRead(MapType map_types = MapType::ALL)
	{
		Base::template gpuRead<TreeBlock>();
		gpuRead(map_types, MapBasesISeq{});
	}

	void gpuReadLeaf(MapType map_types = MapType::ALL)
	{
		Base::template gpuReadLeaf<TreeBlock>();
		gpuReadLeaf(map_types, MapBasesISeq{});
	}

	void gpuReadInner(MapType map_types = MapType::ALL)
	{
		Base::template gpuReadInner<TreeBlock>();
		gpuReadInner(map_types, MapBasesISeq{});
	}

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void gpuRead(Predicate const& pred)
	{
		gpuRead(pred::predicate_map_type_v<Predicate>);
	}

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void gpuReadLeaf(Predicate const& pred)
	{
		gpuReadLeaf(pred::predicate_map_type_v<Predicate>);
	}

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void gpuReadInner(Predicate const& pred)
	{
		gpuReadInner(pred::predicate_map_type_v<Predicate>);
	}

	bool gpuWrite(MapType map_types = MapType::ALL)
	{
		bool updated = Base::template gpuWrite<TreeBlock>();
		return gpuWrite(map_types, MapBasesISeq{}) || updated;
	}

	bool gpuWriteLeaf(MapType map_types = MapType::ALL)
	{
		bool updated = Base::template gpuWriteLeaf<TreeBlock>();
		return gpuWriteLeaf(map_types, MapBasesISeq{}) || updated;
	}

	bool gpuWriteInner(MapType map_types = MapType::ALL)
	{
		bool updated = Base::template gpuWriteInner<TreeBlock>();
		return gpuWriteInner(map_types, MapBasesISeq{}) || updated;
	}

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	bool gpuWrite(Predicate const& pred)
	{
		return gpuWrite(pred::predicate_map_type_v<Predicate>);
	}

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	bool gpuWriteLeaf(Predicate const& pred)
	{
		return gpuWriteLeaf(pred::predicate_map_type_v<Predicate>);
	}

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	bool gpuWriteInner(Predicate const& pred)
	{
		return gpuWriteInner(pred::predicate_map_type_v<Predicate>);
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                             Functions Maps Should Have                              |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot() { onInitRoot(Base::block(), MapBasesISeq{}); }

	template <std::size_t... Is>
	void onInitRoot(pos_type block, std::index_sequence<Is...>)
	{
		return (onInitRoot<std::tuple_element_t<Is, MapBases>>(block), ...);
	}

	template <class Map>
	void onInitRoot(pos_type block)
	{
		Map::onInitRoot(block);
	}

	void onInitLeafChildren(Index node, pos_type children)
	{
		onInitLeafChildren(node, children, MapBasesISeq{});
	}

	template <std::size_t... Is>
	void onInitLeafChildren(Index node, pos_type children, std::index_sequence<Is...>)
	{
		return (onInitLeafChildren<std::tuple_element_t<Is, MapBases>>(node, children), ...);
	}

	template <class Map>
	void onInitLeafChildren(Index node, pos_type children)
	{
		Map::onInitLeafChildren(node, children);
	}

	void onInitInnerChildren(Index node, pos_type children)
	{
		onInitInnerChildren(node, children, MapBasesISeq{});
	}

	template <std::size_t... Is>
	void onInitInnerChildren(Index node, pos_type children, std::index_sequence<Is...>)
	{
		return (onInitInnerChildren<std::tuple_element_t<Is, MapBases>>(node, children), ...);
	}

	template <class Map>
	void onInitInnerChildren(Index node, pos_type children)
	{
		Map::onInitInnerChildren(node, children);
	}

	void onPropagateChildren(Index node, pos_type children, MapType map_types)
	{
		onPropagateChildren(node, children, map_types, MapBasesISeq{});
	}

	template <std::size_t... Is>
	void onPropagateChildren(Index node, pos_type children, MapType map_types,
	                         std::index_sequence<Is...>)
	{
		return (onPropagateChildren<std::tuple_element_t<Is, MapBases>>(node, children,
		                                                                map_types),
		        ...);
	}

	template <class Map>
	void onPropagateChildren(Index node, pos_type children, MapType map_types)
	{
		if (!isMapType<Map>(map_types)) {
			return;
		}

		Map::onPropagateChildren(node, children);
	}

	[[nodiscard]] bool onIsPrunable(pos_type block) const
	{
		return Base::noneParent(block) && onIsPrunable(block, MapBasesISeq{});
	}

	template <std::size_t... Is>
	[[nodiscard]] bool onIsPrunable(pos_type block, std::index_sequence<Is...>) const
	{
		return (onIsPrunable<std::tuple_element_t<Is, MapBases>>(block) && ...);
	}

	template <class Map>
	[[nodiscard]] bool onIsPrunable(pos_type block) const
	{
		return Map::onIsPrunable(block);
	}

	void onPruneLeafChildren(Index node, pos_type children)
	{
		onPruneLeafChildren(node, children, MapBasesISeq{});
	}

	template <std::size_t... Is>
	void onPruneLeafChildren(Index node, pos_type children, std::index_sequence<Is...>)
	{
		(onPruneLeafChildren<std::tuple_element_t<Is, MapBases>>(node, children), ...);
	}

	template <class Map>
	void onPruneLeafChildren(Index node, pos_type children)
	{
		Map::onPruneLeafChildren(node, children);
	}

	void onPruneInnerChildren(Index node, pos_type children)
	{
		onPruneInnerChildren(node, children, MapBasesISeq{});
	}

	template <std::size_t... Is>
	void onPruneInnerChildren(Index node, pos_type children, std::index_sequence<Is...>)
	{
		(onPruneInnerChildren<std::tuple_element_t<Is, MapBases>>(node, children), ...);
	}

	template <class Map>
	void onPruneInnerChildren(Index node, pos_type children)
	{
		Map::onPruneInnerChildren(node, children);
	}

	[[nodiscard]] std::size_t onSerializedSize(SerializedBlocks<BF> const& blocks,
	                                           std::size_t                 num_nodes,
	                                           MapType                     map_types) const
	{
		return onSerializedSize(blocks, num_nodes, map_types, MapBasesISeq{});
	}

	template <std::size_t... Is>
	[[nodiscard]] std::size_t onSerializedSize(SerializedBlocks<BF> const& blocks,
	                                           std::size_t num_nodes, MapType map_types,
	                                           std::index_sequence<Is...>) const
	{
		return (onSerializedSize<std::tuple_element_t<Is, MapBases>>(blocks, num_nodes,
		                                                             map_types) +
		        ...);
	}

	template <class Map>
	[[nodiscard]] std::size_t onSerializedSize(SerializedBlocks<BF> const& blocks,
	                                           std::size_t                 num_nodes,
	                                           MapType                     map_types) const
	{
		return isMapType<Map>(map_types) ? Map::onSerializedSize(blocks, num_nodes) : 0;
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onRead(ExecutionPolicy&& policy, ReadBuffer& in,
	            SerializedBlocks<BF> const& blocks, MapType map_type,
	            std::uint64_t data_size)
	{
		onRead(std::forward<ExecutionPolicy>(policy), in, blocks, map_type, data_size,
		       MapBasesISeq{});
	}

	template <
	    class ExecutionPolicy, std::size_t... Is,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onRead(ExecutionPolicy&& policy, ReadBuffer& in,
	            SerializedBlocks<BF> const& blocks, MapType map_type,
	            std::uint64_t data_size, std::index_sequence<Is...>)
	{
		(onRead<std::tuple_element_t<Is, MapBases>>(policy, in, blocks, map_type,
		                                            data_size) ||
		 ...);
	}

	template <
	    class Map, class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	bool onRead(ExecutionPolicy&& policy, ReadBuffer& in,
	            SerializedBlocks<BF> const& blocks, MapType map_type,
	            std::uint64_t data_size)
	{
		if (!isMapType<Map>(map_type)) {
			return false;
		}

		Map::onRead(std::forward<ExecutionPolicy>(policy), in, blocks);
		return true;
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onWrite(ExecutionPolicy&& policy, WriteBuffer& out,
	             SerializedBlocks<BF> const& blocks, MapType map_type) const
	{
		onWrite(std::forward<ExecutionPolicy>(policy), out, blocks, map_type, MapBasesISeq{});
	}

	template <
	    class ExecutionPolicy, std::size_t... Is,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onWrite(ExecutionPolicy&& policy, WriteBuffer& out,
	             SerializedBlocks<BF> const& blocks, MapType map_type,
	             std::index_sequence<Is...>) const
	{
		(onWrite<std::tuple_element_t<Is, MapBases>>(policy, out, blocks, map_type), ...);
	}

	template <
	    class Map, class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void onWrite(ExecutionPolicy&& policy, WriteBuffer& out,
	             SerializedBlocks<BF> const& blocks, MapType map_type) const
	{
		if (!isMapType<Map>(map_type)) {
			return;
		}

		Map::onWrite(std::forward<ExecutionPolicy>(policy), out, blocks);
	}

	void onDotFile(std::ostream& out, Index node, MapType map_types) const
	{
		out << "<br/>Center: " << Base::center(node);
		out << "<br/>Depth: " << Base::depth(node) << " | Length: " << Base::length(node);
		if (Base::modified(node)) {
			out << "Modified: <font color='green'><b>true</b></font>";
		} else {
			out << "Modified: <font color='red'>false</font>";
		}

		onDotFile(out, node, map_types, MapBasesISeq{});
	}

	template <std::size_t... Is>
	void onDotFile(std::ostream& out, Index node, MapType map_types) const
	{
		(onDotFile<std::tuple_element_t<Is, MapBases>>(out, node, map_types), ...);
	}

	template <class Map>
	void onDotFile(std::ostream& out, Index node, MapType map_types) const
	{
		if (MapType::NONE == (mapType<Map>() & map_types)) {
			return;
		}

		Map::onDotFile((out << "<br/>"), node);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Map Types                                      |
	|                                                                                     |
	**************************************************************************************/

	template <std::size_t... Is>
	[[nodiscard]] static constexpr MapType mapTypes(std::index_sequence<Is...>) noexcept
	{
		return (mapType<std::tuple_element_t<Is, MapBases>>() | ...);
	}

	template <class Map>
	[[nodiscard]] static constexpr MapType mapType() noexcept
	{
		return Map::Type;
	}

	template <class Map>
	[[nodiscard]] static constexpr bool isMapType(MapType map_type) noexcept
	{
		return mapType<Map>() == (mapType<Map>() & map_type);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Propagate                                      |
	|                                                                                     |
	**************************************************************************************/

	void propagate(pos_type block, MapType map_types)
	{
		if (Base::isPureLeaf(block)) {
			return;
		}

		// Const access more optimized
		auto const& b = Base::treeInnerBlockConst(block);

		if (Base::modifiedNone(b)) {
			return;
		}

		for (offset_type i{}; BF > i; ++i) {
			Index n(block, i);
			auto  c = Base::children(b, i);

			if (!Base::valid(c) || !Base::modified(b, i)) {
				continue;
			}

			propagate(c, map_types);
			onPropagateChildren(n, c, map_types);

			if (onIsPrunable(c)) {
				Base::pruneChildren(n, c);
			}
		}
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void propagate(ExecutionPolicy&& policy, pos_type block, MapType map_types,
	               bool is_parallel)
	{
		if (Base::isPureLeaf(block)) {
			return;
		}

		// Const access more optimized
		auto const& b = Base::treeInnerBlockConst(block);

		if (Base::modifiedNone(b)) {
			return;
		}

		auto const d = Base::depth(block);
		if (BF - 2u <= Base::modifiedCount(b) && 2u < d) {
			ufo::for_each(policy, offset_type(0), offset_type(BF),
			              [this, policy, block, map_types, is_parallel, &b](offset_type i) {
				              Index n(block, i);
				              auto  c = Base::children(b, i);

				              if (!Base::valid(c) || !Base::modified(b, i)) {
					              return;
				              }

				              if (is_parallel) {
					              propagate(c, map_types);
				              } else {
					              propagate(policy, c, map_types, true);
				              }
				              onPropagateChildren(n, c, map_types);

				              if (onIsPrunable(c)) {
					              Base::pruneChildren(n, c);
				              }
			              });
		} else {
			for (offset_type i{}; BF > i; ++i) {
				Index n(block, i);
				auto  c = Base::children(b, i);

				if (!Base::valid(c) || !Base::modified(b, i)) {
					return;
				}

				propagate(policy, c, map_types, is_parallel);
				onPropagateChildren(n, c, map_types);

				if (onIsPrunable(c)) {
					Base::pruneChildren(n, c);
				}
			}
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	template <class ExecutionPolicy, class Input>
	void readDataImpl(ExecutionPolicy&& policy, Input& in, MapHeader const& header,
	                  MapType map_types, bool propagate)
	{
		if (Dim != header.leaf_node_length.size()) {
			throw std::runtime_error("Trying to read map with dimension " +
			                         std::to_string(header.leaf_node_length.size()) +
			                         " into a map with dimension " + std::to_string(Dim));
		}

		Length leaf_node_length;
		for (std::size_t i{}; Dim > i; ++i) {
			leaf_node_length[i] = header.leaf_node_length[i];
		}

		if (Base::length(0) != leaf_node_length ||
		    Base::numDepthLevels() != header.num_depth_levels) {
			Base::clear(leaf_node_length, header.num_depth_levels);
		}

		readMaps(policy, in, readBlocks(in, header), header, map_types);

		if (propagate) {
			this->propagate(policy, map_types);
		}
	}

	template <class Input>
	[[nodiscard]] SerializedBlocks<BF> readBlocks(Input& in, MapHeader const& header)
	{
		std::vector<BitSet<BF>> tree(header.map_info[0].size);

		if (0 < header.map_info[0].size) {
			std::size_t size = header.map_info[0].size * sizeof(BitSet<BF>);
			if constexpr (std::is_base_of_v<std::istream, remove_cvref_t<Input>>) {
				in.read(reinterpret_cast<char*>(tree.data()), size);
			} else if constexpr (std::is_base_of_v<ReadBuffer, remove_cvref_t<Input>>) {
				in.read(tree.data(), size);
			} else {
				static_assert(dependent_false_v<Input>, "Wrong input type");
			}
		}

		SerializedBlocks<BF> blocks;

		if (0 < header.num_blocks) {
			blocks.reserve(header.num_blocks);
			readBlocksRecurs(Base::block(), tree.begin(), blocks);

			for (std::size_t i = 1; blocks.size() > i; ++i) {
				blocks[i].data_start = blocks[i - 1].data_start + blocks[i - 1].offsets.count();
			}
		}

		return blocks;
	}

	typename std::vector<BitSet<BF>>::const_iterator readBlocksRecurs(
	    pos_type block, typename std::vector<BitSet<BF>>::const_iterator tree,
	    SerializedBlocks<BF>& blocks)
	{
		BitSet<BF> const valid_return         = *tree++;
		BitSet<BF> const valid_inner          = *tree++;
		BitSet<BF> const valid_erase_children = valid_return & ~valid_inner;

		Base::modified(block) |= (valid_return & valid_inner).data();

		if (valid_return.any()) {
			blocks.emplace_back(0u, block, valid_return);
		}

		if (valid_inner.none() && valid_erase_children.none()) {
			return tree;
		}

		for (offset_type i{}; BF > i; ++i) {
			if (valid_erase_children[i]) {
				Base::eraseChildren(Index(block, i));
			}
		}

		for (offset_type i{}; BF > i; ++i) {
			if (valid_inner[i]) {
				tree = readBlocksRecurs(Base::createChildren(Index(block, i)), tree, blocks);
			}
		}

		return tree;
	}

	template <class ExecutionPolicy>
	void readMaps(ExecutionPolicy&& policy, std::istream& in,
	              SerializedBlocks<BF> const& blocks, MapHeader const& header,
	              MapType map_types)
	{
		Buffer buffer;
		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			auto map_type  = header.map_info[i].type;
			auto data_size = header.map_info[i].size;

			if (hasMapTypes(map_types & map_type)) {
				buffer.clear();
				buffer.write(in, data_size);
				onRead(policy, buffer, blocks, map_type, data_size);
			} else {
				// Skip forward
				in.seekg(static_cast<std::istream::off_type>(data_size), std::istream::cur);
			}
		}
	}

	template <class ExecutionPolicy>
	void readMaps(ExecutionPolicy&& policy, ReadBuffer& in,
	              SerializedBlocks<BF> const& blocks, MapHeader const& header,
	              MapType map_types)
	{
		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			auto map_type  = header.map_info[i].type;
			auto data_size = header.map_info[i].size;
			auto pos       = in.readPos();

			if (hasMapTypes(map_types & map_type)) {
				onRead(policy, in, blocks, map_type, data_size);
			}

			// Skip forward
			in.readSeek(pos + data_size);
		}
	}

	// template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	// [[nodiscard]] std::vector<detail::Fulfill<BF>> fulfills(Predicate pred) const
	// {
	// 	std::vector<detail::Fulfill<BF>> res;

	// 	// TODO: Implement

	// 	return res;
	// }

	// template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	// void fulfillsRecurs(Node parent, Predicate const& pred,
	//                     std::vector<detail::Fulfill<BF>>& fulfills) const
	// {
	// 	// TODO: Implement
	// }

	// void fulfillsModifiedRecurs(pos_type                          block,
	//                             std::vector<detail::Fulfill<BF>>& fulfills) const
	// {
	// 	// TODO: Implement
	// }

	template <class ExecutionPolicy, class Output, class Predicate>
	MapHeader writeImpl(ExecutionPolicy&& policy, Output& out, Predicate const& pred,
	                    MapType map_types, bool write_header) const
	{
		auto [tree, blocks] = writeBlocks(pred);

		for (std::size_t i = 1; blocks.size() > i; ++i) {
			blocks[i].data_start = blocks[i - 1].data_start + blocks[i - 1].offsets.count();
		}

		MapHeader h        = header(map_types);
		h.num_blocks       = blocks.size();
		h.num_nodes        = blocks.back().data_start + blocks.back().offsets.count();
		h.map_info[0].size = tree.size();
		for (std::size_t i = 1; h.map_info.size() > i; ++i) {
			h.map_info[i].size = onSerializedSize(blocks, h.num_nodes, h.map_info[i].type);
		}

		if (write_header) {
			h.write(out);
		}

		if (!tree.empty()) {
			std::size_t size = tree.size() * sizeof(BitSet<BF>);
			if constexpr (std::is_base_of_v<std::ostream, remove_cvref_t<Output>>) {
				out.write(reinterpret_cast<char*>(tree.data()), size);
			} else if constexpr (std::is_base_of_v<WriteBuffer, remove_cvref_t<Output>>) {
				out.write(tree.data(), size);
			} else {
				static_assert(dependent_false_v<Output>, "Wrong output type");
			}
		}

		if (!blocks.empty()) {
			writeMaps(std::forward<ExecutionPolicy>(policy), out, blocks, h);
		}

		return h;
	}

	template <class Predicate>
	[[nodiscard]] std::pair<std::vector<BitSet<BF>>, SerializedBlocks<BF>> writeBlocks(
	    Predicate pred) const
	{
		using P      = remove_cvref_t<Predicate>;
		using Filter = pred::Filter<P>;
		using L      = pred::Leaf;
		using M      = pred::Modified<false>;
		using P1     = decltype(std::declval<L>() && std::declval<M>());
		using P2     = decltype(std::declval<M>() && std::declval<L>());

		std::pair<std::vector<BitSet<BF>>, SerializedBlocks<BF>> res;

		auto& tree   = res.first;
		auto& blocks = res.second;

		if constexpr (contains_type_v<P, P1, P2>) {
			auto root = Base::index();

			bool leaf     = Base::isLeaf(root);
			bool modified = Base::modified(root);

			bool valid_return = leaf && modified;
			bool valid_inner  = !leaf && modified;

			tree.emplace_back(valid_return ? 1u : 0u);
			tree.emplace_back(valid_inner ? 1u : 0u);

			if (valid_return) {
				blocks.emplace_back(0u, root.pos, BitSet<BF>(1u));
			} else if (valid_inner) {
				writeBlocksModifiedRecurs(Base::children(root), tree, blocks);
			}
		} else {
			Filter::init(pred, *this);

			auto root = Base::node();

			bool valid_return = Filter::returnable(pred, *this, root);
			bool valid_inner =
			    Base::isParent(root.index) && Filter::traversable(pred, *this, root);

			tree.emplace_back(valid_return ? 1u : 0u);
			tree.emplace_back(valid_inner ? 1u : 0u);

			if (valid_return) {
				blocks.emplace_back(0u, root.index.pos, BitSet<BF>(1u));
			}
			if (valid_inner) {
				writeBlocksRecurs(root, pred, tree, blocks);
			}
		}

		if (blocks.empty()) {
			tree.clear();
		}

		return res;
	}

	template <class Predicate>
	bool writeBlocksRecurs(Node parent, Predicate const& pred,
	                       std::vector<BitSet<BF>>& tree,
	                       SerializedBlocks<BF>&    blocks) const
	{
		using P      = remove_cvref_t<Predicate>;
		using Filter = pred::Filter<P>;

		auto const& block = Base::TreeInnerBlock(parent.index.pos);
		auto const  c     = Base::children(block, parent.index.offset);

		if (Base::isPureLeaf(c)) {
			BitSet<BF> vr;
			for (offset_type i{}; BF > i; ++i) {
				Node node(Base::child(parent.code, i), Index(c, i));
				vr[i] = Filter::returnable(pred, *this, node);
			}

			if (vr.none()) {
				return false;
			}

			tree.push_back(vr);
			blocks.emplace_back(0u, c, vr);
			return true;
		}

		BitSet<BF>           vr;
		BitSet<BF>           vi;
		std::array<Node, BF> node;
		for (offset_type i{}; BF > i; ++i) {
			node[i].code  = Base::child(parent.code, i);
			node[i].index = Index(c, i);
			vr[i]         = Filter::returnable(pred, *this, node[i]);
			vi[i] = Base::isParent(node.index) && Filter::traversable(pred, *this, node[i]);
		}

		if (vr.none() && vi.none()) {
			return false;
		}

		std::size_t const tree_size_before = tree.size();
		tree.emplace_back(vr);
		BitSet<BF>& valid_inner = tree.emplace_back(vi);

		if (vr.any()) {
			blocks.emplace_back(0u, block, vr);
		}

		if (vi.none()) {
			return true;
		}

		for (offset_type i{}; BF > i; ++i) {
			vi[i] = vi[i] && writeNodesRecurs(node[i], pred, tree, blocks);
		}

		valid_inner = vi;

		if (vr.any() || vi.any()) {
			return true;
		}

		tree.resize(tree_size_before);
		return false;
	}

	bool writeBlocksModifiedRecurs(pos_type block, std::vector<BitSet<BF>>& tree,
	                               SerializedBlocks<BF>& blocks) const
	{
		if (Base::isPureLeaf(block)) {
			auto const m = BitSet<BF>(Base::modified(Base::treeLeafBlock(block)));
			if (m.none()) {
				return false;
			}

			tree.push_back(m);
			blocks.emplace_back(0u, block, m);
			return true;
		}

		auto const& b = Base::treeInnerBlock(block);
		auto const  m = BitSet<BF>(Base::modified(b));
		auto const& c = Base::children(b);

		BitSet<BF> leaf;
		for (offset_type i{}; BF > i; ++i) {
			leaf[i] = !Base::valid(c[i]);
		}

		BitSet<BF> const vr = leaf & m;
		BitSet<BF>       vi = ~leaf & m;

		if (vr.none() && vi.none()) {
			return false;
		}

		std::size_t const tree_size_before = tree.size();
		tree.emplace_back(vr);
		BitSet<BF>& valid_inner = tree.emplace_back(vi);

		if (vr.any()) {
			blocks.emplace_back(0u, block, vr);
		}

		if (vi.none()) {
			return true;
		}

		for (offset_type i{}; BF > i; ++i) {
			vi[i] = vi[i] && writeBlocksModifiedRecurs(c[i], tree, blocks);
		}

		valid_inner = vi;

		if (vr.any() || vi.any()) {
			return true;
		}

		tree.resize(tree_size_before);
		return false;
	}

	template <class ExecutionPolicy>
	void writeMaps(ExecutionPolicy&& policy, std::ostream& out,
	               SerializedBlocks<BF> const& blocks, MapHeader const& header) const
	{
		if (blocks.empty()) {
			return;
		}

		std::uint64_t max_size{};
		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			max_size = std::max(max_size, header.map_info[i].size);
		}

		Buffer buffer;
		buffer.reserve(max_size);

		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			buffer.clear();
			onWrite(policy, buffer, blocks, header.map_info[i].type);
			if (!buffer.empty()) {
				buffer.read(out, buffer.size());
			}
			// TODO: Fill in compressed size
		}
	}

	template <class ExecutionPolicy>
	void writeMaps(ExecutionPolicy&& policy, WriteBuffer& out,
	               SerializedBlocks<BF> const& blocks, MapHeader const& header) const
	{
		if (blocks.empty()) {
			return;
		}

		std::size_t total_size{};
		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			total_size += header.map_info[i].size;
		}

		out.reserve(out.size() + total_size);

		for (std::size_t i = 1; header.map_info.size() > i; ++i) {
			onWrite(policy, out, blocks, header.map_info[i].type);
			// TODO: Fill in compressed size
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Dot file                                       |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate, std::enable_if_t<pred::is_pred_v<Predicate>, bool> = true>
	void saveDotFileRecurs(std::ostream& out, Node node, std::string const& id,
	                       Predicate const& pred, MapType map_types,
	                       std::string const& parent_shape) const
	{
		using Filter = pred::Filter<Predicate>;

		for (std::size_t i{}; BF > i; ++i) {
			Node child        = Base::child(node, i);
			bool valid_return = Filter::returnable(pred, *this, child);
			bool valid_inner = Base::isParent(child) && Filter::traversable(pred, *this, child);

			if (!valid_return && !valid_inner) {
				continue;
			}

			std::string child_id = std::to_string(child.index.pos) + '.' +
			                       std::to_string(child.index.offset) + '.' +
			                       std::to_string(child.code.depth()) + '.' +
			                       std::to_string(child.code.offset());
			out << child_id << " [";

			if (valid_inner) {
				out << parent_shape;
			}

			out << "label=<";
			if (valid_return) {
				// We need to write this node regardless, otherwise we would have disconnected
				// nodes
				onDotFile(out, child.index, map_types);
			} else {
				out << ' ';
			}

			out << ">";
			if (valid_inner) {
				out << " color=darkorange1";
			}
			out << "]\n";
			out << id << " -- " << child_id << '\n';

			if (valid_inner) {
				saveDotFileRecurs(out, child, child_id, pred, map_types, parent_shape);
			}
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         GPU                                         |
	|                                                                                     |
	**************************************************************************************/

	template <std::size_t... Is>
	void gpuRelease(std::index_sequence<Is...>)
	{
		(gpuRelease<std::tuple_element_t<Is, MapBases>>(), ...);
	}

	template <class Map>
	void gpuRelease()
	{
		Map::onGpuRelease();
	}

	template <std::size_t... Is>
	[[nodiscard]] std::size_t gpuNumBuffers(MapType map_type,
	                                        std::index_sequence<Is...>) const
	{
		return std::max({gpuNumBuffers<std::tuple_element_t<Is, MapBases>>(map_type)...});
	}

	template <class Map>
	[[nodiscard]] std::size_t gpuNumBuffers(MapType map_type) const
	{
		return isMapType<Map>(map_type) ? Base::template gpuNumBuffers<typename Map::Block>()
		                                : std::size_t(0);
	}

	template <std::size_t... Is>
	[[nodiscard]] std::size_t gpuNumLeafBuffers(MapType map_type,
	                                            std::index_sequence<Is...>) const
	{
		return std::max({gpuNumLeafBuffers<std::tuple_element_t<Is, MapBases>>(map_type)...});
	}

	template <class Map>
	[[nodiscard]] std::size_t gpuNumLeafBuffers(MapType map_type) const
	{
		return isMapType<Map>(map_type)
		           ? Base::template gpuNumLeafBuffers<typename Map::Block>()
		           : std::size_t(0);
	}

	template <std::size_t... Is>
	[[nodiscard]] std::size_t gpuNumInnerBuffers(MapType map_type,
	                                             std::index_sequence<Is...>) const
	{
		return std::max(
		    {gpuNumInnerBuffers<std::tuple_element_t<Is, MapBases>>(map_type)...});
	}

	template <class Map>
	[[nodiscard]] std::size_t gpuNumInnerBuffers(MapType map_type) const
	{
		return isMapType<Map>(map_type)
		           ? Base::template gpuNumInnerBuffers<typename Map::Block>()
		           : std::size_t(0);
	}

	template <std::size_t... Is>
	[[nodiscard]] WGPUBuffer gpuLeafBuffer(MapType map_type, std::size_t index,
	                                       std::index_sequence<Is...>) const
	{
		return std::max(
		    {gpuLeafBuffer<std::tuple_element_t<Is, MapBases>>(map_type, index)...});
	}

	template <class Map>
	[[nodiscard]] WGPUBuffer gpuLeafBuffer(MapType map_type, std::size_t index) const
	{
		return isMapType<Map>(map_type)
		           ? Base::template gpuLeafBuffer<typename Map::Block>(index)
		           : nullptr;
	}

	template <std::size_t... Is>
	[[nodiscard]] WGPUBuffer gpuInnerBuffer(MapType map_type, std::size_t index,
	                                        std::index_sequence<Is...>) const
	{
		return std::max(
		    {gpuInnerBuffer<std::tuple_element_t<Is, MapBases>>(map_type, index)...});
	}

	template <class Map>
	[[nodiscard]] WGPUBuffer gpuInnerBuffer(MapType map_type, std::size_t index) const
	{
		return isMapType<Map>(map_type)
		           ? Base::template gpuInnerBuffer<typename Map::Block>(index)
		           : nullptr;
	}

	template <std::size_t... Is>
	[[nodiscard]] std::size_t gpuLeafBufferSize(MapType map_type, std::size_t index,
	                                            std::index_sequence<Is...>) const
	{
		return std::max(
		    {gpuLeafBufferSize<std::tuple_element_t<Is, MapBases>>(map_type, index)...});
	}

	template <class Map>
	[[nodiscard]] std::size_t gpuLeafBufferSize(MapType map_type, std::size_t index) const
	{
		return isMapType<Map>(map_type)
		           ? Base::template gpuLeafBufferSize<typename Map::Block>(index)
		           : std::size_t(0);
	}

	template <std::size_t... Is>
	[[nodiscard]] std::size_t gpuInnerBufferSize(MapType map_type, std::size_t index,
	                                             std::index_sequence<Is...>) const
	{
		return std::max(
		    {gpuInnerBufferSize<std::tuple_element_t<Is, MapBases>>(map_type, index)...});
	}

	template <class Map>
	[[nodiscard]] std::size_t gpuInnerBufferSize(MapType map_type, std::size_t index) const
	{
		return isMapType<Map>(map_type)
		           ? Base::template gpuInnerBufferSize<typename Map::Block>(index)
		           : std::size_t(0);
	}

	template <std::size_t... Is>
	void gpuRead(MapType map_types, std::index_sequence<Is...>)
	{
		(gpuRead<std::tuple_element_t<Is, MapBases>>(map_types), ...);
	}

	template <class Map>
	void gpuRead(MapType map_types)
	{
		if (!isMapType<Map>(map_types)) {
			return;
		}

		Map::onGpuRead();
		Base::template gpuRead<typename Map::Block>();
	}

	template <std::size_t... Is>
	void gpuReadLeaf(MapType map_types, std::index_sequence<Is...>)
	{
		(gpuReadLeaf<std::tuple_element_t<Is, MapBases>>(map_types), ...);
	}

	template <class Map>
	void gpuReadLeaf(MapType map_types)
	{
		if (!isMapType<Map>(map_types)) {
			return;
		}

		Map::onGpuReadLeaf();
		Base::template gpuReadLeaf<typename Map::Block>();
	}

	template <std::size_t... Is>
	void gpuReadInner(MapType map_types, std::index_sequence<Is...>)
	{
		(gpuReadLeaf<std::tuple_element_t<Is, MapBases>>(map_types), ...);
	}

	template <class Map>
	void gpuReadInner(MapType map_types)
	{
		if (!isMapType<Map>(map_types)) {
			return;
		}

		Map::onGpuReadInner();
		Base::template gpuReadInner<typename Map::Block>();
	}

	template <std::size_t... Is>
	bool gpuWrite(MapType map_types, std::index_sequence<Is...>)
	{
		return (gpuWrite<std::tuple_element_t<Is, MapBases>>(map_types) | ...);
	}

	template <class Map>
	bool gpuWrite(MapType map_types)
	{
		if (!isMapType<Map>(map_types)) {
			return false;
		}

		Map::onGpuWrite();
		return Base::template gpuWrite<typename Map::Block>();
	}

	template <std::size_t... Is>
	bool gpuWriteLeaf(MapType map_types, std::index_sequence<Is...>)
	{
		return (gpuWriteLeaf<std::tuple_element_t<Is, MapBases>>(map_types) | ...);
	}

	template <class Map>
	bool gpuWriteLeaf(MapType map_types)
	{
		if (!isMapType<Map>(map_types)) {
			return false;
		}

		Map::onGpuWriteLeaf();
		return Base::template gpuWriteLeaf<typename Map::Block>();
	}

	template <std::size_t... Is>
	bool gpuWriteInner(MapType map_types, std::index_sequence<Is...>)
	{
		return (gpuWriteLeaf<std::tuple_element_t<Is, MapBases>>(map_types) | ...);
	}

	template <class Map>
	bool gpuWriteInner(MapType map_types)
	{
		if (!isMapType<Map>(map_types)) {
			return false;
		}

		Map::onGpuWriteInner();
		return Base::template gpuWriteInner<typename Map::Block>();
	}
};
}  // namespace ufo

#endif  // UFO_MAP_HPP