#ifndef UFO_PLAN_GRAPH_HPP
#define UFO_PLAN_GRAPH_HPP

// UFO
#include <ufo/container/tree_map.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/plan/node.hpp>
#include <ufo/plan/path.hpp>

// STL
#include <memory>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ufo
{
template <std::size_t Dim = 3, class T = float>
class PlanGraph
{
 public:
	using Node = std::shared_ptr<PlanNode<Dim, T>>;
	using Edge = PlanEdge<Dim, T>;
	using Path = PlanPath<Dim, T>;

	[[nodiscard]] std::tuple<Path, float, float> plan(Node const& start, Node const& goal,
	                                                  float distance_weight  = 1.0f,
	                                                  float edge_cost_weight = 1.0f,
	                                                  float node_cost_weight = 1.0f,
	                                                  float heuristic_weight = 1.0f) const
	{
		std::vector<Node> starts(1, start);
		return plan(starts, goal, distance_weight, edge_cost_weight, node_cost_weight,
		            heuristic_weight);
	}

	[[nodiscard]] std::tuple<Path, float, float> plan(std::vector<Node> const& starts,
	                                                  Node const&              goal,
	                                                  float distance_weight  = 1.0f,
	                                                  float edge_cost_weight = 1.0f,
	                                                  float node_cost_weight = 1.0f,
	                                                  float heuristic_weight = 1.0f) const
	{
		auto cost_f = [distance_weight, edge_cost_weight,
		               node_cost_weight](Edge const& edge) {
			return distance_weight * edge.distance + edge_cost_weight * edge.cost +
			       node_cost_weight * edge.to->cost;
		};

		return plan(starts, goal, cost_f, heuristic_weight);
	}

	template <class CostFun>
	[[nodiscard]] std::tuple<Path, float, float> plan(Node const& start, Node const& goal,
	                                                  CostFun cost_f,
	                                                  float   heuristic_weight = 1.0f) const
	{
		std::vector<Node> starts(1, start);
		return plan(starts, goal, cost_f, heuristic_weight);
	}

	template <class CostFun>
	[[nodiscard]] std::tuple<Path, float, float> plan(std::vector<Node> const& starts,
	                                                  Node const& goal, CostFun cost_f,
	                                                  float heuristic_weight = 1.0f) const
	{
		auto heuristic_f = [heuristic_weight](Node const& cur, Node const& goal) {
			return heuristic_weight * distance(*cur, *goal);
		};

		return plan(starts, goal, cost_f, heuristic_f);
	}

	template <class CostFun, class HeuristicFun>
	[[nodiscard]] std::tuple<Path, float, float> plan(Node const& start, Node const& goal,
	                                                  CostFun      cost_f,
	                                                  HeuristicFun heuristic_f) const
	{
		std::vector<Node> starts(1, start);
		return plan(starts, goal, cost_f, heuristic_f);
	}

	template <class CostFun, class HeuristicFun>
	[[nodiscard]] std::tuple<Path, float, float> plan(std::vector<Node> const& starts,
	                                                  Node const& goal, CostFun cost_f,
	                                                  HeuristicFun heuristic_f) const
	{
		assert(!starts.empty());
		assert(std::all_of(starts.begin(), starts.end(),
		                   [](Node const& n) { return nullptr != n; }));
		assert(nullptr != goal);

		using NodePtr = PlanNode<Dim, T> const*;

		auto comp = [](auto const& a, auto const& b) { return a.second > b.second; };

		using OpenSet =
		    std::priority_queue<std::pair<Node, float>, std::vector<std::pair<Node, float>>,
		                        decltype(comp)>;
		using ClosedSet = std::unordered_set<NodePtr>;
		using CameFrom  = std::unordered_map<NodePtr, Node>;
		using GScore    = std::unordered_map<NodePtr, float>;

		OpenSet   open(comp);
		ClosedSet closed;
		CameFrom  came_from;
		GScore    g_score;

		for (Node const& start : starts) {
			g_score[start.get()] = 0.0f;
			float f_score        = heuristic_f(start, goal);
			open.emplace(start, f_score);
		}

		Node cur;
		while (!open.empty() && goal != (cur = open.top().first)) {
			open.pop();

			if (!closed.insert(cur.get()).second) {
				// We have already processed this node
				continue;
			}

			for (Edge const& edge : cur->edges) {
				Node const& neighbor = edge.to;

				if (0 < closed.count(neighbor.get())) {
					continue;
				}

				float tentative_g_score = g_score[cur.get()] + cost_f(edge);
				if (auto [it, inserted] = g_score.try_emplace(neighbor.get(), tentative_g_score);
				    inserted || tentative_g_score < it->second) {
					came_from[neighbor.get()] = cur;
					it->second                = tentative_g_score;
					float f_score             = tentative_g_score + heuristic_f(neighbor, goal);
					open.emplace(neighbor, f_score);
				}
			}
		}

		std::tuple<Path, float, float> ret{};
		Path&                          path     = std::get<0>(ret);
		float&                         cost     = std::get<1>(ret);
		float&                         distance = std::get<2>(ret);

		if (goal != cur) {
			// Could not find a path
			distance = -1.0f;
			return ret;
		}

		cost = g_score[goal.get()];

		// Reconstruct paths
		path.push_back(goal);
		for (auto it = came_from.find(goal.get()); came_from.end() != it;
		     it      = came_from.find(path.back().get())) {
			path.push_back(it->second);
		}

		std::reverse(path.begin(), path.end());

		for (std::size_t i = 1; path.size() > i; ++i) {
			for (auto const& edge : path[i - 1]->edges) {
				if (edge.to == path[i]) {
					distance += edge.distance;
				}
			}
		}

		return ret;
	}

	//

	[[nodiscard]] std::tuple<Path, float, float> planST(Node const& start, Node const& goal,
	                                                    float distance_weight  = 1.0f,
	                                                    float edge_cost_weight = 1.0f,
	                                                    float node_cost_weight = 1.0f,
	                                                    float heuristic_weight = 1.0f) const
	{
		std::vector<Node> starts(1, start);
		return planST(starts, goal, distance_weight, edge_cost_weight, node_cost_weight,
		              heuristic_weight);
	}

	[[nodiscard]] std::tuple<Path, float, float> planST(std::vector<Node> const& starts,
	                                                    Node const&              goal,
	                                                    float distance_weight  = 1.0f,
	                                                    float edge_cost_weight = 1.0f,
	                                                    float node_cost_weight = 1.0f,
	                                                    float heuristic_weight = 1.0f) const
	{
		auto cost_f = [distance_weight, edge_cost_weight,
		               node_cost_weight](Edge const& edge) {
			return distance_weight * edge.distance + edge_cost_weight * edge.cost +
			       node_cost_weight * edge.to->cost;
		};

		return planST(starts, goal, cost_f, heuristic_weight);
	}

	template <class CostFun>
	[[nodiscard]] std::tuple<Path, float, float> planST(Node const& start, Node const& goal,
	                                                    CostFun cost_f,
	                                                    float heuristic_weight = 1.0f) const
	{
		std::vector<Node> starts(1, start);
		return planST(starts, goal, cost_f, heuristic_weight);
	}

	template <class CostFun>
	[[nodiscard]] std::tuple<Path, float, float> planST(std::vector<Node> const& starts,
	                                                    Node const& goal, CostFun cost_f,
	                                                    float heuristic_weight = 1.0f) const
	{
		auto heuristic_f = [heuristic_weight](Node const& cur, Node const& goal) {
			return heuristic_weight * distance(*cur, *goal);
		};

		return planST(starts, goal, cost_f, heuristic_f);
	}

	template <class CostFun, class HeuristicFun>
	[[nodiscard]] std::tuple<Path, float, float> planST(Node const& start, Node const& goal,
	                                                    CostFun      cost_f,
	                                                    HeuristicFun heuristic_f) const
	{
		std::vector<Node> starts(1, start);
		return planST(starts, goal, cost_f, heuristic_f);
	}

	template <class CostFun, class HeuristicFun>
	[[nodiscard]] std::tuple<Path, float, float> planST(std::vector<Node> const& starts,
	                                                    Node const& goal, CostFun cost_f,
	                                                    HeuristicFun heuristic_f) const
	{
		assert(!starts.empty());
		assert(std::all_of(starts.begin(), starts.end(),
		                   [](Node const& n) { return nullptr != n; }));
		assert(nullptr != goal);

		using NodePtr = PlanNode<Dim, T>*;

		auto comp = [](auto const& a, auto const& b) { return a.second > b.second; };

		using OpenSet =
		    std::priority_queue<std::pair<NodePtr, float>,
		                        std::vector<std::pair<NodePtr, float>>, decltype(comp)>;

		OpenSet open(comp);

		for (Node const& start : starts) {
			start->g_score_ = T(0);
			float f_score   = heuristic_f(start, goal);
			open.emplace(start.get(), f_score);
			touched_nodes_.push_back(start.get());
		}

		NodePtr cur = nullptr;
		while (!open.empty() && goal.get() != (cur = open.top().first)) {
			open.pop();

			if (cur->visited_) {
				// We have already processed this node
				continue;
			}

			cur->visited_ = true;

			for (Edge const& edge : cur->edges) {
				Node const& neighbor = edge.to;

				if (neighbor->visited_) {
					continue;
				}

				float tentative_g_score = cur->g_score_ + cost_f(edge);
				if (tentative_g_score < neighbor->g_score_) {
					neighbor->came_from_ = cur;
					neighbor->g_score_   = tentative_g_score;
					float f_score        = tentative_g_score + heuristic_f(neighbor, goal);
					open.emplace(neighbor.get(), f_score);
					touched_nodes_.push_back(neighbor.get());
				}
			}
		}

		std::tuple<Path, float, float> ret{};
		Path&                          path     = std::get<0>(ret);
		float&                         cost     = std::get<1>(ret);
		float&                         distance = std::get<2>(ret);

		if (goal.get() != cur) {
			// Could not find a path

			for (NodePtr node : touched_nodes_) {
				node->resetPlanningState();
			}
			touched_nodes_.clear();

			distance = -1.0f;
			return ret;
		}

		cost = goal->g_score_;

		// Reconstruct paths
		path.push_back(goal);
		for (NodePtr it = goal->came_from_; it; it = it->came_from_) {
			path.push_back(it->shared_from_this());
		}

		std::reverse(path.begin(), path.end());

		for (std::size_t i = 1; path.size() > i; ++i) {
			for (auto const& edge : path[i - 1]->edges) {
				if (edge.to == path[i]) {
					distance += edge.distance;
				}
			}
		}

		for (NodePtr node : touched_nodes_) {
			node->resetPlanningState();
		}
		touched_nodes_.clear();

		return ret;
	}

	Node addNode(Vec<Dim, T> const& position, T const& cost)
	{
		Node node = std::make_shared<PlanNode<Dim, T>>(position, cost);
		graph_.emplace(position, node);
		return node;
	}

	bool eraseNode(Node const& node)
	{
		// TODO: Remove from all edges

		Vec<Dim, T> position(node);
		for (auto const& v : graph_.nearest(position)) {
			if (v.second == node) {
				graph_.erase(v);
				return true;
			} else if (v.first != position) {
				return false;
			}
		}
		return false;
	}

	bool addEdge(Node const& from, Node const& to, T const& cost = T(0))
	{
		return from->addEdge(to, cost);
	}

	bool addEdgeDistance(Node const& from, Node const& to, T const& cost, T const& distance)
	{
		return from->addEdge(to, cost, distance);
	}

	bool addBiEdge(Node const& from, Node const& to, T const& cost = T(0))
	{
		return addBiEdge(from, to, cost, cost);
	}

	bool addBiEdgeDistance(Node const& from, Node const& to, T const& cost,
	                       T const& distance)
	{
		return addBiEdgeDistance(from, to, cost, cost, distance, distance);
	}

	int addBiEdge(Node const& from, Node const& to, T const& cost_1, T const& cost_2)
	{
		int res{};
		res += from->addEdge(to, cost_1) ? 1 : 0;
		res += to->addEdge(from, cost_2) ? 1 : 0;
		return res;
	}

	int addBiEdgeDistance(Node const& from, Node const& to, T const& cost_1,
	                      T const& cost_2, T const& distance_1, T const& distance_2)
	{
		int res{};
		res += from->addEdge(to, cost_1, distance_1) ? 1 : 0;
		res += to->addEdge(from, cost_2, distance_2) ? 1 : 0;
		return res;
	}

	int eraseEdge(Node const& from, Node const& to) { return from->eraseEdge(to); }

	int eraseBiEdge(Node const& from, Node const& to)
	{
		int res{};
		res += from->eraseEdge(to) ? 1 : 0;
		res += to->eraseEdge(from) ? 1 : 0;
		return res;
	}

	template <class Geometry>
	[[nodiscard]] Node nearestNode(Geometry const& geometry) const
	{
		for (auto const& [_, node] : graph_.nearest(geometry)) {
			return node;
		}
		return Node();
	}

	template <class Geometry>
	[[nodiscard]] std::vector<Node> kNearestNodes(Geometry const& geometry,
	                                              unsigned        k) const
	{
		std::vector<Node> res;
		res.reserve(k);
		for (auto const& [_, node] : graph_.nearest(geometry)) {
			if (0 == k) {
				break;
			}
			--k;
			res.push_back(node);
		}
		return res;
	}

	template <class Geometry>
	[[nodiscard]] std::vector<Node> intersectingNodes(Geometry const& geometry) const
	{
		std::vector<Node> res;
		for (auto const& [_, node] : graph_.query(pred::Intersects(geometry))) {
			res.push_back(node);
		}
		return res;
	}

 private:
	TreeMap<Dim, Node> graph_;

	// Stateful planning
	mutable std::vector<PlanNode<Dim, T>*> touched_nodes_;
};
}  // namespace ufo

#endif  // UFO_PLAN_GRAPH_HPP