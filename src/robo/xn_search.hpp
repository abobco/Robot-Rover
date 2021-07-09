/*
    Best-first serach:
    - work on weighted graphs
    - seeks path between 2 nodes having the smallest cost

    A* search:
        - best first search
        - maintains a tree of paths starting at a node,
        - extends these paths 1 edge at a time until termination condition is
   met

        At each loop iteration, A* needs to choose which paths to extend. It
   does this by minimizing 2 costs:
            - g(n): cost of the path from start node to n
            - h(n): heuristic function that estimates the cost of the cheapest
   path from n to the goal. Problem-specific, never overestimates the actual
   cost.

        Making the total cost:

            f(n) = g(n)+h(n)

        Typically A* implementations use a priority queue to select min cost
   nodes to expand.
            - this PQ is known as the open set or fring

        At each step:
            1. The node w/ lowest f(x) val is removed from the queue
            2. the f & g values of its neighbors are updated
            3. these neighbors are added to the queue

        Algorithm continues until a removed node (node w/ lowest f value out of
   all nodes in the PQ) is a goal node. The algorithm described so far only
   gives the length of the shortest path. To know the sequence of steps, each
        node on the path must keep track of its predecessor, so the ending node
   can be traced back to the start node.

        Simple heuristic algorithms(h(x)):
            - distance (euclidean, manhattan, octile)

        TLDR:
            equivalent to running Dijkstra's algorithm w/ the reduced cost:
                d'(x,y)= d(x,y) + h(y) - h(x)

*/
#pragma once
#include "xn_gpio.hpp"
#include <queue>
#include <unordered_map>
#include <vector>

namespace xn {

// typedef struct GridNode GridNode;
struct GridNode {
  int x = 0, y = 0;
  std::vector<struct GridNode *> neighbors;
  int cost; // gscore[n] + h(n)

  GridNode(int x = 0, int y = 0) : x(x), y(y), cost(0) {}

  bool operator==(const GridNode &p) const { return x == p.x && y == p.y; }
};

struct GridNodeCompare {
  bool reverse;
  GridNodeCompare(const bool &revparam = true) { reverse = revparam; }
  bool operator()(const GridNode &lhs, const GridNode &rhs) const;
};

struct GridHash {
  std::size_t operator()(const GridNode &node) const;
};

struct GridGraph {
  std::vector<GridNode> graph; // replaces AppState::get().graph
  std::vector<GridNode> path;  // replaces AppState::get().path
  std_vec2d<bool> cells;       // replaces AppState::get().gridgraph
  glm::vec3 offset;            // replaces AppState::get().grid_offset
  float boxSize = 0.25;        // replaces AppState::get().gridbox_size
};

// access underlying container of an std::priority_queue for iteration
template <class T, class S, class C>
S &Container(std::priority_queue<T, S, C> &q) {
  struct IterableQueue : private std::priority_queue<T, S, C> {
    static S &Container(std::priority_queue<T, S, C> &q) {
      return q.*&IterableQueue::c;
    }
  };
  return IterableQueue::Container(q);
};

int manhattan_distance(GridNode &a, GridNode &b);

std::vector<GridNode>
trace_path(std::unordered_map<GridNode, GridNode, GridHash> came_from,
           GridNode current);

std::vector<GridNode> preprocess_graph(std_vec2d<bool> graph);

std::vector<GridNode> A_star(std::vector<GridNode> &processed_graph,
                             int x_start, int y_start, int goal_x, int goal_y);

std::vector<GridNode> A_star(std_vec2d<bool> graph, GridNode start,
                             GridNode goal);

glm::ivec2 world_to_grid(const glm::vec3 &in, float gridbox_size = 1,
                         const glm::vec3 &offset = glm::vec3(0));

glm::vec3 grid_to_world(int x, int y, float gridbox_size,
                        const glm::vec3 &offset = glm::vec3(0));

glm::vec2 screen_to_grid(const glm::vec2 &win_size, const glm::vec2 &pixel,
                         float proj_area, float gridbox_size = 1,
                         const glm::vec3 &offset = glm::vec3(0));
} // namespace xn