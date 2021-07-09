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
#include "xn_math.hpp"
#include <bits/stdc++.h>
#include <queue>
#include <unordered_map>
#include <vector>

namespace xn {
typedef struct GridNode GridNode;
struct GridNode {
    int x = 0, y = 0;
    std::vector<GridNode *> neighbors;
    int cost; // gscore[n] + h(n)

    GridNode() {
        this->x = 0;
        this->y = 0;
        this->neighbors = std::vector<GridNode *>();
        this->cost = 0;
    }

    GridNode(int x, int y) {
        this->x = x;
        this->y = y;
        this->neighbors = std::vector<GridNode *>();
        this->cost = 0;
    }

    bool operator==(const GridNode &p) const { return x == p.x && y == p.y; }
};

struct GridNodeCompare {
    bool reverse;
    GridNodeCompare(const bool &revparam = true) { reverse = revparam; }
    bool operator()(const GridNode &lhs, const GridNode &rhs) const {
        if (reverse)
            return (lhs.cost > rhs.cost);
        else
            return (lhs.cost < rhs.cost);
    }
};

struct GridHash {
    std::size_t operator()(const GridNode &node) const {
        std::size_t h1 = std::hash<int>()(node.x);
        std::size_t h2 = std::hash<int>()(node.y);
        return h1 ^ h2;
    }
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

int manhattan(GridNode &a, GridNode &b) {
    return abs(b.x - a.x) + abs(b.y - a.y);
}

std::vector<GridNode>
trace_path(std::unordered_map<GridNode, GridNode, GridHash> came_from,
           GridNode current) {
    std::vector<GridNode> out;
    out.push_back(current);
    while (came_from.find(current) != came_from.end()) {
        current = came_from[current];
        out.insert(out.begin(), current);
    }
    return out;
}

std::vector<GridNode> preprocess_graph(std_vec2d<bool> graph) {
    std::vector<GridNode> node_graph;
    for (unsigned int i = 0; i < graph.size(); i++) {
        for (unsigned int j = 0; j < graph[i].size(); j++) {
            GridNode n(i, j);
            if (graph[i][j])
                node_graph.push_back(n);
        }
    }

    //  Any 2 neighbors sharing a column will be at most graph.front().size()
    //  elements away
    auto get_x_neighbor = [&node_graph,
                           &graph](std::vector<GridNode>::iterator &i, int a) {
        if (a > 0 ? (unsigned)i->x < graph.front().size() - 1
                  : (unsigned)i->x > 0 && graph[i->x + a][i->y])
            for (auto j = i + a;
                 a > 0 ? (j < node_graph.end()) : (j > node_graph.begin());
                 j += a)
                if (j->x == i->x + a && j->y == i->y) {
                    i->neighbors.push_back(&(*(j)));
                    break;
                }
    };

    for (auto i = node_graph.begin(); i < node_graph.end(); i++) {
        // row neighbors will always be adjacent in memory
        if ((unsigned)i->y > 0 && graph[i->x][i->y - 1])
            i->neighbors.push_back(&(*(i - 1)));
        if ((unsigned)i->y < graph.size() - 1 && graph[i->x][i->y + 1])
            i->neighbors.push_back(&(*(i + 1)));

        get_x_neighbor(i, -1);
        get_x_neighbor(i, 1);
    }

    return node_graph;
}

std::vector<GridNode> A_star(std::vector<GridNode> &processed_graph,
                             int x_start, int y_start, int goal_x, int goal_y) {
    std::priority_queue<GridNode, std::vector<GridNode>, GridNodeCompare>
        open_set;
    std::vector<GridNode> &open_set_contents = Container(open_set);
    std::unordered_map<GridNode, GridNode, GridHash>
        came_from; // came_from[n] = node immediately preceding n on the
                   // cheapest path
    std::unordered_map<GridNode, int, GridHash>
        g_score; // cost of cheapest path from start to key node, default
                 // val=infinity
    GridNode *start = NULL, *goal = NULL;

    for (GridNode &n1 : processed_graph) {
        g_score[n1] = INT_MAX;
        n1.cost = INT_MAX;

        if (x_start == n1.x && y_start == n1.y)
            start = &n1;
        if (goal_x == n1.x && goal_y == n1.y)
            goal = &n1;
    }

    if (start == NULL || goal == NULL)
        return std::vector<GridNode>();

    g_score[*start] = 0;
    start->cost = 0;
    open_set.push(*start);

    while (!open_set.empty()) {
        GridNode current = open_set.top();
        if (current == *goal)
            return trace_path(came_from, current);

        open_set.pop();
        for (GridNode *n_ptr : current.neighbors) {
            GridNode n = *n_ptr;
            int gscore_new = g_score[current] + manhattan(current, n);
            if (gscore_new < g_score[n]) {
                // new best path
                came_from[n] = current;
                g_score[n] = gscore_new;
                n_ptr->cost = g_score[n] + manhattan(*goal, n);

                bool found = false;
                for (auto i = open_set_contents.begin();
                     i < open_set_contents.end(); i++) {
                    if (*i == *n_ptr) {
                        found = true;
                        break;
                    }
                }
                if (!found)
                    open_set.push(*n_ptr);
            }
        }
    }

    // fail case
    return std::vector<GridNode>();
}

std::vector<GridNode> A_star(std_vec2d<bool> graph, GridNode start,
                             GridNode goal) {
    auto pg = preprocess_graph(graph);
    return A_star(pg, start.x, start.y, goal.x, goal.y);
}

glm::vec2 world_to_grid(const glm::vec3 &in, float gridbox_size = 1,
                        const glm::vec3 &offset = glm::vec3(0)) {
    glm::vec2 out(in.x, in.z);
    out.x -= offset.x;
    out.y -= offset.z;
    out /= gridbox_size;
    return glm::floor(out + 0.5f);
}

glm::vec3 grid_to_world(int x, int y, float gridbox_size,
                        const glm::vec3 &offset = glm::vec3(0)) {
    glm::vec3 out(x, 0, y);
    out *= gridbox_size;
    out.x += offset.x;
    out.z += offset.z;
    return out;
}

glm::vec2 screen_to_grid(const glm::vec2 &win_size, const glm::vec2 &pixel,
                         float proj_area, float gridbox_size = 1,
                         const glm::vec3 &offset = glm::vec3(0)) {
    glm::vec2 out = pixel;
    for (int i = 0; i < 2; i++) {
        out[i] /= (win_size[i] / 2);
        out[i] -= 1;
        out[i] *= abs(proj_area);
    }

    out.x -= offset.x;
    out.y -= offset.z;
    out /= gridbox_size;
    return glm::floor(out + 0.5f);
}
} // namespace xn