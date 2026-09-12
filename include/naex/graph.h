#pragma once

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <naex/iterators.h>
#include <naex/map.h>
#include <naex/types.h>
#include <utility>

// Boost graph adapter for naex::Graph / naex::EdgeCosts.
// NB: naex/grid/graph.h provides an equivalent adapter for the grid planner.
namespace boost {

template <> struct graph_traits<naex::Graph> {
  typedef naex::Vertex vertex_descriptor;
  typedef naex::Vertex vertices_size_type;
  typedef naex::Edge edge_descriptor;
  typedef naex::Edge edges_size_type;

  typedef directed_tag directed_category;
  typedef disallow_parallel_edge_tag edge_parallel_category;

  typedef bidirectional_traversal_tag traversal_category;
  typedef naex::VertexIter vertex_iterator;
  typedef naex::EdgeIter out_edge_iterator;
};

inline naex::Vertex num_vertices(const naex::Graph &g) {
  return g.num_vertices();
}

inline std::pair<naex::VertexIter, naex::VertexIter>
vertices(const naex::Graph &g) {
  return g.vertices();
}

inline naex::Vertex source(naex::Edge e, const naex::Graph &g) {
  return g.source(e);
}

inline naex::Vertex target(naex::Edge e, const naex::Graph &g) {
  return g.target(e);
}

inline std::pair<naex::EdgeIter, naex::EdgeIter>
out_edges(naex::Vertex u, const naex::Graph &g) {
  return g.out_edges(u);
}

inline naex::Edge out_degree(naex::Vertex u, const naex::Graph &g) {
  return g.out_degree(u);
}

template <> class property_traits<naex::EdgeCosts> {
public:
  typedef naex::Edge key_type;
  typedef naex::Cost value_type;
  typedef readable_property_map_tag category;
};

inline naex::Cost get(const naex::EdgeCosts &map, const naex::Edge &key) {
  return map[key];
}

} // namespace boost

// Include the Dijkstra header once all used concepts are defined.
// https://groups.google.com/g/boost-developers-archive/c/G2qArovLKzk
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
