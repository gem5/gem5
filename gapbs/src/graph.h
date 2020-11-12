// Copyright (c) 2015, The Regents of the University of California (Regents)
// See LICENSE.txt for license details

#ifndef GRAPH_H_
#define GRAPH_H_

#include <algorithm>
#include <cinttypes>
#include <cstddef>
#include <iostream>
#include <type_traits>

#include "pvector.h"
#include "util.h"


/*
GAP Benchmark Suite
Class:  CSRGraph
Author: Scott Beamer

Simple container for graph in CSR format
 - Intended to be constructed by a Builder
 - To make weighted, set DestID_ template type to NodeWeight
 - MakeInverse parameter controls whether graph stores its inverse
*/


// Used to hold node & weight, with another node it makes a weighted edge
template <typename NodeID_, typename WeightT_>
struct NodeWeight {
  NodeID_ v;
  WeightT_ w;
  NodeWeight() {}
  NodeWeight(NodeID_ v) : v(v), w(1) {}
  NodeWeight(NodeID_ v, WeightT_ w) : v(v), w(w) {}

  bool operator< (const NodeWeight& rhs) const {
    return v == rhs.v ? w < rhs.w : v < rhs.v;
  }

  // doesn't check WeightT_s, needed to remove duplicate edges
  bool operator== (const NodeWeight& rhs) const {
    return v == rhs.v;
  }

  // doesn't check WeightT_s, needed to remove self edges
  bool operator== (const NodeID_& rhs) const {
    return v == rhs;
  }

  operator NodeID_() {
    return v;
  }
};

template <typename NodeID_, typename WeightT_>
std::ostream& operator<<(std::ostream& os,
                         const NodeWeight<NodeID_, WeightT_>& nw) {
  os << nw.v << " " << nw.w;
  return os;
}

template <typename NodeID_, typename WeightT_>
std::istream& operator>>(std::istream& is, NodeWeight<NodeID_, WeightT_>& nw) {
  is >> nw.v >> nw.w;
  return is;
}



// Syntatic sugar for an edge
template <typename SrcT, typename DstT = SrcT>
struct EdgePair {
  SrcT u;
  DstT v;

  EdgePair() {}

  EdgePair(SrcT u, DstT v) : u(u), v(v) {}
};

// SG = serialized graph, these types are for writing graph to file
typedef int32_t SGID;
typedef EdgePair<SGID> SGEdge;
typedef int64_t SGOffset;



template <class NodeID_, class DestID_ = NodeID_, bool MakeInverse = true>
class CSRGraph {
  // Used for *non-negative* offsets within a neighborhood
  typedef std::make_unsigned<std::ptrdiff_t>::type OffsetT;

  // Used to access neighbors of vertex, basically sugar for iterators
  class Neighborhood {
    NodeID_ n_;
    DestID_** g_index_;
    OffsetT start_offset_;
   public:
    Neighborhood(NodeID_ n, DestID_** g_index, OffsetT start_offset) :
        n_(n), g_index_(g_index), start_offset_(0) {
      OffsetT max_offset = end() - begin();
      start_offset_ = std::min(start_offset, max_offset);
    }
    typedef DestID_* iterator;
    iterator begin() { return g_index_[n_] + start_offset_; }
    iterator end()   { return g_index_[n_+1]; }
  };

  void ReleaseResources() {
    if (out_index_ != nullptr)
      delete[] out_index_;
    if (out_neighbors_ != nullptr)
      delete[] out_neighbors_;
    if (directed_) {
      if (in_index_ != nullptr)
        delete[] in_index_;
      if (in_neighbors_ != nullptr)
        delete[] in_neighbors_;
    }
  }


 public:
  CSRGraph() : directed_(false), num_nodes_(-1), num_edges_(-1),
    out_index_(nullptr), out_neighbors_(nullptr),
    in_index_(nullptr), in_neighbors_(nullptr) {}

  CSRGraph(int64_t num_nodes, DestID_** index, DestID_* neighs) :
    directed_(false), num_nodes_(num_nodes),
    out_index_(index), out_neighbors_(neighs),
    in_index_(index), in_neighbors_(neighs) {
      num_edges_ = (out_index_[num_nodes_] - out_index_[0]) / 2;
    }

  CSRGraph(int64_t num_nodes, DestID_** out_index, DestID_* out_neighs,
        DestID_** in_index, DestID_* in_neighs) :
    directed_(true), num_nodes_(num_nodes),
    out_index_(out_index), out_neighbors_(out_neighs),
    in_index_(in_index), in_neighbors_(in_neighs) {
      num_edges_ = out_index_[num_nodes_] - out_index_[0];
    }

  CSRGraph(CSRGraph&& other) : directed_(other.directed_),
    num_nodes_(other.num_nodes_), num_edges_(other.num_edges_),
    out_index_(other.out_index_), out_neighbors_(other.out_neighbors_),
    in_index_(other.in_index_), in_neighbors_(other.in_neighbors_) {
      other.num_edges_ = -1;
      other.num_nodes_ = -1;
      other.out_index_ = nullptr;
      other.out_neighbors_ = nullptr;
      other.in_index_ = nullptr;
      other.in_neighbors_ = nullptr;
  }

  ~CSRGraph() {
    ReleaseResources();
  }

  CSRGraph& operator=(CSRGraph&& other) {
    if (this != &other) {
      ReleaseResources();
      directed_ = other.directed_;
      num_edges_ = other.num_edges_;
      num_nodes_ = other.num_nodes_;
      out_index_ = other.out_index_;
      out_neighbors_ = other.out_neighbors_;
      in_index_ = other.in_index_;
      in_neighbors_ = other.in_neighbors_;
      other.num_edges_ = -1;
      other.num_nodes_ = -1;
      other.out_index_ = nullptr;
      other.out_neighbors_ = nullptr;
      other.in_index_ = nullptr;
      other.in_neighbors_ = nullptr;
    }
    return *this;
  }

  bool directed() const {
    return directed_;
  }

  int64_t num_nodes() const {
    return num_nodes_;
  }

  int64_t num_edges() const {
    return num_edges_;
  }

  int64_t num_edges_directed() const {
    return directed_ ? num_edges_ : 2*num_edges_;
  }

  int64_t out_degree(NodeID_ v) const {
    return out_index_[v+1] - out_index_[v];
  }

  int64_t in_degree(NodeID_ v) const {
    static_assert(MakeInverse, "Graph inversion disabled but reading inverse");
    return in_index_[v+1] - in_index_[v];
  }

  Neighborhood out_neigh(NodeID_ n, OffsetT start_offset = 0) const {
    return Neighborhood(n, out_index_, start_offset);
  }

  Neighborhood in_neigh(NodeID_ n, OffsetT start_offset = 0) const {
    static_assert(MakeInverse, "Graph inversion disabled but reading inverse");
    return Neighborhood(n, in_index_, start_offset);
  }

  void PrintStats() const {
    std::cout << "Graph has " << num_nodes_ << " nodes and "
              << num_edges_ << " ";
    if (!directed_)
      std::cout << "un";
    std::cout << "directed edges for degree: ";
    std::cout << num_edges_/num_nodes_ << std::endl;
  }

  void PrintTopology() const {
    for (NodeID_ i=0; i < num_nodes_; i++) {
      std::cout << i << ": ";
      for (DestID_ j : out_neigh(i)) {
        std::cout << j << " ";
      }
      std::cout << std::endl;
    }
  }

  static DestID_** GenIndex(const pvector<SGOffset> &offsets, DestID_* neighs) {
    NodeID_ length = offsets.size();
    DestID_** index = new DestID_*[length];
    #pragma omp parallel for
    for (NodeID_ n=0; n < length; n++)
      index[n] = neighs + offsets[n];
    return index;
  }

  pvector<SGOffset> VertexOffsets(bool in_graph = false) const {
    pvector<SGOffset> offsets(num_nodes_+1);
    for (NodeID_ n=0; n < num_nodes_+1; n++)
      if (in_graph)
        offsets[n] = in_index_[n] - in_index_[0];
      else
        offsets[n] = out_index_[n] - out_index_[0];
    return offsets;
  }

  Range<NodeID_> vertices() const {
    return Range<NodeID_>(num_nodes());
  }

 private:
  bool directed_;
  int64_t num_nodes_;
  int64_t num_edges_;
  DestID_** out_index_;
  DestID_*  out_neighbors_;
  DestID_** in_index_;
  DestID_*  in_neighbors_;
};

#endif  // GRAPH_H_
