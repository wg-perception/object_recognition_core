#include <iostream>

#include "maximum_clique.h"

int main(int argc, char **argv) {
  object_recognition::maximum_clique::Graph graph("example_graph.asc");
  object_recognition::maximum_clique::Graph::Vertices vertices;
  graph.FindMaximumClique(vertices);
  std::cout << "max clique: " << vertices.size() << " while it should be " << " 4 " << std::endl;
}
