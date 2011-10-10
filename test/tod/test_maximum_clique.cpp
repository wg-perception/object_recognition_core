#include <iostream>

#include <gtest/gtest.h>

#include "maximum_clique.h"

TEST(OR_tod, Graph1)
{
  object_recognition::maximum_clique::Graph graph(10);
  graph.AddEdge(4, 1);
  graph.AddEdge(4, 3);
  graph.AddEdge(5, 3);
  graph.AddEdge(6, 1);
  graph.AddEdge(6, 4);
  graph.AddEdge(7, 0);
  graph.AddEdge(7, 2);
  graph.AddEdge(7, 3);
  graph.AddEdge(7, 4);
  graph.AddEdge(7, 5);
  graph.AddEdge(8, 0);
  graph.AddEdge(8, 2);
  graph.AddEdge(8, 3);
  graph.AddEdge(8, 5);
  graph.AddEdge(8, 6);
  graph.AddEdge(9, 0);
  graph.AddEdge(9, 1);
  graph.AddEdge(9, 2);
  graph.AddEdge(9, 3);
  graph.AddEdge(9, 4);
  graph.AddEdge(9, 6);
  graph.AddEdge(9, 7);
  graph.AddEdge(9, 8);

  object_recognition::maximum_clique::Graph::Vertices vertices;
  graph.FindMaximumClique(vertices);

  EXPECT_EQ(vertices.size(), 4);
}

TEST(OR_tod, Graph2)
{
  object_recognition::maximum_clique::Graph graph(10);
  // Connect all of the vertices
  for (unsigned int i = 0; i < 10; ++i)
    for (unsigned int j = i + 1; j < 10; ++j)
      graph.AddEdge(i, j);
  graph.DeleteEdge(0, 1);

  object_recognition::maximum_clique::Graph::Vertices vertices;
  graph.FindMaximumClique(vertices);

  EXPECT_EQ(vertices.size(), 9);
}
