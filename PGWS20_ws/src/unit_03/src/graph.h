#ifndef GRAPH_H
#define GRAPH_H


#include <iostream>
#include <vector>
#include <queue>
#include <array>
using namespace std;

//class Graph{
// An adjacency list. Each adjList[i] holds a all the friends of node i.
// The first int is the vertex of the friend, the second int is the edge weight.

typedef struct MyArray {
  int* pointer;
  int size;
}MyArray;

vector< vector<pair<int, int> > >  FormAdjList();
    
// Given an Adjacency List, find all shortest paths from "start" to all other vertices.
vector< pair<int, int> > DijkstraSP(vector< vector<pair<int, int> > > &adjList, int &start);
    
    
// void PrintShortestPathAll(vector< pair<int, int> > &dist, int &start);
//Mee

// void PrintShortestPath(vector< pair<int, int> > &dist, int &start, int &end);

//int * shortestPathFromTo(vector< pair<int, int> > &dist, int &start, int &end);
struct MyArray returnShortestPath(vector< pair<int, int> > &dist, int &start, int &end);
//};
#endif //GRAPH_H