// Copyright srcmake.com 2018.
// C++ Example Dijkstra Algorithm For Shortest Path (With PQ/Min-Heap)
// Official webpage for this tutorial: https://www.srcmake.com/home/cpp-shortest-path-dijkstra

/* The Dijkstra algorithm:
    // Initialize the graph adjacency list. adjList[i] = pair<int, int> where first is vertex, second is edge weight.
    // Initialize all source->vertex as infinite.
    // Create a PQ.
    // Add source to pq, where distance is 0.
    // While pq isn't empty...
        // Get min distance vertex from pq. (Call it u.)
        // Visit all of u's friends. For each one (called v)....
            // If the distance to v is shorter by going through u...
                // Update the distance of v.
                // Insert v into the pq. 
*/

// The example graph: https://www.srcmake.com/uploads/5/3/9/0/5390645/spgraph_orig.jpg

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <array>

#include "graph.h"

using namespace std;

// An adjacency list. Each adjList[i] holds a all the friends of node i.
// The first int is the vertex of the friend, the second int is the edge weight.
vector< vector<pair<int, int> > > FormAdjList()
    {
    // Our adjacency list.
    vector< vector<pair<int, int> > > adjList;
    
    // We have 42 vertices, so initialize 42 rows.
    const int n = 42;
    
    for(int i = 0; i < n; i++)
        {
        // Create a vector to represent a row, and add it to the adjList.
        vector<pair<int, int> > row;
        adjList.push_back(row);
        }
    
    
    // Now let's add our actual edges into the adjacency list.
    // See the picture here: https://www.srcmake.com/uploads/5/3/9/0/5390645/spadjlist_orig.jpg
    
    //von                       nach // kosten 
    adjList[0].push_back(make_pair(1,   1));
    adjList[1].push_back(make_pair(2,   1));
    adjList[2].push_back(make_pair(3,   1));
    adjList[3].push_back(make_pair(4,   1));
    adjList[4].push_back(make_pair(5,   1));
    adjList[5].push_back(make_pair(6,   1));
    adjList[6].push_back(make_pair(7,   1));
    adjList[7].push_back(make_pair(8,   1));
    adjList[8].push_back(make_pair(9,   1));
    adjList[9].push_back(make_pair(10,  1));
    adjList[10].push_back(make_pair(11, 1));
    adjList[11].push_back(make_pair(12, 1));
    adjList[12].push_back(make_pair(13, 1));
    adjList[13].push_back(make_pair(11, 1));
    adjList[11].push_back(make_pair(13, 1));
    adjList[13].push_back(make_pair(14, 1));
    adjList[14].push_back(make_pair(15, 1));
    adjList[15].push_back(make_pair(16, 1));
    adjList[16].push_back(make_pair(8,  1));
    adjList[8].push_back(make_pair(16,  1));
    adjList[16].push_back(make_pair(17, 1));
    adjList[17].push_back(make_pair(7,  1));
    adjList[7].push_back(make_pair(17,  1));
    adjList[17].push_back(make_pair(18, 1));
    adjList[18].push_back(make_pair(17, 1));
    adjList[17].push_back(make_pair(6,  1));
    adjList[6].push_back(make_pair(17,  1));
    adjList[7].push_back(make_pair(18,  1));     //neuer knoten
    adjList[18].push_back(make_pair(19, 1));
    adjList[19].push_back(make_pair(20, 1));
    adjList[20].push_back(make_pair(5,  1));
    adjList[5].push_back(make_pair(20,  1));
    adjList[20].push_back(make_pair(21, 1));
    adjList[21].push_back(make_pair(4,  1));
    adjList[4].push_back(make_pair(21,  1));
    adjList[21].push_back(make_pair(22, 1));
    adjList[22].push_back(make_pair(3,  1));
    adjList[3].push_back(make_pair(22,  1));
    adjList[22].push_back(make_pair(23, 1));
    adjList[23].push_back(make_pair(2,  1));
    adjList[2].push_back(make_pair(23,  1));
    adjList[23].push_back(make_pair(24, 1));
    adjList[24].push_back(make_pair(25, 1));
    adjList[25].push_back(make_pair(26, 1));
    adjList[26].push_back(make_pair(27, 1));
    adjList[27].push_back(make_pair(28, 1));
    adjList[28].push_back(make_pair(29, 1));
    adjList[29].push_back(make_pair(25, 1));
    adjList[25].push_back(make_pair(29, 1));
    adjList[29].push_back(make_pair(30, 1));
    adjList[30].push_back(make_pair(37, 1));
    adjList[37].push_back(make_pair(30, 1));
    adjList[30].push_back(make_pair(31, 1));
    adjList[31].push_back(make_pair(32, 1));
    adjList[32].push_back(make_pair(33, 1));
    adjList[33].push_back(make_pair(34, 1));
    adjList[34].push_back(make_pair(35, 1));
    adjList[35].push_back(make_pair(36, 1));
    adjList[36].push_back(make_pair(35, 1));
    adjList[36].push_back(make_pair(37, 1));
    adjList[37].push_back(make_pair(36, 1));
    adjList[36].push_back(make_pair(38, 1));
    adjList[38].push_back(make_pair(33, 1));
    adjList[33].push_back(make_pair(38, 1));
    adjList[38].push_back(make_pair(39, 1));
    adjList[39].push_back(make_pair(32, 1));
    adjList[32].push_back(make_pair(39, 1));
    adjList[39].push_back(make_pair(40, 1));
    adjList[40].push_back(make_pair(41, 1));
    adjList[41].push_back(make_pair(30, 1));
    adjList[30].push_back(make_pair(41, 1));
    adjList[41].push_back(make_pair(2,  1));
    // Our graph is now represented as an adjacency list. Return it.
    return adjList;
    }
    
// Given an Adjacency List, find all shortest paths from "start" to all other vertices.
vector< pair<int, int> > DijkstraSP(vector< vector<pair<int, int> > > &adjList, int &start)
    {
    vector<pair<int, int> > dist; // First int is dist, second is the previous node. 
    
    // Initialize all source->vertex as infinite.
    int n = adjList.size();
    for(int i = 0; i < n; i++)
        {
        dist.push_back(make_pair(1000000007, i)); // Define "infinity" as necessary by constraints.
        }
        
    // Create a PQ.
    priority_queue<pair<int, int>, vector< pair<int, int> >, greater<pair<int, int> > > pq;
    
    // Add source to pq, where distance is 0.
    pq.push(make_pair(start, 0));
    dist[start] = make_pair(0, start);;
    
    // While pq isn't empty...
    while(pq.empty() == false)
        {
        // Get min distance vertex from pq. (Call it u.)
        int u = pq.top().first;
        pq.pop();
        
        // Visit all of u's friends. For each one (called v)....
        for(int i = 0; i < adjList[u].size(); i++)
            {
            int v = adjList[u][i].first;
            int weight = adjList[u][i].second;
            
            // If the distance to v is shorter by going through u...
            if(dist[v].first > dist[u].first + weight)
                {
                // Update the distance of v.
                dist[v].first = dist[u].first + weight;
                // Update the previous node of v.
                dist[v].second = u;
                // Insert v into the pq. 
                pq.push(make_pair(v, dist[v].first));
                }
            }
        }
    
    return dist;
    }
    
// void PrintShortestPathAll(vector< pair<int, int> > &dist, int &start)
//     {
//     cout << "\nPrinting the shortest paths for node " << start << ".\n";
//     for(int i = 0; i < dist.size(); i++)
//         {
//         cout << "The distance from node " << start << " to node " << i << " is: " << dist[i].first << endl;
        
//         int currnode = i;
//         cout << "The path is: " << currnode;
//         while(currnode != start)
//             {
//             currnode = dist[currnode].second;
//             cout << " <- " << currnode;
//             }
//         cout << endl << endl;
//         }
//     }

// //Mee

// void PrintShortestPath(vector< pair<int, int> > &dist, int &start, int &end)
//     {
//     cout << "\nPrinting the shortest paths for node " << start << ".\n";

//         cout << "The distance from node " << start << " to node " << end << " is: " << dist[end].first << endl;
        

//         int currnode = end;
//         int * routeBuffer = new int[42];
//         cout << "The path is: " << currnode;

//         int counter =0;
//         while(currnode != start)
//             {
//             routeBuffer[counter] = currnode;
//             counter++;
//             currnode = dist[currnode].second;
//             cout << " <- " << currnode;
            
//             }
//         cout << endl << endl;
//          int Array[counter];
        
//         cout <<"shortestPath \n";
        
//         for(int i=0; i <=counter; i++ ){
//             Array[i] =  routeBuffer[counter - i];
//             cout<< Array[i];  
//         }       
//                 cout << endl << endl;


//     }



struct MyArray returnShortestPath(vector< pair<int, int> > &dist, int &start, int &end)
    {      
        int currnode = end;
        int * routeBuffer = new int[42];

        int counter =0;
        while(currnode != start)
            {
            routeBuffer[counter] = currnode;
            counter++;
            currnode = dist[currnode].second;
            }
           
        int * Array = new int[counter+1];
       for(int i=0; i <=counter; i++ ){
            Array[i] =  routeBuffer[counter - i];
        }   
        struct MyArray array;
        array.pointer = Array;
        array.size = counter +1;
        free(routeBuffer);
        return array;
    }
