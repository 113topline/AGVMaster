#include "Astar.hpp"

#include <bits/stdc++.h>

#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

/// @brief OPEN List
vector<vertex *> OPEN;
/// @brief CLOSED List
vector<vertex *> CLOSED;
/// @brief Points' List
vector<vertex *> points;
/// @brief Path list
vector<vertex *> path;

/// @brief Constructor for A* class. Here you specify the basic parameters.
/// The grid lines and columns are not recommended to be changed after
/// construction. The function createMaze() is called automatically.
/// @param gl The "height" of the maze (number of lines)
/// @param gc The "width" of the maze (number of columns)
/// @param sx The starting vertex X coordinate
/// @param sy The starting vertex Y coordinate
/// @param gx The goal vertex X coordinate
/// @param gy The vertex X coordinate
Astar::Astar(int gl, int gc, int sx, int sy, int gx, int gy) {
  _sx = sx;
  _sy = sy;
  _gx = gx;
  _gy = gy;
  _gridl = gl;
  _gridc = gc;
  this->createMaze();
}

/// @brief Destructor for A* class. It wipes all data from all vector lists.
Astar::~Astar() {}

void Astar::createMaze() {
  vector<vector<int>> temp;
  vector<int> cols;
  for (int j = 0; j < _gridc; j++) cols.push_back(0);

  for (int i = 0; i < _gridl; i++) temp.push_back(cols);

  maze = temp;
}

void Astar::clearSearch() {
  vector<vertex *>::iterator it = OPEN.begin();
  while (it != OPEN.end()) {
    delete *it;
    it = OPEN.erase(it);
  }
  it = CLOSED.begin();
  while (it != CLOSED.end()) {
    delete *it;
    it = CLOSED.erase(it);
  }
  /*  it = path.begin();
   while (it != path.end()) {
     delete *it;
     it = path.erase(it);
   } */
}

/// @brief Adds a vertex from a list (OPEN or CLOSED)
/// @param v The vertex' pointer
/// @param vList The list of interest
void Astar::add(vertex *v, vector<vertex *> &vList) { vList.push_back(v); }

/// @brief Adds a vertex to the points list
/// @param v The vertex' pointer
void Astar::addVertex(vertex *v) { points.push_back(v); }

/// @brief Removes a vertex from a list (OPEN or CLOSED)
/// @param v The vertex' pointer
/// @param vList The list of interest
void Astar::erase(vertex *v, vector<vertex *> &vList) {
  for (unsigned int i = 0; i != vList.size(); i++) {
    if (vList[i] == v) {
      vList.erase(vList.begin() + i);
      break;
    }
  }
}

/// @brief Creates a vertex
/// @param x The vertex' x value
/// @param y The vertex' y value
/// @return Returns the vertex pointer
vertex *Astar::createVertex(int x, int y) {
  vertex *v = (vertex *)malloc(sizeof(vertex));
  v->x = x;
  v->y = y;
  v->f = v->g = v->h = INFINITY;
  v->cost = 1;
  v->bestparent = nullptr;
  v->parents.clear();
  // v->child = nullptr;
  return v;
}

/// @brief Gets a vertex address using its coordinates
/// @param X The vertex' x value
/// @param Y The vertex' y value
/// @return Returns the vertex pointer
vertex *Astar::getVertex(int X, int Y) {
  vertex *j = nullptr;
  for (auto &i : points) {
    if (i->x == X && i->y == Y) {
      j = i;
      break;
    }
  }
  return j;
}

/// @brief Initializes an array of all vertices
/// @param rows Number of rows of the map
/// @param cols Number of cols of the map
void Astar::initGrid() {

  if (maze.size() == 0) {
    for (int i = 0; i < _gridl; i++) {
      for (int j = 0; j < _gridc; j++) {
        this->addVertex(this->createVertex(i, j));
      }
    }
    this->createMaze();
  }
}

/// @brief Sets new value for goal vertex
/// @param x X coordinate of the vertex
/// @param y Y coordinate of the vertex
void Astar::setGoal(int x, int y) {
  _gx = x;
  _gy = y;
}

/// @brief Sets new value for start vertex
/// @param x X coordinate of the vertex
/// @param y Y coordinate of the vertex
void Astar::setStart(int x, int y) {
  _sx = x;
  _sy = y;
}

/// @brief Updates vertex cost
/// @param v The vertex to be updated
/// @param cost The new cost of the vertex
void Astar::updateVertexCost(vertex *v, float cost) { v->cost = cost; }

/// @brief Updates vertex state on maze
/// @param v The vertex to be updated
/// @param s The state of the vertex (CLEAR = 0, OBSTACLE = 1)
/// @param v2 The second vertex (optional). The function changes then all
/// vertices in between the two vertices informed
void Astar::updateVertex(vertex *v, state s, vertex *v2) {
  if (v2 == nullptr) {
    maze[v->x][v->y] = s;
  } else {
    for (int i = (v->x > v2->x ? v2->x : v->x);
         i <= (v->x > v2->x ? v->x : v2->x); i++)
      for (int j = (v->y > v2->y ? v2->y : v->y);
           j <= (v->y > v2->y ? v->y : v2->y); j++)
        maze[i][j] = s;
  }
}

/// @brief Returns if a vertex is a obstacle or not
/// @param map The maze to be analysed
/// @param v The vertex of interest
/// @return If it is an obstacle (true) or not (false)
bool Astar::isObstacle(vector<vector<int>> map, vertex *v) {
  return (map[v->x][v->y] == OBSTACLE);
}

/// @brief Checks all neighbours of an vertex on four directions
/// @param v The vertex of interest
/// @return A vertex containing all valid neighbours
vector<vertex *> Astar::getNeighbours(vertex *v) {
  vector<vertex *> list;
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      if (i == j || i == -j)
        continue;  // excludes diagonal/center points (manhattan distance
                   // adopted)
      if (i + v->x < 0 || i + v->x >= _gridl)
        continue;  // excludes points outside the grid x-range
      if (j + v->y < 0 || j + v->y >= _gridc)
        continue;  // excludes points outside the grid y-range
      vertex *x = this->getVertex(i + v->x, j + v->y);
      if (!this->isObstacle(maze, x)) {
        list.push_back(x);  // checks if it is an obstacle (1)
      }
    }
  }
  return list;
}

/// @brief Checks if vertex is on OPEN list
/// @param v The vertex of interest
/// @return If it is on OPEN List (true) or not (false)
bool Astar::isOpen(vertex *v) {
  for (auto i : OPEN) {
    if (i->x == v->x && i->y == v->y) return true;
  }
  return false;
}

/// @brief Checks if vertex is on CLOSED list
/// @param v The vertex of interest
/// @return If it is on CLOSED List (true) or not (false)
bool Astar::isClosed(vertex *v) {
  for (auto i : CLOSED) {
    if (i->x == v->x && i->y == v->y) return true;
  }
  return false;
}

/// @brief Checks if vertex is the goal vertex
/// @param v The vertex of interest
/// @return If it is the goal (true) or not (false)
bool Astar::isGoal(vertex *v) { return (v->x == _gx && v->y == _gy); }

/// @brief Checks if vertex is the start vertex
/// @param v The vertex of interest
/// @return If it is the start (true) or not (false)
bool Astar::isStart(vertex *v) { return (v->x == _sx && v->y == _sy); }

/// @brief Calculates the heuristic of a vertex (manhattan distance) to goal
/// @param s The vertex of interest
/// @return Returns the heuristic distance
float Astar::h(vertex *s) { return (abs(s->x - _gx) + abs(s->y - _gy)); }

/// @brief Calculates the g-value of a vertex
/// @param s The vertex of interest
/// @return Returns the g-value
float Astar::g(vertex *s) {
  float g;
  if (this->isStart(s)) return 0;
  g = 0;
  g += s->cost + s->bestparent->g;
  return g;
}

/// @brief Calculates the f-value of a vertex
/// @param s The vertex of interest
/// @return Returns the f-value
float Astar::f(vertex *s) { return s->g + s->h; }

bool Astar::contains(vertex *s, vector<vertex *> list) {
  for (auto i : list) {
    if (i->x == s->x && i->y == s->y) return true;
  }
  return false;
}

/// @brief Finds the vertex with lowest f-cost at OPEN list
/// @return Returns the vertex' pointer
vertex *Astar::lowestF() {
  vertex *v = nullptr;
  float F = INFINITY;
  // float H = INFINITY;
  for (auto &i : OPEN) {
    if (i->f < F) {
      v = i;
      F = i->f;
      // H = i->h;
    } else if (i->f == F) {
      if (i->h < v->h) {
        v = i;
        F = i->f;
        // H = i->h;
      }
    }
  }
  return v;
}

/// @brief Does the expansion of the nodes and searches for the best path
void Astar::computePath() {
  while (true) {
    if (OPEN.empty()) break;
    current = this->lowestF();
    this->erase(current, OPEN);
    this->add(current, CLOSED);

    if (this->isGoal(current)) {
      gotGoal = true;
      this->Path();
      break;
    }

    vector<vertex *> listN = this->getNeighbours(current);
    for (auto &i : listN) {
      if (this->isClosed(i)) {
        continue;
      }

      // looks if new path is shorter
      if ((!this->isOpen(i) && !this->isClosed(i)) ||
          (this->isOpen(i) && i->bestparent->g > current->g)) {
        i->bestparent = current;
        i->h = h(i);
        i->g = g(i);
        i->f = f(i);

        if (!this->isOpen(i)) this->add(i, OPEN);
      }
    }
    last = current;
  }
}

/// @brief Stores the best path into a list
void Astar::Path() {
  vertex *v;
  vector<vertex *> output;
  vector<vertex *> list = CLOSED;
  if (gotGoal == true) {
    for (auto &i : list) {
      if (this->isGoal(i)) {
        v = i;
        break;
      }
    }

    while (v->bestparent != nullptr) {
      output.push_back(v);
      v = v->bestparent;
    }

    if (v->bestparent == nullptr) {
      output.push_back(v);
    }
    path = output;
  }
}

void Astar::setPath(vector<vertex *> list) { path = list; }

/// @brief Gets the best path
/// @returns A vector will all vertices that belongs to the path
vector<vertex *> Astar::getPath() { return path; }

/// @brief Gets all vertices of the maze
/// @returns A vector will all vertices
vector<vertex *> Astar::getVertices() { return points; }

/// @brief Gets the maze structure
/// @returns A vector containing all maze's components
vector<vector<int>> Astar::getMaze() { return maze; }

/// @brief Sets the maze structure like copying from a vector to another
void Astar::setMaze(vector<vector<int>> list) { maze = list; }

/// @brief Dumps vertex data of a list into the terminal
/// @param vList The list that contains vertexes
void Astar::dump(vector<vertex *> vList) {
  if (vList.size() == 0) cout << "No items" << endl;
  for (auto &i : vList) {
    cout << "Addr= " << i << " X=" << i->x << " Y=" << i->y << " F=" << i->f
         << " G=" << i->g << " H=" << i->h << " cost=" << i->cost << endl;
  }
}

/// @brief Starts the OPEN and CLOSED lists
/// and puts the start point into OPEN
void Astar::initStart() {
  OPEN.clear();
  CLOSED.clear();
  path.clear();

  gotGoal = false;
  curr = nullptr;
  next = nullptr;

  for (auto &i : points) {
    if (this->isStart(i)) {
      i->h = this->h(i);
      i->g = 0;
      i->f = this->f(i);
      i->bestparent = nullptr;
      this->add(i, OPEN);
    }
  }
}

/// @brief Check if given vertex is on path list
/// @param v The vertex to be checked
/// @return If the vertex is a path or not
bool Astar::isPath(vertex *v) {
  for (auto &i : path) {
    if (v->x == i->x && v->y == i->y) return true;
  }
  return false;
}

/// @brief Plots the map, as well as the path, in ASCII characters
void Astar::plotmap() {
  // Character set: ═ ║ ╝ ╚ ╗ ╔ █
  cout << "Total cost: ";
  if (!path.empty())
    cout << this->getVertex(_gx, _gy)->f << endl;
  else {
    cout << "NULL" << endl;
    cout << "Not able to find a possible path." << endl;
  }

  cout << "╔";
  for (int i = 0; i < _gridc; i++) cout << "═══";
  cout << "╗" << endl;

  for (int y = _gridl - 1; y >= 0; y--) {
    cout << "║";
    for (int x = 0; x < _gridc; x++) {
      if (this->isObstacle(maze, this->getVertex(x, y))) cout << "███";
      else if (this->isGoal(this->getVertex(x, y))) cout << " G ";
      else if (this->isStart(this->getVertex(x, y))) cout << " S ";
      else if (this->isPath(this->getVertex(x, y))) cout << " x ";
      /* else if (isClosed(getVertex(i, j))) cout << " x ";
      else if (isOpen(getVertex(i, j))) cout << " x "; */
      else
        cout << "   ";
    }
    cout << "║" << endl;
  }

  cout << "╚";
  for (int i = 0; i < _gridc; i++) cout << "═══";
  cout << "╝" << endl;
}

/// @brief Plots the map, as well as the path, in ASCII characters and the current position
/// @param x current X position
/// @param y the current Y position
void Astar::plotmap(int X, int Y) {
  // Character set: ═ ║ ╝ ╚ ╗ ╔ █
  cout << "Total cost: ";
  if (!path.empty())
    cout << this->getVertex(_gx, _gy)->f << endl;
  else {
    cout << "NULL" << endl;
    cout << "Not able to find a possible path." << endl;
  }

  cout << "╔";
  for (int i = 0; i < _gridc; i++) cout << "═══";
  cout << "╗" << endl;

  for (int y = _gridl - 1; y >= 0; y--) {
    cout << "║";
    for (int x = 0; x < _gridc; x++) {
      if (x == X && y == Y) cout << " # ";
      else if (this->isObstacle(maze, this->getVertex(x, y))) cout << "███";
      else if (this->isGoal(this->getVertex(x, y))) cout << " G ";
      else if (this->isStart(this->getVertex(x, y))) cout << " S ";
      else if (this->isPath(this->getVertex(x, y))) cout << " x ";
      /* else if (isClosed(getVertex(i, j))) cout << " x ";
      else if (isOpen(getVertex(i, j))) cout << " x "; */
      else
        cout << "   ";
    }
    cout << "║" << endl;
  }

  cout << "╚";
  for (int i = 0; i < _gridc; i++) cout << "═══";
  cout << "╝" << endl;
}


void Astar::plotparents(vertex *v) {
  cout << "╔";
  for (int i = 0; i < _gridc; i++) cout << "═══";
  cout << "╗" << endl;

  for (int i = 0; i < _gridl; i++) {
    cout << "║";
    for (int j = 0; j < _gridc; j++) {
      if (this->getVertex(i, j) == v)
        cout << " x ";
      else if (this->contains(this->getVertex(i, j), v->parents))
        cout << " o ";
      else
        cout << "   ";
    }
    cout << "║" << endl;
  }

  cout << "╚";
  for (int i = 0; i < _gridc; i++) cout << "═══";
  cout << "╝" << endl;
}

void Astar::begin() {
  maze.clear();
  OPEN.clear();
  CLOSED.clear();
  path.clear();
  points.clear();

  this->setStart(_sx, _sy);
  this->setGoal(_gx, _gy);

  _oldgridc = _gridc;
  _oldgridl = _gridl;
  cout << "MAZE SIZE: " << maze.size() << endl;
  this->initGrid();
  this->initStart();
}

void Astar::recomputePath() {
  this->initStart();
  this->computePath();
}

/// @brief Checks whether the section of path from vertex v to goal stays the same
/// @returns true if it is the same, false otherwise
bool Astar::comparePath(vertex *v) {
  cout << "comparing..." << endl;
  bool same = true;
  vector<vertex *> tempList = this->getPath();
  auto vtemp = find(tempList.begin(), tempList.end(), v);

  this->recomputePath();
  vector<vertex *> newList = this->getPath();
  auto newtemp = find(newList.begin(), newList.end(), v);

  if (vtemp != tempList.end() && newtemp != newList.end()) {
    int tempindex = vtemp - tempList.begin();
    int newindex = newtemp - newList.begin();
    if (tempindex != newindex) {
      same = false;
      // cout << "possible new path" << endl;
    } else {
      // cout << "possibly same path" << endl;
      vector<vertex *> tempsubpath;
      vector<vertex *> newsubpath;
      for (int i = 0; i < tempindex; i++) {
        tempsubpath.push_back(tempList[i]);
        newsubpath.push_back(newList[i]);
      }
      if (tempsubpath != newsubpath)
        same = false;
        // cout << "yeah definitely different" << endl;
      else
        same = true;
        // cout << "yeah definitely equal" << endl;
    }
  } else {
    same = false;
    // cout << "one of them is null so new path" << endl;
  }
  cout << "comparing done" << endl;
  return same;
}

int Astar::getPathSize(){
  return path.size();
}