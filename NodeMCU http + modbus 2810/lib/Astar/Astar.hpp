#include <vector>
#ifndef Astar_hpp
#define Astar_hpp

using namespace std;

typedef enum state { CLEAR = 0, OBSTACLE = 1 } state;

typedef struct point vertex;

typedef struct point {
  int x, y;
  float f, g, h, cost;
  vertex *bestparent;
  vector<vertex *> parents;
  vector<vertex *> children;
} vertex;

class Astar {
 private:
  int _gridc = 10;
  int _gridl = 10;
  float _gx = 0;
  float _gy = 0;
  float _sx = 0;
  float _sy = 0;
  bool gotGoal = false;
  vertex *current = nullptr;
  vertex *last = nullptr;
  vector<vector<int>> maze;
  int _oldgridc = 0;
  int _oldgridl = 0;

  /// @brief OPEN List
  vector<vertex *> OPEN;
  /// @brief CLOSED List
  vector<vertex *> CLOSED;
  /// @brief Points' List
  vector<vertex *> points;
  /// @brief Path list
  vector<vertex *> path;

 public:
  int id;
  vertex *prev = nullptr;
  vertex *curr = nullptr;
  vertex *next = nullptr;
  Astar(int gl, int gc, int sx, int sy, int gx, int gy);
  ~Astar();

  void initGrid();
  void initStart();
  void add(vertex *v, vector<vertex *> &vList);
  void addVertex(vertex *v);
  void begin();
  void computePath();
  void dump(vector<vertex *> vList);
  void erase(vertex *v, vector<vertex *> &vList);
  void setGoal(int x, int y);
  void setStart(int x, int y);
  void updateVertex(vertex *v, state s, vertex *v2 = nullptr);
  void updateVertexCost(vertex *v, float cost);
  void plotmap();
  void plotmap(int X, int Y);
  void recomputePath();
  void Path();
  void setPath(vector<vertex *> list);
  void setMaze(vector<vector<int>> list);
  void plotparents(vertex *v);
  void createMaze();
  void clearSearch();

  vertex *createVertex(int x, int y);
  vertex *getVertex(int X, int Y);
  vertex *lowestF();

  bool isOpen(vertex *v);
  bool isClosed(vertex *v);
  bool isGoal(vertex *v);
  bool isStart(vertex *v);
  bool isObstacle(vector<vector<int>> map, vertex *v);
  bool isPath(vertex *v);
  bool contains(vertex *s, vector<vertex *> list);
  bool comparePath(vertex *v);

  float f(vertex *s);
  float g(vertex *s);
  float h(vertex *s);

  int getPathSize();

  vector<vertex *> getNeighbours(vertex *v);
  vector<vertex *> getPath();
  vector<vertex *> getVertices();
  vector<vector<int>> getMaze();
};

#endif