#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "vectorMath.h"
#include <vector>
#include <unordered_map>
#include <iostream>
#include <emscripten/atomic.h>

//

class OctreeNode;

//

bool containsPoint(const OctreeNode &node, const vm::ivec3 &p);
bool containsNode(const OctreeNode &node, const OctreeNode &other);
bool equalsNode(const OctreeNode &node, const OctreeNode &other);
bool intersectsNode(const OctreeNode &node, const OctreeNode &other);

//

class TrackerTask {
public:
  OctreeNode *maxLodNode;
  std::vector<OctreeNode *> oldNodes;
  std::vector<OctreeNode *> newNodes;

  bool isNop() const;
};

//

class TrackerUpdate {
public:
  std::vector<TrackerTask *> oldTasks;
  std::vector<TrackerTask *> newTasks;

  void *getBuffer();
};

//

extern std::array<vm::ivec3, 8> lodOffsets;

//

OctreeNode *getLeafNodeFromPoint(const std::vector<OctreeNode *> &leafNodes, const vm::ivec3 &p);
OctreeNode *getNode(const std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod);
OctreeNode *createNode(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod, bool isLeaf);
OctreeNode *createNode(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod);
OctreeNode *getOrCreateNode(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod);
void ensureChildren(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, OctreeNode *parentNode);
void constructTreeUpwards(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &leafPosition, int maxLod);

std::vector<OctreeNode *> constructOctreeForLeaf(const vm::ivec3 &position, int lod1Range, int maxLod);
OctreeNode *getMaxLodNode(const std::vector<OctreeNode *> &newLeafNodes, const std::vector<OctreeNode *> &oldLeafNodes, const vm::ivec3 &min);
std::vector<TrackerTask *> diffLeafNodes(const std::vector<OctreeNode *> &newLeafNodes, const std::vector<OctreeNode *> &oldLeafNodes);
std::vector<TrackerTask *> sortTasks(const std::vector<TrackerTask *> &tasks, const vm::vec3 &worldPosition);
std::pair<std::vector<OctreeNode *>, std::vector<TrackerTask *>> updateChunks(const std::vector<OctreeNode *> &oldChunks, const std::vector<TrackerTask *> &tasks);

//

class Tracker {
public:
  int lods;
  int minLodRange;
  bool trackY;
  
  vm::ivec3 lastCoord;
  std::vector<OctreeNode *> chunks;
  std::vector<OctreeNode *> lastOctreeLeafNodes;
  std::vector<TrackerTask *> liveTasks;

  Tracker(int lods, int minLodRange, bool trackY);

  // static methods

  vm::ivec3 getCurrentCoord(const vm::vec3 &position);

  // dynamic methods

  TrackerUpdate updateCoord(const vm::ivec3 &currentCoord);
  TrackerUpdate update(const vm::vec3 &position);
};

#endif // _TRACKER_H_