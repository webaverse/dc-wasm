#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "octree.h"
#include "vectorMath.h"
#include <vector>
#include <unordered_map>
#include <iostream>
#include <functional>
#include <emscripten/atomic.h>

//

class OctreeNode;

//

constexpr size_t numCachedOctreeNodes = 64 * 1024;
typedef std::shared_ptr<OctreeNode> OctreeNodePtr;

class OctreeNodeAllocator {
public:
  std::array<OctreeNode, numCachedOctreeNodes> rawNodes;
  std::deque<OctreeNode *> nodes;
  std::function<void (void *)> deleter;

  OctreeNodeAllocator();
  OctreeNodePtr alloc();
};

class OctreeContext {
public:
  OctreeNodeAllocator &octreeNodeAllocator;
  std::unordered_map<uint64_t, std::shared_ptr<OctreeNode>> nodeMap;

  OctreeContext(OctreeNodeAllocator &octreeNodeAllocator);

  OctreeNodePtr alloc(const vm::ivec3 &min, int lod, bool isLeaf) {
    auto node = this->octreeNodeAllocator.alloc();
    if (node) {
      node->min = min;
      node->size = lod;
      node->type = isLeaf ? Node_Leaf : Node_Internal;
      
      uint64_t hash = hashOctreeMinLod(min, lod);
      nodeMap[hash] = node;
    }
    return node;
  }
  OctreeNodePtr alloc(const vm::ivec3 &min, int lod) {
    return alloc(min, lod, lod == 1);
  }
};

//

class TrackerTask {
public:
  OctreeNodePtr maxLodNode;
  std::vector<OctreeNodePtr> oldNodes;
  std::vector<OctreeNodePtr> newNodes;

  bool isNop() const;
};

class TrackerUpdate {
public:
  std::vector<TrackerTask *> oldTasks;
  std::vector<TrackerTask *> newTasks;

  void *getBuffer();
};

typedef std::shared_ptr<TrackerTask> TrackerTaskPtr;

//

bool containsPoint(const OctreeNode &node, const vm::ivec3 &p);
bool containsNode(const OctreeNode &node, const OctreeNode &other);
bool equalsNode(const OctreeNode &node, const OctreeNode &other);
bool intersectsNode(const OctreeNode &node, const OctreeNode &other);

//

extern std::array<vm::ivec3, 8> lodOffsets;
extern OctreeNodeAllocator octreeNodeAllocator;

//

OctreeNodePtr getLeafNodeFromPoint(const std::vector<OctreeNode *> &leafNodes, const vm::ivec3 &p);
OctreeNodePtr getNode(const std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod);
OctreeNodePtr createNode(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod, bool isLeaf);
OctreeNodePtr createNode(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod);
OctreeNodePtr getOrCreateNode(std::unordered_map<uint64_t, OctreeNodePtr> &nodeMap, const vm::ivec3 &min, int lod);
void ensureChildren(std::unordered_map<uint64_t, OctreeNodePtr> &nodeMap, OctreeNode *parentNode);
void constructTreeUpwards(std::unordered_map<uint64_t, OctreeNodePtr> &nodeMap, const vm::ivec3 &leafPosition, int maxLod);

std::vector<OctreeNodePtr> constructOctreeForLeaf(const vm::ivec3 &position, int lod1Range, int maxLod);
OctreeNodePtr getMaxLodNode(const std::vector<OctreeNodePtr> &newLeafNodes, const std::vector<OctreeNodePtr> &oldLeafNodes, const vm::ivec3 &min);
std::vector<TrackerTask *> diffLeafNodes(const std::vector<OctreeNodePtr> &newLeafNodes, const std::vector<OctreeNodePtr> &oldLeafNodes);
std::vector<TrackerTask *> sortTasks(const std::vector<TrackerTask *> &tasks, const vm::vec3 &worldPosition);
std::pair<std::vector<OctreeNodePtr>, std::vector<TrackerTask *>> updateChunks(const std::vector<OctreeNode *> &oldChunks, const std::vector<TrackerTask *> &tasks);

//

class Tracker {
public:
  int lods;
  int minLodRange;
  bool trackY;
  
  vm::ivec3 lastCoord;
  std::vector<OctreeNodePtr> chunks;
  std::vector<OctreeNodePtr> lastOctreeLeafNodes;
  std::vector<TrackerTask *> liveTasks;

  Tracker(int lods, int minLodRange, bool trackY);

  // static methods

  vm::ivec3 getCurrentCoord(const vm::vec3 &position);

  // dynamic methods

  TrackerUpdate updateCoord(const vm::ivec3 &currentCoord);
  TrackerUpdate update(const vm::vec3 &position);
};

#endif // _TRACKER_H_