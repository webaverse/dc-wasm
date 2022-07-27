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
//   std::array<OctreeNode, numCachedOctreeNodes> rawNodes;
//   std::deque<OctreeNode *> nodes;
//   std::function<void (void *)> deleter;

  // OctreeNodeAllocator();
  static OctreeNodePtr alloc(const vm::ivec3 &min, int lod, bool isLeaf);
};

class OctreeContext {
public:
  std::unordered_map<uint64_t, std::shared_ptr<OctreeNode>> nodeMap;

  OctreeContext();

  OctreeNodePtr alloc(const vm::ivec3 &min, int lod, bool isLeaf) {
    auto node = OctreeNodeAllocator::alloc(min, lod, isLeaf);
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

class DataRequest {
public:
  OctreeNodePtr node;
  
  std::vector<uint8_t> getBuffer() const;
};

typedef std::shared_ptr<DataRequest> DataRequestPtr;

class DataRequestUpdate {
public:
  std::vector<DataRequestPtr> newDataRequests;
  std::vector<DataRequestPtr> cancelDataRequests;
  std::vector<DataRequestPtr> keepDataRequests;

  std::vector<uint8_t> getBuffer() const;
};

//

class Dominator {
public:
  std::vector<OctreeNodePtr> newChunks;
  std::vector<OctreeNodePtr> oldChunks;

  std::vector<uint8_t> getBuffer() const;
};

//

extern std::atomic<int> nextTrackerId;

enum TrackerTaskType {
  ADD = 1,
  REMOVE = 2,
  OUTRANGE = 3
};

class TrackerTask {
public:
  int id;
  TrackerTaskType type;
  OctreeNodePtr maxLodNode;
  std::vector<OctreeNodePtr> oldNodes;
  std::vector<OctreeNodePtr> newNodes;

  bool isNop() const;

  std::vector<uint8_t> getBuffer() const;
};
typedef std::shared_ptr<TrackerTask> TrackerTaskPtr;

class TrackerUpdate {
public:
  std::vector<OctreeNodePtr> leafNodes;

  std::vector<DataRequestPtr> newDataRequests;
  std::vector<DataRequestPtr> keepDataRequests;
  std::vector<DataRequestPtr> oldDataRequests;

  std::unordered_map<uint64_t, Dominator> dominators;

  uint8_t *getBuffer() const;
};

//

bool containsPoint(const OctreeNode &node, const vm::ivec3 &p);
bool containsNode(const OctreeNode &node, const OctreeNode &other);
bool equalsNode(const OctreeNode &node, const OctreeNode &other);
bool equalsNodeLod(const OctreeNode &node, const OctreeNode &other);
// bool intersectsNode(const OctreeNode &node, const OctreeNode &other);

//

extern std::array<vm::ivec3, 8> lodOffsets;
extern OctreeNodeAllocator octreeNodeAllocator;

//

enum ConstructTreeFlags {
  ConstructTreeFlags_None = 0,
  ConstructTreeFlags_Children = 1,
  ConstructTreeFlags_Peers = 2,
};

OctreeNodePtr getLeafNodeFromPoint(const std::vector<OctreeNode *> &leafNodes, const vm::ivec3 &p);
OctreeNodePtr getNode(OctreeContext &octreeContext, const vm::ivec3 &min, int lod);
OctreeNodePtr createNode(OctreeContext &octreeContext, const vm::ivec3 &min, int lod, bool isLeaf);
OctreeNodePtr getOrCreateNode(OctreeContext &octreeContext, const vm::ivec3 &min, int lod, bool isLeaf);
void ensureChildren(OctreeContext &octreeContext, OctreeNode *parentNode);
void ensurePeers(OctreeContext &octreeContext, OctreeNode *node, int maxLod);
void constructTreeUpwards(OctreeContext &octreeContext, const vm::ivec3 &leafPosition, int minLod, int maxLod, int flags);

std::vector<OctreeNodePtr> constructOctreeForLeaf(const vm::ivec3 &position, int lod1Range, int maxLod);
OctreeNodePtr getMaxLodNode(const std::vector<OctreeNodePtr> &newLeafNodes, const std::vector<OctreeNodePtr> &oldLeafNodes, const vm::ivec3 &min);
std::vector<TrackerTaskPtr> diffLeafNodes(const std::vector<OctreeNodePtr> &newLeafNodes, const std::vector<OctreeNodePtr> &oldLeafNodes);
// std::vector<TrackerTaskPtr> sortTasks(const std::vector<TrackerTaskPtr> &tasks, const vm::vec3 &worldPosition);
std::pair<std::vector<OctreeNodePtr>, std::vector<TrackerTaskPtr>> updateChunks(const std::vector<OctreeNodePtr> &oldChunks, const std::vector<TrackerTaskPtr> &tasks);

//

class Tracker {
public:
  int lods;
  int minLodRange;
  bool trackY;
  DCInstance *inst;
  
  vm::ivec3 lastCoord;
  std::vector<OctreeNodePtr> chunks;
  std::unordered_map<uint64_t, DataRequestPtr> dataRequests;

  Tracker(int lods, int minLodRange, bool trackY, DCInstance *inst);

  // static methods

  vm::ivec3 getCurrentCoord(const vm::vec3 &position);

  // dynamic methods

  void sortNodes(std::vector<OctreeNodePtr> &nodes);
  TrackerUpdate updateCoord(const vm::ivec3 &currentCoord);
  TrackerUpdate update(const vm::vec3 &position);
};

#endif // _TRACKER_H_