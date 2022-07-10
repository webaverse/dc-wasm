#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "octree.h"
#include <iostream>
#include <emscripten/atomic.h>

bool containsPoint(const OctreeNode &node, const vm::ivec3 &p) {
  return p.x >= node.min.x && p.x < node.min.x + node.size &&
    p.y >= node.min.y && p.y < node.min.y + node.size &&
    p.z >= node.min.z && p.z < node.min.z + node.size;
}
bool containsNode(const OctreeNode &node, const OctreeNode &other) {
  return containsPoint(node, other.min);
}
bool equalsNode(const OctreeNode &node, const OctreeNode &other) {
  return node.min == other.min &&
    std::all_of(node.lodArray.begin(), node.lodArray.end(), [&](int lod) -> bool {
      return node.lodArray[lod] == other.lodArray[lod];
    });
}
bool intersectsNode(const OctreeNode &node, const OctreeNode &other) {
  return containsNode(node, other) || containsNode(other, node);
}

//

class TrackerTask {
public:
  OctreeNode *maxLodNode;
  std::vector<OctreeNode *> oldNodes;
  std::vector<OctreeNode *> newNodes;

  bool isNop() const {
    auto task = this;
    return task->newNodes.size() == task->oldNodes.size() &&
      std::all_of(task->newNodes.begin(), task->newNodes.end(), [&](OctreeNode *newNode) -> bool {
        return std::any_of(task->oldNodes.begin(), task->oldNodes.end(), [&](OctreeNode *oldNode) -> bool {
          return equalsNode(*oldNode, *newNode);
        });
      });
  }
};

//

class TrackerUpdate {
public:
  std::vector<TrackerTask *> oldTasks;
  std::vector<TrackerTask *> newTasks;

  void *getBuffer() {
    // XXX serialize to a buffer
    return nullptr;
  }
};

//

std::array<vm::ivec3, 8> lodOffsets = {
    vm::ivec3{0, 0, 0},
    vm::ivec3{1, 0, 0},
    vm::ivec3{0, 0, 1},
    vm::ivec3{1, 0, 1},
    vm::ivec3{0, 1, 0},
    vm::ivec3{1, 1, 0},
    vm::ivec3{0, 1, 1},
    vm::ivec3{1, 1, 1}
};
// const _octreeNodeMinHash = (min, lod) => `${min.x},${min.y},${min.z}:${lod}`;
OctreeNode *getLeafNodeFromPoint(const std::vector<OctreeNode *> &leafNodes, const vm::ivec3 &p) {
    for (size_t i = 0; i < leafNodes.size(); i++) {
        OctreeNode *leafNode = leafNodes[i];
        if (containsPoint(*leafNode, p)) {
            return leafNode;
        }
    }
    return nullptr;
}
OctreeNode *getNode(const std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod) {
    uint64_t hash = hashOctreeMinLod(min, lod);
    auto iter = nodeMap.find(hash);
    if (iter != nodeMap.end()) {
      return iter->second;
    } else {
      return nullptr;
    }
}
OctreeNode *createNode(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod, bool isLeaf) {
    OctreeNode *node = new OctreeNode();
    node->min = min;
    node->size = lod;
    node->type = isLeaf ? Node_Leaf : Node_Internal;
    uint64_t hash = hashOctreeMinLod(min, lod);
    if (nodeMap.find(hash) != nodeMap.end()) {
      // throw new Error(`Node already exists: ${hash}`);
      EM_ASM({
        console.log('Node already exists:', $0);
      }, hash);
      abort();
    }
    nodeMap[hash] = node;
    return node;
}
OctreeNode *createNode(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod) {
    return createNode(nodeMap, min, lod, lod == 1);
}
OctreeNode *getOrCreateNode(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &min, int lod) {
    OctreeNode *node = getNode(nodeMap, min, lod);
    if (!node) {
        node = createNode(nodeMap, min, lod);
    }
    return node;
}
void ensureChildren(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, OctreeNode *parentNode) {
    const vm::ivec3 &lodMin = parentNode->min;
    const int &lod = parentNode->size;

    for (int dx = 0; dx < 2; dx++) {
      for (int dy = 0; dy < 2; dy++) {
        for (int dz = 0; dz < 2; dz++) {
           int childIndex = dx + 2 * (dy + 2 * dz);
           if (parentNode->children[childIndex] == nullptr) {
              parentNode->children[childIndex] = createNode(
                nodeMap,
                lodMin + vm::ivec3{dx, dy, dz} * (lod / 2),
                lod / 2,
                true
              );
            //   parentNode->children[childIndex].parent = parentNode;
           }
        }
      }
    }
}
void constructTreeUpwards(std::unordered_map<uint64_t, OctreeNode *> &nodeMap, const vm::ivec3 &leafPosition, int maxLod) {
    OctreeNode *rootNode = getOrCreateNode(nodeMap, leafPosition, 1);
    for (int lod = 2; lod <= maxLod; lod *= 2) {
      vm::ivec3 lodMin = rootNode->min;
      lodMin.x = (lodMin.x / lod) * lod;
      lodMin.y = (lodMin.y / lod) * lod;
      lodMin.z = (lodMin.z / lod) * lod;

      const vm::ivec3 &lodCenter = lodMin * (lod / 2);
      const int childIndex = (rootNode->min.x < lodCenter.x ? 0 : 1) +
        (rootNode->min.y < lodCenter.y ? 0 : 2) +
        (rootNode->min.z < lodCenter.z ? 0 : 4);

      OctreeNode *parentNode = getOrCreateNode(nodeMap, lodMin, lod);
      // parentNode.isLeaf = false;
      parentNode->type = Node_Internal;
      if (parentNode->children[childIndex] == nullptr) { // children not set yet
        parentNode->children[childIndex] = rootNode;
        ensureChildren(nodeMap, parentNode);
      }
      rootNode = parentNode;
    }
    // return rootNode;
}

std::vector<OctreeNode *> constructOctreeForLeaf(const vm::ivec3 &position, int lod1Range, int maxLod) {
  std::unordered_map<uint64_t, OctreeNode *> nodeMap;

  // sample base leaf nodes to generate octree upwards
  const vm::ivec3 &rangeMin = position - vm::ivec3{lod1Range, lod1Range, lod1Range};
  const vm::ivec3 &rangeMax = position + vm::ivec3{lod1Range, lod1Range, lod1Range};
  for (int dx = rangeMin.x; dx <= rangeMax.x; dx++) {
    for (int dy = rangeMin.y; dy <= rangeMax.y; dy++) {
      for (int dz = rangeMin.z; dz <= rangeMax.z; dz++) {
        vm::ivec3 leafPosition = vm::ivec3{dx, dy, dz};
        // leafPosition.x = Math.floor(leafPosition.x);
        // leafPosition.y = Math.floor(leafPosition.y);
        // leafPosition.z = Math.floor(leafPosition.z);
        constructTreeUpwards(nodeMap, leafPosition, maxLod);
      }
    }
  }

  std::vector<OctreeNode *> rootNodes;
  for (const auto &iter : nodeMap) {
    OctreeNode *node = iter.second;
    if (node->size == maxLod) {
      rootNodes.push_back(node);
    }
  }

  std::vector<OctreeNode *> lod1Nodes;
  for (const auto &iter : nodeMap) {
    OctreeNode *node = iter.second;
    if (node->size == 1) {
      lod1Nodes.push_back(node);
    }
  }

  // sanity check lod1Nodes for duplicates
  {
    std::unordered_map<uint64_t, OctreeNode *> lod1NodeMap;
    for (OctreeNode *node : lod1Nodes) {
      uint64_t hash = hashOctreeMinLod(node->min, node->size);
      if (lod1NodeMap.find(hash) != lod1NodeMap.end()) {
        // throw new Error(`Duplicate lod1 node: ${hash}`);
        EM_ASM({
            console.log('Duplicate lod1 node:', $0);
        }, hash);
        abort();
      }
      lod1NodeMap[hash] = node;
    }
  }

  std::vector<OctreeNode *> leafNodes;
  for (const auto &iter : nodeMap) {
    OctreeNode *node = iter.second;
    if (node->type == Node_Leaf) {
      leafNodes.push_back(node);
    }
  }

  // sanity check that no leaf node contains another leaf node
  for (OctreeNode *leafNode : leafNodes) {
    for (OctreeNode *childNode : leafNode->children) {
      if (childNode != nullptr && childNode->type == Node_Leaf) {
        // throw new Error(`Leaf node contains another leaf node 1: ${leafNode.min.toArray().join(',')}`);
        EM_ASM({
            console.log('Leaf node contains another leaf node 1:', $0, $1, $2);
        }, leafNode->min.x, leafNode->min.y, leafNode->min.z);
      }
    }
    for (OctreeNode *leafNode2 : leafNodes) {
      if (leafNode != leafNode2 && containsNode(*leafNode, *leafNode2)) {
        // throw new Error(`Leaf node contains another leaf node 2: ${leafNode.min.toArray().join(',')}`);
        EM_ASM({
            console.log('Leaf node contains another leaf node 2:', $0, $1, $2);
        }, leafNode->min.x, leafNode->min.y, leafNode->min.z);
      }
    }
  }

  // assign lodArray for each node based on the minimum lod of the target point in the world
  // vm::ivec3(0, 0, 0),
  // vm::ivec3(1, 0, 0),
  // vm::ivec3(0, 0, 1),
  // vm::ivec3(1, 0, 1),
  // vm::ivec3(0, 1, 0),
  // vm::ivec3(1, 1, 0),
  // vm::ivec3(0, 1, 1),
  // vm::ivec3(1, 1, 1),
  for (const auto &iter : nodeMap) {
    OctreeNode *node = iter.second;

    std::array<int, 8> lodArray;
    for (int i = 0; i < 8; i++) {
      const vm::ivec3 &offset = lodOffsets[i];
      int &lod = lodArray[i]; // output
      
      const vm::ivec3 &p = node->min + offset * node->size;
      OctreeNode *containingLeafNode = getLeafNodeFromPoint(leafNodes, p);
      if (containingLeafNode) {
        lod = containingLeafNode->size;
      } else {
        lod = node->size;
      }
    }
  }

  return leafNodes;
  /* return {
    rootNodes,
    lod1Nodes,
    leafNodes,
    remapNodes(nodes) {
      for (let i = 0; i < nodes.length; i++) {
        const node = nodes[i];
        const hash = _octreeNodeMinHash(node.min, node.lod);
        const otherNode = nodeMap.get(hash);
        if (otherNode) {
          nodes[i] = otherNode;
        }
      }
    },
    getOutrangedNodes(nodes) {
      const remainderNodes = [];
      for (let i = 0; i < nodes.length; i++) {
        const node = nodes[i];
        const hash = _octreeNodeMinHash(node.min, node.lod);
        const otherNode = nodeMap.get(hash);
        if (!otherNode) {
          remainderNodes.push(node);
        }
      }
      return remainderNodes;
    },
  }; */
}
OctreeNode *getMaxLodNode(const std::vector<OctreeNode *> &newLeafNodes, const std::vector<OctreeNode *> &oldLeafNodes, const vm::ivec3 &min) {
    OctreeNode *newLeafNode = getLeafNodeFromPoint(newLeafNodes, min);
    OctreeNode *oldLeafNode = getLeafNodeFromPoint(oldLeafNodes, min);
    if (newLeafNode != nullptr && oldLeafNode != nullptr) {
      return newLeafNode->size > oldLeafNode->size ? newLeafNode : oldLeafNode;
    } else if (newLeafNode != nullptr) {
      return newLeafNode;
    } else if (oldLeafNode != nullptr) {
      return oldLeafNode;
    } else {
      return nullptr;
    }
}
std::vector<TrackerTask *> diffLeafNodes(const std::vector<OctreeNode *> &newLeafNodes, const std::vector<OctreeNode *> &oldLeafNodes) {
  // map from min lod hash to task containing new nodes and old nodes
  std::unordered_map<uint64_t, TrackerTask *> taskMap;

  for (OctreeNode *newNode : newLeafNodes) {
    OctreeNode *maxLodNode = getMaxLodNode(newLeafNodes, oldLeafNodes, newNode->min);
    const uint64_t hash = hashOctreeMinLod(maxLodNode->min, maxLodNode->size);
    
    TrackerTask *task;
    const auto &iter = taskMap.find(hash);
    if (iter != taskMap.end()) {
      task = iter->second;
    } else {
      task = new TrackerTask();
      task->maxLodNode = maxLodNode;
      taskMap[hash] = task;
    }
    task->newNodes.push_back(newNode);
  }
  for (OctreeNode *oldNode : oldLeafNodes) {
    OctreeNode *maxLodNode = getMaxLodNode(newLeafNodes, oldLeafNodes, oldNode->min);
    const uint64_t hash = hashOctreeMinLod(maxLodNode->min, maxLodNode->size);
    
    TrackerTask *task;
    const auto &iter = taskMap.find(hash);
    if (iter != taskMap.end()) {
      task = iter->second;
    } else {
      task = new TrackerTask();
      task->maxLodNode = maxLodNode;
      taskMap[hash] = task;
    }
    task->oldNodes.push_back(oldNode);
  }

  std::vector<TrackerTask *> tasks;
  for (const auto &iter : taskMap) {
    tasks.push_back(iter.second);
  }
  return tasks;
}
// sort tasks by distance to world position of the central max lod node
std::vector<TrackerTask *> sortTasks(const std::vector<TrackerTask *> &tasks, const vm::vec3 &worldPosition) {
  std::vector<std::pair<TrackerTask *, float>> taskDistances;
  taskDistances.reserve(tasks.size());

  for (TrackerTask *task : tasks) {
    const vm::ivec3 &min = task->maxLodNode->min;
    const int &lod = task->maxLodNode->size;

    vm::vec3 center = vm::vec3{(float)min.x, (float)min.y, (float)min.z} +
      vm::vec3{0.5, 0.5, 0.5} * (float)lod;
    vm::vec3 delta = worldPosition - center;
    float distance = vm::lengthSq(delta);

    taskDistances.push_back(std::make_pair(task, distance));
  }

  std::sort(
    taskDistances.begin(),
    taskDistances.end(),
    [](const std::pair<TrackerTask *, float> &a, const std::pair<TrackerTask *, float> &b) {
      return a.second < b.second;
    }
  );

  std::vector<TrackerTask *> sortedTasks;
  for (const auto &iter : taskDistances) {
    sortedTasks.push_back(iter.first);
  }
  return sortedTasks;
}
std::pair<std::vector<OctreeNode *>, std::vector<TrackerTask *>> updateChunks(const std::vector<OctreeNode *> &oldChunks, const std::vector<TrackerTask *> &tasks) {
  std::vector<OctreeNode *> newChunks = oldChunks;
  
  for (TrackerTask *task : tasks) {
    if (!task->isNop()) {
      const std::vector<OctreeNode *> &newNodes = task->newNodes;
      const std::vector<OctreeNode *> &oldNodes = task->oldNodes;

      for (OctreeNode *oldNode : oldNodes) {
        const auto &iter = std::find_if(
            newChunks.begin(),
            newChunks.end(),
            [&](OctreeNode *chunk) -> bool {
                return equalsNode(*chunk, *oldNode);
            }
        );
        if (iter != newChunks.end()) {
          // newChunks.splice(index, 1);
          newChunks.erase(iter);
        } else {
          // debugger;
          abort();
        }
      }
      for (OctreeNode *newNode : newNodes) {
        newChunks.push_back(newNode);
      }
    }
  }

  std::vector<OctreeNode *> removedChunks;
  for (OctreeNode *oldChunk : oldChunks) {
    if (std::any_of(newChunks.begin(), newChunks.end(), [&](OctreeNode *newChunk) {
      return newChunk->min == oldChunk->min;
    })) {
      removedChunks.push_back(oldChunk);
    }
  }
  std::vector<TrackerTask *> extraTasks;
  for (OctreeNode *chunk : removedChunks) {
    TrackerTask *task = new TrackerTask();
    task->maxLodNode = chunk;
    task->oldNodes.push_back(chunk);
    extraTasks.push_back(task);
  }

  return std::make_pair(std::move(newChunks), std::move(extraTasks));
  /* return {
    chunks: newChunks,
    extraTasks,
  }; */
}

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

  Tracker(int lods, int minLodRange, bool trackY) :
    lods(lods),
    minLodRange(minLodRange),
    trackY(trackY),
    lastCoord{
      INT32_MAX,
      INT32_MAX,
      INT32_MAX
    }
  {}

  // static methods

  vm::ivec3 getCurrentCoord(const vm::vec3 &position) {
    const int cx = std::floor(position.x / (float)chunkSize);
    const int cy = this->trackY ? std::floor(position.y / (float)chunkSize) : 0;
    const int cz = std::floor(position.z / (float)chunkSize);
    return vm::ivec3{cx, cy, cz};
  }

  // dynamic methods

  TrackerUpdate updateCoord(const vm::ivec3 &currentCoord) {
    std::vector<OctreeNode *> octreeLeafNodes = constructOctreeForLeaf(currentCoord, this->minLodRange, 1 << (this->lods - 1));

    std::vector<TrackerTask *> tasks = diffLeafNodes(
      octreeLeafNodes,
      this->lastOctreeLeafNodes
    );

    {
      std::pair<std::vector<OctreeNode *>, std::vector<TrackerTask *>> chunksUpdate = updateChunks(this->chunks, tasks);
      std::vector<OctreeNode *> &newChunks = chunksUpdate.first;
      std::vector<TrackerTask *> &newTasks = chunksUpdate.second;
      this->chunks = std::move(newChunks);
      for (TrackerTask *task : newTasks) {
        tasks.push_back(task);
      }
    }

    vm::vec3 worldPosition = vm::vec3{
        (float)currentCoord.x,
        (float)currentCoord.y,
        (float)currentCoord.z
    } + vm::vec3{0.5, 0.5, 0.5} * ((float)chunkSize / 2.0f);
    tasks = sortTasks(tasks, worldPosition);

    std::vector<TrackerTask *> oldTasks;
    for (size_t i = 0; i < tasks.size(); i++) {
      TrackerTask *task = tasks[i];
      if (!task->isNop()) {
        std::vector<TrackerTask *> overlappingTasks;
        for (TrackerTask *lastTask : this->liveTasks) {
            if (containsNode(*task->maxLodNode, *lastTask->maxLodNode)) {
              overlappingTasks.push_back(task);
            }
        }
        
        for (TrackerTask *oldTask : overlappingTasks) {
          oldTasks.push_back(oldTask);
          
          const auto &iter = std::find(
            this->liveTasks.begin(),
            this->liveTasks.end(),
            oldTask
          );
          this->liveTasks.erase(iter);
        }
        this->liveTasks.push_back(task);
      }
    }

    /* for (const task of tasks) {
      if (!task.isNop()) {
        this.dispatchEvent(new MessageEvent('chunkrelod', {
          data: {
            task,
          },
        }));
      }
    } */

    lastOctreeLeafNodes = std::move(octreeLeafNodes);

    // this.dispatchEvent(new MessageEvent('update'));

    // XXX return old tasks and new tasks
    TrackerUpdate result;
    result.oldTasks = std::move(oldTasks);
    result.newTasks = std::move(tasks);
    return result;
  }
  TrackerUpdate update(const vm::vec3 &position) {
    const vm::ivec3 &currentCoord = getCurrentCoord(position);

    // if we moved across a chunk boundary, update needed chunks
    if (currentCoord != this->lastCoord) {
      TrackerUpdate trackerUpdate = updateCoord(currentCoord);
      this->lastCoord = currentCoord;
      return trackerUpdate;
    } else {
      return TrackerUpdate();
    }
  }
};

//

/* class OctreeNode {
  constructor(min, lod, isLeaf) {
    // super();
    
    this.min = min;
    this.lod = lod;
    this.isLeaf = isLeaf;
    this.lodArray = Array(8).fill(-1);

    this.children = Array(8).fill(null);
  }
  equalsNode(p) {
    return p.min.x === this.min.x && p.min.y === this.min.y && p.min.z === this.min.z &&
      p.lodArray.every((lod, i) => lod === this.lodArray[i]);
  }
  intersectsNode(p) {
    return this.containsNode(p) || p.containsNode(this);
  }
} */

/* const _octreeNodeMinHash = (min, lod) => `${min.x},${min.y},${min.z}:${lod}`;
const _getLeafNodeFromPoint = (leafNodes, p) => leafNodes.find(node => node.containsPoint(p));
const constructOctreeForLeaf = (position, lod1Range, maxLod) => {
  const nodeMap = new Map();
  
  const _getNode = (min, lod) => {
    const hash = _octreeNodeMinHash(min, lod);
    return nodeMap.get(hash);
  };
  const _createNode = (min, lod, isLeaf = lod === 1) => {
    const node = new OctreeNode(min, lod, isLeaf);
    const hash = _octreeNodeMinHash(min, lod);
    if (nodeMap.has(hash)) {
      throw new Error(`Node already exists: ${hash}`);
    }
    nodeMap.set(hash, node);
    return node;
  };
  const _getOrCreateNode = (min, lod) => _getNode(min, lod) ?? _createNode(min, lod);
  const _ensureChildren = parentNode => {
    const lodMin = parentNode.min;
    const lod = parentNode.lod;
    for (let dx = 0; dx < 2; dx++) {
      for (let dy = 0; dy < 2; dy++) {
        for (let dz = 0; dz < 2; dz++) {
           const childIndex = dx + 2 * (dy + 2 * dz);
           if (parentNode.children[childIndex] === null) {
              parentNode.children[childIndex] = _createNode(
                lodMin.clone().add(
                  new THREE.Vector3(dx, dy, dz).multiplyScalar(lod / 2)
                ),
                lod / 2,
                true
              );
              parentNode.children[childIndex].parent = parentNode;
           }
        }
      }
    }
  };
  const _constructTreeUpwards = leafPosition => {
    let rootNode = _getOrCreateNode(leafPosition, 1);
    for (let lod = 2; lod <= maxLod; lod *= 2) {
      const lodMin = rootNode.min.clone();
      lodMin.x = Math.floor(lodMin.x / lod) * lod;
      lodMin.y = Math.floor(lodMin.y / lod) * lod;
      lodMin.z = Math.floor(lodMin.z / lod) * lod;

      const lodCenter = lodMin.clone().addScalar(lod / 2);
      const childIndex = (rootNode.min.x < lodCenter.x ? 0 : 1) +
        (rootNode.min.y < lodCenter.y ? 0 : 2) +
        (rootNode.min.z < lodCenter.z ? 0 : 4);

      const parentNode = _getOrCreateNode(lodMin, lod);
      parentNode.isLeaf = false;
      if (parentNode.children[childIndex] === null) { // children not set yet
        parentNode.children[childIndex] = rootNode;
        _ensureChildren(parentNode);
      }
      rootNode = parentNode;
    }
    return rootNode;
  };

  // sample base leaf nodes to generate octree upwards
  const rangeMin = position.clone()
    .sub(new THREE.Vector3(lod1Range, lod1Range, lod1Range));
  const rangeMax = position.clone()
    .add(new THREE.Vector3(lod1Range, lod1Range, lod1Range));
  for (let dx = rangeMin.x; dx <= rangeMax.x; dx++) {
    for (let dy = rangeMin.y; dy <= rangeMax.y; dy++) {
      for (let dz = rangeMin.z; dz <= rangeMax.z; dz++) {
        const leafPosition = new THREE.Vector3(dx, dy, dz);
        leafPosition.x = Math.floor(leafPosition.x);
        leafPosition.y = Math.floor(leafPosition.y);
        leafPosition.z = Math.floor(leafPosition.z);
        _constructTreeUpwards(leafPosition);
      }
    }
  }

  const rootNodes = [];
  for (const node of nodeMap.values()) {
    if (node.lod === maxLod) {
      rootNodes.push(node);
    }
  }

  const lod1Nodes = [];
  for (const node of nodeMap.values()) {
    if (node.lod === 1) {
      lod1Nodes.push(node);
    }
  }

  // sanity check lod1Nodes for duplicates
  {
    const lod1NodeMap = new Map();
    for (const node of lod1Nodes) {
      const hash = _octreeNodeMinHash(node.min, node.lod);
      if (lod1NodeMap.has(hash)) {
        throw new Error(`Duplicate lod1 node: ${hash}`);
      }
      lod1NodeMap.set(hash, node);
    }
  }

  const leafNodes = [];
  for (const node of nodeMap.values()) {
    if (node.isLeaf) {
      leafNodes.push(node);
    }
  }

  // sanity check that no leaf node contains another leaf node
  for (const leafNode of leafNodes) {
    for (const childNode of leafNode.children) {
      if (childNode?.isLeaf) {
        throw new Error(`Leaf node contains another leaf node 1: ${leafNode.min.toArray().join(',')}`);
      }
    }
    for (const leafNode2 of leafNodes) {
      if (leafNode !== leafNode2 && leafNode.containsNode(leafNode2)) {
        throw new Error(`Leaf node contains another leaf node 2: ${leafNode.min.toArray().join(',')}`);
      }
    }
  }

  // assign lodArray for each node based on the minimum lod of the target point in the world
  // vm::ivec3(0, 0, 0),
  // vm::ivec3(1, 0, 0),
  // vm::ivec3(0, 0, 1),
  // vm::ivec3(1, 0, 1),
  // vm::ivec3(0, 1, 0),
  // vm::ivec3(1, 1, 0),
  // vm::ivec3(0, 1, 1),
  // vm::ivec3(1, 1, 1),
  for (const node of nodeMap.values()) {
    node.lodArray = [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 1),
      new THREE.Vector3(1, 0, 1),
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(1, 1, 0),
      new THREE.Vector3(0, 1, 1),
      new THREE.Vector3(1, 1, 1),
    ].map(offset => {
      const containingLeafNode = _getLeafNodeFromPoint(leafNodes, node.min.clone().add(offset.clone().multiplyScalar(node.lod)));
      if (containingLeafNode) {
        return containingLeafNode.lod;
      } else {
        return node.lod;
      }
    });
  }

  return {
    rootNodes,
    lod1Nodes,
    leafNodes,
    remapNodes(nodes) {
      for (let i = 0; i < nodes.length; i++) {
        const node = nodes[i];
        const hash = _octreeNodeMinHash(node.min, node.lod);
        const otherNode = nodeMap.get(hash);
        if (otherNode) {
          nodes[i] = otherNode;
        }
      }
    },
    getOutrangedNodes(nodes) {
      const remainderNodes = [];
      for (let i = 0; i < nodes.length; i++) {
        const node = nodes[i];
        const hash = _octreeNodeMinHash(node.min, node.lod);
        const otherNode = nodeMap.get(hash);
        if (!otherNode) {
          remainderNodes.push(node);
        }
      }
      return remainderNodes;
    },
  };
};
class Task {
  constructor(maxLodNode) {
    super();

    this.maxLodNode = maxLodNode;
    
    this.newNodes = [];
    this.oldNodes = [];

    this.abortController = new AbortController();
    this.signal = this.abortController.signal;
  }
  equals(t) {
    return this.newNodes.length === this.oldNodes.length && this.newNodes.every(node => {
      return t.newNodes.some(node2 => node.equalsNode(node2));
    }) && this.oldNodes.every(node => {
      return t.oldNodes.some(node2 => node.equalsNode(node2));
    });
  }
  cancel() {
    this.abortController.abort(abortError);
  }
  isNop() {
    const task = this;
    return task.newNodes.length === task.oldNodes.length && task.newNodes.every(newNode => {
      return task.oldNodes.some(oldNode => oldNode.equalsNode(newNode));
    });
  }
  commit() {
    this.dispatchEvent(new MessageEvent('finish'));
  }
  waitForLoad() {
    const p = makePromise();
    this.addEventListener('finish', () => {
      p.accept();
    }, {
      once: true,
    });
    return p;
  }
}
const diffLeafNodes = (newLeafNodes, oldLeafNodes) => {
  // map from min lod hash to task containing new nodes and old nodes
  const taskMap = new Map();

  const _getMaxLodNode = min => {
    const newLeafNode = _getLeafNodeFromPoint(newLeafNodes, min);
    const oldLeafNode = _getLeafNodeFromPoint(oldLeafNodes, min);
    if (newLeafNode && oldLeafNode) {
      return newLeafNode.lod > oldLeafNode.lod ? newLeafNode : oldLeafNode;
    } else if (newLeafNode) {
      return newLeafNode;
    } else {
      return oldLeafNode;
    }
  };
  for (const newNode of newLeafNodes) {
    const maxLodNode = _getMaxLodNode(newNode.min);
    const hash = _octreeNodeMinHash(maxLodNode.min, maxLodNode.lod);
    let task = taskMap.get(hash);
    if (!task) {
      task = new Task(maxLodNode);
      taskMap.set(hash, task);
    }
    task.newNodes.push(newNode);
  }
  for (const oldNode of oldLeafNodes) {
    const maxLodNode = _getMaxLodNode(oldNode.min);
    const hash = _octreeNodeMinHash(maxLodNode.min, maxLodNode.lod);
    let task = taskMap.get(hash);
    if (!task) {
      task = new Task(maxLodNode);
      taskMap.set(hash, task);
    }
    task.oldNodes.push(oldNode);
  }

  let tasks = Array.from(taskMap.values());
  return tasks;
};
// sort tasks by distance to world position of the central max lod node
const sortTasks = (tasks, worldPosition) => {
  const taskDistances = tasks.map(task => {
    const distance = localVector2.copy(task.maxLodNode.min)
      .add(localVector3.setScalar(task.maxLodNode.lod / 2))
      .distanceToSquared(worldPosition);
    return {
      task,
      distance,
    };
  });
  taskDistances.sort((a, b) => {
    return a.distance - b.distance;
  });
  return taskDistances.map(taskDistance => taskDistance.task);
};
const updateChunks = (oldChunks, tasks) => {
  const newChunks = oldChunks.slice();
  
  for (const task of tasks) {
    if (!task.isNop()) {
      let {newNodes, oldNodes} = task;
      for (const oldNode of oldNodes) {
        const index = newChunks.findIndex(chunk => chunk.equalsNode(oldNode));
        if (index !== -1) {
          newChunks.splice(index, 1);
        } else {
          debugger;
        }
      }
      newChunks.push(...newNodes);
    }
  }

  const removedChunks = [];
  for (const oldChunk of oldChunks) {
    if (!newChunks.some(newChunk => newChunk.min.equals(oldChunk.min))) {
      removedChunks.push(oldChunk);
    }
  }
  const extraTasks = removedChunks.map(chunk => {
    const task = new Task(chunk);
    task.oldNodes.push(chunk);
    return task;
  });

  return {
    chunks: newChunks,
    extraTasks,
  }
};

export class LodChunk extends THREE.Vector3 {
  constructor(x, y, z, lod, lodArray) {
    
    super(x, y, z);
    this.lod = lod;
    this.lodArray = lodArray;

    this.name = `chunk:${this.x}:${this.y}:${this.z}`;
    this.binding = null;
    this.items = [];
    this.physicsObjects = [];
  }
  lodEquals(chunk) {
    return this.lod === chunk.lod &&
      this.lodArray.length === chunk.lodArray.length && this.lodArray.every((lod, i) => lod === chunk.lodArray[i]);
  }
  containsPoint(p) {
    return p.x >= this.x && p.x < this.x + this.lod &&
      p.y >= this.y && p.y < this.y + this.lod &&
      p.z >= this.z && p.z < this.z + this.lod;
  }
}
export class LodChunkTracker extends EventTarget {
  constructor({
    chunkSize = defaultChunkSize,
    lods = 1,
    minLodRange = 2,
    trackY = false,
    debug = false,
  } = {}) {
    super();

    this.chunkSize = chunkSize;
    this.lods = lods;
    this.minLodRange = minLodRange;
    this.trackY = trackY;

    this.chunks = [];
    this.lastUpdateCoord = new THREE.Vector3(NaN, NaN, NaN);

    if (debug) {
      const maxChunks = 512;
      const instancedCubeGeometry = new THREE.InstancedBufferGeometry();
      {
        const cubeGeometry = new THREE.BoxBufferGeometry(1, 1, 1)
          .translate(0.5, 0.5, 0.5);
        for (const k in cubeGeometry.attributes) {
          instancedCubeGeometry.setAttribute(k, cubeGeometry.attributes[k]);
        }
        instancedCubeGeometry.setIndex(cubeGeometry.index);
      }
      const redMaterial = new THREE.MeshBasicMaterial({
        color: 0xFFFFFF,
        // transparent: true,
        // opacity: 0.1,
        wireframe: true,
      });
      const debugMesh = new THREE.InstancedMesh(instancedCubeGeometry, redMaterial, maxChunks);
      debugMesh.count = 0;
      debugMesh.frustumCulled = false;
      this.debugMesh = debugMesh;

      {
        const localVector = new THREE.Vector3();
        const localVector2 = new THREE.Vector3();
        const localVector3 = new THREE.Vector3();
        const localQuaternion = new THREE.Quaternion();
        const localMatrix = new THREE.Matrix4();
        const localColor = new THREE.Color();

        const _getChunkColorHex = chunk => {
          if (chunk.lod === 1) {
            return 0xFF0000;
          } else if (chunk.lod === 2) {
            return 0x00FF00;
          } else if (chunk.lod === 4) {
            return 0x0000FF;
          } else {
            return 0x0;
          }
        };
        const _flushChunks = () => {
          debugMesh.count = 0;
          for (let i = 0; i < this.chunks.length; i++) {
            const chunk = this.chunks[i];
            localMatrix.compose(
              localVector.copy(chunk.min)
                .multiplyScalar(this.chunkSize),
                // .add(localVector2.set(0, -60, 0)),
              localQuaternion.identity(),
              localVector3.set(1, 1, 1)
                .multiplyScalar(chunk.lod * this.chunkSize * 0.9)
            );
            localColor.setHex(_getChunkColorHex(chunk));
            debugMesh.setMatrixAt(debugMesh.count, localMatrix);
            debugMesh.setColorAt(debugMesh.count, localColor);
            debugMesh.count++;
          }
          debugMesh.instanceMatrix.needsUpdate = true;
          debugMesh.instanceColor && (debugMesh.instanceColor.needsUpdate = true);
        };
        this.addEventListener('update', e => {
          _flushChunks();
        });
      }
    }

    this.lastOctree = {
      leafNodes: [],
    };
    this.liveTasks = [];
  }
  #getCurrentCoord(position, target) {
    const cx = Math.floor(position.x / this.chunkSize);
    const cy = this.trackY ? Math.floor(position.y / this.chunkSize) : 0;
    const cz = Math.floor(position.z / this.chunkSize);
    return target.set(cx, cy, cz);
  }
  emitChunkDestroy(chunk) {
    const hash = chunk.min.toArray().join(','); // _octreeNodeMinHash(chunk.min, chunk.lod);
    this.dispatchEvent(new MessageEvent('destroy.' + hash));
  }
  listenForChunkDestroy(chunk, fn) {
    const hash = chunk.min.toArray().join(','); // _octreeNodeMinHash(chunk.min, chunk.lod);
    this.addEventListener('destroy.' + hash, e => {
      fn(e);
    }, {once: true});
  }
  updateCoord(currentCoord) {
    const octree = constructOctreeForLeaf(currentCoord, this.minLodRange, 2 ** (this.lods - 1));

    let tasks = diffLeafNodes(
      octree.leafNodes,
      this.lastOctree.leafNodes,
    );

    const {
      chunks,
      extraTasks,
    } = updateChunks(this.chunks, tasks);
    this.chunks = chunks;
    tasks.push(...extraTasks);

    sortTasks(tasks, camera.position);

    for (size_t i = 0; i < tasks.size(); i++) {
      const task = tasks[i];
      if (!task.isNop()) {
        const overlappingTasks = this.liveTasks.filter(lastTask => task.maxLodNode.containsNode(lastTask.maxLodNode));
        for (const oldTask of overlappingTasks) {
          oldTask.cancel();
          this.liveTasks.splice(this.liveTasks.indexOf(oldTask), 1);
        }
        this.liveTasks.push(task);
      }
    }

    for (const task of tasks) {
      if (!task.isNop()) {
        this.dispatchEvent(new MessageEvent('chunkrelod', {
          data: {
            task,
          },
        }));
      }
    }

    this.lastOctree = octree;

    this.dispatchEvent(new MessageEvent('update'));
  }
  async waitForLoad() {
    // console.log('wait for live tasks 1', this.liveTasks.length);
    await Promise.all(this.liveTasks.map(task => task.waitForLoad()));
    // console.log('wait for live tasks 2', this.liveTasks.length);
  }
  update(position) {
    const currentCoord = this.#getCurrentCoord(position, localVector).clone();

    // if we moved across a chunk boundary, update needed chunks
    if (!currentCoord.equals(this.lastUpdateCoord)) {
      this.updateCoord(currentCoord);
      this.lastUpdateCoord.copy(currentCoord);
    }
  }
  destroy() {
    for (const chunk of this.chunks) {
      const task = new Task(chunk);
      task.oldNodes.push(chunk);
      
      this.dispatchEvent(new MessageEvent('chunkrelod', {
        data: {
          task,
        },
      }));
    }
    this.chunks.length = 0;
  }
} */

#endif // _TRACKER_H_