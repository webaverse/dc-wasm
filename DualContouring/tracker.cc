#include "tracker.h"
#include "octree.h"
#include <iostream>

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

//

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
}

//

Tracker::Tracker(int lods, int minLodRange, bool trackY) :
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
vm::ivec3 Tracker::getCurrentCoord(const vm::vec3 &position) {
  const int cx = std::floor(position.x / (float)chunkSize);
  const int cy = this->trackY ? std::floor(position.y / (float)chunkSize) : 0;
  const int cz = std::floor(position.z / (float)chunkSize);
  return vm::ivec3{cx, cy, cz};
}
// dynamic methods
TrackerUpdate Tracker::updateCoord(const vm::ivec3 &currentCoord) {
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

  this->lastOctreeLeafNodes = std::move(octreeLeafNodes);

  // this.dispatchEvent(new MessageEvent('update'));

  // XXX return old tasks and new tasks
  TrackerUpdate result;
  result.oldTasks = std::move(oldTasks);
  result.newTasks = std::move(tasks);
  return result;
}
TrackerUpdate Tracker::update(const vm::vec3 &position) {
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