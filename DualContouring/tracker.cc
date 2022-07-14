#include "tracker.h"
#include "octree.h"
#include <iostream>

/*
tracker = await dcWorkerManager.createTracker(3, 2, true); trackerUpdate = await dcWorkerManager.trackerUpdate(tracker, new THREE.Vector3(1, 2, 3));
*/

//

std::atomic<int> nextTrackerId(0);

//

bool TrackerTask::isNop() const {
  auto task = this;
  return task->newNodes.size() == task->oldNodes.size() &&
    std::all_of(task->newNodes.begin(), task->newNodes.end(), [&](OctreeNodePtr newNode) -> bool {
      return std::any_of(task->oldNodes.begin(), task->oldNodes.end(), [&](OctreeNodePtr oldNode) -> bool {
        return equalsNode(*oldNode, *newNode);
      });
    });
}
std::vector<uint8_t> TrackerTask::getBuffer() const {
  size_t size = 0;
  // header
  size += sizeof(int); // id
  size += sizeof(int); // type
  // max lod node
  size += sizeof(vm::ivec3); // min
  size += sizeof(int); // lod
  size += sizeof(int); // isLeaf
  size += sizeof(int[8]); // lodArray
  // old nodes
  size += sizeof(uint32_t); // numOldNodes
  size += sizeof(vm::ivec3) * oldNodes.size(); // min
  size += sizeof(int) * oldNodes.size(); // lod
  size += sizeof(int) * oldNodes.size(); // isLeaf
  size += sizeof(int[8]) * oldNodes.size(); // lodArray
  // new nodes
  size += sizeof(uint32_t); // numNewNodes
  size += sizeof(vm::ivec3) * newNodes.size(); // min
  size += sizeof(int) * newNodes.size(); // lod
  size += sizeof(int) * newNodes.size(); // isLeaf
  size += sizeof(int[8]) * newNodes.size(); // lodArray

  std::vector<uint8_t> result(size);
  int index = 0;
  // id
  *((int *)(result.data() + index)) = id;
  index += sizeof(int);
  // type
  *((int *)(result.data() + index)) = type;
  index += sizeof(int);
  // max lod node
  std::memcpy(result.data() + index, &maxLodNode->min, sizeof(vm::ivec3));
  index += sizeof(vm::ivec3);
  *((int *)(result.data() + index)) = maxLodNode->size;
  index += sizeof(int);
  *((int *)(result.data() + index)) = (maxLodNode->type == Node_Leaf) ? 1 : 0;
  index += sizeof(int);
  std::memcpy(result.data() + index, &maxLodNode->lodArray[0], sizeof(int[8]));
  index += sizeof(int[8]);
  // old nodes
  *((uint32_t *)(result.data() + index)) = oldNodes.size();
  index += sizeof(uint32_t);
  for (auto oldNode : oldNodes) {
    std::memcpy(result.data() + index, &oldNode->min, sizeof(vm::ivec3));
    index += sizeof(vm::ivec3);
    *((int *)(result.data() + index)) = oldNode->size;
    index += sizeof(int);
    *((int *)(result.data() + index)) = (oldNode->type == Node_Leaf) ? 1 : 0;
    index += sizeof(int);
    std::memcpy(result.data() + index, &oldNode->lodArray[0], sizeof(int[8]));
    index += sizeof(int[8]);
  }
  // new nodes
  *((uint32_t *)(result.data() + index)) = newNodes.size();
  index += sizeof(uint32_t);
  for (auto newNode : newNodes) {
    std::memcpy(result.data() + index, &newNode->min, sizeof(vm::ivec3));
    index += sizeof(vm::ivec3);
    *((int *)(result.data() + index)) = newNode->size;
    index += sizeof(int);
    *((int *)(result.data() + index)) = (newNode->type == Node_Leaf) ? 1 : 0;
    index += sizeof(int);
    std::memcpy(result.data() + index, &newNode->lodArray[0], sizeof(int[8]));
    index += sizeof(int[8]);
  }

  return result;
}
uint8_t *TrackerUpdate::getBuffer() const {
  std::vector<std::vector<uint8_t>> oldTaskBuffers;
  for (const auto &task : oldTasks) {
    oldTaskBuffers.push_back(task->getBuffer());
  }

  std::vector<std::vector<uint8_t>> newTaskBuffers;
  for (const auto &task : newTasks) {
    newTaskBuffers.push_back(task->getBuffer());
  }

  size_t size = 0;
  size += sizeof(vm::ivec3); // currentCoord
  size += sizeof(uint32_t); // numOldTasks
  size += sizeof(uint32_t); // numNewTasks
  for (auto &buffer : oldTaskBuffers) {
    size += buffer.size();
  }
  for (auto &buffer : newTaskBuffers) {
    size += buffer.size();
  }

  uint8_t *ptr = (uint8_t *)malloc(size);
  int index = 0;
  memcpy(ptr + index, &currentCoord, sizeof(vm::ivec3));
  index += sizeof(vm::ivec3);
  *((uint32_t *)(ptr + index)) = oldTasks.size();
  index += sizeof(uint32_t);
  *((uint32_t *)(ptr + index)) = newTasks.size();
  index += sizeof(uint32_t);
  for (size_t i = 0; i < oldTaskBuffers.size(); i++) {
    auto &buffer = oldTaskBuffers[i];
    memcpy(ptr + index, buffer.data(), buffer.size() * sizeof(buffer[0]));
    index += buffer.size() * sizeof(buffer[0]);
  }
  for (size_t i = 0; i < newTaskBuffers.size(); i++) {
    auto &buffer = newTaskBuffers[i];
    memcpy(ptr + index, buffer.data(), buffer.size() * sizeof(buffer[0]));
    index += buffer.size() * sizeof(buffer[0]);
  }
  return ptr;
}

//

bool containsPoint(const OctreeNode &node, const vm::ivec3 &p) {
  return p.x >= node.min.x && p.x < node.min.x + node.size &&
    p.y >= node.min.y && p.y < node.min.y + node.size &&
    p.z >= node.min.z && p.z < node.min.z + node.size;
}
bool containsNode(const OctreeNode &node, const OctreeNode &other) {
  return containsPoint(node, other.min);
}
bool equalsNode(const OctreeNode &node, const OctreeNode &other) {
  if (node.min == other.min) {
    for (size_t i = 0; i < node.lodArray.size(); i++) {
      if (node.lodArray[i] != other.lodArray[i]) {
        return false;
      }
    }
    return true;
  } else {
    return false;
  }
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
// OctreeNodeAllocator octreeNodeAllocator;

//

/* OctreeNodeAllocator::OctreeNodeAllocator() {
  for (int i = 0; i < rawNodes.size(); i++) {
    OctreeNode *node = &rawNodes[i];
    nodes.push_back(node);
  }
  this->deleter = [this](void *ptr) -> void {
    this->nodes.push_back((OctreeNode *)ptr);
  };
} */
OctreeNodePtr OctreeNodeAllocator::alloc(const vm::ivec3 &min, int lod, bool isLeaf) {
  OctreeNode *octreeNode = new OctreeNode();
  octreeNode->min = min;
  octreeNode->size = lod;
  octreeNode->type = isLeaf ? Node_Leaf : Node_Internal;
  if (isLeaf) {
    for (size_t i = 0; i < 8; i++) {
      octreeNode->lodArray[i] = -1;
    }
  } else {
    for (size_t i = 0; i < 8; i++) {
      octreeNode->children[i] = nullptr;
    }
  }
  return std::shared_ptr<OctreeNode>(octreeNode);
}
OctreeContext::OctreeContext() {}

//

OctreeNodePtr getLeafNodeFromPoint(const std::vector<OctreeNodePtr> &leafNodes, const vm::ivec3 &p) {
    for (size_t i = 0; i < leafNodes.size(); i++) {
        auto leafNode = leafNodes[i];
        if (containsPoint(*leafNode, p)) {
            return leafNode;
        }
    }
    return nullptr;
}
OctreeNodePtr getNode(OctreeContext &octreeContext, const vm::ivec3 &min, int lod) {
    auto &nodeMap = octreeContext.nodeMap;

    uint64_t hash = hashOctreeMinLod(min, lod);
    auto iter = nodeMap.find(hash);
    if (iter != nodeMap.end()) {
      return iter->second;
    } else {
      return nullptr;
    }
}
OctreeNodePtr createNode(OctreeContext &octreeContext, const vm::ivec3 &min, int lod, bool isLeaf) {
    auto &nodeMap = octreeContext.nodeMap;

    auto node = OctreeNodeAllocator::alloc(min, lod, isLeaf);
    // node->min = min;
    // node->size = lod;
    // node->type = isLeaf ? Node_Leaf : Node_Internal;
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
OctreeNodePtr createNode(OctreeContext &octreeContext, const vm::ivec3 &min, int lod) {
    return createNode(octreeContext, min, lod, lod == 1);
}
OctreeNodePtr getOrCreateNode(OctreeContext &octreeContext, const vm::ivec3 &min, int lod) {
    OctreeNodePtr node = getNode(octreeContext, min, lod);
    if (!node) {
        node = createNode(octreeContext, min, lod);
    }
    return node;
}
void ensureChildren(OctreeContext &octreeContext, OctreeNode *parentNode) {
    // auto &nodeMap = octreeContext.nodeMap;

    const vm::ivec3 &lodMin = parentNode->min;
    const int &lod = parentNode->size;

    for (int dx = 0; dx < 2; dx++) {
      for (int dy = 0; dy < 2; dy++) {
        for (int dz = 0; dz < 2; dz++) {
           int childIndex = dx + 2 * (dy + 2 * dz);
           if (parentNode->children[childIndex] == nullptr) {
              parentNode->children[childIndex] = createNode(
                octreeContext,
                lodMin + vm::ivec3{dx, dy, dz} * (lod / 2),
                lod / 2,
                true
              ).get();
            //   parentNode->children[childIndex].parent = parentNode;
           }
        }
      }
    }
}
void constructTreeUpwards(OctreeContext &octreeContext, const vm::ivec3 &leafPosition, int maxLod) {
    auto &nodeMap = octreeContext.nodeMap;

    OctreeNodePtr rootNode = getOrCreateNode(octreeContext, leafPosition, 1);
    for (int lod = 2; lod <= maxLod; lod *= 2) {
      vm::ivec3 lodMin = rootNode->min;
      lodMin.x = (int)std::floor((float)lodMin.x / (float)lod) * lod;
      lodMin.y = (int)std::floor((float)lodMin.y / (float)lod) * lod;
      lodMin.z = (int)std::floor((float)lodMin.z / (float)lod) * lod;

      const vm::ivec3 &lodCenter = lodMin + (lod / 2);
      const int childIndex = (rootNode->min.x < lodCenter.x ? 0 : 1) +
        (rootNode->min.y < lodCenter.y ? 0 : 2) +
        (rootNode->min.z < lodCenter.z ? 0 : 4);

      OctreeNodePtr parentNode = getOrCreateNode(octreeContext, lodMin, lod);
      // parentNode.isLeaf = false;
      parentNode->type = Node_Internal;
      if (parentNode->children[childIndex] == nullptr) { // children not set yet
        parentNode->children[childIndex] = rootNode.get();
        ensureChildren(octreeContext, parentNode.get());
      }
      rootNode = parentNode;
    }
    // return rootNode;
}

std::vector<OctreeNodePtr> constructOctreeForLeaf(const vm::ivec3 &position, int lod1Range, int maxLod) {
  OctreeContext octreeContext;
  auto &nodeMap = octreeContext.nodeMap;

  // sample base leaf nodes to generate octree upwards
  const vm::ivec3 &rangeMin = position - vm::ivec3{lod1Range, lod1Range, lod1Range};
  const vm::ivec3 &rangeMax = position + vm::ivec3{lod1Range, lod1Range, lod1Range};
  for (int dx = rangeMin.x; dx <= rangeMax.x; dx++) {
    for (int dy = rangeMin.y; dy <= rangeMax.y; dy++) {
      for (int dz = rangeMin.z; dz <= rangeMax.z; dz++) {
        vm::ivec3 leafPosition = vm::ivec3{dx, dy, dz};
        constructTreeUpwards(octreeContext, leafPosition, maxLod);
      }
    }
  }

  std::vector<std::shared_ptr<OctreeNode>> rootNodes;
  for (const auto &iter : nodeMap) {
    auto node = iter.second;
    if (node->size == maxLod) {
      rootNodes.push_back(node);
    }
  }

  std::vector<std::shared_ptr<OctreeNode>> lod1Nodes;
  for (const auto &iter : nodeMap) {
    auto node = iter.second;
    if (node->size == 1) {
      lod1Nodes.push_back(node);
    }
  }

  // sanity check lod1Nodes for duplicates
  {
    std::unordered_map<uint64_t, OctreeNodePtr> lod1NodeMap;
    for (auto node : lod1Nodes) {
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

  std::vector<OctreeNodePtr> leafNodes;
  for (const auto &iter : nodeMap) {
    auto node = iter.second;
    if (node->type == Node_Leaf) {
      leafNodes.push_back(node);
    }
  }

  // sanity check that no leaf node contains another leaf node
  for (auto leafNode : leafNodes) {
    /* for (auto childNode : leafNode->children) {
      if (childNode != nullptr && childNode->type == Node_Leaf) {
        // throw new Error(`Leaf node contains another leaf node 1: ${leafNode.min.toArray().join(',')}`);
        EM_ASM({
            console.log('Leaf node contains another leaf node 1:', $0, $1, $2);
        }, leafNode->min.x, leafNode->min.y, leafNode->min.z);
      }
    } */
    for (auto leafNode2 : leafNodes) {
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
    auto node = iter.second;

    for (int i = 0; i < 8; i++) {
      const vm::ivec3 &offset = lodOffsets[i];
      int &lod = node->lodArray[i]; // output
      
      const vm::ivec3 &p = node->min + offset * node->size;
      OctreeNodePtr containingLeafNode = getLeafNodeFromPoint(leafNodes, p);
      if (containingLeafNode) {
        lod = containingLeafNode->size;
      } else {
        lod = node->size;
      }
    }
  }

  return leafNodes;
}
OctreeNodePtr getMaxLodNode(const std::vector<OctreeNodePtr> &newLeafNodes, const std::vector<OctreeNodePtr> &oldLeafNodes, const vm::ivec3 &min) {
    auto newLeafNode = getLeafNodeFromPoint(newLeafNodes, min);
    auto oldLeafNode = getLeafNodeFromPoint(oldLeafNodes, min);
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
std::vector<TrackerTaskPtr> diffLeafNodes(const std::vector<OctreeNodePtr> &newLeafNodes, const std::vector<OctreeNodePtr> &oldLeafNodes) {
  // map from min lod hash to task containing new nodes and old nodes
  std::unordered_map<uint64_t, TrackerTaskPtr> taskMap;

  for (OctreeNodePtr newNode : newLeafNodes) {
    OctreeNodePtr maxLodNode = getMaxLodNode(newLeafNodes, oldLeafNodes, newNode->min);
    const uint64_t hash = hashOctreeMinLod(maxLodNode->min, maxLodNode->size);
    
    TrackerTaskPtr task;
    const auto &iter = taskMap.find(hash);
    if (iter != taskMap.end()) {
      task = iter->second;
    } else {
      TrackerTask *trackerTask = new TrackerTask();
      trackerTask->id = ++nextTrackerId;
      // std::cout << "increment 1 " << trackerTask->id << std::endl;
      trackerTask->maxLodNode = maxLodNode;
      trackerTask->type = TrackerTaskType::ADD;
      
      task = std::shared_ptr<TrackerTask>(trackerTask);
      
      taskMap[hash] = task;
    }
    task->newNodes.push_back(newNode);
  }
  for (OctreeNodePtr oldNode : oldLeafNodes) {
    OctreeNodePtr maxLodNode = getMaxLodNode(newLeafNodes, oldLeafNodes, oldNode->min);
    const uint64_t hash = hashOctreeMinLod(maxLodNode->min, maxLodNode->size);
    
    TrackerTaskPtr task;
    const auto &iter = taskMap.find(hash);
    if (iter != taskMap.end()) {
      task = iter->second;
    } else {
      TrackerTask *trackerTask = new TrackerTask();
      trackerTask->id = ++nextTrackerId;
      // std::cout << "increment 2 " << trackerTask->id << std::endl;
      trackerTask->maxLodNode = maxLodNode;
      trackerTask->type = TrackerTaskType::REMOVE;
      
      task = std::shared_ptr<TrackerTask>(trackerTask);
      
      taskMap[hash] = task;
    }
    task->oldNodes.push_back(oldNode);
  }

  std::vector<TrackerTaskPtr> tasks;
  for (const auto &iter : taskMap) {
    tasks.push_back(iter.second);
  }
  return tasks;
}
// sort tasks by distance to world position of the central max lod node
std::vector<TrackerTaskPtr> sortTasks(const std::vector<TrackerTaskPtr> &tasks, const vm::vec3 &worldPosition) {
  std::vector<std::pair<TrackerTaskPtr, float>> taskDistances;
  taskDistances.reserve(tasks.size());

  for (const auto &task : tasks) {
    const vm::ivec3 &min = task->maxLodNode->min;
    const int &lod = task->maxLodNode->size;

    vm::vec3 center = vm::vec3{(float)min.x, (float)min.y, (float)min.z} +
      vm::vec3{0.5, 0.5, 0.5} * (float)lod;
    vm::vec3 delta = worldPosition - center;
    float distance = vm::lengthSq(delta);

    taskDistances.push_back(std::pair<TrackerTaskPtr, float>(task, distance));
  }

  std::sort(
    taskDistances.begin(),
    taskDistances.end(),
    [](const std::pair<TrackerTaskPtr, float> &a, const std::pair<TrackerTaskPtr, float> &b) -> bool {
      return a.second < b.second;
    }
  );

  std::vector<TrackerTaskPtr> sortedTasks;
  for (const auto &iter : taskDistances) {
    sortedTasks.push_back(iter.first);
  }
  return sortedTasks;
}
std::pair<std::vector<OctreeNodePtr>, std::vector<TrackerTaskPtr>> updateChunks(const std::vector<OctreeNodePtr> &oldChunks, const std::vector<TrackerTaskPtr> &tasks) {
  std::vector<OctreeNodePtr> newChunks = oldChunks;
  
  for (const auto &task : tasks) {
    if (!task->isNop()) {
      const std::vector<OctreeNodePtr> &newNodes = task->newNodes;
      const std::vector<OctreeNodePtr> &oldNodes = task->oldNodes;

      for (OctreeNodePtr oldNode : oldNodes) {
        const auto &iter = std::find_if(
            newChunks.begin(),
            newChunks.end(),
            [&](auto &chunk) -> bool {
                return equalsNode(*chunk, *oldNode);
            }
        );
        if (iter != newChunks.end()) {
          // newChunks.splice(index, 1);
          newChunks.erase(iter);
        } else {
          // debugger;
          std::cout << "failed to erase new chunk" << std::endl;
          abort();
        }
      }
      for (OctreeNodePtr newNode : newNodes) {
        newChunks.push_back(newNode);
      }
    }
  }

  std::vector<OctreeNodePtr> removedChunks;
  for (auto &oldChunk : oldChunks) {
    if (std::any_of(newChunks.begin(), newChunks.end(), [&](auto &newChunk) {
      return newChunk->min == oldChunk->min;
    })) {
      removedChunks.push_back(oldChunk);
    }
  }
  std::vector<TrackerTaskPtr> extraTasks;
  for (OctreeNodePtr chunk : removedChunks) {
    TrackerTask *trackerTask = new TrackerTask();
    trackerTask->id = ++nextTrackerId;
    // std::cout << "increment 3 " << trackerTask->id << std::endl;
    trackerTask->maxLodNode = chunk;
    trackerTask->type = TrackerTaskType::OUTRANGE;
    trackerTask->oldNodes.push_back(chunk);

    TrackerTaskPtr task = std::shared_ptr<TrackerTask>(trackerTask);

    extraTasks.push_back(task);
  }

  return std::pair<std::vector<OctreeNodePtr>, std::vector<TrackerTaskPtr>>(
    std::move(newChunks),
    std::move(extraTasks)
  );
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
  std::vector<OctreeNodePtr> octreeLeafNodes = constructOctreeForLeaf(currentCoord, this->minLodRange, 1 << (this->lods - 1));

  std::vector<TrackerTaskPtr> tasks = diffLeafNodes(
    octreeLeafNodes,
    this->lastOctreeLeafNodes
  );

  {
    std::pair<std::vector<OctreeNodePtr>, std::vector<TrackerTaskPtr>> chunksUpdate = updateChunks(this->chunks, tasks);
    std::vector<OctreeNodePtr> &newChunks = chunksUpdate.first;
    std::vector<TrackerTaskPtr> &newTasks = chunksUpdate.second;
    this->chunks = std::move(newChunks);
    for (const auto &task : newTasks) {
      tasks.push_back(task);
    }
  }

  vm::vec3 worldPosition = vm::vec3{
      (float)currentCoord.x,
      (float)currentCoord.y,
      (float)currentCoord.z
  } + vm::vec3{0.5, 0.5, 0.5} * ((float)chunkSize / 2.0f);
  tasks = sortTasks(tasks, worldPosition);

  // JS version. Below it is ported to C++.
  /* for (size_t i = 0; i < tasks.size(); i++) {
    auto &task = tasks[i];
    if (!task->isNop()) {
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
  } */

  std::vector<TrackerTaskPtr> oldTasks;
  for (size_t i = 0; i < tasks.size(); i++) {
    auto &task = tasks[i];
    if (!task->isNop()) {
      std::vector<TrackerTaskPtr> overlappingTasks;
      for (TrackerTaskPtr liveTask : this->liveTasks) {
        if (containsNode(*task->maxLodNode, *liveTask->maxLodNode)) {
          overlappingTasks.push_back(liveTask);
        }
      }

      for (TrackerTaskPtr oldTask : overlappingTasks) {
        const auto &iter = std::find(
          this->liveTasks.begin(),
          this->liveTasks.end(),
          oldTask
        );
        if (iter == this->liveTasks.end()) {
          std::cout << "bad live task to remove" << std::endl;
          abort();
        }
        this->liveTasks.erase(iter);

        std::cout << "wasm old task " <<
          task->maxLodNode->min.x << " " << task->maxLodNode->min.y << " " << task->maxLodNode->min.z << " : " <<
          task->maxLodNode->size << " " <<
          task->oldNodes.size() << " " << task->newNodes.size() << " " <<
          (void *)task.get() << " " <<
          oldTask->id << std::endl;

        oldTasks.push_back(oldTask);
      }

      std::cout << "wasm new task " <<
        task->maxLodNode->min.x << " " << task->maxLodNode->min.y << " " << task->maxLodNode->min.z << " : " <<
        task->maxLodNode->size << " " <<
        task->oldNodes.size() << " " << task->newNodes.size() << " " <<
        (void *)task.get() << " " <<
        task->id << std::endl;
      this->liveTasks.push_back(task);
    }
  }

  this->lastOctreeLeafNodes = std::move(octreeLeafNodes);

  TrackerUpdate result;
  result.currentCoord = currentCoord;
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