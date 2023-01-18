#ifndef CONTEXT_H
#define CONTEXT_H

#include "vectorMath.h"
#include "mesh.h"
#include "density.h"
#include "constants.h"

#include <vector>

//

class SdfAccessor {
public:
  std::vector<float> &sdf;
  int chunkSize;

  SdfAccessor(std::vector<float> &sdf, int chunkSize);
  float get(float x, float y, float z) const;
};

//

// the context determines the density function and output of the mesh
class TerrainDCContext {
public:
    std::vector<float> sdf;
    SdfAccessor sdfAccessor;
    TerrainVertexBuffer vertexBuffer;

    TerrainDCContext(const std::vector<vm::vec3> &pointcloud, int chunkSize, int range, float fatness);

    float densityFn(const vm::vec3 &position);
};

#endif // CONTEXT_H
