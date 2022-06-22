#ifndef CONTEXT_H
#define CONTEXT_H

#include "vectorMath.h"
#include "mesh.h"
#include "density.h"

#include <vector>

//

// the context determines the density function and output of the mesh
class TerrainVertexContext {
public:
    TerrainVertexBuffer vertexBuffer;

    static float densityFn(const vm::vec3 &position, DCInstance *inst, Chunk &chunk);
};

class LiquidVertexContext {
public:
    LiquidVertexBuffer vertexBuffer;

    static float densityFn(const vm::vec3 &position, DCInstance *inst, Chunk &chunk);
};

#endif // CONTEXT_H
