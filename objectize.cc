#include <emscripten.h>
// #include "DualContouring/tracker.h"
#include "DualContouring/main.h"
#include "DualContouring/vectorMath.h"
#include "MC.h"
#include "VHACD.h"

extern "C" {

EMSCRIPTEN_KEEPALIVE DCInstance *createInstance(
    int chunkSize,
    int range,
    float fatness
) {
    return DualContouring::createInstance(
        chunkSize,
        range,
        fatness
    );
}
EMSCRIPTEN_KEEPALIVE void destroyInstance(DCInstance *instance) {
    DualContouring::destroyInstance(instance);
}

//

EMSCRIPTEN_KEEPALIVE uint8_t *createPointCloudMesh(DCInstance *instance, float *points, unsigned int pointsSize, unsigned int *resultSize) {
    unsigned int numPoints = pointsSize / 3;
    std::vector<vm::vec3> pointcloud(numPoints);
    for (int i = 0; i < numPoints; i++) {
        pointcloud[i] = vm::vec3{
            points[i * 3],
            points[i * 3 + 1],
            points[i * 3 + 2]
        };
    }
    uint8_t *result = instance->createPointCloudMesh(pointcloud, resultSize);
    return result;
}

// typedef struct mcVec3f
// {
// public:
//     union 
//     {
//         MC_FLOAT v[3];
//         struct 
//         {
//             MC_FLOAT x, y, z;
//         };
//     };
//     inline mcVec3f& operator+=(const mcVec3f& r)
//     {
//         x += r.x; y += r.y; z += r.z;
//         return *this;
//     }
//     inline MC_FLOAT& operator[](int i)
//     {
//         return v[i];
//     }
// } mcVec3f;

// typedef struct mcMesh
// {
// public:
//     std::vector<mcVec3f> vertices;
//     std::vector<mcVec3f> normals;
//     std::vector<muint> indices;
// } mcMesh;

EMSCRIPTEN_KEEPALIVE uint8_t *marchCubes(DCInstance *instance, float *points, unsigned int pointsSize, unsigned int *resultSize) {
    // fill field
    const int chunkSize = instance->chunkSize;
    const int range = instance->range;
    const float fatness = instance->fatness;
    MC::MC_FLOAT *field = new MC::MC_FLOAT[chunkSize * chunkSize * chunkSize];
    for (int i = 0; i < chunkSize * chunkSize * chunkSize; i++) {
        field[i] = -fatness + (float)range;
    }
    for (int i = 0; i < pointsSize; i += 3) {
        const vm::vec3 p{
            points[i],
            points[i + 1],
            points[i + 2]
        };
        const vm::ivec3 ip{
            (int)p.x,
            (int)p.y,
            (int)p.z
        };

        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                for (int dz = -range; dz <= range; dz++) {
                    const vm::ivec3 ip2{
                        ip.x + dx,
                        ip.y + dy,
                        ip.z + dz
                    };
                    if (
                        ip2.x >= 0 && ip2.x < chunkSize &&
                        ip2.y >= 0 && ip2.y < chunkSize &&
                        ip2.z >= 0 && ip2.z < chunkSize
                    ) {
                        const vm::vec3 p2{
                            (float)ip2.x + 0.5f,
                            (float)ip2.y + 0.5f,
                            (float)ip2.z + 0.5f
                        };
                        float distance = vm::length(p2 - p);

                        const int index = ip2.x + ip2.y * chunkSize + ip2.z * chunkSize * chunkSize;
                        field[index] = std::min(
                            field[index],
                            -fatness + distance
                        );
                    }
                }
            }
        }
    }

    // compute mesh
    MC::mcMesh mesh;
	MC::marching_cube(field, chunkSize, chunkSize, chunkSize, mesh);

    // compute needed size
    unsigned int neededSize = 0;
    neededSize += sizeof(uint32_t); // num positions
    for (uint32_t j = 0; j < mesh.vertices.size(); j++) {
        neededSize += sizeof(mesh.vertices[0]);
    }
    neededSize += sizeof(uint32_t); // num normals
    for (uint32_t j = 0; j < mesh.normals.size(); j++) {
        neededSize += sizeof(mesh.normals[0]);
    }
    neededSize += sizeof(uint32_t); // num indices
    for (uint32_t j = 0; j < mesh.indices.size(); j++) {
        neededSize += sizeof(mesh.indices[0]);
    }

    // output result
    uint8_t *result = (uint8_t *)malloc(neededSize);
    int index = 0;

    uint32_t numPositions = mesh.vertices.size();
    memcpy(result + index, &numPositions, sizeof(numPositions));
    index += sizeof(numPositions);
    for (uint32_t j = 0; j < mesh.vertices.size(); j++) {
        memcpy(result + index, &mesh.vertices[j], sizeof(mesh.vertices[0]));
        index += sizeof(mesh.vertices[0]);
    }

    uint32_t numNormals = mesh.normals.size();
    memcpy(result + index, &numNormals, sizeof(numNormals));
    index += sizeof(numNormals);
    for (uint32_t j = 0; j < mesh.normals.size(); j++) {
        memcpy(result + index, &mesh.normals[j], sizeof(mesh.normals[0]));
        index += sizeof(mesh.normals[0]);
    }

    uint32_t numIndices = mesh.indices.size();
    memcpy(result + index, &numIndices, sizeof(numIndices));
    index += sizeof(numIndices);
    for (uint32_t j = 0; j < mesh.indices.size(); j++) {
        memcpy(result + index, &mesh.indices[j], sizeof(mesh.indices[0]));
        index += sizeof(mesh.indices[0]);
    }

    *resultSize = neededSize;

    delete[] field;

    return result;
}

//

EMSCRIPTEN_KEEPALIVE void *doMalloc(size_t size) {
    return malloc(size);
}

EMSCRIPTEN_KEEPALIVE void doFree(void *ptr) {
    free(ptr);
}

//

int main() {
    // DualContouring::start();
    return 0;
}

} // extern "C"