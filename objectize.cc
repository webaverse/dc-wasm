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

EMSCRIPTEN_KEEPALIVE uint8_t *vhacd(
    float *positions,
    unsigned int positionsSize,
    unsigned int *indices,
    unsigned int indicesSize,
    unsigned int *resultSize,

    uint32_t maxConvexHulls,
    uint32_t resolution,
    float minimumVolumePercentErrorAllowed,
    uint32_t maxRecursionDepth,
    uint32_t shrinkWrapInt,
    uint32_t fillModeIndex,
    uint32_t maxNumVerticesPerCH,
    uint32_t minEdgeLength,
    uint32_t findBestPlaneInt
) {
    // std::cout << "compute " << positionsSize << " " << indicesSize << std::endl;

    // initialize
    VHACD::IVHACD *iface = VHACD::CreateVHACD();

    VHACD::IVHACD::Parameters parameters;
    parameters.m_maxConvexHulls = maxConvexHulls;
    parameters.m_resolution = resolution;
    parameters.m_minimumVolumePercentErrorAllowed = minimumVolumePercentErrorAllowed;
    parameters.m_maxRecursionDepth = maxRecursionDepth;
    parameters.m_shrinkWrap = (bool)shrinkWrapInt;
    parameters.m_fillMode = (VHACD::FillMode)fillModeIndex;
    parameters.m_maxNumVerticesPerCH = maxNumVerticesPerCH;
    parameters.m_minEdgeLength = minEdgeLength;
    parameters.m_findBestPlane = (bool)findBestPlaneInt;

    // compute hulls
    bool ok = iface->Compute(
        positions,
        positionsSize / 3,
        indices,
        indicesSize / 3,
        parameters
    );
    bool ready = iface->IsReady();
    std::cout << "vhacd done " << (+ok) << " " << (+ready) << std::endl;

    // extract hulls
    unsigned int numHulls = iface->GetNConvexHulls();
    std::vector<VHACD::IVHACD::ConvexHull> hulls(numHulls);
    for (uint32_t i = 0; i < iface->GetNConvexHulls(); i++) {
        iface->GetConvexHull(i, hulls[i]);
    }

    // compute needed size
    unsigned int neededSize = 0;
    neededSize += sizeof(uint32_t); // num hulls
    for (uint32_t i = 0; i < hulls.size(); i++) {
        VHACD::IVHACD::ConvexHull &ch = hulls[i];

        neededSize += sizeof(uint32_t); // num positions
        for (uint32_t j = 0; j < ch.m_points.size(); j++) {
          neededSize += sizeof(float) * 3;
        }

        neededSize += sizeof(uint32_t); // num triangles
        for (uint32_t j = 0; j < ch.m_triangles.size(); j++) {
            neededSize += sizeof(uint32_t) * 3;
        }
    }

    // read result
    uint8_t *resultBuffer = (uint8_t *)malloc(neededSize);
    int index = 0;

    uint32_t numHulls32 = (uint32_t)hulls.size();
    memcpy(resultBuffer + index, &numHulls32, sizeof(uint32_t));
    index += sizeof(uint32_t);

    for (uint32_t i = 0; i < iface->GetNConvexHulls(); i++) {
        VHACD::IVHACD::ConvexHull &ch = hulls[i];

        // positions
        uint32_t numPositions = ch.m_points.size();
        uint32_t numPositionFloats = numPositions * 3;
        memcpy(resultBuffer + index, &numPositionFloats, sizeof(uint32_t));
        index += sizeof(uint32_t);

        for (uint32_t j = 0; j < ch.m_points.size(); j++) {
            VHACD::Vertex &v = ch.m_points[j];
            
            float mX = (float)v.mX;
            memcpy(resultBuffer + index, &mX, sizeof(float));
            index += sizeof(float);

            float mY = (float)v.mY;
            memcpy(resultBuffer + index, &mY, sizeof(float));
            index += sizeof(float);

            float mZ = (float)v.mZ;
            memcpy(resultBuffer + index, &mZ, sizeof(float));
            index += sizeof(float);
        }

        // triangles
        uint32_t numTriangles = ch.m_triangles.size();
        uint32_t numTriangleUints = numTriangles * 3;
        memcpy(resultBuffer + index, &numTriangleUints, sizeof(uint32_t));
        index += sizeof(uint32_t);

        for (uint32_t j = 0; j < ch.m_triangles.size(); j++) {
            VHACD::Triangle &t = ch.m_triangles[j];

            uint32_t mI0 = t.mI0;
            memcpy(resultBuffer + index, &mI0, sizeof(uint32_t));
            index += sizeof(uint32_t);

            uint32_t mI1 = t.mI1;
            memcpy(resultBuffer + index, &mI1, sizeof(uint32_t));
            index += sizeof(uint32_t);

            uint32_t mI2 = t.mI2;
            memcpy(resultBuffer + index, &mI2, sizeof(uint32_t));
            index += sizeof(uint32_t);
        }
    }

    *resultSize = neededSize;

    iface->Release();

    return resultBuffer;
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