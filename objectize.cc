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

// struct Vertex
// {
//     double mX;
//     double mY;
//     double mZ;

//     Vertex() = default;
//     Vertex(double x, double y, double z) : mX(x), mY(y), mZ(z) {}

//     const double& operator[](size_t idx) const
//     {
//         switch(idx)
//         {
//             case 0: return mX;
//             case 1: return mY;
//             case 2: return mZ;
//         };
//         return mX;
//     }
// };

// struct Triangle
// {
//     uint32_t mI0;
//     uint32_t mI1;
//     uint32_t mI2;

//     Triangle() = default;
//     Triangle(uint32_t i0, uint32_t i1, uint32_t i2) : mI0(i0), mI1(i1), mI2(i2) {}
// };

// class ConvexHull {
// public:
//     std::vector<VHACD::Vertex>      m_points;
//     std::vector<VHACD::Triangle>    m_triangles;

//     double                          m_volume{ 0 };          // The volume of the convex hull
//     VHACD::Vect3                    m_center{ 0, 0, 0 };    // The centroid of the convex hull
//     uint32_t                        m_meshId{ 0 };          // A unique id for this convex hull
//     VHACD::Vect3            mBmin;                  // Bounding box minimum of the AABB
//     VHACD::Vect3            mBmax;                  // Bounding box maximum of the AABB
// };

// bool Compute(
//     const float* const points,
//     const uint32_t countPoints,
//     const uint32_t* const triangles,
//     const uint32_t countTriangles,
//     const Parameters& params);

// class Parameters
// {
// public:
//     IUserCallback*      m_callback{nullptr};            // Optional user provided callback interface for progress
//     IUserLogger*        m_logger{nullptr};              // Optional user provided callback interface for log messages
//     IUserTaskRunner*    m_taskRunner{nullptr};          // Optional user provided interface for creating tasks
//     uint32_t            m_maxConvexHulls{ 64 };         // The maximum number of convex hulls to produce
//     uint32_t            m_resolution{ 400000 };         // The voxel resolution to use
//     double              m_minimumVolumePercentErrorAllowed{ 1 }; // if the voxels are within 1% of the volume of the hull, we consider this a close enough approximation
//     uint32_t            m_maxRecursionDepth{ 10 };        // The maximum recursion depth
//     bool                m_shrinkWrap{true};             // Whether or not to shrinkwrap the voxel positions to the source mesh on output
//     FillMode            m_fillMode{ FillMode::FLOOD_FILL }; // How to fill the interior of the voxelized mesh
//     uint32_t            m_maxNumVerticesPerCH{ 64 };    // The maximum number of vertices allowed in any output convex hull
//     bool                m_asyncACD{ true };             // Whether or not to run asynchronously, taking advantage of additional cores
//     uint32_t            m_minEdgeLength{ 2 };           // Once a voxel patch has an edge length of less than 4 on all 3 sides, we don't keep recursing
//     bool                m_findBestPlane{ false };       // Whether or not to attempt to split planes along the best location. Experimental feature. False by default.
// };

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