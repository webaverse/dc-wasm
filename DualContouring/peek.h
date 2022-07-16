#ifndef PEEK_H
#define PEEK_H
#include "../util.h"
#include "instance.h"
#include <memory>

int getIndex(const vm::ivec3 &pos, const int &size, const int &lod)
{
    return (pos.x + pos.y * size + pos.z * size * size) / lod;
}

template <typename DCContextType>
inline void floodFill(DCInstance *inst,
                      int x, int y, int z,
                      int startFace,
                      const vm::ivec3 &chunkMin, const vm::ivec3 &chunkMax,
                      unsigned char *peeks, unsigned char *seenPeeks, int *PEEK_FACE_INDICES, const int &lod, const int &size)
{
    std::unique_ptr<int[]> queue(new int[size * size * size * 4]);
    unsigned int queueEnd = 0;

    const vm::ivec3 localPos = vm::ivec3{x, y, z} - chunkMin;

    const int index = getIndex(localPos, size, lod);

    queue[queueEnd * 4 + 0] = x;
    queue[queueEnd * 4 + 1] = y;
    queue[queueEnd * 4 + 2] = z;
    queue[queueEnd * 4 + 3] = index;
    queueEnd++;
    seenPeeks[index] = 1;

    for (unsigned int queueStart = 0; queueStart < queueEnd; queueStart++)
    {
        const int x = queue[queueStart * 4 + 0];
        const int y = queue[queueStart * 4 + 1];
        const int z = queue[queueStart * 4 + 2];
        const int index = queue[queueStart * 4 + 3];

        const float sdf = DCContextType::densityFn(vm::vec3{(float)x, (float)y, (float)z}, lod, inst);

        if (sdf >= 0)
        { // if empty space
            if (z == chunkMin.z && startFace != (int)PEEK_FACES::BACK)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::BACK]] = 1;
            }
            if (z == chunkMax.z && startFace != (int)PEEK_FACES::FRONT)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::FRONT]] = 1;
            }
            if (x == chunkMin.x && startFace != (int)PEEK_FACES::LEFT)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::LEFT]] = 1;
            }
            if (x == chunkMax.x && startFace != (int)PEEK_FACES::RIGHT)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::RIGHT]] = 1;
            }
            if (y == chunkMax.y && startFace != (int)PEEK_FACES::TOP)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::TOP]] = 1;
            }
            if (y == chunkMin.y && startFace != (int)PEEK_FACES::BOTTOM)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::BOTTOM]] = 1;
            }

            for (int dx = -lod; dx <= lod; dx += lod)
            {
                const int ax = x + dx;
                if (ax >= chunkMin.x && ax <= chunkMax.x)
                {
                    for (int dy = -lod; dy <= lod; dy += lod)
                    {
                        const int ay = y + dy;
                        if (ay >= chunkMin.y && ay <= chunkMax.y)
                        {
                            for (int dz = -lod; dz <= lod; dz += lod)
                            {
                                const int az = z + dz;
                                if (az >= chunkMin.z && az <= chunkMax.z)
                                {
                                    const vm::ivec3 localPos = vm::ivec3{ax, ay, az} - chunkMin;
                                    const int index = getIndex(localPos, size, lod);
                                    if (!seenPeeks[index])
                                    {
                                        queue[queueEnd * 4 + 0] = ax;
                                        queue[queueEnd * 4 + 1] = ay;
                                        queue[queueEnd * 4 + 2] = az;
                                        queue[queueEnd * 4 + 3] = index;
                                        queueEnd++;
                                        seenPeeks[index] = 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

template <typename DCContextType>
void setPeeks(DCInstance *inst, const vm::ivec3 &chunkMin, const vm::ivec3 &chunkMax, const int &lod, unsigned char *peeks, int *PEEK_FACE_INDICES)
{

    const int size = chunkSize + 1;
    unsigned char seenPeeks[size * size * size];

    for (int x = chunkMin.x; x <= chunkMax.x; x += lod)
    {
        for (int y = chunkMin.y; y <= chunkMax.y; y += lod)
        {
            floodFill<DCContextType>(inst, x, y, chunkMax.z, (int)PEEK_FACES::FRONT, chunkMin, chunkMax, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
        }
    }
    for (int x = chunkMin.x; x <= chunkMax.x; x += lod)
    {
        for (int y = chunkMin.y; y <= chunkMax.y; y += lod)
        {
            floodFill<DCContextType>(inst, x, y, chunkMin.z, (int)PEEK_FACES::BACK, chunkMin, chunkMax, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
        }
    }
    for (int z = chunkMin.z; z <= chunkMax.z; z += lod)
    {
        for (int y = chunkMin.y; y <= chunkMax.y; y += lod)
        {
            floodFill<DCContextType>(inst, chunkMin.x, y, z, (int)PEEK_FACES::LEFT, chunkMin, chunkMax, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
        }
    }
    for (int z = chunkMin.z; z <= chunkMax.z; z += lod)
    {
        for (int y = chunkMin.y; y <= chunkMax.y; y += lod)
        {
            floodFill<DCContextType>(inst, chunkMax.x, y, z, (int)PEEK_FACES::RIGHT, chunkMin, chunkMax, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
        }
    }

    for (int x = chunkMin.x; x <= chunkMax.x; x += lod)
    {
        for (int z = chunkMin.z; z <= chunkMax.z; z += lod)
        {
            floodFill<DCContextType>(inst, x, chunkMax.y, z, (int)PEEK_FACES::TOP, chunkMin, chunkMax, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
        }
    }
    for (int x = chunkMin.x; x <= chunkMax.x; x += lod)
    {
        for (int z = chunkMin.z; z <= chunkMax.z; z += lod)
        {
            floodFill<DCContextType>(inst, x, chunkMin.y, z, (int)PEEK_FACES::BOTTOM, chunkMin, chunkMax, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
        }
    }

    for (int startFace = 0; startFace < 6; startFace++)
    {
        for (int endFace = 0; endFace < 6; endFace++)
        {
            if (endFace != startFace)
            {
                if (peeks[PEEK_FACE_INDICES[startFace << 3 | endFace]] == 1)
                {
                    peeks[PEEK_FACE_INDICES[endFace << 3 | startFace]] = 1;

                    for (int crossFace = 0; crossFace < 6; crossFace++)
                    {
                        if (crossFace != startFace && crossFace != endFace)
                        {
                            if (peeks[PEEK_FACE_INDICES[startFace << 3 | crossFace]] == 1)
                            {
                                peeks[PEEK_FACE_INDICES[crossFace << 3 | startFace]] = 1;
                                peeks[PEEK_FACE_INDICES[crossFace << 3 | endFace]] = 1;
                                peeks[PEEK_FACE_INDICES[endFace << 3 | crossFace]] = 1;
                            }
                            else if (peeks[PEEK_FACE_INDICES[endFace << 3 | crossFace]] == 1)
                            {
                                peeks[PEEK_FACE_INDICES[crossFace << 3 | startFace]] = 1;
                                peeks[PEEK_FACE_INDICES[crossFace << 3 | endFace]] = 1;
                                peeks[PEEK_FACE_INDICES[startFace << 3 | crossFace]] = 1;
                            }
                        }
                    }
                }
            }
        }
    }
}

#endif
