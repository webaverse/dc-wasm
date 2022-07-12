#ifndef PEEK_H
#define PEEK_H
#include "../util.h"
#include "instance.h"
#include <memory>

int getIndex(const int &x, const int &y, const int &z, const int &size)
{
    return x + z * size + y * size * size;
}

template <typename DCContextType>
void floodFill(DCInstance *inst,
               int x, int y, int z,
               int startFace,
               int minX, int maxX, int minY, int maxY, int minZ, int maxZ,
               unsigned char *peeks, unsigned char *seenPeeks, int *PEEK_FACE_INDICES,
               const int &lod, const int &size)
{
    std::unique_ptr<int[]> queue(new int[size * size * size * 4]);
    unsigned int queueEnd = 0;

    const int index = getIndex(x, y, z, size);
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
            if (z == minZ && startFace != (int)PEEK_FACES::BACK)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::BACK]] = 1;
            }
            if (z == maxZ && startFace != (int)PEEK_FACES::FRONT)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::FRONT]] = 1;
            }
            if (x == minX && startFace != (int)PEEK_FACES::LEFT)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::LEFT]] = 1;
            }
            if (x == maxX && startFace != (int)PEEK_FACES::RIGHT)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::RIGHT]] = 1;
            }
            if (y == maxY && startFace != (int)PEEK_FACES::TOP)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::TOP]] = 1;
            }
            if (y == minY && startFace != (int)PEEK_FACES::BOTTOM)
            {
                peeks[PEEK_FACE_INDICES[startFace << 3 | (int)PEEK_FACES::BOTTOM]] = 1;
            }

            // for (int dx = -lod; dx <= lod; dx += lod)
            // {
            //     const int ax = x + dx;
            //     if (ax >= minX && ax <= maxX)
            //     {
            //         for (int dz = -lod; dz <= lod; dz += lod)
            //         {
            //             const int az = z + dz;
            //             if (az >= minZ && az <= maxZ)
            //             {
            //                 for (int dy = -lod; dy <= lod; dy += lod)
            //                 {
            //                     const int ay = y + dy;
            //                     if (ay >= minY && ay <= maxY)
            //                     {
            //                         const int index = getIndex(ax, ay, az, size);
            //                         if (!seenPeeks[index])
            //                         {
            //                             queue[queueEnd * 4 + 0] = ax;
            //                             queue[queueEnd * 4 + 1] = ay;
            //                             queue[queueEnd * 4 + 2] = az;
            //                             queue[queueEnd * 4 + 3] = index;
            //                             queueEnd++;
            //                             seenPeeks[index] = 1;
            //                         }
            //                     }
            //                 }
            //             }
            //         }
            //     }
            // }
        }
    }
}

template <typename DCContextType>
void setPeeks(DCInstance *inst, const vm::ivec3 &chunkMin, const vm::ivec3 &chunkMax, const int &lod, unsigned char *peeks)
{

    int PEEK_FACE_INDICES[8 * 8];
    for (int i = 0; i < 8 * 8; i++)
    {
        PEEK_FACE_INDICES[i] = 0xFF;
    }

    int peekIndex = 0;
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (i != j)
            {
                int otherEntry = PEEK_FACE_INDICES[j << 3 | i];
                PEEK_FACE_INDICES[i << 3 | j] = otherEntry != 0xFF ? otherEntry : peekIndex++;
            }
        }
    }

    const int minX = chunkMin.x;
    const int maxX = chunkMax.x;

    const int minY = chunkMin.y;
    const int maxY = chunkMax.y;

    const int minZ = chunkMin.z;
    const int maxZ = chunkMax.z;

    const int csize = (chunkSize * lod);
    const int size = csize + lod;

    // EM_ASM({
    //     console.log("Error 1\n");
    // });

    // std::cout << "limits " << minX << ":" << maxX << ":" << minY << ":" << maxY << ":" << minZ << ":" << maxZ << "\n";

    // const geometry = geometries[i];
    // const peeks = new Uint8Array(16);
    unsigned char seenPeeks[size * size * size];
    // const minY = i * NUM_CELLS;
    // const maxY = (i + 1) * NUM_CELLS;
    for (int x = minX; x <= maxX; x += lod)
    {
        for (int y = minY; y <= maxY; y += lod)
        {
            floodFill<DCContextType>(inst, x, y, maxZ, (int)PEEK_FACES::FRONT, minX, maxX, minY, maxY, minZ, maxZ, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
        }
    }
    // for (int x = minX; x <= maxX; x+=lod)
    // {
    //     for (int y = minY; y <= maxY; y+=lod)
    //     {
    //         floodFill<DCContextType>(inst, x, y, minZ, (int)PEEK_FACES::BACK, minX, maxX, minY, maxY, minZ, maxZ, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
    //     }
    // }
    // for (int z = minZ; z <= maxZ; z+=lod)
    // {
    //     for (int y = minY; y <= maxY; y+=lod)
    //     {
    //         floodFill<DCContextType>(inst, minX, y, z, (int)PEEK_FACES::LEFT, minX, maxX, minY, maxY, minZ, maxZ, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
    //     }
    // }
    // for (int z = minZ; z <= maxZ; z+=lod)
    // {
    //     for (int y = minY; y <= maxY; y+=lod)
    //     {
    //         floodFill<DCContextType>(inst, maxX, y, z, (int)PEEK_FACES::RIGHT, minX, maxX, minY, maxY, minZ, maxZ, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
    //     }
    // }
    // for (int x = minX; x <= maxX; x++)
    // {
    //     for (int z = minZ; z <= maxZ; z+=lod)
    //     {
    //         floodFill<DCContextType>(inst, x, maxY, z, (int)PEEK_FACES::TOP, minX, maxX, minY, maxY, minZ, maxZ, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
    //     }
    // }
    // for (int x = minX; x <= maxX; x++)
    // {
    //     for (int z = minZ; z <= maxZ; z+=lod)
    //     {
    //         floodFill<DCContextType>(inst, x, minY, z, (int)PEEK_FACES::BOTTOM, minX, maxX, minY, maxY, minZ, maxZ, peeks, seenPeeks, PEEK_FACE_INDICES, lod, size);
    //     }
    // }

    // EM_ASM({
    //     console.log("Error 2\n");
    // });

    // for (int startFace = 0; startFace < 6; startFace++)
    // {
    //     for (int endFace = 0; endFace < 6; endFace++)
    //     {
    //         if (endFace != startFace)
    //         {
    //             if (peeks[PEEK_FACE_INDICES[startFace << 3 | endFace]] == 1)
    //             {
    //                 peeks[PEEK_FACE_INDICES[endFace << 3 | startFace]] = 1;

    //                 for (int crossFace = 0; crossFace < 6; crossFace++)
    //                 {
    //                     if (crossFace != startFace && crossFace != endFace)
    //                     {
    //                         if (peeks[PEEK_FACE_INDICES[startFace << 3 | crossFace]] == 1)
    //                         {
    //                             peeks[PEEK_FACE_INDICES[crossFace << 3 | startFace]] = 1;
    //                             peeks[PEEK_FACE_INDICES[crossFace << 3 | endFace]] = 1;
    //                             peeks[PEEK_FACE_INDICES[endFace << 3 | crossFace]] = 1;
    //                         }
    //                         else if (peeks[PEEK_FACE_INDICES[endFace << 3 | crossFace]] == 1)
    //                         {
    //                             peeks[PEEK_FACE_INDICES[crossFace << 3 | startFace]] = 1;
    //                             peeks[PEEK_FACE_INDICES[crossFace << 3 | endFace]] = 1;
    //                             peeks[PEEK_FACE_INDICES[startFace << 3 | crossFace]] = 1;
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
}

#endif
