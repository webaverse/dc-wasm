#include "newOctree.h"

uint64_t hashOctreeMin(const vm::ivec3 &min)
{
	uint64_t result = uint16_t(min.x);
	result = (result << 16) + uint16_t(min.y);
	result = (result << 16) + uint16_t(min.z);
	return result;
}