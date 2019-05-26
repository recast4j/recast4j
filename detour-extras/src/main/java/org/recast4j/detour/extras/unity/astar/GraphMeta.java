/*
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.detour.extras.unity.astar;

import org.recast4j.detour.extras.Vector3f;

class GraphMeta {
    float characterRadius;
    float contourMaxError;
    float cellSize;
    float walkableHeight;
    float walkableClimb;
    float maxSlope;
    float maxEdgeLength;
    float minRegionSize;
    /** Size of tile along X axis in voxels */
    float tileSizeX;
    /** Size of tile along Z axis in voxels */
    float tileSizeZ;
    boolean useTiles;
    Vector3f rotation;
    Vector3f forcedBoundsCenter;
    Vector3f forcedBoundsSize;
}
