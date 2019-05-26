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

import org.recast4j.detour.MeshData;
import org.recast4j.detour.Poly;

class GraphMeshData {

    final int tileXCount;
    final int tileZCount;

    final MeshData[] tiles;

    GraphMeshData(int tileXCount, int tileZCount, MeshData[] tiles) {
        this.tileXCount = tileXCount;
        this.tileZCount = tileZCount;
        this.tiles = tiles;
    }

    int countNodes() {
        int polyCount = 0;
        for (MeshData t : tiles) {
            polyCount += t.header.polyCount;
        }
        return polyCount;
    }

    public Poly getNode(int node) {
        int index = 0;
        for (MeshData t : tiles) {
            if (node - index >= 0 && node - index < t.header.polyCount) {
                return t.polys[node - index];
            }
            index += t.header.polyCount;
        }
        return null;
    }

    public MeshData getTile(int node) {
        int index = 0;
        for (MeshData t : tiles) {
            if (node - index >= 0 && node - index < t.header.polyCount) {
                return t;
            }
            index += t.header.polyCount;
        }
        return null;
    }

}
