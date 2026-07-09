/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.recast.geom;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class PartitionedMesh implements TriMesh {

    private final float[] verts;
    private final int[] tris;
    private final List<PartitionedMeshNode> nodes;

    private static class BoundsItem {
        private final float[] bmin = new float[2];
        private final float[] bmax = new float[2];
        private int i;
    }

    private static class PartitionedMeshNode {
        private final float[] bmin = new float[2];
        private final float[] bmax = new float[2];
        private int i;
        public int[] tris;
    }

    private class CompareItemX implements Comparator<BoundsItem> {
        @Override
        public int compare(BoundsItem a, BoundsItem b) {
            return Float.compare(a.bmin[0], b.bmin[0]);
        }
    }

    private class CompareItemY implements Comparator<BoundsItem> {
        @Override
        public int compare(BoundsItem a, BoundsItem b) {
            return Float.compare(a.bmin[1], b.bmin[1]);
        }
    }

    private void calcExtends(BoundsItem[] items, int imin, int imax, float[] bmin, float[] bmax) {
        bmin[0] = items[imin].bmin[0];
        bmin[1] = items[imin].bmin[1];

        bmax[0] = items[imin].bmax[0];
        bmax[1] = items[imin].bmax[1];

        for (int i = imin + 1; i < imax; ++i) {
            BoundsItem it = items[i];
            if (it.bmin[0] < bmin[0]) {
                bmin[0] = it.bmin[0];
            }
            if (it.bmin[1] < bmin[1]) {
                bmin[1] = it.bmin[1];
            }

            if (it.bmax[0] > bmax[0]) {
                bmax[0] = it.bmax[0];
            }
            if (it.bmax[1] > bmax[1]) {
                bmax[1] = it.bmax[1];
            }
        }
    }

    private int longestAxis(float x, float y) {
        return y > x ? 1 : 0;
    }

    private void subdivide(BoundsItem[] items, int imin, int imax, int trisPerChunk, List<PartitionedMeshNode> nodes,
            int[] inTris) {
        int inum = imax - imin;

        PartitionedMeshNode node = new PartitionedMeshNode();
        nodes.add(node);

        if (inum <= trisPerChunk) {
            // Leaf
            calcExtends(items, imin, imax, node.bmin, node.bmax);

            // Copy triangles.
            node.i = nodes.size();
            node.tris = new int[inum * 3];

            int dst = 0;
            for (int i = imin; i < imax; ++i) {
                int src = items[i].i * 3;
                node.tris[dst++] = inTris[src];
                node.tris[dst++] = inTris[src + 1];
                node.tris[dst++] = inTris[src + 2];
            }
        } else {
            // Split
            calcExtends(items, imin, imax, node.bmin, node.bmax);

            int axis = longestAxis(node.bmax[0] - node.bmin[0], node.bmax[1] - node.bmin[1]);

            int isplit = imin + inum / 2;
            NthElement.nthElement(items, imin, isplit, imax, axis == 0 ? new CompareItemX(): new CompareItemY());

            // Left
            subdivide(items, imin, isplit, trisPerChunk, nodes, inTris);
            // Right
            subdivide(items, isplit, imax, trisPerChunk, nodes, inTris);

            // Negative index means escape.
            node.i = -nodes.size();
        }
    }

    public PartitionedMesh(float[] verts, int[] tris) {
        this(verts, tris, 32);
    }

    public PartitionedMesh(float[] verts, int[] tris, int trisPerChunk) {
        this.verts = verts;
        this.tris = tris;
        int ntris = tris.length / 3;
        int nchunks = (ntris + trisPerChunk - 1) / trisPerChunk;

        nodes = new ArrayList<>(nchunks);

        // Build tree
        BoundsItem[] items = new BoundsItem[ntris];

        for (int i = 0; i < ntris; i++) {
            int t = i * 3;
            BoundsItem it = items[i] = new BoundsItem();
            it.i = i;
            // Calc triangle XZ bounds.
            it.bmin[0] = it.bmax[0] = verts[tris[t] * 3 + 0];
            it.bmin[1] = it.bmax[1] = verts[tris[t] * 3 + 2];
            for (int j = 1; j < 3; ++j) {
                int v = tris[t + j] * 3;
                if (verts[v] < it.bmin[0]) {
                    it.bmin[0] = verts[v];
                }
                if (verts[v + 2] < it.bmin[1]) {
                    it.bmin[1] = verts[v + 2];
                }

                if (verts[v] > it.bmax[0]) {
                    it.bmax[0] = verts[v];
                }
                if (verts[v + 2] > it.bmax[1]) {
                    it.bmax[1] = verts[v + 2];
                }
            }
        }

        subdivide(items, 0, ntris, trisPerChunk, nodes, tris);

    }

    private boolean checkOverlapRect(float[] amin, float[] amax, float[] bmin, float[] bmax) {
        boolean overlap = true;
        overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
        overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
        return overlap;
    }

    @Override
    public List<int[]> getChunksOverlappingRect(float[] bmin, float[] bmax) {
        List<int[]> ids = new ArrayList<>();
        int i = 0;
        while (i < nodes.size()) {
            PartitionedMeshNode node = nodes.get(i);
            boolean overlap = checkOverlapRect(bmin, bmax, node.bmin, node.bmax);
            boolean isLeafNode = node.i >= 0;

            if (isLeafNode && overlap) {
                ids.add(node.tris);
            }

            if (overlap || isLeafNode) {
                i++;
            } else {
                i = -node.i;
            }
        }
        return ids;
    }

    @Override
    public int[] getTris() {
        return tris;
    }

    @Override
    public float[] getVerts() {
        return verts;
    }

}
