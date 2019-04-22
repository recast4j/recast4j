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
package org.recast4j.recast;

import static org.recast4j.recast.RecastCommon.GetCon;
import static org.recast4j.recast.RecastCommon.GetDirOffsetX;
import static org.recast4j.recast.RecastCommon.GetDirOffsetY;
import static org.recast4j.recast.RecastConstants.RC_NOT_CONNECTED;
import static org.recast4j.recast.RecastConstants.RC_NULL_AREA;
import static org.recast4j.recast.RecastVectors.copy;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.recast4j.recast.HeightfieldLayerSet.HeightfieldLayer;
import org.recast4j.recast.RecastRegion.SweepSpan;

public class RecastLayers {

    static final int RC_MAX_LAYERS = RecastConstants.RC_NOT_CONNECTED;
    static final int RC_MAX_NEIS = 16;

    static class LayerRegion {
        int id;
        int layerId;
        boolean base;
        int ymin, ymax;
        List<Integer> layers;
        List<Integer> neis;

        LayerRegion(int i) {
            id = i;
            ymin = 0xFFFF;
            layerId = 0xff;
            layers = new ArrayList<>();
            neis = new ArrayList<>();
        }

    };

    private static void addUnique(List<Integer> a, int v) {
        if (!a.contains(v)) {
            a.add(v);
        }
    }

    private static boolean contains(List<Integer> a, int v) {
        return a.contains(v);
    }

    private static boolean overlapRange(int amin, int amax, int bmin, int bmax) {
        return (amin > bmax || amax < bmin) ? false : true;
    }

    public static HeightfieldLayerSet buildHeightfieldLayers(Context ctx, CompactHeightfield chf, int borderSize,
            int walkableHeight) {

        ctx.startTimer("RC_TIMER_BUILD_LAYERS");
        int w = chf.width;
        int h = chf.height;
        int[] srcReg = new int[chf.spanCount];
        Arrays.fill(srcReg, 0xFF);
        int nsweeps = chf.width;// Math.max(chf.width, chf.height);
        SweepSpan[] sweeps = new SweepSpan[nsweeps];
        for (int i = 0; i < sweeps.length; i++) {
            sweeps[i] = new SweepSpan();
        }
        // Partition walkable area into monotone regions.
        int[] prevCount = new int[256];
        int regId = 0;
        // Sweep one line at a time.
        for (int y = borderSize; y < h - borderSize; ++y) {
            // Collect spans from this row.
            Arrays.fill(prevCount, 0, regId, 0);
            int sweepId = 0;

            for (int x = borderSize; x < w - borderSize; ++x) {
                CompactCell c = chf.cells[x + y * w];

                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    if (chf.areas[i] == RC_NULL_AREA)
                        continue;
                    int sid = 0xFF;
                    // -x

                    if (GetCon(s, 0) != RC_NOT_CONNECTED) {
                        int ax = x + GetDirOffsetX(0);
                        int ay = y + GetDirOffsetY(0);
                        int ai = chf.cells[ax + ay * w].index + GetCon(s, 0);
                        if (chf.areas[ai] != RC_NULL_AREA && srcReg[ai] != 0xff)
                            sid = srcReg[ai];
                    }

                    if (sid == 0xff) {
                        sid = sweepId++;
                        sweeps[sid].nei = 0xff;
                        sweeps[sid].ns = 0;
                    }

                    // -y
                    if (GetCon(s, 3) != RC_NOT_CONNECTED) {
                        int ax = x + GetDirOffsetX(3);
                        int ay = y + GetDirOffsetY(3);
                        int ai = chf.cells[ax + ay * w].index + GetCon(s, 3);
                        int nr = srcReg[ai];
                        if (nr != 0xff) {
                            // Set neighbour when first valid neighbour is
                            // encoutered.
                            if (sweeps[sid].ns == 0)
                                sweeps[sid].nei = nr;

                            if (sweeps[sid].nei == nr) {
                                // Update existing neighbour
                                sweeps[sid].ns++;
                                prevCount[nr]++;
                            } else {
                                // This is hit if there is nore than one
                                // neighbour.
                                // Invalidate the neighbour.
                                sweeps[sid].nei = 0xff;
                            }
                        }
                    }

                    srcReg[i] = sid;
                }
            }

            // Create unique ID.
            for (int i = 0; i < sweepId; ++i) {
                // If the neighbour is set and there is only one continuous
                // connection to it,
                // the sweep will be merged with the previous one, else new
                // region is created.
                if (sweeps[i].nei != 0xff && prevCount[sweeps[i].nei] == sweeps[i].ns) {
                    sweeps[i].id = sweeps[i].nei;
                } else {
                    if (regId == 255) {
                        throw new RuntimeException("rcBuildHeightfieldLayers: Region ID overflow.");
                    }
                    sweeps[i].id = regId++;
                }
            }

            // Remap local sweep ids to region ids.
            for (int x = borderSize; x < w - borderSize; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    if (srcReg[i] != 0xff)
                        srcReg[i] = sweeps[srcReg[i]].id;
                }
            }
        }
        int nregs = regId;
        LayerRegion[] regs = new LayerRegion[nregs];

        // Construct regions
        for (int i = 0; i < nregs; ++i) {
            regs[i] = new LayerRegion(i);
        }

        // Find region neighbours and overlapping regions.
        List<Integer> lregs = new ArrayList<>();
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];

                lregs.clear();

                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    int ri = srcReg[i];
                    if (ri == 0xff)
                        continue;

                    regs[ri].ymin = Math.min(regs[ri].ymin, s.y);
                    regs[ri].ymax = Math.max(regs[ri].ymax, s.y);

                    // Collect all region layers.
                    lregs.add(ri);

                    // Update neighbours
                    for (int dir = 0; dir < 4; ++dir) {
                        if (GetCon(s, dir) != RC_NOT_CONNECTED) {
                            int ax = x + GetDirOffsetX(dir);
                            int ay = y + GetDirOffsetY(dir);
                            int ai = chf.cells[ax + ay * w].index + GetCon(s, dir);
                            int rai = srcReg[ai];
                            if (rai != 0xff && rai != ri)
                                addUnique(regs[ri].neis, rai);
                        }
                    }

                }

                // Update overlapping regions.
                for (int i = 0; i < lregs.size() - 1; ++i) {
                    for (int j = i + 1; j < lregs.size(); ++j) {
                        if (lregs.get(i).intValue() != lregs.get(j).intValue()) {
                            LayerRegion ri = regs[lregs.get(i)];
                            LayerRegion rj = regs[lregs.get(j)];
                            addUnique(ri.layers, lregs.get(j));
                            addUnique(rj.layers, lregs.get(i));
                        }
                    }
                }

            }
        }

        // Create 2D layers from regions.
        int layerId = 0;

        List<Integer> stack = new ArrayList<>();

        for (int i = 0; i < nregs; ++i) {
            LayerRegion root = regs[i];
            // Skip already visited.
            if (root.layerId != 0xff)
                continue;

            // Start search.
            root.layerId = layerId;
            root.base = true;

            stack.add(i);

            while (!stack.isEmpty()) {
                // Pop front
                LayerRegion reg = regs[stack.remove(0)];

                for (int nei : reg.neis) {
                    LayerRegion regn = regs[nei];
                    // Skip already visited.
                    if (regn.layerId != 0xff)
                        continue;
                    // Skip if the neighbour is overlapping root region.
                    if (contains(root.layers, nei))
                        continue;
                    // Skip if the height range would become too large.
                    int ymin = Math.min(root.ymin, regn.ymin);
                    int ymax = Math.max(root.ymax, regn.ymax);
                    if ((ymax - ymin) >= 255)
                        continue;

                    // Deepen
                    stack.add(nei);

                    // Mark layer id
                    regn.layerId = layerId;
                    // Merge current layers to root.
                    for (int layer : regn.layers)
                        addUnique(root.layers, layer);
                    root.ymin = Math.min(root.ymin, regn.ymin);
                    root.ymax = Math.max(root.ymax, regn.ymax);
                }
            }

            layerId++;
        }

        // Merge non-overlapping regions that are close in height.
        int mergeHeight = walkableHeight * 4;

        for (int i = 0; i < nregs; ++i) {
            LayerRegion ri = regs[i];
            if (!ri.base)
                continue;

            int newId = ri.layerId;

            for (;;) {
                int oldId = 0xff;

                for (int j = 0; j < nregs; ++j) {
                    if (i == j)
                        continue;
                    LayerRegion rj = regs[j];
                    if (!rj.base)
                        continue;

                    // Skip if the regions are not close to each other.
                    if (!overlapRange(ri.ymin, ri.ymax + mergeHeight, rj.ymin, rj.ymax + mergeHeight))
                        continue;
                    // Skip if the height range would become too large.
                    int ymin = Math.min(ri.ymin, rj.ymin);
                    int ymax = Math.max(ri.ymax, rj.ymax);
                    if ((ymax - ymin) >= 255)
                        continue;

                    // Make sure that there is no overlap when merging 'ri' and
                    // 'rj'.
                    boolean overlap = false;
                    // Iterate over all regions which have the same layerId as
                    // 'rj'
                    for (int k = 0; k < nregs; ++k) {
                        if (regs[k].layerId != rj.layerId)
                            continue;
                        // Check if region 'k' is overlapping region 'ri'
                        // Index to 'regs' is the same as region id.
                        if (contains(ri.layers, k)) {
                            overlap = true;
                            break;
                        }
                    }
                    // Cannot merge of regions overlap.
                    if (overlap)
                        continue;

                    // Can merge i and j.
                    oldId = rj.layerId;
                    break;
                }

                // Could not find anything to merge with, stop.
                if (oldId == 0xff)
                    break;

                // Merge
                for (int j = 0; j < nregs; ++j) {
                    LayerRegion rj = regs[j];
                    if (rj.layerId == oldId) {
                        rj.base = false;
                        // Remap layerIds.
                        rj.layerId = newId;
                        // Add overlaid layers from 'rj' to 'ri'.
                        for (int layer : rj.layers)
                            addUnique(ri.layers, layer);
                        // Update height bounds.
                        ri.ymin = Math.min(ri.ymin, rj.ymin);
                        ri.ymax = Math.max(ri.ymax, rj.ymax);
                    }
                }
            }
        }

        // Compact layerIds
        int[] remap = new int[256];

        // Find number of unique layers.
        layerId = 0;
        for (int i = 0; i < nregs; ++i)
            remap[regs[i].layerId] = 1;
        for (int i = 0; i < 256; ++i) {
            if (remap[i] != 0)
                remap[i] = layerId++;
            else
                remap[i] = 0xff;
        }
        // Remap ids.
        for (int i = 0; i < nregs; ++i)
            regs[i].layerId = remap[regs[i].layerId];

        // No layers, return empty.
        if (layerId == 0) {
            // ctx.stopTimer(RC_TIMER_BUILD_LAYERS);
            return null;
        }

        // Create layers.
        // rcAssert(lset.layers == 0);

        int lw = w - borderSize * 2;
        int lh = h - borderSize * 2;

        // Build contracted bbox for layers.
        float[] bmin = new float[3];
        float[] bmax = new float[3];
        copy(bmin, chf.bmin);
        copy(bmax, chf.bmax);
        bmin[0] += borderSize * chf.cs;
        bmin[2] += borderSize * chf.cs;
        bmax[0] -= borderSize * chf.cs;
        bmax[2] -= borderSize * chf.cs;

        HeightfieldLayerSet lset = new HeightfieldLayerSet();
        lset.layers = new HeightfieldLayer[layerId];
        for (int i = 0; i < lset.layers.length; i++) {
            lset.layers[i] = new HeightfieldLayer();
        }

        // Store layers.
        for (int i = 0; i < lset.layers.length; ++i) {
            int curId = i;

            HeightfieldLayer layer = lset.layers[i];

            int gridSize = lw * lh;

            layer.heights = new int[gridSize];
            Arrays.fill(layer.heights, 0xFF);
            layer.areas = new int[gridSize];
            layer.cons = new int[gridSize];

            // Find layer height bounds.
            int hmin = 0, hmax = 0;
            for (int j = 0; j < nregs; ++j) {
                if (regs[j].base && regs[j].layerId == curId) {
                    hmin = regs[j].ymin;
                    hmax = regs[j].ymax;
                }
            }

            layer.width = lw;
            layer.height = lh;
            layer.cs = chf.cs;
            layer.ch = chf.ch;

            // Adjust the bbox to fit the heightfield.
            copy(layer.bmin, bmin);
            copy(layer.bmax, bmax);
            layer.bmin[1] = bmin[1] + hmin * chf.ch;
            layer.bmax[1] = bmin[1] + hmax * chf.ch;
            layer.hmin = hmin;
            layer.hmax = hmax;

            // Update usable data region.
            layer.minx = layer.width;
            layer.maxx = 0;
            layer.miny = layer.height;
            layer.maxy = 0;

            // Copy height and area from compact heightfield.
            for (int y = 0; y < lh; ++y) {
                for (int x = 0; x < lw; ++x) {
                    int cx = borderSize + x;
                    int cy = borderSize + y;
                    CompactCell c = chf.cells[cx + cy * w];
                    for (int j = c.index, nj = c.index + c.count; j < nj; ++j) {
                        CompactSpan s = chf.spans[j];
                        // Skip unassigned regions.
                        if (srcReg[j] == 0xff)
                            continue;
                        // Skip of does nto belong to current layer.
                        int lid = regs[srcReg[j]].layerId;
                        if (lid != curId)
                            continue;

                        // Update data bounds.
                        layer.minx = Math.min(layer.minx, x);
                        layer.maxx = Math.max(layer.maxx, x);
                        layer.miny = Math.min(layer.miny, y);
                        layer.maxy = Math.max(layer.maxy, y);

                        // Store height and area type.
                        int idx = x + y * lw;
                        layer.heights[idx] = (char) (s.y - hmin);
                        layer.areas[idx] = chf.areas[j];

                        // Check connection.
                        char portal = 0;
                        char con = 0;
                        for (int dir = 0; dir < 4; ++dir) {
                            if (GetCon(s, dir) != RC_NOT_CONNECTED) {
                                int ax = cx + GetDirOffsetX(dir);
                                int ay = cy + GetDirOffsetY(dir);
                                int ai = chf.cells[ax + ay * w].index + GetCon(s, dir);
                                int alid = srcReg[ai] != 0xff ? regs[srcReg[ai]].layerId : 0xff;
                                // Portal mask
                                if (chf.areas[ai] != RC_NULL_AREA && lid != alid) {
                                    portal |= (1 << dir);
                                    // Update height so that it matches on both
                                    // sides of the portal.
                                    CompactSpan as = chf.spans[ai];
                                    if (as.y > hmin)
                                        layer.heights[idx] = Math.max(layer.heights[idx], (char) (as.y - hmin));
                                }
                                // Valid connection mask
                                if (chf.areas[ai] != RC_NULL_AREA && lid == alid) {
                                    int nx = ax - borderSize;
                                    int ny = ay - borderSize;
                                    if (nx >= 0 && ny >= 0 && nx < lw && ny < lh)
                                        con |= (1 << dir);
                                }
                            }
                        }
                        layer.cons[idx] = (portal << 4) | con;
                    }
                }
            }

            if (layer.minx > layer.maxx)
                layer.minx = layer.maxx = 0;
            if (layer.miny > layer.maxy)
                layer.miny = layer.maxy = 0;
        }

        // ctx->stopTimer(RC_TIMER_BUILD_LAYERS);
        return lset;
    }
}
