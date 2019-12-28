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

import static org.recast4j.recast.RecastConstants.RC_BORDER_REG;
import static org.recast4j.recast.RecastConstants.RC_NOT_CONNECTED;
import static org.recast4j.recast.RecastConstants.RC_NULL_AREA;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RecastRegion {

    static final int RC_NULL_NEI = 0xffff;

    static class SweepSpan {
        int rid; // row id
        int id; // region id
        int ns; // number samples
        int nei; // neighbour id
    }

    public static int calculateDistanceField(CompactHeightfield chf, int[] src) {
        int maxDist;
        int w = chf.width;
        int h = chf.height;

        // Init distance and points.
        for (int i = 0; i < chf.spanCount; ++i) {
            src[i] = 0xffff;
        }

        // Mark boundary cells.
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    int area = chf.areas[i];

                    int nc = 0;
                    for (int dir = 0; dir < 4; ++dir) {
                        if (RecastCommon.GetCon(s, dir) != RC_NOT_CONNECTED) {
                            int ax = x + RecastCommon.GetDirOffsetX(dir);
                            int ay = y + RecastCommon.GetDirOffsetY(dir);
                            int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, dir);
                            if (area == chf.areas[ai]) {
                                nc++;
                            }
                        }
                    }
                    if (nc != 4) {
                        src[i] = 0;
                    }
                }
            }
        }

        // Pass 1
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];

                    if (RecastCommon.GetCon(s, 0) != RC_NOT_CONNECTED) {
                        // (-1,0)
                        int ax = x + RecastCommon.GetDirOffsetX(0);
                        int ay = y + RecastCommon.GetDirOffsetY(0);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, 0);
                        CompactSpan as = chf.spans[ai];
                        if (src[ai] + 2 < src[i]) {
                            src[i] = src[ai] + 2;
                        }

                        // (-1,-1)
                        if (RecastCommon.GetCon(as, 3) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.GetDirOffsetX(3);
                            int aay = ay + RecastCommon.GetDirOffsetY(3);
                            int aai = chf.cells[aax + aay * w].index + RecastCommon.GetCon(as, 3);
                            if (src[aai] + 3 < src[i]) {
                                src[i] = src[aai] + 3;
                            }
                        }
                    }
                    if (RecastCommon.GetCon(s, 3) != RC_NOT_CONNECTED) {
                        // (0,-1)
                        int ax = x + RecastCommon.GetDirOffsetX(3);
                        int ay = y + RecastCommon.GetDirOffsetY(3);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, 3);
                        CompactSpan as = chf.spans[ai];
                        if (src[ai] + 2 < src[i]) {
                            src[i] = src[ai] + 2;
                        }

                        // (1,-1)
                        if (RecastCommon.GetCon(as, 2) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.GetDirOffsetX(2);
                            int aay = ay + RecastCommon.GetDirOffsetY(2);
                            int aai = chf.cells[aax + aay * w].index + RecastCommon.GetCon(as, 2);
                            if (src[aai] + 3 < src[i]) {
                                src[i] = src[aai] + 3;
                            }
                        }
                    }
                }
            }
        }

        // Pass 2
        for (int y = h - 1; y >= 0; --y) {
            for (int x = w - 1; x >= 0; --x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];

                    if (RecastCommon.GetCon(s, 2) != RC_NOT_CONNECTED) {
                        // (1,0)
                        int ax = x + RecastCommon.GetDirOffsetX(2);
                        int ay = y + RecastCommon.GetDirOffsetY(2);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, 2);
                        CompactSpan as = chf.spans[ai];
                        if (src[ai] + 2 < src[i]) {
                            src[i] = src[ai] + 2;
                        }

                        // (1,1)
                        if (RecastCommon.GetCon(as, 1) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.GetDirOffsetX(1);
                            int aay = ay + RecastCommon.GetDirOffsetY(1);
                            int aai = chf.cells[aax + aay * w].index + RecastCommon.GetCon(as, 1);
                            if (src[aai] + 3 < src[i]) {
                                src[i] = src[aai] + 3;
                            }
                        }
                    }
                    if (RecastCommon.GetCon(s, 1) != RC_NOT_CONNECTED) {
                        // (0,1)
                        int ax = x + RecastCommon.GetDirOffsetX(1);
                        int ay = y + RecastCommon.GetDirOffsetY(1);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, 1);
                        CompactSpan as = chf.spans[ai];
                        if (src[ai] + 2 < src[i]) {
                            src[i] = src[ai] + 2;
                        }

                        // (-1,1)
                        if (RecastCommon.GetCon(as, 0) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.GetDirOffsetX(0);
                            int aay = ay + RecastCommon.GetDirOffsetY(0);
                            int aai = chf.cells[aax + aay * w].index + RecastCommon.GetCon(as, 0);
                            if (src[aai] + 3 < src[i]) {
                                src[i] = src[aai] + 3;
                            }
                        }
                    }
                }
            }
        }

        maxDist = 0;
        for (int i = 0; i < chf.spanCount; ++i) {
            maxDist = Math.max(src[i], maxDist);
        }

        return maxDist;
    }

    private static int[] boxBlur(CompactHeightfield chf, int thr, int[] src) {
        int w = chf.width;
        int h = chf.height;
        int[] dst = new int[chf.spanCount];

        thr *= 2;

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    int cd = src[i];
                    if (cd <= thr) {
                        dst[i] = cd;
                        continue;
                    }

                    int d = cd;
                    for (int dir = 0; dir < 4; ++dir) {
                        if (RecastCommon.GetCon(s, dir) != RC_NOT_CONNECTED) {
                            int ax = x + RecastCommon.GetDirOffsetX(dir);
                            int ay = y + RecastCommon.GetDirOffsetY(dir);
                            int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, dir);
                            d += src[ai];

                            CompactSpan as = chf.spans[ai];
                            int dir2 = (dir + 1) & 0x3;
                            if (RecastCommon.GetCon(as, dir2) != RC_NOT_CONNECTED) {
                                int ax2 = ax + RecastCommon.GetDirOffsetX(dir2);
                                int ay2 = ay + RecastCommon.GetDirOffsetY(dir2);
                                int ai2 = chf.cells[ax2 + ay2 * w].index + RecastCommon.GetCon(as, dir2);
                                d += src[ai2];
                            } else {
                                d += cd;
                            }
                        } else {
                            d += cd * 2;
                        }
                    }
                    dst[i] = ((d + 5) / 9);
                }
            }
        }
        return dst;
    }

    private static boolean floodRegion(int x, int y, int i, int level, int r, CompactHeightfield chf, int[] srcReg,
            int[] srcDist, List<Integer> stack) {
        int w = chf.width;

        int area = chf.areas[i];

        // Flood fill mark region.
        stack.clear();
        stack.add(x);
        stack.add(y);
        stack.add(i);
        srcReg[i] = r;
        srcDist[i] = 0;

        int lev = level >= 2 ? level - 2 : 0;
        int count = 0;

        while (stack.size() > 0) {
            int ci = stack.remove(stack.size() - 1);
            int cy = stack.remove(stack.size() - 1);
            int cx = stack.remove(stack.size() - 1);

            CompactSpan cs = chf.spans[ci];

            // Check if any of the neighbours already have a valid region set.
            int ar = 0;
            for (int dir = 0; dir < 4; ++dir) {
                // 8 connected
                if (RecastCommon.GetCon(cs, dir) != RC_NOT_CONNECTED) {
                    int ax = cx + RecastCommon.GetDirOffsetX(dir);
                    int ay = cy + RecastCommon.GetDirOffsetY(dir);
                    int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(cs, dir);
                    if (chf.areas[ai] != area) {
                        continue;
                    }
                    int nr = srcReg[ai];
                    if ((nr & RC_BORDER_REG) != 0) {
                        continue;
                    }
                    if (nr != 0 && nr != r) {
                        ar = nr;
                        break;
                    }

                    CompactSpan as = chf.spans[ai];

                    int dir2 = (dir + 1) & 0x3;
                    if (RecastCommon.GetCon(as, dir2) != RC_NOT_CONNECTED) {
                        int ax2 = ax + RecastCommon.GetDirOffsetX(dir2);
                        int ay2 = ay + RecastCommon.GetDirOffsetY(dir2);
                        int ai2 = chf.cells[ax2 + ay2 * w].index + RecastCommon.GetCon(as, dir2);
                        if (chf.areas[ai2] != area) {
                            continue;
                        }
                        int nr2 = srcReg[ai2];
                        if (nr2 != 0 && nr2 != r) {
                            ar = nr2;
                            break;
                        }
                    }
                }
            }
            if (ar != 0) {
                srcReg[ci] = 0;
                continue;
            }

            count++;

            // Expand neighbours.
            for (int dir = 0; dir < 4; ++dir) {
                if (RecastCommon.GetCon(cs, dir) != RC_NOT_CONNECTED) {
                    int ax = cx + RecastCommon.GetDirOffsetX(dir);
                    int ay = cy + RecastCommon.GetDirOffsetY(dir);
                    int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(cs, dir);
                    if (chf.areas[ai] != area) {
                        continue;
                    }
                    if (chf.dist[ai] >= lev && srcReg[ai] == 0) {
                        srcReg[ai] = r;
                        srcDist[ai] = 0;
                        stack.add(ax);
                        stack.add(ay);
                        stack.add(ai);
                    }
                }
            }
        }

        return count > 0;
    }

    private static int[] expandRegions(int maxIter, int level, CompactHeightfield chf, int[] srcReg, int[] srcDist,
            List<Integer> stack, boolean fillStack) {
        int w = chf.width;
        int h = chf.height;

        if (fillStack) {
            // Find cells revealed by the raised level.
            stack.clear();
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    CompactCell c = chf.cells[x + y * w];
                    for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                        if (chf.dist[i] >= level && srcReg[i] == 0 && chf.areas[i] != RC_NULL_AREA) {
                            stack.add(x);
                            stack.add(y);
                            stack.add(i);
                        }
                    }
                }
            }
        } else // use cells in the input stack
        {
            // mark all cells which already have a region
            for (int j = 0; j < stack.size(); j += 3) {
                int i = stack.get(j + 2);
                if (srcReg[i] != 0) {
                    stack.set(j + 2, -1);
                }
            }
        }

        List<Integer> dirtyEntries = new ArrayList<>();
        int iter = 0;
        while (stack.size() > 0) {
            int failed = 0;
            dirtyEntries.clear();

            for (int j = 0; j < stack.size(); j += 3) {
                int x = stack.get(j + 0);
                int y = stack.get(j + 1);
                int i = stack.get(j + 2);
                if (i < 0) {
                    failed++;
                    continue;
                }

                int r = srcReg[i];
                int d2 = 0xffff;
                int area = chf.areas[i];
                CompactSpan s = chf.spans[i];
                for (int dir = 0; dir < 4; ++dir) {
                    if (RecastCommon.GetCon(s, dir) == RC_NOT_CONNECTED) {
                        continue;
                    }
                    int ax = x + RecastCommon.GetDirOffsetX(dir);
                    int ay = y + RecastCommon.GetDirOffsetY(dir);
                    int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, dir);
                    if (chf.areas[ai] != area) {
                        continue;
                    }
                    if (srcReg[ai] > 0 && (srcReg[ai] & RC_BORDER_REG) == 0) {
                        if (srcDist[ai] + 2 < d2) {
                            r = srcReg[ai];
                            d2 = srcDist[ai] + 2;
                        }
                    }
                }
                if (r != 0) {
                    stack.set(j + 2, -1); // mark as used
                    dirtyEntries.add(i);
                    dirtyEntries.add(r);
                    dirtyEntries.add(d2);
                } else {
                    failed++;
                }
            }

            // Copy entries that differ between src and dst to keep them in sync.
            for (int i = 0; i < dirtyEntries.size(); i += 3) {
                int idx = dirtyEntries.get(i);
                srcReg[idx] = dirtyEntries.get(i + 1);
                srcDist[idx] = dirtyEntries.get(i + 2);
            }

            if (failed * 3 == stack.size()) {
                break;
            }

            if (level > 0) {
                ++iter;
                if (iter >= maxIter) {
                    break;
                }
            }
        }

        return srcReg;
    }

    private static void sortCellsByLevel(int startLevel, CompactHeightfield chf, int[] srcReg, int nbStacks,
            List<List<Integer>> stacks, int loglevelsPerStack) // the levels per stack (2 in our case) as a bit shift
    {
        int w = chf.width;
        int h = chf.height;
        startLevel = startLevel >> loglevelsPerStack;

        for (int j = 0; j < nbStacks; ++j) {
            stacks.get(j).clear();
        }
        ;

        // put all cells in the level range into the appropriate stacks
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    if (chf.areas[i] == RC_NULL_AREA || srcReg[i] != 0) {
                        continue;
                    }

                    int level = chf.dist[i] >> loglevelsPerStack;
                    int sId = startLevel - level;
                    if (sId >= nbStacks) {
                        continue;
                    }
                    if (sId < 0) {
                        sId = 0;
                    }

                    stacks.get(sId).add(x);
                    stacks.get(sId).add(y);
                    stacks.get(sId).add(i);
                }
            }
        }
    }

    private static void appendStacks(List<Integer> srcStack, List<Integer> dstStack, int[] srcReg) {
        for (int j = 0; j < srcStack.size(); j += 3) {
            int i = srcStack.get(j + 2);
            if ((i < 0) || (srcReg[i] != 0)) {
                continue;
            }
            dstStack.add(srcStack.get(j));
            dstStack.add(srcStack.get(j + 1));
            dstStack.add(srcStack.get(j + 2));
        }
    }

    static class Region {
        int spanCount; // Number of spans belonging to this region
        int id; // ID of the region
        int areaType; // Are type.
        boolean remap;
        boolean visited;
        boolean overlap;
        boolean connectsToBorder;
        int ymin, ymax;
        List<Integer> connections;
        List<Integer> floors;

        Region(int i) {
            id = i;
            ymin = 0xFFFF;
            connections = new ArrayList<>();
            floors = new ArrayList<>();
        }

    };

    private static void removeAdjacentNeighbours(Region reg) {
        // Remove adjacent duplicates.
        for (int i = 0; i < reg.connections.size() && reg.connections.size() > 1;) {
            int ni = (i + 1) % reg.connections.size();
            if (reg.connections.get(i) == reg.connections.get(ni)) {
                reg.connections.remove(i);
            } else {
                ++i;
            }
        }
    }

    private static void replaceNeighbour(Region reg, int oldId, int newId) {
        boolean neiChanged = false;
        for (int i = 0; i < reg.connections.size(); ++i) {
            if (reg.connections.get(i) == oldId) {
                reg.connections.set(i, newId);
                neiChanged = true;
            }
        }
        for (int i = 0; i < reg.floors.size(); ++i) {
            if (reg.floors.get(i) == oldId) {
                reg.floors.set(i, newId);
            }
        }
        if (neiChanged) {
            removeAdjacentNeighbours(reg);
        }
    }

    private static boolean canMergeWithRegion(Region rega, Region regb) {
        if (rega.areaType != regb.areaType) {
            return false;
        }
        int n = 0;
        for (int i = 0; i < rega.connections.size(); ++i) {
            if (rega.connections.get(i) == regb.id) {
                n++;
            }
        }
        if (n > 1) {
            return false;
        }
        for (int i = 0; i < rega.floors.size(); ++i) {
            if (rega.floors.get(i) == regb.id) {
                return false;
            }
        }
        return true;
    }

    private static void addUniqueFloorRegion(Region reg, int n) {
        if (!reg.floors.contains(n)) {
            reg.floors.add(n);
        }
    }

    private static boolean mergeRegions(Region rega, Region regb) {
        int aid = rega.id;
        int bid = regb.id;

        // Duplicate current neighbourhood.
        List<Integer> acon = new ArrayList<>(rega.connections);
        List<Integer> bcon = regb.connections;

        // Find insertion point on A.
        int insa = -1;
        for (int i = 0; i < acon.size(); ++i) {
            if (acon.get(i) == bid) {
                insa = i;
                break;
            }
        }
        if (insa == -1) {
            return false;
        }

        // Find insertion point on B.
        int insb = -1;
        for (int i = 0; i < bcon.size(); ++i) {
            if (bcon.get(i) == aid) {
                insb = i;
                break;
            }
        }
        if (insb == -1) {
            return false;
        }

        // Merge neighbours.
        rega.connections.clear();
        for (int i = 0, ni = acon.size(); i < ni - 1; ++i) {
            rega.connections.add(acon.get((insa + 1 + i) % ni));
        }

        for (int i = 0, ni = bcon.size(); i < ni - 1; ++i) {
            rega.connections.add(bcon.get((insb + 1 + i) % ni));
        }

        removeAdjacentNeighbours(rega);

        for (int j = 0; j < regb.floors.size(); ++j) {
            addUniqueFloorRegion(rega, regb.floors.get(j));
        }
        rega.spanCount += regb.spanCount;
        regb.spanCount = 0;
        regb.connections.clear();

        return true;
    }

    private static boolean isRegionConnectedToBorder(Region reg) {
        // Region is connected to border if
        // one of the neighbours is null id.
        return reg.connections.contains(0);
    }

    private static boolean isSolidEdge(CompactHeightfield chf, int[] srcReg, int x, int y, int i, int dir) {
        CompactSpan s = chf.spans[i];
        int r = 0;
        if (RecastCommon.GetCon(s, dir) != RC_NOT_CONNECTED) {
            int ax = x + RecastCommon.GetDirOffsetX(dir);
            int ay = y + RecastCommon.GetDirOffsetY(dir);
            int ai = chf.cells[ax + ay * chf.width].index + RecastCommon.GetCon(s, dir);
            r = srcReg[ai];
        }
        if (r == srcReg[i]) {
            return false;
        }
        return true;
    }

    private static void walkContour(int x, int y, int i, int dir, CompactHeightfield chf, int[] srcReg,
            List<Integer> cont) {
        int startDir = dir;
        int starti = i;

        CompactSpan ss = chf.spans[i];
        int curReg = 0;
        if (RecastCommon.GetCon(ss, dir) != RC_NOT_CONNECTED) {
            int ax = x + RecastCommon.GetDirOffsetX(dir);
            int ay = y + RecastCommon.GetDirOffsetY(dir);
            int ai = chf.cells[ax + ay * chf.width].index + RecastCommon.GetCon(ss, dir);
            curReg = srcReg[ai];
        }
        cont.add(curReg);

        int iter = 0;
        while (++iter < 40000) {
            CompactSpan s = chf.spans[i];

            if (isSolidEdge(chf, srcReg, x, y, i, dir)) {
                // Choose the edge corner
                int r = 0;
                if (RecastCommon.GetCon(s, dir) != RC_NOT_CONNECTED) {
                    int ax = x + RecastCommon.GetDirOffsetX(dir);
                    int ay = y + RecastCommon.GetDirOffsetY(dir);
                    int ai = chf.cells[ax + ay * chf.width].index + RecastCommon.GetCon(s, dir);
                    r = srcReg[ai];
                }
                if (r != curReg) {
                    curReg = r;
                    cont.add(curReg);
                }

                dir = (dir + 1) & 0x3; // Rotate CW
            } else {
                int ni = -1;
                int nx = x + RecastCommon.GetDirOffsetX(dir);
                int ny = y + RecastCommon.GetDirOffsetY(dir);
                if (RecastCommon.GetCon(s, dir) != RC_NOT_CONNECTED) {
                    CompactCell nc = chf.cells[nx + ny * chf.width];
                    ni = nc.index + RecastCommon.GetCon(s, dir);
                }
                if (ni == -1) {
                    // Should not happen.
                    return;
                }
                x = nx;
                y = ny;
                i = ni;
                dir = (dir + 3) & 0x3; // Rotate CCW
            }

            if (starti == i && startDir == dir) {
                break;
            }
        }

        // Remove adjacent duplicates.
        if (cont.size() > 1) {
            for (int j = 0; j < cont.size();) {
                int nj = (j + 1) % cont.size();
                if (cont.get(j) == cont.get(nj)) {
                    cont.remove(j);
                } else {
                    ++j;
                }
            }
        }
    }

    private static int mergeAndFilterRegions(Context ctx, int minRegionArea, int mergeRegionSize, int maxRegionId,
            CompactHeightfield chf, int[] srcReg, List<Integer> overlaps) {
        int w = chf.width;
        int h = chf.height;

        int nreg = maxRegionId + 1;
        Region[] regions = new Region[nreg];

        // Construct regions
        for (int i = 0; i < nreg; ++i) {
            regions[i] = new Region(i);
        }

        // Find edge of a region and find connections around the contour.
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    int r = srcReg[i];
                    if (r == 0 || r >= nreg) {
                        continue;
                    }

                    Region reg = regions[r];
                    reg.spanCount++;

                    // Update floors.
                    for (int j = c.index; j < ni; ++j) {
                        if (i == j) {
                            continue;
                        }
                        int floorId = srcReg[j];
                        if (floorId == 0 || floorId >= nreg) {
                            continue;
                        }
                        if (floorId == r) {
                            reg.overlap = true;
                        }
                        addUniqueFloorRegion(reg, floorId);
                    }

                    // Have found contour
                    if (reg.connections.size() > 0) {
                        continue;
                    }

                    reg.areaType = chf.areas[i];

                    // Check if this cell is next to a border.
                    int ndir = -1;
                    for (int dir = 0; dir < 4; ++dir) {
                        if (isSolidEdge(chf, srcReg, x, y, i, dir)) {
                            ndir = dir;
                            break;
                        }
                    }

                    if (ndir != -1) {
                        // The cell is at border.
                        // Walk around the contour to find all the neighbours.
                        walkContour(x, y, i, ndir, chf, srcReg, reg.connections);
                    }
                }
            }
        }

        // Remove too small regions.
        List<Integer> stack = new ArrayList<>(32);
        List<Integer> trace = new ArrayList<>(32);
        for (int i = 0; i < nreg; ++i) {
            Region reg = regions[i];
            if (reg.id == 0 || (reg.id & RC_BORDER_REG) != 0) {
                continue;
            }
            if (reg.spanCount == 0) {
                continue;
            }
            if (reg.visited) {
                continue;
            }

            // Count the total size of all the connected regions.
            // Also keep track of the regions connects to a tile border.
            boolean connectsToBorder = false;
            int spanCount = 0;
            stack.clear();
            trace.clear();

            reg.visited = true;
            stack.add(i);

            while (stack.size() > 0) {
                // Pop
                int ri = stack.remove(stack.size() - 1);

                Region creg = regions[ri];

                spanCount += creg.spanCount;
                trace.add(ri);

                for (int j = 0; j < creg.connections.size(); ++j) {
                    if ((creg.connections.get(j) & RC_BORDER_REG) != 0) {
                        connectsToBorder = true;
                        continue;
                    }
                    Region neireg = regions[creg.connections.get(j)];
                    if (neireg.visited) {
                        continue;
                    }
                    if (neireg.id == 0 || (neireg.id & RC_BORDER_REG) != 0) {
                        continue;
                    }
                    // Visit
                    stack.add(neireg.id);
                    neireg.visited = true;
                }
            }

            // If the accumulated regions size is too small, remove it.
            // Do not remove areas which connect to tile borders
            // as their size cannot be estimated correctly and removing them
            // can potentially remove necessary areas.
            if (spanCount < minRegionArea && !connectsToBorder) {
                // Kill all visited regions.
                for (int j = 0; j < trace.size(); ++j) {
                    regions[trace.get(j)].spanCount = 0;
                    regions[trace.get(j)].id = 0;
                }
            }
        }

        // Merge too small regions to neighbour regions.
        int mergeCount = 0;
        do {
            mergeCount = 0;
            for (int i = 0; i < nreg; ++i) {
                Region reg = regions[i];
                if (reg.id == 0 || (reg.id & RC_BORDER_REG) != 0) {
                    continue;
                }
                if (reg.overlap) {
                    continue;
                }
                if (reg.spanCount == 0) {
                    continue;
                }

                // Check to see if the region should be merged.
                if (reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg)) {
                    continue;
                }

                // Small region with more than 1 connection.
                // Or region which is not connected to a border at all.
                // Find smallest neighbour region that connects to this one.
                int smallest = 0xfffffff;
                int mergeId = reg.id;
                for (int j = 0; j < reg.connections.size(); ++j) {
                    if ((reg.connections.get(j) & RC_BORDER_REG) != 0) {
                        continue;
                    }
                    Region mreg = regions[reg.connections.get(j)];
                    if (mreg.id == 0 || (mreg.id & RC_BORDER_REG) != 0 || mreg.overlap) {
                        continue;
                    }
                    if (mreg.spanCount < smallest && canMergeWithRegion(reg, mreg) && canMergeWithRegion(mreg, reg)) {
                        smallest = mreg.spanCount;
                        mergeId = mreg.id;
                    }
                }
                // Found new id.
                if (mergeId != reg.id) {
                    int oldId = reg.id;
                    Region target = regions[mergeId];

                    // Merge neighbours.
                    if (mergeRegions(target, reg)) {
                        // Fixup regions pointing to current region.
                        for (int j = 0; j < nreg; ++j) {
                            if (regions[j].id == 0 || (regions[j].id & RC_BORDER_REG) != 0) {
                                continue;
                            }
                            // If another region was already merged into current region
                            // change the nid of the previous region too.
                            if (regions[j].id == oldId) {
                                regions[j].id = mergeId;
                            }
                            // Replace the current region with the new one if the
                            // current regions is neighbour.
                            replaceNeighbour(regions[j], oldId, mergeId);
                        }
                        mergeCount++;
                    }
                }
            }
        } while (mergeCount > 0);

        // Compress region Ids.
        for (int i = 0; i < nreg; ++i) {
            regions[i].remap = false;
            if (regions[i].id == 0) {
                continue; // Skip nil regions.
            }
            if ((regions[i].id & RC_BORDER_REG) != 0) {
                continue; // Skip external regions.
            }
            regions[i].remap = true;
        }

        int regIdGen = 0;
        for (int i = 0; i < nreg; ++i) {
            if (!regions[i].remap) {
                continue;
            }
            int oldId = regions[i].id;
            int newId = ++regIdGen;
            for (int j = i; j < nreg; ++j) {
                if (regions[j].id == oldId) {
                    regions[j].id = newId;
                    regions[j].remap = false;
                }
            }
        }
        maxRegionId = regIdGen;

        // Remap regions.
        for (int i = 0; i < chf.spanCount; ++i) {
            if ((srcReg[i] & RC_BORDER_REG) == 0) {
                srcReg[i] = regions[srcReg[i]].id;
            }
        }

        // Return regions that we found to be overlapping.
        for (int i = 0; i < nreg; ++i) {
            if (regions[i].overlap) {
                overlaps.add(regions[i].id);
            }
        }

        return maxRegionId;
    }

    private static void addUniqueConnection(Region reg, int n) {
        if (!reg.connections.contains(n)) {
            reg.connections.add(n);
        }
    }

    private static int mergeAndFilterLayerRegions(Context ctx, int minRegionArea, int maxRegionId,
            CompactHeightfield chf, int[] srcReg, List<Integer> overlaps) {
        int w = chf.width;
        int h = chf.height;

        int nreg = maxRegionId + 1;
        Region[] regions = new Region[nreg];

        // Construct regions
        for (int i = 0; i < nreg; ++i) {
            regions[i] = new Region(i);
        }

        // Find region neighbours and overlapping regions.
        List<Integer> lregs = new ArrayList<>(32);
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];

                lregs.clear();

                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    int ri = srcReg[i];
                    if (ri == 0 || ri >= nreg) {
                        continue;
                    }
                    Region reg = regions[ri];

                    reg.spanCount++;

                    reg.ymin = Math.min(reg.ymin, s.y);
                    reg.ymax = Math.max(reg.ymax, s.y);
                    // Collect all region layers.
                    lregs.add(ri);

                    // Update neighbours
                    for (int dir = 0; dir < 4; ++dir) {
                        if (RecastCommon.GetCon(s, dir) != RC_NOT_CONNECTED) {
                            int ax = x + RecastCommon.GetDirOffsetX(dir);
                            int ay = y + RecastCommon.GetDirOffsetY(dir);
                            int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, dir);
                            int rai = srcReg[ai];
                            if (rai > 0 && rai < nreg && rai != ri) {
                                addUniqueConnection(reg, rai);
                            }
                            if ((rai & RC_BORDER_REG) != 0) {
                                reg.connectsToBorder = true;
                            }
                        }
                    }

                }

                // Update overlapping regions.
                for (int i = 0; i < lregs.size() - 1; ++i) {
                    for (int j = i + 1; j < lregs.size(); ++j) {
                        if (lregs.get(i) != lregs.get(j)) {
                            Region ri = regions[lregs.get(i)];
                            Region rj = regions[lregs.get(j)];
                            addUniqueFloorRegion(ri, lregs.get(j));
                            addUniqueFloorRegion(rj, lregs.get(i));
                        }
                    }
                }

            }
        }

        // Create 2D layers from regions.
        int layerId = 1;

        for (int i = 0; i < nreg; ++i) {
            regions[i].id = 0;
        }

        // Merge montone regions to create non-overlapping areas.
        List<Integer> stack = new ArrayList<>(32);
        for (int i = 1; i < nreg; ++i) {
            Region root = regions[i];
            // Skip already visited.
            if (root.id != 0) {
                continue;
            }

            // Start search.
            root.id = layerId;

            stack.clear();
            stack.add(i);

            while (stack.size() > 0) {
                // Pop front
                Region reg = regions[stack.remove(0)];

                int ncons = reg.connections.size();
                for (int j = 0; j < ncons; ++j) {
                    int nei = reg.connections.get(j);
                    Region regn = regions[nei];
                    // Skip already visited.
                    if (regn.id != 0) {
                        continue;
                    }
                    // Skip if the neighbour is overlapping root region.
                    boolean overlap = false;
                    for (int k = 0; k < root.floors.size(); k++) {
                        if (root.floors.get(k) == nei) {
                            overlap = true;
                            break;
                        }
                    }
                    if (overlap) {
                        continue;
                    }

                    // Deepen
                    stack.add(nei);

                    // Mark layer id
                    regn.id = layerId;
                    // Merge current layers to root.
                    for (int k = 0; k < regn.floors.size(); ++k) {
                        addUniqueFloorRegion(root, regn.floors.get(k));
                    }
                    root.ymin = Math.min(root.ymin, regn.ymin);
                    root.ymax = Math.max(root.ymax, regn.ymax);
                    root.spanCount += regn.spanCount;
                    regn.spanCount = 0;
                    root.connectsToBorder = root.connectsToBorder || regn.connectsToBorder;
                }
            }

            layerId++;
        }

        // Remove small regions
        for (int i = 0; i < nreg; ++i) {
            if (regions[i].spanCount > 0 && regions[i].spanCount < minRegionArea && !regions[i].connectsToBorder) {
                int reg = regions[i].id;
                for (int j = 0; j < nreg; ++j) {
                    if (regions[j].id == reg) {
                        regions[j].id = 0;
                    }
                }
            }
        }

        // Compress region Ids.
        for (int i = 0; i < nreg; ++i) {
            regions[i].remap = false;
            if (regions[i].id == 0) {
                continue; // Skip nil regions.
            }
            if ((regions[i].id & RC_BORDER_REG) != 0) {
                continue; // Skip external regions.
            }
            regions[i].remap = true;
        }

        int regIdGen = 0;
        for (int i = 0; i < nreg; ++i) {
            if (!regions[i].remap) {
                continue;
            }
            int oldId = regions[i].id;
            int newId = ++regIdGen;
            for (int j = i; j < nreg; ++j) {
                if (regions[j].id == oldId) {
                    regions[j].id = newId;
                    regions[j].remap = false;
                }
            }
        }
        maxRegionId = regIdGen;

        // Remap regions.
        for (int i = 0; i < chf.spanCount; ++i) {
            if ((srcReg[i] & RC_BORDER_REG) == 0) {
                srcReg[i] = regions[srcReg[i]].id;
            }
        }

        return maxRegionId;
    }

    /// @par
    ///
    /// This is usually the second to the last step in creating a fully built
    /// compact heightfield. This step is required before regions are built
    /// using #rcBuildRegions or #rcBuildRegionsMonotone.
    ///
    /// After this step, the distance data is available via the rcCompactHeightfield::maxDistance
    /// and rcCompactHeightfield::dist fields.
    ///
    /// @see rcCompactHeightfield, rcBuildRegions, rcBuildRegionsMonotone
    public static void buildDistanceField(Context ctx, CompactHeightfield chf) {

        ctx.startTimer("BUILD_DISTANCEFIELD");
        int[] src = new int[chf.spanCount];
        ctx.startTimer("DISTANCEFIELD_DIST");

        int maxDist = calculateDistanceField(chf, src);
        chf.maxDistance = maxDist;

        ctx.stopTimer("DISTANCEFIELD_DIST");

        ctx.startTimer("DISTANCEFIELD_BLUR");

        // Blur
        src = boxBlur(chf, 1, src);

        // Store distance.
        chf.dist = src;

        ctx.stopTimer("DISTANCEFIELD_BLUR");

        ctx.stopTimer("BUILD_DISTANCEFIELD");

    }

    private static void paintRectRegion(int minx, int maxx, int miny, int maxy, int regId, CompactHeightfield chf,
            int[] srcReg) {
        int w = chf.width;
        for (int y = miny; y < maxy; ++y) {
            for (int x = minx; x < maxx; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    if (chf.areas[i] != RC_NULL_AREA) {
                        srcReg[i] = regId;
                    }
                }
            }
        }
    }

    /// @par
    ///
    /// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
    /// Contours will form simple polygons.
    ///
    /// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
    /// re-assigned to the zero (null) region.
    ///
    /// Partitioning can result in smaller than necessary regions. @p mergeRegionArea helps
    /// reduce unecessarily small regions.
    ///
    /// See the #rcConfig documentation for more information on the configuration parameters.
    ///
    /// The region data will be available via the rcCompactHeightfield::maxRegions
    /// and rcCompactSpan::reg fields.
    ///
    /// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
    ///
    /// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
    public static void buildRegionsMonotone(Context ctx, CompactHeightfield chf, int borderSize, int minRegionArea,
            int mergeRegionArea) {
        ctx.startTimer("BUILD_REGIONS");

        int w = chf.width;
        int h = chf.height;
        int id = 1;

        int[] srcReg = new int[chf.spanCount];

        int nsweeps = Math.max(chf.width, chf.height);
        SweepSpan[] sweeps = new SweepSpan[nsweeps];
        for (int i = 0; i < sweeps.length; i++) {
            sweeps[i] = new SweepSpan();
        }

        // Mark border regions.
        if (borderSize > 0) {
            // Make sure border will not overflow.
            int bw = Math.min(w, borderSize);
            int bh = Math.min(h, borderSize);
            // Paint regions
            paintRectRegion(0, bw, 0, h, id | RC_BORDER_REG, chf, srcReg);
            id++;
            paintRectRegion(w - bw, w, 0, h, id | RC_BORDER_REG, chf, srcReg);
            id++;
            paintRectRegion(0, w, 0, bh, id | RC_BORDER_REG, chf, srcReg);
            id++;
            paintRectRegion(0, w, h - bh, h, id | RC_BORDER_REG, chf, srcReg);
            id++;

        }

        chf.borderSize = borderSize;

        int[] prev = new int[256];

        // Sweep one line at a time.
        for (int y = borderSize; y < h - borderSize; ++y) {
            // Collect spans from this row.
            Arrays.fill(prev, 0, id, 0);
            int rid = 1;

            for (int x = borderSize; x < w - borderSize; ++x) {
                CompactCell c = chf.cells[x + y * w];

                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    if (chf.areas[i] == RC_NULL_AREA) {
                        continue;
                    }

                    // -x
                    int previd = 0;
                    if (RecastCommon.GetCon(s, 0) != RC_NOT_CONNECTED) {
                        int ax = x + RecastCommon.GetDirOffsetX(0);
                        int ay = y + RecastCommon.GetDirOffsetY(0);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, 0);
                        if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai]) {
                            previd = srcReg[ai];
                        }
                    }

                    if (previd == 0) {
                        previd = rid++;
                        sweeps[previd].rid = previd;
                        sweeps[previd].ns = 0;
                        sweeps[previd].nei = 0;
                    }

                    // -y
                    if (RecastCommon.GetCon(s, 3) != RC_NOT_CONNECTED) {
                        int ax = x + RecastCommon.GetDirOffsetX(3);
                        int ay = y + RecastCommon.GetDirOffsetY(3);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, 3);
                        if (srcReg[ai] != 0 && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai]) {
                            int nr = srcReg[ai];
                            if (sweeps[previd].nei == 0 || sweeps[previd].nei == nr) {
                                sweeps[previd].nei = nr;
                                sweeps[previd].ns++;
                                prev[nr]++;
                            } else {
                                sweeps[previd].nei = RC_NULL_NEI;
                            }
                        }
                    }

                    srcReg[i] = previd;
                }
            }

            // Create unique ID.
            for (int i = 1; i < rid; ++i) {
                if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 && prev[sweeps[i].nei] == sweeps[i].ns) {
                    sweeps[i].id = sweeps[i].nei;
                } else {
                    sweeps[i].id = id++;
                }
            }

            // Remap IDs
            for (int x = borderSize; x < w - borderSize; ++x) {
                CompactCell c = chf.cells[x + y * w];

                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    if (srcReg[i] > 0 && srcReg[i] < rid) {
                        srcReg[i] = sweeps[srcReg[i]].id;
                    }
                }
            }
        }

        ctx.startTimer("BUILD_REGIONS_FILTER");

        // Merge regions and filter out small regions.
        List<Integer> overlaps = new ArrayList<>();
        chf.maxRegions = mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, id, chf, srcReg, overlaps);

        // Monotone partitioning does not generate overlapping regions.

        ctx.stopTimer("BUILD_REGIONS_FILTER");

        // Store the result out.
        for (int i = 0; i < chf.spanCount; ++i) {
            chf.spans[i].reg = srcReg[i];
        }

        ctx.stopTimer("BUILD_REGIONS");

    }

    /// @par
    ///
    /// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
    /// Contours will form simple polygons.
    ///
    /// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
    /// re-assigned to the zero (null) region.
    ///
    /// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors.
    /// @p mergeRegionArea helps reduce unecessarily small regions.
    ///
    /// See the #rcConfig documentation for more information on the configuration parameters.
    ///
    /// The region data will be available via the rcCompactHeightfield::maxRegions
    /// and rcCompactSpan::reg fields.
    ///
    /// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
    ///
    /// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
    public static void buildRegions(Context ctx, CompactHeightfield chf, int borderSize, int minRegionArea,
            int mergeRegionArea) {
        ctx.startTimer("BUILD_REGIONS");

        int w = chf.width;
        int h = chf.height;

        ctx.startTimer("REGIONS_WATERSHED");

        int LOG_NB_STACKS = 3;
        int NB_STACKS = 1 << LOG_NB_STACKS;
        List<List<Integer>> lvlStacks = new ArrayList<>();
        for (int i = 0; i < NB_STACKS; ++i) {
            lvlStacks.add(new ArrayList<>(1024));
        }

        List<Integer> stack = new ArrayList<>(1024);

        int[] srcReg = new int[chf.spanCount];
        int[] srcDist = new int[chf.spanCount];

        int regionId = 1;
        int level = (chf.maxDistance + 1) & ~1;

        // TODO: Figure better formula, expandIters defines how much the
        // watershed "overflows" and simplifies the regions. Tying it to
        // agent radius was usually good indication how greedy it could be.
        // const int expandIters = 4 + walkableRadius * 2;
        int expandIters = 8;

        if (borderSize > 0) {
            // Make sure border will not overflow.
            int bw = Math.min(w, borderSize);
            int bh = Math.min(h, borderSize);
            // Paint regions
            paintRectRegion(0, bw, 0, h, regionId | RC_BORDER_REG, chf, srcReg);
            regionId++;
            paintRectRegion(w - bw, w, 0, h, regionId | RC_BORDER_REG, chf, srcReg);
            regionId++;
            paintRectRegion(0, w, 0, bh, regionId | RC_BORDER_REG, chf, srcReg);
            regionId++;
            paintRectRegion(0, w, h - bh, h, regionId | RC_BORDER_REG, chf, srcReg);
            regionId++;

        }

        chf.borderSize = borderSize;

        int sId = -1;
        while (level > 0) {
            level = level >= 2 ? level - 2 : 0;
            sId = (sId + 1) & (NB_STACKS - 1);

            // ctx->startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

            if (sId == 0) {
                sortCellsByLevel(level, chf, srcReg, NB_STACKS, lvlStacks, 1);
            } else {
                appendStacks(lvlStacks.get(sId - 1), lvlStacks.get(sId), srcReg); // copy left overs from last level
            }

            // ctx->stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

            ctx.startTimer("BUILD_REGIONS_EXPAND");

            // Expand current regions until no empty connected cells found.
            expandRegions(expandIters, level, chf, srcReg, srcDist, lvlStacks.get(sId), false);

            ctx.stopTimer("BUILD_REGIONS_EXPAND");

            ctx.startTimer("BUILD_REGIONS_FLOOD");

            // Mark new regions with IDs.
            for (int j = 0; j < lvlStacks.get(sId).size(); j += 3) {
                int x = lvlStacks.get(sId).get(j);
                int y = lvlStacks.get(sId).get(j + 1);
                int i = lvlStacks.get(sId).get(j + 2);
                if (i >= 0 && srcReg[i] == 0) {
                    if (floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, stack)) {
                        regionId++;
                    }
                }
            }

            ctx.stopTimer("BUILD_REGIONS_FLOOD");
        }

        // Expand current regions until no empty connected cells found.
        expandRegions(expandIters * 8, 0, chf, srcReg, srcDist, stack, true);

        ctx.stopTimer("BUILD_REGIONS_WATERSHED");

        ctx.startTimer("BUILD_REGIONS_FILTER");

        // Merge regions and filter out smalle regions.
        List<Integer> overlaps = new ArrayList<>();
        chf.maxRegions = mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, regionId, chf, srcReg, overlaps);

        // If overlapping regions were found during merging, split those regions.
        if (overlaps.size() > 0) {
            ctx.warn("rcBuildRegions: " + overlaps.size() + " overlapping regions.");
        }

        ctx.stopTimer("BUILD_REGIONS_FILTER");

        // Write the result out.
        for (int i = 0; i < chf.spanCount; ++i) {
            chf.spans[i].reg = srcReg[i];
        }

        ctx.stopTimer("BUILD_REGIONS");

    }

    public static void buildLayerRegions(Context ctx, CompactHeightfield chf, int borderSize, int minRegionArea) {

        ctx.startTimer("BUILD_REGIONS");

        int w = chf.width;
        int h = chf.height;
        int id = 1;

        int[] srcReg = new int[chf.spanCount];
        int nsweeps = Math.max(chf.width, chf.height);
        SweepSpan[] sweeps = new SweepSpan[nsweeps];
        for (int i = 0; i < sweeps.length; i++) {
            sweeps[i] = new SweepSpan();
        }

        // Mark border regions.
        if (borderSize > 0) {
            // Make sure border will not overflow.
            int bw = Math.min(w, borderSize);
            int bh = Math.min(h, borderSize);
            // Paint regions
            paintRectRegion(0, bw, 0, h, id | RC_BORDER_REG, chf, srcReg);
            id++;
            paintRectRegion(w - bw, w, 0, h, id | RC_BORDER_REG, chf, srcReg);
            id++;
            paintRectRegion(0, w, 0, bh, id | RC_BORDER_REG, chf, srcReg);
            id++;
            paintRectRegion(0, w, h - bh, h, id | RC_BORDER_REG, chf, srcReg);
            id++;

        }

        chf.borderSize = borderSize;

        int[] prev = new int[256];

        // Sweep one line at a time.
        for (int y = borderSize; y < h - borderSize; ++y) {
            // Collect spans from this row.
            Arrays.fill(prev, 0, id, 0);
            int rid = 1;

            for (int x = borderSize; x < w - borderSize; ++x) {
                CompactCell c = chf.cells[x + y * w];

                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    if (chf.areas[i] == RC_NULL_AREA) {
                        continue;
                    }

                    // -x
                    int previd = 0;
                    if (RecastCommon.GetCon(s, 0) != RC_NOT_CONNECTED) {
                        int ax = x + RecastCommon.GetDirOffsetX(0);
                        int ay = y + RecastCommon.GetDirOffsetY(0);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, 0);
                        if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai]) {
                            previd = srcReg[ai];
                        }
                    }

                    if (previd == 0) {
                        previd = rid++;
                        sweeps[previd].rid = previd;
                        sweeps[previd].ns = 0;
                        sweeps[previd].nei = 0;
                    }

                    // -y
                    if (RecastCommon.GetCon(s, 3) != RC_NOT_CONNECTED) {
                        int ax = x + RecastCommon.GetDirOffsetX(3);
                        int ay = y + RecastCommon.GetDirOffsetY(3);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, 3);
                        if (srcReg[ai] != 0 && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai]) {
                            int nr = srcReg[ai];
                            if (sweeps[previd].nei == 0 || sweeps[previd].nei == nr) {
                                sweeps[previd].nei = nr;
                                sweeps[previd].ns++;
                                prev[nr]++;
                            } else {
                                sweeps[previd].nei = RC_NULL_NEI;
                            }
                        }
                    }

                    srcReg[i] = previd;
                }
            }

            // Create unique ID.
            for (int i = 1; i < rid; ++i) {
                if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 && prev[sweeps[i].nei] == sweeps[i].ns) {
                    sweeps[i].id = sweeps[i].nei;
                } else {
                    sweeps[i].id = id++;
                }
            }

            // Remap IDs
            for (int x = borderSize; x < w - borderSize; ++x) {
                CompactCell c = chf.cells[x + y * w];

                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    if (srcReg[i] > 0 && srcReg[i] < rid) {
                        srcReg[i] = sweeps[srcReg[i]].id;
                    }
                }
            }
        }

        ctx.startTimer("BUILD_REGIONS_FILTER");

        // Merge monotone regions to layers and remove small regions.
        List<Integer> overlaps = new ArrayList<>();
        chf.maxRegions = mergeAndFilterLayerRegions(ctx, minRegionArea, id, chf, srcReg, overlaps);

        ctx.stopTimer("BUILD_REGIONS_FILTER");

        // Store the result out.
        for (int i = 0; i < chf.spanCount; ++i) {
            chf.spans[i].reg = srcReg[i];
        }

        ctx.stopTimer("BUILD_REGIONS");

    }
}
