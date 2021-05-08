/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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
package org.recast4j.detour.tilecache;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import org.junit.Test;
import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;
import org.recast4j.detour.StraightPathItem;
import org.recast4j.detour.tilecache.io.TileCacheReader;

public class TileCacheFindPathTest extends AbstractTileCacheTest {

    private final float[] start = { 39.44734f, 9.998177f, -0.784811f };
    private final float[] end = { 19.292645f, 11.611748f, -57.750366f };
    private final NavMesh navmesh;
    private final NavMeshQuery query;

    public TileCacheFindPathTest() throws IOException {
        InputStream is = getClass().getClassLoader().getResourceAsStream("dungeon_all_tiles_tilecache.bin");
        TileCache tcC = new TileCacheReader().read(is, 6, new TestTileCacheMeshProcess());
        navmesh = tcC.getNavMesh();
        query = new NavMeshQuery(navmesh);
    }

    @Test
    public void testFindPath() {
        QueryFilter filter = new DefaultQueryFilter();
        float[] extents = new float[] { 2f, 4f, 2f };
        Result<FindNearestPolyResult> findPolyStart = query.findNearestPoly(start, extents, filter);
        Result<FindNearestPolyResult> findPolyEnd = query.findNearestPoly(end, extents, filter);
        long startRef = findPolyStart.result.getNearestRef();
        long endRef = findPolyEnd.result.getNearestRef();
        float[] startPos = findPolyStart.result.getNearestPos();
        float[] endPos = findPolyEnd.result.getNearestPos();
        Result<List<Long>> path = query.findPath(startRef, endRef, startPos, endPos, filter);
        int maxStraightPath = 256;
        int options = 0;
        Result<List<StraightPathItem>> pathStr = query.findStraightPath(startPos, endPos, path.result, maxStraightPath,
                options);
        assertEquals(8, pathStr.result.size());
    }

}