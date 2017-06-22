package org.recast4j.detour.tilecache;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import org.junit.Test;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.FindPathResult;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
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
        this.navmesh = tcC.getNavMesh();
        this.query = new NavMeshQuery(navmesh);
    }

    @Test
    public void testFindPath() {
        QueryFilter filter = new QueryFilter();
        float[] extents = new float[] { 2f, 4f, 2f };
        FindNearestPolyResult findPolyStart = query.findNearestPoly(start, extents, filter);
        FindNearestPolyResult findPolyEnd = query.findNearestPoly(end, extents, filter);
        long startRef = findPolyStart.getNearestRef();
        long endRef = findPolyEnd.getNearestRef();
        float[] startPos = findPolyStart.getNearestPos();
        float[] endPos = findPolyEnd.getNearestPos();
        FindPathResult path = query.findPath(startRef, endRef, startPos, endPos, filter);
        int maxStraightPath = 256;
        int options = 0;
        List<StraightPathItem> pathStr = query.findStraightPath(startPos, endPos, path.getRefs(), maxStraightPath, options);
        assertEquals(8, pathStr.size());
    }

}