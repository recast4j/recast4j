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

import java.io.IOException;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;
import org.recast4j.detour.Status;
import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.geom.InputGeomProvider;

public class TileCacheNavigationTest extends AbstractTileCacheTest {

    protected final long[] startRefs = { 281475006070787L };
    protected final long[] endRefs = { 281474986147841L };
    protected final float[][] startPoss = { { 39.447338f, 9.998177f, -0.784811f } };
    protected final float[][] endPoss = { { 19.292645f, 11.611748f, -57.750366f } };
    private final Status[] statuses = { Status.SUCCSESS, Status.PARTIAL_RESULT, Status.SUCCSESS, Status.SUCCSESS,
            Status.SUCCSESS };
    private final long[][] results = { { 281475006070787L, 281475006070785L, 281475005022208L, 281475005022209L,
            281475003973633L, 281475003973634L, 281475003973632L, 281474996633604L, 281474996633605L, 281474996633603L,
            281474995585027L, 281474995585029L, 281474995585026L, 281474995585028L, 281474995585024L, 281474991390721L,
            281474991390722L, 281474991390725L, 281474991390720L, 281474987196418L, 281474987196417L, 281474988244995L,
            281474988245001L, 281474988244997L, 281474988244998L, 281474988245002L, 281474988245000L, 281474988244999L,
            281474988244994L, 281474985099264L, 281474985099266L, 281474986147841L } };

    protected NavMesh navmesh;
    protected NavMeshQuery query;

    @Before
    public void setUp() throws IOException {

        boolean cCompatibility = true;
        InputGeomProvider geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream("dungeon.obj"));
        TestTileLayerBuilder layerBuilder = new TestTileLayerBuilder(geom);
        List<byte[]> layers = layerBuilder.build(ByteOrder.LITTLE_ENDIAN, cCompatibility, 1);
        TileCache tc = getTileCache(geom, ByteOrder.LITTLE_ENDIAN, cCompatibility);
        for (byte[] data : layers) {
            tc.addTile(data, 0);
        }
        for (int y = 0; y < layerBuilder.getTh(); ++y) {
            for (int x = 0; x < layerBuilder.getTw(); ++x) {
                for (Long ref : tc.getTilesAt(x, y)) {
                    tc.buildNavMeshTile(ref);
                }
            }
        }
        navmesh = tc.getNavMesh();
        query = new NavMeshQuery(navmesh);

    }

    @Test
    public void testFindPath() {
        QueryFilter filter = new DefaultQueryFilter();
        for (int i = 0; i < startRefs.length; i++) {
            long startRef = startRefs[i];
            long endRef = endRefs[i];
            float[] startPos = startPoss[i];
            float[] endPos = endPoss[i];
            Result<List<Long>> path = query.findPath(startRef, endRef, startPos, endPos, filter);
            Assert.assertEquals(statuses[i], path.status);
            Assert.assertEquals(results[i].length, path.result.size());
            for (int j = 0; j < results[i].length; j++) {
                Assert.assertEquals(results[i][j], path.result.get(j).longValue());
            }
        }
    }

}
