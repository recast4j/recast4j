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
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.Test;
import org.recast4j.detour.MeshTile;
import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.geom.InputGeomProvider;

public class TempObstaclesTest extends AbstractTileCacheTest {

    @Test
    public void testDungeon() throws IOException {
        boolean cCompatibility = true;
        InputGeomProvider geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream("dungeon.obj"));
        TestTileLayerBuilder layerBuilder = new TestTileLayerBuilder(geom);
        List<byte[]> layers = layerBuilder.build(ByteOrder.LITTLE_ENDIAN, cCompatibility, 1);
        TileCache tc = getTileCache(geom, ByteOrder.LITTLE_ENDIAN, cCompatibility);
        for (byte[] data : layers) {
            long ref = tc.addTile(data, 0);
            tc.buildNavMeshTile(ref);
        }
        List<MeshTile> tiles = tc.getNavMesh().getTilesAt(1, 4);
        MeshTile tile = tiles.get(0);
        assertEquals(16, tile.data.header.vertCount);
        assertEquals(6, tile.data.header.polyCount);
        long o = tc.addObstacle(new float[] { -1.815208f, 9.998184f, -20.307983f }, 1f, 2f);
        boolean upToDate = tc.update();
        assertTrue(upToDate);
        tiles = tc.getNavMesh().getTilesAt(1, 4);
        tile = tiles.get(0);
        assertEquals(22, tile.data.header.vertCount);
        assertEquals(11, tile.data.header.polyCount);
        tc.removeObstacle(o);
        upToDate = tc.update();
        assertTrue(upToDate);
        tiles = tc.getNavMesh().getTilesAt(1, 4);
        tile = tiles.get(0);
        assertEquals(16, tile.data.header.vertCount);
        assertEquals(6, tile.data.header.polyCount);
    }

    @Test
    public void testDungeonBox() throws IOException {
        boolean cCompatibility = true;
        InputGeomProvider geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream("dungeon.obj"));
        TestTileLayerBuilder layerBuilder = new TestTileLayerBuilder(geom);
        List<byte[]> layers = layerBuilder.build(ByteOrder.LITTLE_ENDIAN, cCompatibility, 1);
        TileCache tc = getTileCache(geom, ByteOrder.LITTLE_ENDIAN, cCompatibility);
        for (byte[] data : layers) {
            long ref = tc.addTile(data, 0);
            tc.buildNavMeshTile(ref);
        }
        List<MeshTile> tiles = tc.getNavMesh().getTilesAt(1, 4);
        MeshTile tile = tiles.get(0);
        assertEquals(16, tile.data.header.vertCount);
        assertEquals(6, tile.data.header.polyCount);
        long o = tc.addBoxObstacle(new float[] { -2.315208f, 9.998184f, -20.807983f },
                new float[] { -1.315208f, 11.998184f, -19.807983f });
        boolean upToDate = tc.update();
        assertTrue(upToDate);
        tiles = tc.getNavMesh().getTilesAt(1, 4);
        tile = tiles.get(0);
        assertEquals(22, tile.data.header.vertCount);
        assertEquals(11, tile.data.header.polyCount);
        tc.removeObstacle(o);
        upToDate = tc.update();
        assertTrue(upToDate);
        tiles = tc.getNavMesh().getTilesAt(1, 4);
        tile = tiles.get(0);
        assertEquals(16, tile.data.header.vertCount);
        assertEquals(6, tile.data.header.polyCount);
    }
}
