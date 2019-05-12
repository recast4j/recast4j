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
package org.recast4j.detour.tilecache.io;

import static org.junit.Assert.assertEquals;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.Test;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.MeshHeader;
import org.recast4j.detour.MeshTile;
import org.recast4j.detour.tilecache.AbstractTileCacheTest;
import org.recast4j.detour.tilecache.TestTileLayerBuilder;
import org.recast4j.detour.tilecache.TileCache;
import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.geom.InputGeomProvider;

public class TileCacheReaderWriterTest extends AbstractTileCacheTest {

    private final TileCacheReader reader = new TileCacheReader();
    private final TileCacheWriter writer = new TileCacheWriter();

    @Test
    public void testFastLz() throws IOException {
        testDungeon(false);
        testDungeon(true);
    }

    @Test
    public void testLZ4() throws IOException {
        testDungeon(true);
        testDungeon(false);
    }

    private void testDungeon(boolean cCompatibility) throws IOException {
        InputGeomProvider geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream("dungeon.obj"));
        TestTileLayerBuilder layerBuilder = new TestTileLayerBuilder(geom);
        List<byte[]> layers = layerBuilder.build(ByteOrder.LITTLE_ENDIAN, cCompatibility, 1);
        TileCache tc = getTileCache(geom, ByteOrder.LITTLE_ENDIAN, cCompatibility);
        for (byte[] data : layers) {
            long ref = tc.addTile(data, 0);
            tc.buildNavMeshTile(ref);
        }
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        writer.write(baos, tc, ByteOrder.LITTLE_ENDIAN, cCompatibility);
        ByteArrayInputStream bais = new ByteArrayInputStream(baos.toByteArray());
        tc = reader.read(bais, 6, null);
        assertEquals(256, tc.getNavMesh().getMaxTiles());
        assertEquals(16384, tc.getNavMesh().getParams().maxPolys);
        assertEquals(14.4f, tc.getNavMesh().getParams().tileWidth, 0.001f);
        assertEquals(14.4f, tc.getNavMesh().getParams().tileHeight, 0.001f);
        assertEquals(6, tc.getNavMesh().getMaxVertsPerPoly());
        assertEquals(0.3f, tc.getParams().cs, 0.0f);
        assertEquals(0.2f, tc.getParams().ch, 0.0f);
        assertEquals(0.9f, tc.getParams().walkableClimb, 0.0f);
        assertEquals(2f, tc.getParams().walkableHeight, 0.0f);
        assertEquals(0.6f, tc.getParams().walkableRadius, 0.0f);
        assertEquals(48, tc.getParams().width);
        assertEquals(6 * 7 * 4, tc.getParams().maxTiles);
        assertEquals(128, tc.getParams().maxObstacles);
        assertEquals(168, tc.getTileCount());
        // Tile0: Tris: 8, Verts: 18 Detail Meshed: 8 Detail Verts: 0 Detail Tris: 14
        MeshTile tile = tc.getNavMesh().getTile(0);
        MeshData data = tile.data;
        MeshHeader header = data.header;
        assertEquals(18, header.vertCount);
        assertEquals(8, header.polyCount);
        assertEquals(8, header.detailMeshCount);
        assertEquals(0, header.detailVertCount);
        assertEquals(14, header.detailTriCount);
        assertEquals(8, data.polys.length);
        assertEquals(3 * 18, data.verts.length);
        assertEquals(8, data.detailMeshes.length);
        assertEquals(0, data.detailVerts.length);
        assertEquals(4 * 14, data.detailTris.length);
        // Tile8: Tris: 3, Verts: 8 Detail Meshed: 3 Detail Verts: 0 Detail Tris: 6
        tile = tc.getNavMesh().getTile(8);
        data = tile.data;
        header = data.header;
        assertEquals(8, header.vertCount);
        assertEquals(3, header.polyCount);
        assertEquals(3, header.detailMeshCount);
        assertEquals(0, header.detailVertCount);
        assertEquals(6, header.detailTriCount);
        assertEquals(3, data.polys.length);
        assertEquals(3 * 8, data.verts.length);
        assertEquals(3, data.detailMeshes.length);
        assertEquals(0, data.detailVerts.length);
        assertEquals(4 * 6, data.detailTris.length);
        // Tile16: Tris: 10, Verts: 20 Detail Meshed: 10 Detail Verts: 0 Detail Tris: 18
        tile = tc.getNavMesh().getTile(16);
        data = tile.data;
        header = data.header;
        assertEquals(20, header.vertCount);
        assertEquals(10, header.polyCount);
        assertEquals(10, header.detailMeshCount);
        assertEquals(0, header.detailVertCount);
        assertEquals(18, header.detailTriCount);
        assertEquals(10, data.polys.length);
        assertEquals(3 * 20, data.verts.length);
        assertEquals(10, data.detailMeshes.length);
        assertEquals(0, data.detailVerts.length);
        assertEquals(4 * 18, data.detailTris.length);
        // Tile29: Tris: 1, Verts: 5 Detail Meshed: 1 Detail Verts: 0 Detail Tris: 3
        tile = tc.getNavMesh().getTile(29);
        data = tile.data;
        header = data.header;
        assertEquals(5, header.vertCount);
        assertEquals(1, header.polyCount);
        assertEquals(1, header.detailMeshCount);
        assertEquals(0, header.detailVertCount);
        assertEquals(3, header.detailTriCount);
        assertEquals(1, data.polys.length);
        assertEquals(3 * 5, data.verts.length);
        assertEquals(1, data.detailMeshes.length);
        assertEquals(0, data.detailVerts.length);
        assertEquals(4 * 3, data.detailTris.length);
    }

}
