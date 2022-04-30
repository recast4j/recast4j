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

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.offset;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.MeshHeader;
import org.recast4j.detour.MeshTile;
import org.recast4j.detour.tilecache.AbstractTileCacheTest;
import org.recast4j.detour.tilecache.ObjImporter;
import org.recast4j.detour.tilecache.TestTileLayerBuilder;
import org.recast4j.detour.tilecache.TileCache;
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
        InputGeomProvider geom = new ObjImporter().load(TileCacheReaderWriterTest.class.getClassLoader().getResourceAsStream("dungeon.obj"));
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
        assertThat(tc.getNavMesh().getMaxTiles()).isEqualTo(256);
        assertThat(tc.getNavMesh().getParams().maxPolys).isEqualTo(16384);
        assertThat(tc.getNavMesh().getParams().tileWidth).isEqualTo(14.4f, offset(0.001f));
        assertThat(tc.getNavMesh().getParams().tileHeight).isEqualTo(14.4f, offset(0.001f));
        assertThat(tc.getNavMesh().getMaxVertsPerPoly()).isEqualTo(6);
        assertThat(tc.getParams().cs).isEqualTo(0.3f, offset(0.0f));
        assertThat(tc.getParams().ch).isEqualTo(0.2f, offset(0.0f));
        assertThat(tc.getParams().walkableClimb).isEqualTo(0.9f, offset(0.0f));
        assertThat(tc.getParams().walkableHeight).isEqualTo(2f, offset(0.0f));
        assertThat(tc.getParams().walkableRadius).isEqualTo(0.6f, offset(0.0f));
        assertThat(tc.getParams().width).isEqualTo(48);
        assertThat(tc.getParams().maxTiles).isEqualTo(6 * 7 * 4);
        assertThat(tc.getParams().maxObstacles).isEqualTo(128);
        assertThat(tc.getTileCount()).isEqualTo(168);
        // Tile0: Tris: 8, Verts: 18 Detail Meshed: 8 Detail Verts: 0 Detail Tris: 14
        MeshTile tile = tc.getNavMesh().getTile(0);
        MeshData data = tile.data;
        MeshHeader header = data.header;
        assertThat(header.vertCount).isEqualTo(18);
        assertThat(header.polyCount).isEqualTo(8);
        assertThat(header.detailMeshCount).isEqualTo(8);
        assertThat(header.detailVertCount).isEqualTo(0);
        assertThat(header.detailTriCount).isEqualTo(14);
        assertThat(data.polys.length).isEqualTo(8);
        assertThat(data.verts.length).isEqualTo(3 * 18);
        assertThat(data.detailMeshes.length).isEqualTo(8);
        assertThat(data.detailVerts.length).isEqualTo(0);
        assertThat(data.detailTris.length).isEqualTo(4 * 14);
        // Tile8: Tris: 3, Verts: 8 Detail Meshed: 3 Detail Verts: 0 Detail Tris: 6
        tile = tc.getNavMesh().getTile(8);
        data = tile.data;
        header = data.header;
        assertThat(header.vertCount).isEqualTo(8);
        assertThat(header.polyCount).isEqualTo(3);
        assertThat(header.detailMeshCount).isEqualTo(3);
        assertThat(header.detailVertCount).isEqualTo(0);
        assertThat(header.detailTriCount).isEqualTo(6);
        assertThat(data.polys.length).isEqualTo(3);
        assertThat(data.verts.length).isEqualTo(3 * 8);
        assertThat(data.detailMeshes.length).isEqualTo(3);
        assertThat(data.detailVerts.length).isEqualTo(0);
        assertThat(data.detailTris.length).isEqualTo(4 * 6);
        // Tile16: Tris: 10, Verts: 20 Detail Meshed: 10 Detail Verts: 0 Detail Tris: 18
        tile = tc.getNavMesh().getTile(16);
        data = tile.data;
        header = data.header;
        assertThat(header.vertCount).isEqualTo(20);
        assertThat(header.polyCount).isEqualTo(10);
        assertThat(header.detailMeshCount).isEqualTo(10);
        assertThat(header.detailVertCount).isEqualTo(0);
        assertThat(header.detailTriCount).isEqualTo(18);
        assertThat(data.polys.length).isEqualTo(10);
        assertThat(data.verts.length).isEqualTo(3 * 20);
        assertThat(data.detailMeshes.length).isEqualTo(10);
        assertThat(data.detailVerts.length).isEqualTo(0);
        assertThat(data.detailTris.length).isEqualTo(4 * 18);
        // Tile29: Tris: 1, Verts: 5 Detail Meshed: 1 Detail Verts: 0 Detail Tris: 3
        tile = tc.getNavMesh().getTile(29);
        data = tile.data;
        header = data.header;
        assertThat(header.vertCount).isEqualTo(5);
        assertThat(header.polyCount).isEqualTo(1);
        assertThat(header.detailMeshCount).isEqualTo(1);
        assertThat(header.detailVertCount).isEqualTo(0);
        assertThat(header.detailTriCount).isEqualTo(3);
        assertThat(data.polys.length).isEqualTo(1);
        assertThat(data.verts.length).isEqualTo(3 * 5);
        assertThat(data.detailMeshes.length).isEqualTo(1);
        assertThat(data.detailVerts.length).isEqualTo(0);
        assertThat(data.detailTris.length).isEqualTo(4 * 3);
    }

}
