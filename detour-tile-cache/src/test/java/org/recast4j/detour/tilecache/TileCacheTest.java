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

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.offset;

import java.io.IOException;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.MeshHeader;
import org.recast4j.detour.MeshTile;
import org.recast4j.recast.geom.InputGeomProvider;

public class TileCacheTest extends AbstractTileCacheTest {

    @Test
    public void testFastLz() throws IOException {
        testDungeon(ByteOrder.LITTLE_ENDIAN, false);
        testDungeon(ByteOrder.LITTLE_ENDIAN, true);
        testDungeon(ByteOrder.BIG_ENDIAN, false);
        testDungeon(ByteOrder.BIG_ENDIAN, true);
        test(ByteOrder.LITTLE_ENDIAN, false);
        test(ByteOrder.LITTLE_ENDIAN, true);
        test(ByteOrder.BIG_ENDIAN, false);
        test(ByteOrder.BIG_ENDIAN, true);
    }

    @Test
    public void testLZ4() throws IOException {
        testDungeon(ByteOrder.LITTLE_ENDIAN, false);
        testDungeon(ByteOrder.LITTLE_ENDIAN, true);
        testDungeon(ByteOrder.BIG_ENDIAN, false);
        testDungeon(ByteOrder.BIG_ENDIAN, true);
        test(ByteOrder.LITTLE_ENDIAN, false);
        test(ByteOrder.LITTLE_ENDIAN, true);
        test(ByteOrder.BIG_ENDIAN, false);
        test(ByteOrder.BIG_ENDIAN, true);
    }

    private void testDungeon(ByteOrder order, boolean cCompatibility) throws IOException {
        InputGeomProvider geom = new ObjImporter().load(TileCacheTest.class.getClassLoader().getResourceAsStream("dungeon.obj"));
        TileCache tc = getTileCache(geom, order, cCompatibility);
        TestTileLayerBuilder layerBuilder = new TestTileLayerBuilder(geom);
        List<byte[]> layers = layerBuilder.build(order, cCompatibility, 1);
        int cacheLayerCount = 0;
        int cacheCompressedSize = 0;
        int cacheRawSize = 0;
        for (byte[] data : layers) {
            long ref = tc.addTile(data, 0);
            tc.buildNavMeshTile(ref);
            cacheLayerCount++;
            cacheCompressedSize += data.length;
            cacheRawSize += 4 * 48 * 48 + 56; // FIXME
        }
        System.out.println("Compressor: " + tc.getCompressor().getClass().getSimpleName() + " C Compatibility: " + cCompatibility
                + " Layers: " + cacheLayerCount + " Raw Size: " + cacheRawSize + " Compressed: " + cacheCompressedSize);
        assertThat(tc.getNavMesh().getMaxTiles()).isEqualTo(256);
        assertThat(tc.getNavMesh().getParams().maxPolys).isEqualTo(16384);
        assertThat(tc.getNavMesh().getParams().tileWidth).isEqualTo(14.4f, offset(0.001f));
        assertThat(tc.getNavMesh().getParams().tileHeight).isEqualTo(14.4f, offset(0.001f));
        assertThat(tc.getNavMesh().getMaxVertsPerPoly()).isEqualTo(6);
        assertThat(tc.getParams().cs).isEqualTo(0.3f);
        assertThat(tc.getParams().ch).isEqualTo(0.2f);
        assertThat(tc.getParams().walkableClimb).isEqualTo(0.9f);
        assertThat(tc.getParams().walkableHeight).isEqualTo(2f);
        assertThat(tc.getParams().walkableRadius).isEqualTo(0.6f);
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
        assertThat(data.verts[1]).isEqualTo(14.997294f, offset(0.0001f));
        assertThat(data.verts[6]).isEqualTo(15.484785f, offset(0.0001f));
        assertThat(data.verts[9]).isEqualTo(15.484785f, offset(0.0001f));
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

    private void test(ByteOrder order, boolean cCompatibility) throws IOException {
        InputGeomProvider geom = new ObjImporter().load(TileCacheTest.class.getClassLoader().getResourceAsStream("nav_test.obj"));
        TileCache tc = getTileCache(geom, order, cCompatibility);
        TestTileLayerBuilder layerBuilder = new TestTileLayerBuilder(geom);
        List<byte[]> layers = layerBuilder.build(order, cCompatibility, 1);
        int cacheLayerCount = 0;
        int cacheCompressedSize = 0;
        int cacheRawSize = 0;
        for (byte[] data : layers) {
            long ref = tc.addTile(data, 0);
            tc.buildNavMeshTile(ref);
            cacheLayerCount++;
            cacheCompressedSize += data.length;
            cacheRawSize += 4 * 48 * 48 + 56;
        }
        System.out.println("Compressor: " + tc.getCompressor().getClass().getSimpleName() + " C Compatibility: " + cCompatibility
                + " Layers: " + cacheLayerCount + " Raw Size: " + cacheRawSize + " Compressed: " + cacheCompressedSize);
    }

    @Test
    public void testPerformance() throws IOException {
        int threads = 4;
        ByteOrder order = ByteOrder.LITTLE_ENDIAN;
        boolean cCompatibility = false;
        InputGeomProvider geom = new ObjImporter().load(TileCacheTest.class.getClassLoader().getResourceAsStream("dungeon.obj"));
        TestTileLayerBuilder layerBuilder = new TestTileLayerBuilder(geom);
        for (int i = 0; i < 4; i++) {
            layerBuilder.build(order, cCompatibility, 1);
            layerBuilder.build(order, cCompatibility, threads);
        }
        long t1 = System.nanoTime();
        List<byte[]> layers = null;
        for (int i = 0; i < 8; i++) {
            layers = layerBuilder.build(order, cCompatibility, 1);
        }
        long t2 = System.nanoTime();
        for (int i = 0; i < 8; i++) {
            layers = layerBuilder.build(order, cCompatibility, threads);
        }
        long t3 = System.nanoTime();
        System.out.println(" Time ST : " + (t2 - t1) / 1000000);
        System.out.println(" Time MT : " + (t3 - t2) / 1000000);
        TileCache tc = getTileCache(geom, order, cCompatibility);
        for (byte[] data : layers) {
            long ref = tc.addTile(data, 0);
            tc.buildNavMeshTile(ref);
        }
        assertThat(tc.getNavMesh().getMaxTiles()).isEqualTo(256);
        assertThat(tc.getNavMesh().getParams().maxPolys).isEqualTo(16384);
        assertThat(tc.getNavMesh().getParams().tileWidth).isEqualTo(14.4f, offset(0.001f));
        assertThat(tc.getNavMesh().getParams().tileHeight).isEqualTo(14.4f, offset(0.001f));
        assertThat(tc.getNavMesh().getMaxVertsPerPoly()).isEqualTo(6);
        assertThat(tc.getParams().cs).isEqualTo(0.3f);
        assertThat(tc.getParams().ch).isEqualTo(0.2f);
        assertThat(tc.getParams().walkableClimb).isEqualTo(0.9f);
        assertThat(tc.getParams().walkableHeight).isEqualTo(2f);
        assertThat(tc.getParams().walkableRadius).isEqualTo(0.6f);
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
        assertThat(data.verts[1]).isEqualTo(14.997294f, offset(0.0001f));
        assertThat(data.verts[6]).isEqualTo(15.484785f, offset(0.0001f));
        assertThat(data.verts[9]).isEqualTo(15.484785f, offset(0.0001f));
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
