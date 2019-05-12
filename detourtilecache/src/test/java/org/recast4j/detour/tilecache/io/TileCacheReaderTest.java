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

import java.io.IOException;
import java.io.InputStream;

import org.junit.Test;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.MeshHeader;
import org.recast4j.detour.MeshTile;
import org.recast4j.detour.tilecache.TileCache;

public class TileCacheReaderTest {

    private final TileCacheReader reader = new TileCacheReader();

    @Test
    public void testNavmesh() throws IOException {

        InputStream is = getClass().getClassLoader().getResourceAsStream("all_tiles_tilecache.bin");
        TileCache tc = reader.read(is, 6, null);
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
        // Tile0: Tris: 1, Verts: 4 Detail Meshed: 1 Detail Verts: 0 Detail Tris: 2
        // Verts: -2.269517, 28.710686, 28.710686
        MeshTile tile = tc.getNavMesh().getTile(0);
        MeshData data = tile.data;
        MeshHeader header = data.header;
        assertEquals(4, header.vertCount);
        assertEquals(1, header.polyCount);
        assertEquals(1, header.detailMeshCount);
        assertEquals(0, header.detailVertCount);
        assertEquals(2, header.detailTriCount);
        assertEquals(1, data.polys.length);
        assertEquals(3 * 4, data.verts.length);
        assertEquals(1, data.detailMeshes.length);
        assertEquals(0, data.detailVerts.length);
        assertEquals(4 * 2, data.detailTris.length);
        assertEquals(-2.269517f, data.verts[1], 0.0001f);
        assertEquals(28.710686f, data.verts[6], 0.0001f);
        assertEquals(28.710686f, data.verts[9], 0.0001f);
        // Tile8: Tris: 7, Verts: 10 Detail Meshed: 7 Detail Verts: 0 Detail Tris: 10
        // Verts: 0.330483, 43.110687, 43.110687
        tile = tc.getNavMesh().getTile(8);
        data = tile.data;
        header = data.header;
        System.out.println(data.header.x + "  " + data.header.y + "  " + data.header.layer);
        assertEquals(4, header.x);
        assertEquals(1, header.y);
        assertEquals(0, header.layer);
        assertEquals(10, header.vertCount);
        assertEquals(7, header.polyCount);
        assertEquals(7, header.detailMeshCount);
        assertEquals(0, header.detailVertCount);
        assertEquals(10, header.detailTriCount);
        assertEquals(7, data.polys.length);
        assertEquals(3 * 10, data.verts.length);
        assertEquals(7, data.detailMeshes.length);
        assertEquals(0, data.detailVerts.length);
        assertEquals(4 * 10, data.detailTris.length);
        assertEquals(0.330483f, data.verts[1], 0.0001f);
        assertEquals(43.110687f, data.verts[6], 0.0001f);
        assertEquals(43.110687f, data.verts[9], 0.0001f);
        // Tile16: Tris: 13, Verts: 33 Detail Meshed: 13 Detail Verts: 0 Detail Tris: 25
        // Verts: 1.130483, 5.610685, 6.510685
        tile = tc.getNavMesh().getTile(16);
        data = tile.data;
        header = data.header;
        assertEquals(33, header.vertCount);
        assertEquals(13, header.polyCount);
        assertEquals(13, header.detailMeshCount);
        assertEquals(0, header.detailVertCount);
        assertEquals(25, header.detailTriCount);
        assertEquals(13, data.polys.length);
        assertEquals(3 * 33, data.verts.length);
        assertEquals(13, data.detailMeshes.length);
        assertEquals(0, data.detailVerts.length);
        assertEquals(4 * 25, data.detailTris.length);
        assertEquals(1.130483f, data.verts[1], 0.0001f);
        assertEquals(5.610685f, data.verts[6], 0.0001f);
        assertEquals(6.510685f, data.verts[9], 0.0001f);
        // Tile29: Tris: 5, Verts: 15 Detail Meshed: 5 Detail Verts: 0 Detail Tris: 11
        // Verts: 10.330483, 10.110685, 10.110685
        tile = tc.getNavMesh().getTile(29);
        data = tile.data;
        header = data.header;
        assertEquals(15, header.vertCount);
        assertEquals(5, header.polyCount);
        assertEquals(5, header.detailMeshCount);
        assertEquals(0, header.detailVertCount);
        assertEquals(11, header.detailTriCount);
        assertEquals(5, data.polys.length);
        assertEquals(3 * 15, data.verts.length);
        assertEquals(5, data.detailMeshes.length);
        assertEquals(0, data.detailVerts.length);
        assertEquals(4 * 11, data.detailTris.length);
        assertEquals(10.330483f, data.verts[1], 0.0001f);
        assertEquals(10.110685f, data.verts[6], 0.0001f);
        assertEquals(10.110685f, data.verts[9], 0.0001f);
    }

    @Test
    public void testDungeon() throws IOException {
        InputStream is = getClass().getClassLoader().getResourceAsStream("dungeon_all_tiles_tilecache.bin");
        TileCache tc = reader.read(is, 6, null);
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
        // Verts: 14.997294, 15.484785, 15.484785
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
        assertEquals(14.997294f, data.verts[1], 0.0001f);
        assertEquals(15.484785f, data.verts[6], 0.0001f);
        assertEquals(15.484785f, data.verts[9], 0.0001f);
        // Tile8: Tris: 3, Verts: 8 Detail Meshed: 3 Detail Verts: 0 Detail Tris: 6
        // Verts: 13.597294, 17.584785, 17.584785
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
        assertEquals(13.597294f, data.verts[1], 0.0001f);
        assertEquals(17.584785f, data.verts[6], 0.0001f);
        assertEquals(17.584785f, data.verts[9], 0.0001f);
        // Tile16: Tris: 10, Verts: 20 Detail Meshed: 10 Detail Verts: 0 Detail Tris: 18
        // Verts: 6.197294, -22.315216, -22.315216
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
        assertEquals(6.197294f, data.verts[1], 0.0001f);
        assertEquals(-22.315216f, data.verts[6], 0.0001f);
        assertEquals(-22.315216f, data.verts[9], 0.0001f);
        // Tile29: Tris: 1, Verts: 5 Detail Meshed: 1 Detail Verts: 0 Detail Tris: 3
        // Verts: 10.197294, 48.484783, 48.484783
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
        assertEquals(10.197294f, data.verts[1], 0.0001f);
        assertEquals(48.484783f, data.verts[6], 0.0001f);
        assertEquals(48.484783f, data.verts[9], 0.0001f);
    }

}
