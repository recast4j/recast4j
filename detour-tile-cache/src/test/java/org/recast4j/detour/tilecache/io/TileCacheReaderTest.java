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

import java.io.IOException;
import java.io.InputStream;

import org.junit.jupiter.api.Test;
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
        // Tile0: Tris: 1, Verts: 4 Detail Meshed: 1 Detail Verts: 0 Detail Tris: 2
        // Verts: -2.269517, 28.710686, 28.710686
        MeshTile tile = tc.getNavMesh().getTile(0);
        MeshData data = tile.data;
        MeshHeader header = data.header;
        assertThat(header.vertCount).isEqualTo(4);
        assertThat(header.polyCount).isEqualTo(1);
        assertThat(header.detailMeshCount).isEqualTo(1);
        assertThat(header.detailVertCount).isEqualTo(0);
        assertThat(header.detailTriCount).isEqualTo(2);
        assertThat(data.polys.length).isEqualTo(1);
        assertThat(data.verts.length).isEqualTo(3 * 4);
        assertThat(data.detailMeshes.length).isEqualTo(1);
        assertThat(data.detailVerts.length).isEqualTo(0);
        assertThat(data.detailTris.length).isEqualTo(4 * 2);
        assertThat(data.verts[1]).isEqualTo(-2.269517f, offset(0.0001f));
        assertThat(data.verts[6]).isEqualTo(28.710686f, offset(0.0001f));
        assertThat(data.verts[9]).isEqualTo(28.710686f, offset(0.0001f));
        // Tile8: Tris: 7, Verts: 10 Detail Meshed: 7 Detail Verts: 0 Detail Tris: 10
        // Verts: 0.330483, 43.110687, 43.110687
        tile = tc.getNavMesh().getTile(8);
        data = tile.data;
        header = data.header;
        System.out.println(data.header.x + "  " + data.header.y + "  " + data.header.layer);
        assertThat(header.x).isEqualTo(4);
        assertThat(header.y).isEqualTo(1);
        assertThat(header.layer).isEqualTo(0);
        assertThat(header.vertCount).isEqualTo(10);
        assertThat(header.polyCount).isEqualTo(7);
        assertThat(header.detailMeshCount).isEqualTo(7);
        assertThat(header.detailVertCount).isEqualTo(0);
        assertThat(header.detailTriCount).isEqualTo(10);
        assertThat(data.polys.length).isEqualTo(7);
        assertThat(data.verts.length).isEqualTo(3 * 10);
        assertThat(data.detailMeshes.length).isEqualTo(7);
        assertThat(data.detailVerts.length).isEqualTo(0);
        assertThat(data.detailTris.length).isEqualTo(4 * 10);
        assertThat(data.verts[1]).isEqualTo(0.330483f, offset(0.0001f));
        assertThat(data.verts[6]).isEqualTo(43.110687f, offset(0.0001f));
        assertThat(data.verts[9]).isEqualTo(43.110687f, offset(0.0001f));
        // Tile16: Tris: 13, Verts: 33 Detail Meshed: 13 Detail Verts: 0 Detail Tris: 25
        // Verts: 1.130483, 5.610685, 6.510685
        tile = tc.getNavMesh().getTile(16);
        data = tile.data;
        header = data.header;
        assertThat(header.vertCount).isEqualTo(33);
        assertThat(header.polyCount).isEqualTo(13);
        assertThat(header.detailMeshCount).isEqualTo(13);
        assertThat(header.detailVertCount).isEqualTo(0);
        assertThat(header.detailTriCount).isEqualTo(25);
        assertThat(data.polys.length).isEqualTo(13);
        assertThat(data.verts.length).isEqualTo(3 * 33);
        assertThat(data.detailMeshes.length).isEqualTo(13);
        assertThat(data.detailVerts.length).isEqualTo(0);
        assertThat(data.detailTris.length).isEqualTo(4 * 25);
        assertThat(data.verts[1]).isEqualTo(1.130483f, offset(0.0001f));
        assertThat(data.verts[6]).isEqualTo(5.610685f, offset(0.0001f));
        assertThat(data.verts[9]).isEqualTo(6.510685f, offset(0.0001f));
        // Tile29: Tris: 5, Verts: 15 Detail Meshed: 5 Detail Verts: 0 Detail Tris: 11
        // Verts: 10.330483, 10.110685, 10.110685
        tile = tc.getNavMesh().getTile(29);
        data = tile.data;
        header = data.header;
        assertThat(header.vertCount).isEqualTo(15);
        assertThat(header.polyCount).isEqualTo(5);
        assertThat(header.detailMeshCount).isEqualTo(5);
        assertThat(header.detailVertCount).isEqualTo(0);
        assertThat(header.detailTriCount).isEqualTo(11);
        assertThat(data.polys.length).isEqualTo(5);
        assertThat(data.verts.length).isEqualTo(3 * 15);
        assertThat(data.detailMeshes.length).isEqualTo(5);
        assertThat(data.detailVerts.length).isEqualTo(0);
        assertThat(data.detailTris.length).isEqualTo(4 * 11);
        assertThat(data.verts[1]).isEqualTo(10.330483f, offset(0.0001f));
        assertThat(data.verts[6]).isEqualTo(10.110685f, offset(0.0001f));
        assertThat(data.verts[9]).isEqualTo(10.110685f, offset(0.0001f));
    }

    @Test
    public void testDungeon() throws IOException {
        InputStream is = getClass().getClassLoader().getResourceAsStream("dungeon_all_tiles_tilecache.bin");
        TileCache tc = reader.read(is, 6, null);
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
        // Verts: 14.997294, 15.484785, 15.484785
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
        // Verts: 13.597294, 17.584785, 17.584785
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
        assertThat(data.verts[1]).isEqualTo(13.597294f, offset(0.0001f));
        assertThat(data.verts[6]).isEqualTo(17.584785f, offset(0.0001f));
        assertThat(data.verts[9]).isEqualTo(17.584785f, offset(0.0001f));
        // Tile16: Tris: 10, Verts: 20 Detail Meshed: 10 Detail Verts: 0 Detail Tris: 18
        // Verts: 6.197294, -22.315216, -22.315216
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
        assertThat(data.verts[1]).isEqualTo(6.197294f, offset(0.0001f));
        assertThat(data.verts[6]).isEqualTo(-22.315216f, offset(0.0001f));
        assertThat(data.verts[9]).isEqualTo(-22.315216f, offset(0.0001f));
        // Tile29: Tris: 1, Verts: 5 Detail Meshed: 1 Detail Verts: 0 Detail Tris: 3
        // Verts: 10.197294, 48.484783, 48.484783
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
        assertThat(data.verts[1]).isEqualTo(10.197294f, offset(0.0001f));
        assertThat(data.verts[6]).isEqualTo(48.484783f, offset(0.0001f));
        assertThat(data.verts[9]).isEqualTo(48.484783f, offset(0.0001f));
    }

}
