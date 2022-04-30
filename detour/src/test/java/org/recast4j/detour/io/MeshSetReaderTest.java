/*
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
package org.recast4j.detour.io;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.offset;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.recast4j.detour.MeshTile;
import org.recast4j.detour.NavMesh;

public class MeshSetReaderTest {

    private final MeshSetReader reader = new MeshSetReader();

    @Test
    public void testNavmesh() throws IOException {
        InputStream is = getClass().getClassLoader().getResourceAsStream("all_tiles_navmesh.bin");
        NavMesh mesh = reader.read(is, 6);
        assertThat(mesh.getMaxTiles()).isEqualTo(128);
        assertThat(mesh.getParams().maxPolys).isEqualTo(0x8000);
        assertThat(mesh.getParams().tileWidth).isEqualTo(9.6f, offset(0.001f));
        List<MeshTile> tiles = mesh.getTilesAt(4, 7);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(7);
        assertThat(tiles.get(0).data.verts).hasSize(22 * 3);
        tiles = mesh.getTilesAt(1, 6);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(7);
        assertThat(tiles.get(0).data.verts).hasSize(26 * 3);
        tiles = mesh.getTilesAt(6, 2);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(1);
        assertThat(tiles.get(0).data.verts).hasSize(4 * 3);
        tiles = mesh.getTilesAt(7, 6);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(8);
        assertThat(tiles.get(0).data.verts).hasSize(24 * 3);
    }

    @Test
    public void testDungeon() throws IOException {
        InputStream is = getClass().getClassLoader().getResourceAsStream("dungeon_all_tiles_navmesh.bin");
        NavMesh mesh = reader.read(is, 6);
        assertThat(mesh.getMaxTiles()).isEqualTo(128);
        assertThat(mesh.getParams().maxPolys).isEqualTo(0x8000);
        assertThat(mesh.getParams().tileWidth).isEqualTo(9.6f, offset(0.001f));
        List<MeshTile> tiles = mesh.getTilesAt(6, 9);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(2);
        assertThat(tiles.get(0).data.verts).hasSize(7 * 3);
        tiles = mesh.getTilesAt(2, 9);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(2);
        assertThat(tiles.get(0).data.verts).hasSize(9 * 3);
        tiles = mesh.getTilesAt(4, 3);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(3);
        assertThat(tiles.get(0).data.verts).hasSize(6 * 3);
        tiles = mesh.getTilesAt(2, 8);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(5);
        assertThat(tiles.get(0).data.verts).hasSize(17 * 3);
    }

    @Test
    public void testDungeon32Bit() throws IOException {
        InputStream is = getClass().getClassLoader().getResourceAsStream("dungeon_all_tiles_navmesh_32bit.bin");
        NavMesh mesh = reader.read32Bit(is, 6);
        assertThat(mesh.getMaxTiles()).isEqualTo(128);
        assertThat(mesh.getParams().maxPolys).isEqualTo(0x8000);
        assertThat(mesh.getParams().tileWidth).isEqualTo(9.6f, offset(0.001f));
        List<MeshTile> tiles = mesh.getTilesAt(6, 9);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(2);
        assertThat(tiles.get(0).data.verts).hasSize(7 * 3);
        tiles = mesh.getTilesAt(2, 9);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(2);
        assertThat(tiles.get(0).data.verts).hasSize(9 * 3);
        tiles = mesh.getTilesAt(4, 3);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(3);
        assertThat(tiles.get(0).data.verts).hasSize(6 * 3);
        tiles = mesh.getTilesAt(2, 8);
        assertThat(tiles).hasSize(1);
        assertThat(tiles.get(0).data.polys).hasSize(5);
        assertThat(tiles.get(0).data.verts).hasSize(17 * 3);
    }
}
