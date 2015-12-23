package org.recast4j.detour.io;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.Test;
import org.recast4j.detour.MeshTile;
import org.recast4j.detour.NavMesh;

public class MeshSetReaderTest {

	private final MeshSetReader reader = new MeshSetReader();

	@Test
	public void testNavmesh() throws IOException {
		InputStream is = getClass().getClassLoader().getResourceAsStream("all_tiles_navmesh.bin");
		NavMesh mesh = reader.read(is, ByteOrder.LITTLE_ENDIAN, true);
		assertEquals(128, mesh.getMaxTiles());
		assertEquals(0x8000, mesh.getParams().maxPolys);
		assertEquals(9.6, mesh.getParams().tileWidth, 0.001);
		List<MeshTile> tiles = mesh.getTilesAt(4, 7);
		assertEquals(1, tiles.size());
		assertEquals(7, tiles.get(0).data.polys.length);
		assertEquals(22 * 3, tiles.get(0).data.verts.length);
		tiles = mesh.getTilesAt(1, 6);
		assertEquals(1, tiles.size());
		assertEquals(7, tiles.get(0).data.polys.length);
		assertEquals(26 * 3, tiles.get(0).data.verts.length);
		tiles = mesh.getTilesAt(6, 2);
		assertEquals(1, tiles.size());
		assertEquals(1, tiles.get(0).data.polys.length);
		assertEquals(4 * 3, tiles.get(0).data.verts.length);
		tiles = mesh.getTilesAt(7, 6);
		assertEquals(1, tiles.size());
		assertEquals(8, tiles.get(0).data.polys.length);
		assertEquals(24 * 3, tiles.get(0).data.verts.length);
	}

	@Test
	public void testDungeon() throws IOException {
		InputStream is = getClass().getClassLoader().getResourceAsStream("dungeon_all_tiles_navmesh.bin");
		NavMesh mesh = reader.read(is, ByteOrder.LITTLE_ENDIAN, true);
		assertEquals(128, mesh.getMaxTiles());
		assertEquals(0x8000, mesh.getParams().maxPolys);
		assertEquals(9.6, mesh.getParams().tileWidth, 0.001);
		List<MeshTile> tiles = mesh.getTilesAt(6, 9);
		assertEquals(1, tiles.size());
		assertEquals(2, tiles.get(0).data.polys.length);
		assertEquals(7 * 3, tiles.get(0).data.verts.length);
		tiles = mesh.getTilesAt(2, 9);
		assertEquals(1, tiles.size());
		assertEquals(2, tiles.get(0).data.polys.length);
		assertEquals(9 * 3, tiles.get(0).data.verts.length);
		tiles = mesh.getTilesAt(4, 3);
		assertEquals(1, tiles.size());
		assertEquals(3, tiles.get(0).data.polys.length);
		assertEquals(6 * 3, tiles.get(0).data.verts.length);
		tiles = mesh.getTilesAt(2, 8);
		assertEquals(1, tiles.size());
		assertEquals(5, tiles.get(0).data.polys.length);
		assertEquals(17 * 3, tiles.get(0).data.verts.length);
	}
}
