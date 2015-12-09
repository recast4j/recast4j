package org.recast4j.detour.tilecache.io;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteOrder;

import org.junit.Test;
import org.recast4j.detour.MeshTile;
import org.recast4j.detour.NavMesh;

public class MeshSetReaderTest {

	private final MeshSetReader reader= new MeshSetReader();
	
	@Test
	public void testCCompatibility() throws IOException {
		InputStream is = getClass().getClassLoader().getResourceAsStream("all_tiles_navmesh.bin");
		NavMesh mesh = reader.read(is, ByteOrder.LITTLE_ENDIAN, true);
		assertEquals(128, mesh.getMaxTiles());
		assertEquals(0x8000, mesh.getParams().maxPolys);
		assertEquals(9,6, mesh.getParams().tileWidth);
		MeshTile tile0 = mesh.getTile(0);
		
		MeshTile tile20 = mesh.getTile(20);
	}
}
