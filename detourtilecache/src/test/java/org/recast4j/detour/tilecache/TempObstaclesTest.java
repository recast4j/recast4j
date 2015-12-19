package org.recast4j.detour.tilecache;

import org.junit.Test;
import org.recast4j.recast.InputGeom;
import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.RecastBuilder;

public class TempObstaclesTest extends AbstractTileCacheTest {

	@Test
	public void testDungeon() {
		InputGeom geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream("dungeon.obj"));
	}
}
