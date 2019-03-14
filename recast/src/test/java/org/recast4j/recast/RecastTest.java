package org.recast4j.recast;

import static org.junit.Assert.assertEquals;
import static org.recast4j.recast.RecastConstants.RC_NULL_AREA;

import org.junit.Test;

public class RecastTest {

    @Test
    public void textClearUnwalkableTriangles() {
        float walkableSlopeAngle = 45;
        float verts[] = { 0, 0, 0, 1, 0, 0, 0, 0, -1 };
        int nv = 3;
        int walkable_tri[] = { 0, 1, 2 };
        int unwalkable_tri[] = { 0, 2, 1 };
        int nt = 1;

        Context ctx = new Context();
        {
            int areas[] = { 42 };
            Recast.clearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas);
            assertEquals("Sets area ID of unwalkable triangle to RC_NULL_AREA", RC_NULL_AREA, areas[0]);
        }
        {
            int areas[] = { 42 };
            Recast.clearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
            assertEquals("Does not modify walkable triangle aread ID's", 42, areas[0]);
        }
        {
            int areas[] = { 42 };
            walkableSlopeAngle = 0;
            Recast.clearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
            assertEquals("Slopes equal to the max slope are considered unwalkable.", RC_NULL_AREA, areas[0]);
        }
    }
}
