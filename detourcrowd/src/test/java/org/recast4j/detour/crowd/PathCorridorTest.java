/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org

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

package org.recast4j.detour.crowd;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Matchers.any;
import static org.mockito.Matchers.anyInt;
import static org.mockito.Mockito.when;

import java.util.ArrayList;
import java.util.List;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.runners.MockitoJUnitRunner;
import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;
import org.recast4j.detour.StraightPathItem;

@RunWith(MockitoJUnitRunner.class)
public class PathCorridorTest {

    private final PathCorridor corridor = new PathCorridor();
    private final QueryFilter filter = new DefaultQueryFilter();
    @Mock
    private NavMeshQuery query;

    @Before
    public void setUp() {
        corridor.m_pos[0] = 10;
        corridor.m_pos[1] = 20;
        corridor.m_pos[2] = 30;
    }

    @Test
    public void shouldKeepOriginalPathInFindCornersWhenNothingCanBePruned() {
        List<StraightPathItem> straightPath = new ArrayList<>();
        straightPath.add(new StraightPathItem(new float[] { 11, 20, 30.00001f }, 0, 0));
        straightPath.add(new StraightPathItem(new float[] { 12, 20, 30.00002f }, 0, 0));
        straightPath.add(new StraightPathItem(new float[] { 11f, 21, 32f }, 0, 0));
        straightPath.add(new StraightPathItem(new float[] { 11f, 21, 32f }, 0, 0));
        Result<List<StraightPathItem>> result = Result.success(straightPath);
        when(query.findStraightPath(any(), any(), any(), anyInt(), anyInt())).thenReturn(result);
        List<StraightPathItem> path = corridor.findCorners(Integer.MAX_VALUE, query, filter);
        assertThat(path).hasSize(4);
        assertThat(path).containsExactlyElementsOf(straightPath);
    }

    @Test
    public void shouldPrunePathInFindCorners() {
        List<StraightPathItem> straightPath = new ArrayList<>();
        straightPath.add(new StraightPathItem(new float[] { 10, 20, 30.00001f }, 0, 0)); // too close
        straightPath.add(new StraightPathItem(new float[] { 10, 20, 30.00002f }, 0, 0)); // too close
        straightPath.add(new StraightPathItem(new float[] { 11f, 21, 32f }, 0, 0));
        straightPath.add(new StraightPathItem(new float[] { 12f, 22, 33f }, NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION, 0)); // offmesh
        straightPath.add(new StraightPathItem(new float[] { 11f, 21, 32f }, NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION, 0)); // offmesh
        Result<List<StraightPathItem>> result = Result.success(straightPath);
        when(query.findStraightPath(any(), any(), any(), anyInt(), anyInt())).thenReturn(result);
        List<StraightPathItem> path = corridor.findCorners(Integer.MAX_VALUE, query, filter);
        assertThat(path).hasSize(2);
        assertThat(path).containsExactly(straightPath.get(2), straightPath.get(3));
    }

}
