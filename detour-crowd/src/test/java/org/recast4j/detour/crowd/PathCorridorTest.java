/*
recast4j copyright (c) 2021-2026 Piotr Piastucki piotr@jtilia.org

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
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.when;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;
import org.recast4j.detour.StraightPathItem;

@ExtendWith(MockitoExtension.class)
public class PathCorridorTest {

    private final PathCorridor corridor = new PathCorridor();
    private final QueryFilter filter = new DefaultQueryFilter();
    @Mock
    private NavMeshQuery query;

    @BeforeEach
    public void setUp() {
        corridor.reset(0, new float[] {10,20,30});
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

    @Test
    public void testMergeCorridorStartMovedEmptyInput() {
        List<Long> path = new ArrayList<>();
        List<Long> visited = new ArrayList<>();
        List<Long> result = PathCorridor.mergeCorridorStartMoved(path, visited);
        assertThat(result).isEmpty();
    }

    @Test
    public void testMergeCorridorStartMovedEmptyVisited() {
        List<Long> path = new ArrayList<>();
        path.add(1L);
        List<Long> visited = new ArrayList<>();
        List<Long> result = PathCorridor.mergeCorridorStartMoved(path, visited);
        assertThat(result).hasSize(1).containsExactly(1L);
    }

    @Test
    public void testMergeCorridorStartMovedEmptyPath() {
        List<Long> path = new ArrayList<>();
        List<Long> visited = new ArrayList<>();
        visited.add(1L);
        List<Long> result = PathCorridor.mergeCorridorStartMoved(path, visited);
        assertThat(result).isEmpty();
    }

    @Test
    public void testMergeCorridorStartMovedStripVisitedPointsFromPath() {
        List<Long> path = new ArrayList<>();
        path.add(1L);
        path.add(2L);
        List<Long> visited = new ArrayList<>();
        visited.add(1L);
        visited.add(2L);
        List<Long> result = PathCorridor.mergeCorridorStartMoved(path, visited);
        assertThat(result).hasSize(1).containsExactly(2L);
    }

    @Test
    public void testMergeCorridorStartMovedAddVisitedPointsNotInPath() {
        List<Long> path = new ArrayList<>();
        path.add(1L);
        path.add(2L);
        List<Long> visited = new ArrayList<>();
        visited.add(1L);
        visited.add(2L);
        visited.add(3L);
        visited.add(4L);
        List<Long> result = PathCorridor.mergeCorridorStartMoved(path, visited);
        assertThat(result).hasSize(3).containsExactly(4L, 3L, 2L);
    }

    @Test
    public void testMergeCorridorStartMovedAddVisitedPointsUpToCapacity() {
        List<Long> path = new ArrayList<>();
        path.add(1L);
        path.add(2L);
        List<Long> visited = new ArrayList<>();
        visited.add(1L);
        visited.add(2L);
        visited.add(3L);
        visited.add(4L);
        visited.add(5L);
        List<Long> result = PathCorridor.mergeCorridorStartMoved(path, visited);
        // All visited items after the common point (2) plus the path from that point onwards
        assertThat(result).hasSize(4).containsExactly(5L, 4L, 3L, 2L);
    }

    @Test
    public void testMergeCorridorStartMovedNoIntersectionWithVisited() {
        List<Long> path = new ArrayList<>();
        path.add(1L);
        path.add(2L);
        List<Long> visited = new ArrayList<>();
        visited.add(3L);
        visited.add(4L);
        List<Long> result = PathCorridor.mergeCorridorStartMoved(path, visited);
        // Should return original path if there's no intersection
        assertThat(result).hasSize(2).containsExactly(1L, 2L);
    }

    @Test
    public void testMergeCorridorStartMovedSaveUnvisitedPathPoints() {
        List<Long> path = new ArrayList<>();
        path.add(1L);
        path.add(2L);
        List<Long> visited = new ArrayList<>();
        visited.add(1L);
        visited.add(3L);
        List<Long> result = PathCorridor.mergeCorridorStartMoved(path, visited);
        assertThat(result).hasSize(3).containsExactly(3L, 1L, 2L);
    }

}