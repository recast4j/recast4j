/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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
package org.recast4j.recast;

import static org.recast4j.recast.RecastConstants.RC_NULL_AREA;
import static org.recast4j.recast.RecastConstants.SPAN_MAX_HEIGHT;

public class RecastFilter {

    /// @par
    ///
    /// Allows the formation of walkable regions that will flow over low lying
    /// objects such as curbs, and up structures such as stairways.
    ///
    /// Two neighboring spans are walkable if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) < walkableClimb</tt>
    ///
    /// @warning Will override the effect of #rcFilterLedgeSpans. So if both filters are used, call
    /// #rcFilterLedgeSpans after calling this filter.
    ///
    /// @see rcHeightfield, rcConfig
    public static void filterLowHangingWalkableObstacles(Telemetry ctx, int walkableClimb, Heightfield heightfield) {

        ctx.startTimer("FILTER_LOW_OBSTACLES");

        int xSize = heightfield.width;
        int zSize = heightfield.height;

        for (int z = 0; z < zSize; ++z) {
            for (int x = 0; x < xSize; ++x) {
                Span previousSpan = null;
                boolean previousWasWalkable = false;
                int previousArea = RC_NULL_AREA;

                for (Span span = heightfield.spans[x + z * xSize]; span != null; previousSpan = span, span = span.next) {
                    boolean walkable = span.area != RC_NULL_AREA;
                    // If current span is not walkable, but there is walkable
                    // span just below it, mark the span above it walkable too.
                    if (!walkable && previousWasWalkable) {
                        if (Math.abs(span.smax - previousSpan.smax) <= walkableClimb)
                            span.area = previousArea;
                    }
                    // Copy walkable flag so that it cannot propagate
                    // past multiple non-walkable objects.
                    previousWasWalkable = walkable;
                    previousArea = span.area;
                }
            }
        }

        ctx.stopTimer("FILTER_LOW_OBSTACLES");
    }

    /// @par
    ///
    /// A ledge is a span with one or more neighbors whose maximum is further away than @p walkableClimb
    /// from the current span's maximum.
    /// This method removes the impact of the overestimation of conservative voxelization
    /// so the resulting mesh will not have regions hanging in the air over ledges.
    ///
    /// A span is a ledge if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
    ///
    /// @see rcHeightfield, rcConfig
    public static void filterLedgeSpans(Telemetry ctx, int walkableHeight, int walkableClimb, Heightfield heightfield) {
        ctx.startTimer("FILTER_LEDGE");

        int xSize = heightfield.width;
        int zSize = heightfield.height;

        // Mark border spans.
        for (int z = 0; z < zSize; ++z) {
            for (int x = 0; x < xSize; ++x) {
                for (Span span = heightfield.spans[x + z * xSize]; span != null; span = span.next) {
                    // Skip non walkable spans.
                    if (span.area == RC_NULL_AREA)
                        continue;

                    int bot = (span.smax);
                    int top = span.next != null ? span.next.smin : SPAN_MAX_HEIGHT;

                    // Find neighbours minimum height.
                    int minNeighborHeight = SPAN_MAX_HEIGHT;

                    // Min and max height of accessible neighbours.
                    int accessibleNeighborMinHeight = span.smax;
                    int accessibleNeighborMaxHeight = span.smax;

                    for (int direction = 0; direction < 4; ++direction) {
                        int dx = x + RecastCommon.GetDirOffsetX(direction);
                        int dz = z + RecastCommon.GetDirOffsetY(direction);
                        // Skip neighbours which are out of bounds.
                        if (dx < 0 || dz < 0 || dx >= xSize || dz >= zSize) {
                            minNeighborHeight = -walkableClimb - 1;
                            break;
                        }

                        // From minus infinity to the first span.
                        Span neighborSpan = heightfield.spans[dx + dz * xSize];
                        int neighborTop = neighborSpan != null ? neighborSpan.smin : SPAN_MAX_HEIGHT;
                        // Skip neightbour if the gap between the spans is too small.
                        if (Math.min(top, neighborTop) - bot >= walkableHeight) {
                            minNeighborHeight = -walkableClimb - 1;
                            break;
                        }

                        // Rest of the spans.
                        for (neighborSpan = heightfield.spans[dx + dz * xSize]; neighborSpan != null; neighborSpan = neighborSpan.next) {
                            int neighborBot = neighborSpan.smax;
                            neighborTop = neighborSpan.next != null ? neighborSpan.next.smin : SPAN_MAX_HEIGHT;
                            // Skip neightbour if the gap between the spans is too small.
                            if (Math.min(top, neighborTop) - Math.max(bot, neighborBot) >= walkableHeight) {
                                int accessibleNeighbourHeight = neighborBot - bot;
                                minNeighborHeight = Math.min(minNeighborHeight, accessibleNeighbourHeight);

                                // Find min/max accessible neighbour height.
                                if (Math.abs(accessibleNeighbourHeight) <= walkableClimb) {
                                    if (neighborBot < accessibleNeighborMinHeight)
                                        accessibleNeighborMinHeight = neighborBot;
                                    if (neighborBot > accessibleNeighborMaxHeight)
                                        accessibleNeighborMaxHeight = neighborBot;
                                } else if (accessibleNeighbourHeight < -walkableClimb) {
                                    break;
                                }

                            }
                        }
                    }

                    // The current span is close to a ledge if the drop to any
                    // neighbour span is less than the walkableClimb.
                    if (minNeighborHeight < -walkableClimb)
                        span.area = RC_NULL_AREA;

                    // If the difference between all neighbours is too large,
                    // we are at steep slope, mark the span as ledge.
                    if ((accessibleNeighborMaxHeight - accessibleNeighborMinHeight) > walkableClimb) {
                        span.area = RC_NULL_AREA;
                    }
                }
            }
        }

        ctx.stopTimer("FILTER_LEDGE");
    }

    /// @par
    ///
    /// For this filter, the clearance above the span is the distance from the span's
    /// maximum to the next higher span's minimum. (Same grid column.)
    ///
    /// @see rcHeightfield, rcConfig
    public static void filterWalkableLowHeightSpans(Telemetry ctx, int walkableHeight, Heightfield heightfield) {
        ctx.startTimer("FILTER_WALKABLE");

        int xSize = heightfield.width;
        int zSize = heightfield.height;

        // Remove walkable flag from spans which do not have enough
        // space above them for the agent to stand there.
        for (int z = 0; z < zSize; ++z) {
            for (int x = 0; x < xSize; ++x) {
                for (Span span = heightfield.spans[x + z * xSize]; span != null; span = span.next) {
                    int bot = (span.smax);
                    int top = span.next != null ? span.next.smin : SPAN_MAX_HEIGHT;
                    if ((top - bot) < walkableHeight)
                        span.area = RC_NULL_AREA;
                }
            }
        }
        ctx.stopTimer("FILTER_WALKABLE");
    }
}
