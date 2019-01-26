/*
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
package org.recast4j.demo.settings;

import static org.lwjgl.nuklear.Nuklear.NK_TEXT_ALIGN_LEFT;
import static org.lwjgl.nuklear.Nuklear.NK_TEXT_ALIGN_RIGHT;
import static org.lwjgl.nuklear.Nuklear.NK_WINDOW_BORDER;
import static org.lwjgl.nuklear.Nuklear.NK_WINDOW_MOVABLE;
import static org.lwjgl.nuklear.Nuklear.NK_WINDOW_TITLE;
import static org.lwjgl.nuklear.Nuklear.nk_begin;
import static org.lwjgl.nuklear.Nuklear.nk_button_text;
import static org.lwjgl.nuklear.Nuklear.nk_check_text;
import static org.lwjgl.nuklear.Nuklear.nk_end;
import static org.lwjgl.nuklear.Nuklear.nk_label;
import static org.lwjgl.nuklear.Nuklear.nk_layout_row_dynamic;
import static org.lwjgl.nuklear.Nuklear.nk_option_text;
import static org.lwjgl.nuklear.Nuklear.nk_property_float;
import static org.lwjgl.nuklear.Nuklear.nk_property_int;
import static org.lwjgl.nuklear.Nuklear.nk_rect;
import static org.lwjgl.nuklear.Nuklear.nk_rgb;
import static org.lwjgl.nuklear.Nuklear.nk_rgba;
import static org.lwjgl.nuklear.Nuklear.nk_spacing;
import static org.lwjgl.nuklear.Nuklear.nk_style_item_color;
import static org.lwjgl.nuklear.Nuklear.nk_window_get_bounds;
import static org.lwjgl.system.MemoryStack.stackPush;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import org.lwjgl.BufferUtils;
import org.lwjgl.nuklear.NkColor;
import org.lwjgl.nuklear.NkContext;
import org.lwjgl.nuklear.NkRect;
import org.lwjgl.nuklear.NkStyleItem;
import org.lwjgl.system.MemoryStack;
import org.recast4j.demo.draw.DrawMode;
import org.recast4j.demo.ui.NuklearUIHelper;
import org.recast4j.demo.ui.NuklearUIModule;
import org.recast4j.recast.RecastConstants.PartitionType;

public class SettingsUI implements NuklearUIModule {

    private final FloatBuffer cellSize = BufferUtils.createFloatBuffer(1).put(0, 0.3f);
    private final FloatBuffer cellHeight = BufferUtils.createFloatBuffer(1).put(0, 0.2f);

    private final FloatBuffer agentHeight = BufferUtils.createFloatBuffer(1).put(0, 2f);
    private final FloatBuffer agentRadius = BufferUtils.createFloatBuffer(1).put(0, 0.6f);
    private final FloatBuffer agentMaxClimb = BufferUtils.createFloatBuffer(1).put(0, 0.9f);
    private final FloatBuffer agentMaxSlope = BufferUtils.createFloatBuffer(1).put(0, 45f);

    private final IntBuffer minRegionSize = BufferUtils.createIntBuffer(1).put(0, 8);
    private final IntBuffer mergedRegionSize = BufferUtils.createIntBuffer(1).put(0, 20);

    private PartitionType partitioning = PartitionType.WATERSHED;

    private boolean filterLowHangingObstacles = true;
    private boolean filterLedgeSpans = true;
    private boolean filterWalkableLowHeightSpans = true;

    private final FloatBuffer edgeMaxLen = BufferUtils.createFloatBuffer(1).put(0, 12f);
    private final FloatBuffer edgeMaxError = BufferUtils.createFloatBuffer(1).put(0, 1.3f);
    private final IntBuffer vertsPerPoly = BufferUtils.createIntBuffer(1).put(0, 6);

    private final FloatBuffer detailSampleDist = BufferUtils.createFloatBuffer(1).put(0, 6f);
    private final FloatBuffer detailSampleMaxError = BufferUtils.createFloatBuffer(1).put(0, 1f);

    private boolean tiled = false;
    private final IntBuffer tileSize = BufferUtils.createIntBuffer(1).put(0, 32);

    public final NkColor white = NkColor.create();
    public final NkColor background = NkColor.create();
    public final NkColor transparent = NkColor.create();
    private boolean buildTriggered;
    private long buildTime;
    private final int[] voxels = new int[2];
    private final int[] tiles = new int[2];
    private int maxTiles;
    private int maxPolys;

    private DrawMode drawMode = DrawMode.DRAWMODE_MESH;
    private boolean meshInputTrigerred;

    @Override
    public boolean layout(NkContext ctx, int x, int y, int width, int height, int mouseX, int mouseY) {
        boolean mouseInside = false;
        nk_rgb(255, 255, 255, white);
        nk_rgba(0, 0, 0, 192, background);
        nk_rgba(255, 0, 0, 0, transparent);
        try (MemoryStack stack = stackPush()) {
            ctx.style().text().color().set(white);
            ctx.style().option().text_normal().set(white);
            ctx.style().property().label_normal().set(white);
            ctx.style().window().background().set(background);
            NkStyleItem styleItem = NkStyleItem.mallocStack(stack);
            nk_style_item_color(background, styleItem);
            ctx.style().window().fixed_background().set(styleItem);
            nk_style_item_color(white, styleItem);
            ctx.style().option().cursor_hover().set(styleItem);
            ctx.style().option().cursor_normal().set(styleItem);
            nk_style_item_color(transparent, styleItem);
            ctx.style().tab().node_minimize_button().normal().set(styleItem);
            ctx.style().tab().node_minimize_button().active().set(styleItem);
            ctx.style().tab().node_maximize_button().normal().set(styleItem);
            ctx.style().tab().node_maximize_button().active().set(styleItem);
        }
        try (MemoryStack stack = stackPush()) {
            NkRect rect = NkRect.mallocStack(stack);
            if (nk_begin(ctx, "Properties", nk_rect(width - 255, 5, 250, height - 10, rect),
                    NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_TITLE)) {

                nk_layout_row_dynamic(ctx, 5, 1);
                nk_spacing(ctx, 1);
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, "Input Mesh", NK_TEXT_ALIGN_LEFT);
                nk_layout_row_dynamic(ctx, 20, 1);
                meshInputTrigerred = nk_button_text(ctx, "Choose Mesh...");
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, String.format("Verts: %d Tris: %d", 0, 0), NK_TEXT_ALIGN_RIGHT);

                nk_layout_row_dynamic(ctx, 5, 1);
                nk_spacing(ctx, 1);
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, "Rasterization", NK_TEXT_ALIGN_LEFT);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_float(ctx, "Cell Size", 0.1f, cellSize, 1f, 0.01f, 0.01f);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_float(ctx, "Cell Height", 0.1f, cellHeight, 1f, 0.01f, 0.01f);
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, String.format("Voxels %d x %d", voxels[0], voxels[1]), NK_TEXT_ALIGN_RIGHT);

                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, "Agent", NK_TEXT_ALIGN_LEFT);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_float(ctx, "Height", 0.1f, agentHeight, 5f, 0.1f, 0.1f);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_float(ctx, "Radius", 0.0f, agentRadius, 5f, 0.1f, 0.1f);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_float(ctx, "Max Climb", 0.1f, agentMaxClimb, 5f, 0.1f, 0.1f);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_float(ctx, "Max Slope", 0f, agentMaxSlope, 90f, 1f, 1f);

                nk_layout_row_dynamic(ctx, 5, 1);
                nk_spacing(ctx, 1);
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, "Region", NK_TEXT_ALIGN_LEFT);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_int(ctx, "Min Region Size", 0, minRegionSize, 150, 1, 1f);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_int(ctx, "Merged Region Size", 0, mergedRegionSize, 150, 1, 1f);

                nk_layout_row_dynamic(ctx, 5, 1);
                nk_spacing(ctx, 1);
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, "Partitioning", NK_TEXT_ALIGN_LEFT);
                partitioning = NuklearUIHelper.nk_radio(ctx, PartitionType.values(), partitioning,
                        p -> p.name().substring(0, 1) + p.name().substring(1).toLowerCase());

                nk_layout_row_dynamic(ctx, 5, 1);
                nk_spacing(ctx, 1);
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, "Filtering", NK_TEXT_ALIGN_LEFT);
                nk_layout_row_dynamic(ctx, 20, 1);
                filterLowHangingObstacles = nk_option_text(ctx, "Low Hanging Obstacles", filterLowHangingObstacles);
                nk_layout_row_dynamic(ctx, 20, 1);
                filterLedgeSpans = nk_option_text(ctx, "Ledge Spans", filterLedgeSpans);
                nk_layout_row_dynamic(ctx, 20, 1);
                filterWalkableLowHeightSpans = nk_option_text(ctx, "Walkable Low Height Spans",
                        filterWalkableLowHeightSpans);

                nk_layout_row_dynamic(ctx, 5, 1);
                nk_spacing(ctx, 1);
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, "Polygonization", NK_TEXT_ALIGN_LEFT);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_float(ctx, "Max Edge Length", 0f, edgeMaxLen, 50f, 1f, 1f);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_float(ctx, "Max Edge Error", 0.1f, edgeMaxError, 3f, 0.1f, 0.1f);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_int(ctx, "Vert Per Poly", 3, vertsPerPoly, 12, 1, 1);

                nk_layout_row_dynamic(ctx, 5, 1);
                nk_spacing(ctx, 1);
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, "Detail Mesh", NK_TEXT_ALIGN_LEFT);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_float(ctx, "Sample Distance", 0f, detailSampleDist, 16f, 1f, 1f);
                nk_layout_row_dynamic(ctx, 20, 1);
                nk_property_float(ctx, "Max Sample Error", 0f, detailSampleMaxError, 16f, 1f, 1f);

                nk_layout_row_dynamic(ctx, 5, 1);
                nk_spacing(ctx, 1);
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, "Tiling", NK_TEXT_ALIGN_LEFT);
                nk_layout_row_dynamic(ctx, 20, 1);
                tiled = nk_check_text(ctx, "Enable", tiled);
                if (tiled) {
                    nk_layout_row_dynamic(ctx, 20, 1);
                    nk_property_int(ctx, "Tile Size", 16, tileSize, 1024, 16, 16);
                    nk_layout_row_dynamic(ctx, 18, 1);
                    nk_label(ctx, String.format("Tiles %d x %d", tiles[0], tiles[1]), NK_TEXT_ALIGN_RIGHT);
                    nk_layout_row_dynamic(ctx, 18, 1);
                    nk_label(ctx, String.format("Max Tiles %d", maxTiles), NK_TEXT_ALIGN_RIGHT);
                    nk_layout_row_dynamic(ctx, 18, 1);
                    nk_label(ctx, String.format("Max Polys %d", maxPolys), NK_TEXT_ALIGN_RIGHT);
                }
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, String.format("Build Time: %d ms", buildTime), NK_TEXT_ALIGN_LEFT);

                nk_layout_row_dynamic(ctx, 20, 1);
                buildTriggered = nk_button_text(ctx, "Build");

                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, "Draw", NK_TEXT_ALIGN_LEFT);
                drawMode = NuklearUIHelper.nk_radio(ctx, DrawMode.values(), drawMode, dm -> dm.toString());

                nk_window_get_bounds(ctx, rect);
                if (mouseX >= rect.x() && mouseX <= rect.x() + rect.w() && mouseY >= rect.y()
                        && mouseY <= rect.y() + rect.h()) {
                    mouseInside = true;
                }
            }
            nk_end(ctx);
        }
        return mouseInside;
    }

    public float getCellSize() {
        return cellSize.get(0);
    }

    public float getCellHeight() {
        return cellHeight.get(0);
    }

    public float getAgentHeight() {
        return agentHeight.get(0);
    }

    public float getAgentRadius() {
        return agentRadius.get(0);
    }

    public float getAgentMaxClimb() {
        return agentMaxClimb.get(0);
    }

    public float getAgentMaxSlope() {
        return agentMaxSlope.get(0);
    }

    public int getMinRegionSize() {
        return minRegionSize.get(0);
    }

    public int getMergedRegionSize() {
        return mergedRegionSize.get(0);
    }

    public PartitionType getPartitioning() {
        return partitioning;
    }

    public boolean isBuildTriggered() {
        return buildTriggered;
    }

    public boolean isFilterLowHangingObstacles() {
        return filterLowHangingObstacles;
    }

    public boolean isFilterLedgeSpans() {
        return filterLedgeSpans;
    }

    public boolean isFilterWalkableLowHeightSpans() {
        return filterWalkableLowHeightSpans;
    }

    public void setBuildTime(long buildTime) {
        this.buildTime = buildTime;
    }

    public DrawMode getDrawMode() {
        return drawMode;
    }

    public float getEdgeMaxLen() {
        return edgeMaxLen.get(0);
    }

    public float getEdgeMaxError() {
        return edgeMaxError.get(0);
    }

    public int getVertsPerPoly() {
        return vertsPerPoly.get(0);
    }

    public float getDetailSampleDist() {
        return detailSampleDist.get(0);
    }

    public float getDetailSampleMaxError() {
        return detailSampleMaxError.get(0);
    }

    public void setVoxels(int[] voxels) {
        this.voxels[0] = voxels[0];
        this.voxels[1] = voxels[1];
    }

    public boolean isTiled() {
        return tiled;
    }

    public int getTileSize() {
        return tileSize.get(0);
    }

    public void setTiles(int[] tiles) {
        this.tiles[0] = tiles[0];
        this.tiles[1] = tiles[1];
    }

    public void setMaxTiles(int maxTiles) {
        this.maxTiles = maxTiles;
    }

    public void setMaxPolys(int maxPolys) {
        this.maxPolys = maxPolys;
    }

    public boolean isMeshInputTrigerred() {
        return meshInputTrigerred;
    }

}
