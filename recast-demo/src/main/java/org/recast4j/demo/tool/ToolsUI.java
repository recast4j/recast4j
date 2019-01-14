package org.recast4j.demo.tool;

import static org.lwjgl.nuklear.Nuklear.NK_WINDOW_BORDER;
import static org.lwjgl.nuklear.Nuklear.NK_WINDOW_MOVABLE;
import static org.lwjgl.nuklear.Nuklear.NK_WINDOW_TITLE;
import static org.lwjgl.nuklear.Nuklear.nk_begin;
import static org.lwjgl.nuklear.Nuklear.nk_end;
import static org.lwjgl.nuklear.Nuklear.nk_layout_row_dynamic;
import static org.lwjgl.nuklear.Nuklear.nk_option_label;
import static org.lwjgl.nuklear.Nuklear.nk_rect;
import static org.lwjgl.nuklear.Nuklear.nk_rgb;
import static org.lwjgl.nuklear.Nuklear.nk_spacing;
import static org.lwjgl.nuklear.Nuklear.nk_window_get_bounds;
import static org.lwjgl.system.MemoryStack.stackPush;

import java.util.Arrays;

import org.lwjgl.nuklear.NkColor;
import org.lwjgl.nuklear.NkContext;
import org.lwjgl.nuklear.NkRect;
import org.lwjgl.system.MemoryStack;
import org.recast4j.demo.sample.Sample;
import org.recast4j.demo.ui.NuklearUIModule;

public class ToolsUI implements NuklearUIModule {

    private final NkColor white = NkColor.create();
    private Tool currentTool;
    private boolean enabled;
    private final Tool[] tools;

    public ToolsUI(Tool... tools) {
        this.tools = tools;
    }

    @Override
    public boolean layout(NkContext ctx, int x, int y, int width, int height, int mouseX, int mouseY) {
        boolean mouseInside = false;
        nk_rgb(255, 255, 255, white);
        try (MemoryStack stack = stackPush()) {
            NkRect rect = NkRect.mallocStack(stack);
            if (nk_begin(ctx, "Tools", nk_rect(5, 5, 250, height - 10, rect), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_TITLE)) {
                if (enabled) {
                    for (Tool tool : tools) {
                        nk_layout_row_dynamic(ctx, 20, 1);
                        if (nk_option_label(ctx, tool.getName(), tool == currentTool)) {
                            currentTool = tool;
                        }
                    }
                    nk_layout_row_dynamic(ctx, 5, 1);
                    nk_spacing(ctx, 1);
                    if (currentTool != null) {
                        currentTool.layout(ctx);
                    }
                }
                nk_window_get_bounds(ctx, rect);
                if (mouseX >= rect.x() && mouseX <= rect.x() + rect.w() && mouseY >= rect.y() && mouseY <= rect.y() + rect.h()) {
                    mouseInside = true;
                }
            }
            nk_end(ctx);
        }
        return mouseInside;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public Tool getTool() {
        return currentTool;
    }

    public void setSample(Sample sample) {
        Arrays.stream(tools).forEach(t -> t.setSample(sample));
    }

    public void handleUpdate(float dt) {
        Arrays.stream(tools).forEach(t -> t.handleUpdate(dt));
    }

}
