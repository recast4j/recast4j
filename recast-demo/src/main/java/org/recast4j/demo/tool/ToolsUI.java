/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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
package org.recast4j.demo.tool;

import static org.lwjgl.nuklear.Nuklear.*;
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
            NkRect rect = NkRect.malloc(stack);
            if (nk_begin(ctx, "Tools", nk_rect(5, 5, 250, height - 10, rect), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_TITLE)) {
                if (enabled) {
                    for (Tool tool : tools) {
                        nk_layout_row_dynamic(ctx, 20, 1);
                        if (nk_option_label(ctx, tool.getName(), tool == currentTool)) {
                            currentTool = tool;
                        }
                    }
                    nk_layout_row_dynamic(ctx, 3, 1);
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
