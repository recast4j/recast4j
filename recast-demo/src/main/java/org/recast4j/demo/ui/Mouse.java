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
package org.recast4j.demo.ui;

import static org.lwjgl.glfw.GLFW.GLFW_PRESS;
import static org.lwjgl.glfw.GLFW.GLFW_RELEASE;
import static org.lwjgl.glfw.GLFW.glfwSetCursorPosCallback;
import static org.lwjgl.glfw.GLFW.glfwSetMouseButtonCallback;
import static org.lwjgl.glfw.GLFW.glfwSetScrollCallback;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Mouse {

    private double x;
    private double y;
    private double scrollX;
    private double scrollY;

    private double px;
    private double py;
    private double pScrollX;
    private double pScrollY;
    private final Set<Integer> pressed = new HashSet<>();
    private final List<MouseListener> listeners = new ArrayList<>();

    public Mouse(long window) {
        glfwSetMouseButtonCallback(window, (win, button, action, mods) -> {
            if (action == GLFW_PRESS) {
                buttonPress(button, mods);
            } else if (action == GLFW_RELEASE) {
                buttonRelease(button, mods);
            }
        });
        glfwSetCursorPosCallback(window, (win, x, y) -> cursorPos(x, y));
        glfwSetScrollCallback(window, (win, x, y) -> scroll(x, y));
    }

    public void cursorPos(double x, double y) {
        for (MouseListener l : listeners) {
            l.position(x, y);
        }
        this.x = x;
        this.y = y;
    }

    public void scroll(double xoffset, double yoffset) {
        for (MouseListener l : listeners) {
            l.scroll(xoffset, yoffset);
        }
        scrollX += xoffset;
        scrollY += yoffset;
    }

    public double getDX() {
        return x - px;
    }

    public double getDY() {
        return y - py;
    }

    public double getDScrollX() {
        return scrollX - pScrollX;
    }

    public double getDScrollY() {
        return scrollY - pScrollY;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setDelta() {
        px = x;
        py = y;
        pScrollX = scrollX;
        pScrollY = scrollY;
    }

    public void buttonPress(int button, int mods) {
        for (MouseListener l : listeners) {
            l.button(button, mods, true);
        }
        pressed.add(button);
    }

    public void buttonRelease(int button, int mods) {
        for (MouseListener l : listeners) {
            l.button(button, mods, false);
        }
        pressed.remove(button);
    }

    public boolean isPressed(int button) {
        return pressed.contains(button);
    }

    public void addListener(MouseListener listener) {
        listeners.add(listener);
    }
}
