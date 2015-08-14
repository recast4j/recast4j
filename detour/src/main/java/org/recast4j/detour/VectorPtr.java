/*
Recast4J Copyright (c) 2015 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.detour;

/**
 * Wrapper for 3-element pieces (3D vectors) of a bigger float array.
 *
 */
public class VectorPtr {

	private final float[] array;
	private final int index;

	public VectorPtr(float[] array) {
		this(array, 0);
	}

	public VectorPtr(float[] array, int index) {
		this.array = array;
		this.index = index;
	}

	public float get(int offset) {
		return array[index + offset];
	}

	public void set(int offset, float f) {
		array[index + offset] = f;
	}
}
