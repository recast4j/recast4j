/*
recast4j copyright (c) 2021-2026 Piotr Piastucki piotr@recast4j.org

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

import java.util.Comparator;

public class NthElement {

    public static <T> void nthElement(T[] array, int low, int nth, int high, Comparator<? super T> comparator) {
        while ((high - low) > 3) {
            T midValue = medianOfThree(array[low], array[low + ((high - low) >> 1)], array[high - 1], comparator);
            int first = low;
            int last = high;
            for (;;) {
                while (comparator.compare(array[first], midValue) < 0) {
                    first++;
                }
                do {
                    last--;
                } while (comparator.compare(midValue, array[last]) < 0);
                if (first >= last) {
                    break;
                }
                T temp = array[first];
                array[first] = array[last];
                array[last] = temp;
                first++;
            }
            if (first <= nth) {
                low = first;
            } else {
                high = first;
            }
        }
        insertionSort(array, low, high, comparator);
    }

    private static <T> void insertionSort(T[] array, int low, int high, Comparator<? super T> comparator) {
        for (int i = low + 1; i < high; i++) {
            T key = array[i];
            int j = i - 1;
            while (j >= low && comparator.compare(array[j], key) > 0) {
                array[j + 1] = array[j];
                j--;
            }
            array[j + 1] = key;
        }
    }

    private static <T> T medianOfThree(T a, T b, T c, Comparator<? super T> comparator) {
        if (comparator.compare(a, b) < 0) {
            if (comparator.compare(b, c) < 0) {
                return b;
            } else if (comparator.compare(a, c) < 0) {
                return c;
            } else {
                return a;
            }
        } else if (comparator.compare(a, c) < 0) {
            return a;
        } else if (comparator.compare(b, c) < 0) {
            return c;
        }
        return b;
    }

}
