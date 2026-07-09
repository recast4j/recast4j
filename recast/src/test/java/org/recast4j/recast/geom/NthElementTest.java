package org.recast4j.recast.geom;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.Comparator;
import java.util.Random;

import org.junit.jupiter.api.Test;

public class NthElementTest {

    @Test
    public void testNthElementSimple() {
        Integer[] a = {5, 3, 1, 4, 2};
        NthElement.nthElement(a, 0, 2, a.length, Comparator.naturalOrder());
        Integer[] sorted = a.clone();
        Arrays.sort(sorted);
        assertEquals(sorted[2], a[2]);
        // partition property
        for (int i = 0; i < 2; i++) {
            assertTrue(a[i].compareTo(a[2]) <= 0, "element before nth should be <= nth");
        }
        for (int i = 3; i < a.length; i++) {
            assertTrue(a[2].compareTo(a[i]) <= 0, "element after nth should be >= nth");
        }
    }

    @Test
    public void testNthElementRandom() {
        Random rnd = new Random(12345);
        for (int size = 1; size < 100; size++) {
            Integer[] a = new Integer[size];
            for (int i = 0; i < size; i++) {
                a[i] = rnd.nextInt(1000) - 500; // include negatives
            }
            Integer[] sorted = a.clone();
            Arrays.sort(sorted);
            for (int n = 0; n < size; n++) {
                Integer[] copy = a.clone();
                NthElement.nthElement(copy, 0, n, copy.length, Comparator.naturalOrder());
                assertEquals(sorted[n], copy[n], "nth element should match sorted value for size=" + size + " n=" + n);
                // partition property
                for (int i = 0; i < n; i++) {
                    assertTrue(copy[i].compareTo(copy[n]) <= 0);
                }
                for (int i = n + 1; i < size; i++) {
                    assertTrue(copy[n].compareTo(copy[i]) <= 0);
                }
            }
        }
    }

    @Test
    public void testNthElementWithDuplicatesAndComparator() {
        class Item {
            final int key;
            final String id;
            Item(int k, String id) { key = k; this.id = id; }
        }

        Item[] items = new Item[] {
                new Item(2, "a"), new Item(1, "b"), new Item(2, "c"), new Item(3, "d"), new Item(1, "e")
        };
        Comparator<Item> comp = Comparator.comparingInt(i -> i.key);
        Item[] expected = items.clone();
        Arrays.sort(expected, comp);
        NthElement.nthElement(items, 0, 2, items.length, comp);
        assertEquals(expected[2].key, items[2].key);
        // ensure partition property holds for keys
        for (int i = 0; i < 2; i++) {
            assertTrue(items[i].key <= items[2].key);
        }
        for (int i = 3; i < items.length; i++) {
            assertTrue(items[2].key <= items[i].key);
        }
    }

}
