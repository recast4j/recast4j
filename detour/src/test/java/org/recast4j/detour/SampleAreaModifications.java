package org.recast4j.detour;

import org.recast4j.recast.AreaModification;

public class SampleAreaModifications {

	public static int SAMPLE_POLYAREA_TYPE_MASK = 0x07;
	public static int SAMPLE_POLYAREA_TYPE_GROUND = 0x1;
	public static int SAMPLE_POLYAREA_TYPE_WATER = 0x2;
	public static int SAMPLE_POLYAREA_TYPE_ROAD = 0x3;
	public static int SAMPLE_POLYAREA_TYPE_DOOR = 0x4;
	public static int SAMPLE_POLYAREA_TYPE_GRASS = 0x5;
	public static int SAMPLE_POLYAREA_TYPE_JUMP = 0x6;

	public static AreaModification SAMPLE_AREAMOD_GROUND = new AreaModification(SAMPLE_POLYAREA_TYPE_GROUND,
			SAMPLE_POLYAREA_TYPE_MASK);
	public static AreaModification SAMPLE_AREAMOD_WATER = new AreaModification(SAMPLE_POLYAREA_TYPE_WATER,
			SAMPLE_POLYAREA_TYPE_MASK);
	public static AreaModification SAMPLE_AREAMOD_ROAD = new AreaModification(SAMPLE_POLYAREA_TYPE_ROAD,
			SAMPLE_POLYAREA_TYPE_MASK);
	public static AreaModification SAMPLE_AREAMOD_GRASS = new AreaModification(SAMPLE_POLYAREA_TYPE_GRASS,
			SAMPLE_POLYAREA_TYPE_MASK);
	public static AreaModification SAMPLE_AREAMOD_DOOR = new AreaModification(SAMPLE_POLYAREA_TYPE_DOOR,
			SAMPLE_POLYAREA_TYPE_DOOR);
	public static AreaModification SAMPLE_AREAMOD_JUMP = new AreaModification(SAMPLE_POLYAREA_TYPE_JUMP,
			SAMPLE_POLYAREA_TYPE_JUMP);

	public static final int SAMPLE_POLYFLAGS_WALK = 0x01;	// Ability to walk (ground, grass, road)
	public static final int SAMPLE_POLYFLAGS_SWIM = 0x02;   // Ability to swim (water).
	public static final int SAMPLE_POLYFLAGS_DOOR = 0x04;   // Ability to move through doors.
	public static final int SAMPLE_POLYFLAGS_JUMP = 0x08;   // Ability to jump.
	public static final int SAMPLE_POLYFLAGS_DISABLED = 0x10; // Disabled polygon
	public static final int SAMPLE_POLYFLAGS_ALL = 0xffff; // All abilities.
}
