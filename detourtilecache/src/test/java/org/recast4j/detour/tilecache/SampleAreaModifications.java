package org.recast4j.detour.tilecache;

import org.recast4j.recast.AreaModification;

public class SampleAreaModifications {

	public static int SAMPLE_POLYAREA_TYPE_MASK = 0x07;
	/// Value for the kind of ceil "ground"
	public static int SAMPLE_POLYAREA_TYPE_GROUND = 0x1;
	/// Value for the kind of ceil "water"
	public static int SAMPLE_POLYAREA_TYPE_WATER = 0x2;
	/// Value for the kind of ceil "road"
	public static int SAMPLE_POLYAREA_TYPE_ROAD = 0x3;
	/// Value for the kind of ceil "grass"
	public static int SAMPLE_POLYAREA_TYPE_GRASS = 0x4;
	/// Flag for door area. Can be combined with area types and jump flag.
	public static int SAMPLE_POLYAREA_FLAG_DOOR = 0x08;
	/// Flag for jump area. Can be combined with area types and door flag.
	public static int SAMPLE_POLYAREA_FLAG_JUMP = 0x10;

	public static AreaModification SAMPLE_AREAMOD_GROUND = new AreaModification(SAMPLE_POLYAREA_TYPE_GROUND,
			SAMPLE_POLYAREA_TYPE_MASK);
	public static AreaModification SAMPLE_AREAMOD_WATER = new AreaModification(SAMPLE_POLYAREA_TYPE_WATER,
			SAMPLE_POLYAREA_TYPE_MASK);
	public static AreaModification SAMPLE_AREAMOD_ROAD = new AreaModification(SAMPLE_POLYAREA_TYPE_ROAD,
			SAMPLE_POLYAREA_TYPE_MASK);
	public static AreaModification SAMPLE_AREAMOD_GRASS = new AreaModification(SAMPLE_POLYAREA_TYPE_GRASS,
			SAMPLE_POLYAREA_TYPE_MASK);
	public static AreaModification SAMPLE_AREAMOD_DOOR = new AreaModification(SAMPLE_POLYAREA_FLAG_DOOR,
			SAMPLE_POLYAREA_FLAG_DOOR);
	public static AreaModification SAMPLE_AREAMOD_JUMP = new AreaModification(SAMPLE_POLYAREA_FLAG_JUMP,
			SAMPLE_POLYAREA_FLAG_JUMP);

}
