package org.recast4j.recast;

public class AreaModification {

	public static final int RC_AREA_FLAGS_MASK = 0x3F;
	private int value;
	private int mask;

	/**
	 * Mask is set to all available bits, which means value is fully applied
	 * 
	 * @param value
	 *            The area id to apply. [Limit: <= #RC_AREA_FLAGS_MASK]
	 */
	public AreaModification(int value) {
		this.value = value;
		mask = RC_AREA_FLAGS_MASK;
	}

	/**
	 * 
	 * @param value
	 *            The area id to apply. [Limit: <= #RC_AREA_FLAGS_MASK]
	 * @param mask
	 *            Bitwise mask used when applying value. [Limit: <= #RC_AREA_FLAGS_MASK]
	 */
	public AreaModification(int value, int mask) {
		this.value = value;
		this.mask = mask;
	}

	public AreaModification(AreaModification other) {
		this.value = other.value;
		this.mask = other.mask;
	}

	public int getMaskedValue() {
		return value & mask;
	}

	public int apply(int area) {
		return ((value & mask) | (area & ~mask));
	}
}
