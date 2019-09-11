	.arch armv8-a
	.text
	.global	randomx_program_aarch64
	.global	randomx_program_aarch64_vm_instructions
	.global	randomx_program_aarch64_vm_instructions_end
	.global randomx_program_aarch64_update_spMix1
	.global	randomx_program_aarch64_end

# Register allocation

# x0  -> pointer to reg buffer
# x1  -> pointer to mem buffer and then to dataset
# x2  -> pointer to scratchpad
# x3  -> loop counter
# x4  -> "r0"
# x5  -> "r1"
# x6  -> "r2"
# x7  -> "r3"
# x8  -> temporary
# x9  -> mx, ma
# x10 -> spMix1
# x11 -> spMix2
# x12 -> "r4"
# x13 -> "r5"
# x14 -> "r6"
# x15 -> "r7"
# x16 -> ScratchpadL1Mask64
# x17 -> ScratchpadL2Mask64
# x18 -> ScratchpadL3Mask64
# x19 -> spAddr0
# x20 -> spAddr1
# x21 -> temporary
# x22 -> temporary
# x23 -> temporary
# x24 -> temporary
# x25 -> temporary
# x26 -> ScratchpadL1Mask8
# x27 -> ScratchpadL2Mask8
# x28 -> ScratchpadL3Mask8
# x29 -> CacheLineAlignMask

# v0-v7 -> temporary
# v8-v15 -> not used
# v16 -> "f0"
# v17 -> "f1"
# v18 -> "f2"
# v19 -> "f3"
# v20 -> "e0"
# v21 -> "e1"
# v22 -> "e2"
# v23 -> "e3"
# v24 -> "a0"
# v25 -> "a1"
# v26 -> "a2"
# v27 -> "a3"
# v28 -> temporary
# v29 -> E 'and' mask = 0x00ffffffffffffff00ffffffffffffff
# v30 -> E 'or' mask  = 0x3*00000000******3*00000000******
# v31 -> scale mask   = 0x81f000000000000081f0000000000000

randomx_program_aarch64:
	# Save callee-saved registers
	sub	sp, sp, 128
	stp	x16, x17, [sp]
	stp	x18, x19, [sp, 16]
	stp	x20, x21, [sp, 32]
	stp	x22, x23, [sp, 48]
	stp	x24, x25, [sp, 64]
	stp	x26, x27, [sp, 80]
	stp	x28, x29, [sp, 96]
	stp	x8, x30, [sp, 112]

	# Zero integer registers
	mov	x4, xzr
	mov	x5, xzr
	mov	x6, xzr
	mov	x7, xzr
	mov	x12, xzr
	mov	x13, xzr
	mov	x14, xzr
	mov	x15, xzr

	# Load ma, mx and dataset pointer
	ldp	x9, x1, [x1]

	# Load initial spMix value
	mov	x10, x9

	# Load Scratchpad masks
	mov	x16, 16384 - 64
	mov	x17, 262144 - 64
	mov	x18, 2097152 - 64
	mov	x26, 16384 - 8
	mov	x27, 262144 - 8
	mov	x28, 2097152 - 8

	# Load CacheLineAlignMask
	mov	x29, 0x7FFFFFC0

	# Load group A registers
	ldp	q24, q25, [x0, 192]
	ldp	q26, q27, [x0, 224]

	# Load E 'and' mask
	mov	x21, 0x00FFFFFFFFFFFFFF
	ins	v29.d[0], x21
	ins	v29.d[1], x21

	# Load E 'or' mask (stored in reg.f[0])
	ldr	q30, [x0, 64]

	# Load scale mask
	mov	x21, 0x81f0000000000000
	ins	v31.d[0], x21
	ins	v31.d[1], x21

main_loop:
	# spAddr0 = spMix1 & ScratchpadL3Mask64;
	# spAddr1 = (spMix1 >> 32) & ScratchpadL3Mask64;
	lsr	x21, x10, 32
	and	w19, w10, w18
	and	w20, w21, w18

	# x19 = scratchpad + spAddr0
	# x20 = scratchpad + spAddr1
	add	x19, x19, x2
	add	x20, x20, x2

	# xor integer registers with scratchpad data (spAddr0)
	ldp	x21, x22, [x19]
	ldp	x23, x24, [x19, 16]
	eor	x4, x4, x21
	eor	x5, x5, x22
	eor	x6, x6, x23
	eor	x7, x7, x24
	ldp	x21, x22, [x19, 32]
	ldp	x23, x24, [x19, 48]
	eor	x12, x12, x21
	eor	x13, x13, x22
	eor	x14, x14, x23
	eor	x15, x15, x24

	# Load group F registers (spAddr1)
	ldpsw	x21, x22, [x20]
	ldpsw	x23, x24, [x20, 8]
	ins	v16.d[0], x21
	ins	v16.d[1], x22
	ins	v17.d[0], x23
	ins	v17.d[1], x24
	ldpsw	x21, x22, [x20, 16]
	ldpsw	x23, x24, [x20, 24]
	ins	v18.d[0], x21
	ins	v18.d[1], x22
	ins	v19.d[0], x23
	ins	v19.d[1], x24
	scvtf	v16.2d, v16.2d
	scvtf	v17.2d, v17.2d
	scvtf	v18.2d, v18.2d
	scvtf	v19.2d, v19.2d

	# Load group E registers (spAddr1)
	ldpsw	x21, x22, [x20, 32]
	ldpsw	x23, x24, [x20, 40]
	ins	v20.d[0], x21
	ins	v20.d[1], x22
	ins	v21.d[0], x23
	ins	v21.d[1], x24
	ldpsw	x21, x22, [x20, 48]
	ldpsw	x23, x24, [x20, 56]
	ins	v22.d[0], x21
	ins	v22.d[1], x22
	ins	v23.d[0], x23
	ins	v23.d[1], x24
	scvtf	v20.2d, v20.2d
	scvtf	v21.2d, v21.2d
	scvtf	v22.2d, v22.2d
	scvtf	v23.2d, v23.2d
	and	v20.16b, v20.16b, v29.16b
	and	v21.16b, v21.16b, v29.16b
	and	v22.16b, v22.16b, v29.16b
	and	v23.16b, v23.16b, v29.16b
	orr	v20.16b, v20.16b, v30.16b
	orr	v21.16b, v21.16b, v30.16b
	orr	v22.16b, v22.16b, v30.16b
	orr	v23.16b, v23.16b, v30.16b

	# Execute VM instructions
randomx_program_aarch64_vm_instructions:

	# 16 KB buffer for generated instructions
	.fill 4096,4,0

randomx_program_aarch64_vm_instructions_end:

	# mx ^= r[readReg2] ^ r[readReg3];
	eor	x9, x9, x11

	# Calculate dataset pointer for dataset prefetch
	mov	w25, w9
	and	x25, x25, x29
	add	x25, x25, x1

	# Prefetch dataset data
	prfm	pldl2strm, [x25]

	# mx <-> ma
	ror	x9, x9, 32

	# Calculate dataset pointer for dataset read
	mov	w25, w9
	and	x25, x25, x29
	add	x25, x25, x1

	# xor integer registers with dataset data
	ldp	x21, x22, [x25]
	ldp	x23, x24, [x25, 16]
	eor	x4, x4, x21
	eor	x5, x5, x22
	eor	x6, x6, x23
	eor	x7, x7, x24
	ldp	x21, x22, [x25, 32]
	ldp	x23, x24, [x25, 48]
	eor	x12, x12, x21
	eor	x13, x13, x22
	eor	x14, x14, x23
	eor	x15, x15, x24

randomx_program_aarch64_update_spMix1:
	eor	x10, x0, x0

	# Store integer registers to scratchpad (spAddr1)
	stp	x4, x5, [x20, 0]
	stp	x6, x7, [x20, 16]
	stp	x12, x13, [x20, 32]
	stp	x14, x15, [x20, 48]

	# xor group F and group E registers
	eor	v16.16b, v16.16b, v20.16b
	eor	v17.16b, v17.16b, v21.16b
	eor	v18.16b, v18.16b, v22.16b
	eor	v19.16b, v19.16b, v23.16b

	# Store FP registers to scratchpad (spAddr0)
	stp	q16, q17, [x19, 0]
	stp	q18, q19, [x19, 32]

	subs	x3, x3, 1
	bne	main_loop
	
	# Store integer registers
	stp	x4, x5, [x0, 0]
	stp	x6, x7, [x0, 16]
	stp	x12, x13, [x0, 32]
	stp	x14, x15, [x0, 48]

	# Store FP registers
	stp	q16, q17, [x0, 64]
	stp	q18, q19, [x0, 96]
	stp	q20, q21, [x0, 128]
	stp	q22, q23, [x0, 160]

	# Restore callee-saved registers
	ldp	x16, x17, [sp]
	ldp	x18, x19, [sp, 16]
	ldp	x20, x21, [sp, 32]
	ldp	x22, x23, [sp, 48]
	ldp	x24, x25, [sp, 64]
	ldp	x26, x27, [sp, 80]
	ldp	x28, x29, [sp, 96]
	ldp	x8, x30, [sp, 112]
	add	sp, sp, 128

	ret

randomx_program_aarch64_end:
