	.section        .xiptext,"ax",%progbits
	.globl	do_flash_cmd
	.p2align	2
	.type	do_flash_cmd,%function
do_flash_cmd:
	push	{r4, r5, r6, r7, lr}
	add	r7, sp, #12
	sub	sp, #20
	mov	r6, r2
	str	r1, [sp, #16]
	str	r0, [sp, #12]
	movs	r4, #20
	ldrh	r0, [r4]
	movs	r5, #24
	ldrh	r2, [r5]
	ldr	r1, .XLCPI0_0
	blx	r2
	str	r0, [sp, #4]
	ldrh	r2, [r5]
	ldrh	r0, [r4]
	ldr	r1, .XLCPI0_1
	adds	r1, r1, #2
	blx	r2
	str	r0, [sp]
	ldrh	r2, [r5]
	ldrh	r0, [r4]
	ldr	r1, .XLCPI0_1
	blx	r2
	str	r0, [sp, #8]
	@ COMPILER BARRIER
	ldr	r0, [sp, #4]
	blx	r0
	ldr	r0, [sp]
	blx	r0
	ldr	r0, .XLCPI0_2
	ldr	r1, [r0]
	ldr	r2, .XLCPI0_3
	ands	r1, r2
	movs	r2, #1
	lsls	r3, r2, #9
	adds	r1, r1, r3
	str	r1, [r0]
	cmp	r6, #0
	beq	.XLBB0_12
	mov	r3, r6
	mov	r5, r2
	mov	r4, r2
.XLBB0_2:
	ldr	r0, .XLCPI0_4
	ldr	r1, [r0]
	lsls	r0, r1, #30
	bpl	.XLBB0_6
	lsls	r0, r5, #31
	beq	.XLBB0_6
	movs	r0, #8
	ands	r1, r0
	lsrs	r1, r1, #3
	ands	r1, r4
	mov	r5, r3
	subs	r0, r6, r3
	cmp	r0, #14
	bhs	.XLBB0_11
	ldr	r3, [sp, #12]
	ldrb	r0, [r3]
	ldr	r4, .XLCPI0_4
	str	r0, [r4, #56]
	subs	r5, r5, #1
	adds	r3, r3, #1
	cmp	r1, #0
	str	r3, [sp, #12]
	bne	.XLBB0_7
	b	.XLBB0_8
.XLBB0_6:
	movs	r0, #8
	ands	r1, r0
	lsrs	r0, r1, #3
	mov	r5, r3
	tst	r0, r4
	beq	.XLBB0_8
.XLBB0_7:
	ldr	r1, [sp, #16]
	ldr	r0, .XLCPI0_4
	ldr	r0, [r0, #56]
	strb	r0, [r1]
	subs	r6, r6, #1
	adds	r1, r1, #1
	str	r1, [sp, #16]
.XLBB0_8:
	subs	r0, r6, #1
	mov	r4, r6
	sbcs	r4, r0
	subs	r0, r5, #1
	mov	r3, r5
	sbcs	r5, r0
	cmp	r6, #0
	mov	r1, r2
	bne	.XLBB0_10
	mov	r1, r5
.XLBB0_10:
	cmp	r1, #0
	bne	.XLBB0_2
	b	.XLBB0_12
.XLBB0_11:
	cmp	r1, #0
	bne	.XLBB0_7
	b	.XLBB0_8
.XLBB0_12:
	ldr	r0, .XLCPI0_2
	ldr	r1, [r0]
	ldr	r2, .XLCPI0_3
	ands	r1, r2
	str	r1, [r0]
	ldr	r0, [sp, #8]
	blx	r0
	@ COMPILER BARRIER
	add	sp, #20
	pop	{r4, r5, r6, r7, pc}
	.p2align	2
.XLCPI0_0:
	.long	17993
.XLCPI0_1:
	.long	22595
.XLCPI0_2:
	.long	1073840140
.XLCPI0_3:
	.long	4294966527
.XLCPI0_4:
	.long	402653224