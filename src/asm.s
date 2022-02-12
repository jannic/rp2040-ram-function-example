	push	{r4, r5, r6, r7}
	add	r7, sp, #16
	sub	sp, #20
	mov	r6, r2
	str	r1, [sp, #16]
	str	r0, [sp, #12]
	movs	r4, #20
	ldrh	r0, [r4]
	movs	r5, #24
	ldrh	r2, [r5]
	ldr	r1, 7f
	blx	r2
	str	r0, [sp, #4]
	ldrh	r2, [r5]
	ldrh	r0, [r4]
	ldr	r1, 8f
	adds	r1, r1, #2
	blx	r2
	str	r0, [sp]
	ldrh	r2, [r5]
	ldrh	r0, [r4]
	ldr	r1, 8f
	blx	r2
	str	r0, [sp, #8]
	ldr	r0, [sp, #4]
	blx	r0
	ldr	r0, [sp]
	blx	r0
	ldr	r0, 9f
	ldr	r1, [r0]
	ldr	r2, 4f
	ands	r1, r2
	movs	r2, #1
	lsls	r3, r2, #9
	adds	r1, r1, r3
	str	r1, [r0]
	cmp	r6, #0
	beq	6f
	mov	r3, r6
	mov	r5, r2
	mov	r4, r2
0:
	ldr	r0, 5f
	ldr	r1, [r0]
	lsls	r0, r1, #30
	bpl	1f
	lsls	r0, r5, #31
	beq	1f
	movs	r0, #8
	ands	r1, r0
	lsrs	r1, r1, #3
	ands	r1, r4
	mov	r5, r3
	subs	r0, r6, r3
	cmp	r0, #14
	bhs	0f
	ldr	r3, [sp, #12]
	ldrb	r0, [r3]
	ldr	r4, 5f
	str	r0, [r4, #56]
	subs	r5, r5, #1
	adds	r3, r3, #1
	cmp	r1, #0
	str	r3, [sp, #12]
	bne	2f
	b	3f
1:
	movs	r0, #8
	ands	r1, r0
	lsrs	r0, r1, #3
	mov	r5, r3
	tst	r0, r4
	beq	3f
2:
	ldr	r1, [sp, #16]
	ldr	r0, 5f
	ldr	r0, [r0, #56]
	strb	r0, [r1]
	subs	r6, r6, #1
	adds	r1, r1, #1
	str	r1, [sp, #16]
3:
	subs	r0, r6, #1
	mov	r4, r6
	sbcs	r4, r0
	subs	r0, r5, #1
	mov	r3, r5
	sbcs	r5, r0
	cmp	r6, #0
	mov	r1, r2
	bne	1f
	mov	r1, r5
1:
	cmp	r1, #0
	bne	0b
	b	6f
0:
	cmp	r1, #0
	bne	2b
	b	3b
6:
	ldr	r0, 9f
	ldr	r1, [r0]
	ldr	r2, 4f
	ands	r1, r2
	str	r1, [r0]
	ldr	r0, [sp, #8]
	blx	r0
	b	0f
	.p2align	2
7:
	.long	17993
8:
	.long	22595
9:
	.long	1073840140
4:
	.long	4294966527
5:
	.long	402653224
	.p2align	2
0:
	add	sp, #20
	pop	{r4, r5, r6, r7}
