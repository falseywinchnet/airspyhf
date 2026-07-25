	.section .rodata

	.global	m0_i2c_wait_bin
	.align  4
m0_i2c_wait_bin:
	.incbin "../airspy_m0/airspy_m0_i2c_wait.bin"

	.global m0_i2c_wait_bin_size
	.align  4
m0_i2c_wait_bin_size:
	.int	m0_i2c_wait_bin_size - m0_i2c_wait_bin
