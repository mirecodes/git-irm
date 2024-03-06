// lab00, lab procedure, tasks 1 & 3

// source file

/*
 * this file contains the implementation of the functions relevant for
 * lab00.
 * The declarations of the functions are in the corresponding header
 * file (functions.h).
 */

// the header file needs to be included
#include "functions.h"

/* task 1
 * The function print_bits() accepts an input of type uint16_t
 * (arg_word) and has no return value (void).
 * The function simply writes the binary and hexadecimal number of the
 * input to the terminal.
 */
void print_bits(uint16_t arg_word)
{
	printf("hex: 0x%04x, bin: ", arg_word);

	// only a simple implementation of a for loop is given
	// you are free to use it or solve the problem in another way

	for (int i=3; i>-1; i--) {
		char bin_digits[4] = {};

		bin_digits[0] = 0 != (arg_word & 1<<(3+i*4));
		bin_digits[1] = 0 != (arg_word & 1<<(2+i*4));
		bin_digits[2] = 0 != (arg_word & 1<<(1+i*4));
		bin_digits[3] = 0 != (arg_word & 1<<(0+i*4));

		for (int j=0; j<4; j++) {
			printf("%d", bin_digits[j]);
		}
		printf(" ");
	}
	printf("\n");
}

/* task 3
 * The function bit_merge() accepts two uint16_t as inputs (lsb and msb)
 * and combines them to a uint32_t number by merging them.
 * The return value is a uint32_t number.
 */
uint32_t bit_merge(uint16_t lsb, uint16_t msb)
{
  uint32_t merged = msb << 16 | lsb;
  return merged;
}
