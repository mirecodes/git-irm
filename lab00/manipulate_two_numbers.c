// lab00, lab procesure, task 4

// source file for the program manipulate_two_numbers

/*
 * This file is used to generate an executable program that reads two
 * numbers from the terminal and output the merged result in
 * hexadecimal, and the sum in hexadecimal and binary format.
 */

// again, the header of our function library is already included
#include "functions.h"

// main
int main(int argc, char *argv[])
{
	// ***** enter your code here ***** //
	uint16_t lsb = 0;
	uint16_t msb = 0;

	// scan the terminal for inputs from the user
	scanf("%hx %hx", &lsb, &msb);

	// use the function bit_merge() to merge the numbers
	uint32_t merged = bit_merge(lsb, msb);

	// use the function print_bits to print the sum to the terminal
	printf("merging 0x%04x and 0x%04x results in 0x%08x\n", lsb, msb, merged);
	printf("\tthe sum is ");
	print_bits(lsb+msb);

	// ***** end of your code ***** //

	return 0;
}
