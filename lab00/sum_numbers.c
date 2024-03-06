// lab00, lap procedure, task 2

// source file for the program sum_numbers

/*
 * This file is used to generate an executable program that prints the
 * sum of the two numbers in binary and hexadecimal format.
 */

// Since we will be using our own functions, we need to add the header
// file where the functions are declared.
// Keep in mind that the header file already includes standard libraries
// so they do not to be included here.
#include "functions.h"

// main function
int main(int argc, char *argv[])
{
	// ***** enter your code here ***** //
	uint16_t num1 = 0x2001;
	uint16_t num2 = 0x35FA;

	uint16_t sum = num1 + num2;
	
	print_bits(sum);
	// ***** end of your code ***** //

	return 0;
}
