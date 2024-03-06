#include "functions.h"

int main(int argc, char *argv[])
{
    int iter = 0;
	
    while (1) {
        printf("Iteration: %d\n", ++iter);

	    uint16_t lsb = 0;
	    uint16_t msb = 0;

		scanf("%hx %hx", &lsb, &msb);

        // break condition
        if(lsb == 0 && msb == 0) break;

	    uint32_t merged = bit_merge(lsb, msb);

		printf("merging 0x%04x and 0x%04x results in 0x%08x\n", lsb, msb, merged);
		printf("\tthe sum is ");
		print_bits(lsb+msb);
		printf("\n");
    }
    printf("Program ended with %d iteration(s)\n", iter-1);


	return 0;
}
