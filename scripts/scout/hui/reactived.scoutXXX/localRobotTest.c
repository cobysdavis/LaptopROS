
# include <stdio.h>

# include "scout.h"

main()
{
	int i;
	printf("starting\n");
	initializeHardware();

	setPose(0.0f, 0.0f, 0.0f);
	setGoal(1.0f, 0.0f, 0.0f);
	for(i=0;i<5;i++) {
		robotTimeSlice();
		printf("slice\n");
	}
	shutdownHardware();
}
