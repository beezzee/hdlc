#include <stdio.h>
#include "assert.h"

void assert(int c) {
  if (!c) {
    printf("Error: Assertion failed\n");
  }

}
