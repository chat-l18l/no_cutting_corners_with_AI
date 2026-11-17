/*
 * test_model.c — draait de selftest van de rbt_model-registry
 *
 * De inhoudelijke checks (compleetheid van elke registry-entry, geldige
 * defaults, naam-lookup) staan in rbt_model_test() onderaan rbt_model.c,
 * conform de CLASS-conventie.
 *
 * Build:
 *   gcc -Wall -Wextra -I./include -I./src/mpc \
 *       test/native/test_model.c src/mpc/rbt_model.c \
 *       src/mpc/rbt_model_ackermann.c src/mpc/rbt_vehicle_model.c \
 *       -lm -o test_model && ./test_model
 */

#include "rbt_model.h"

int main(void)
{
    rbt_model_test(true);
    return 0;
}
