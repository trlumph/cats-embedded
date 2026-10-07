// Host test stubs for the Vega USB console used by cli/settings.cpp (print_cats_config).
#include "cli/cli.hpp"

void cli_printf(const char* /*format*/, ...) {}
void cli_print_linefeed() {}
void cli_print_var(const char* /*cmdName*/, const cats_config_t* /*cfg*/, const cli_value_t* /*var*/, bool /*full*/) {}
