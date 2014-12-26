//Simple wrapper file to conditionally include the real CATCH for if we're doing a build with tests enabled.

#ifndef PRINTIPI_CATCH_HPP
#define PRINTIPI_CATCH_HPP

#include "compileflags.h"

#if DO_TESTS
	#include "catch/catch.hpp"
#else
	//Taken from catch.hpp to generate unique variable/function names:
	#define INTERNAL_CATCH_UNIQUE_NAME_LINE2( name, line ) name##line
	#define INTERNAL_CATCH_UNIQUE_NAME_LINE( name, line ) INTERNAL_CATCH_UNIQUE_NAME_LINE2( name, line )
	#define INTERNAL_CATCH_UNIQUE_NAME( name ) INTERNAL_CATCH_UNIQUE_NAME_LINE( name, __LINE__ )
	//reimplement public catch functions as nop's:
	#define TEST_CASE(...) static inline void INTERNAL_CATCH_UNIQUE_NAME(__DEAD_TEST_CODE)()
	#define SCENARIO(...) TEST_CASE()
	#define SECTION(...) if (false)
	#define GIVEN(...) SECTION()
	#define WHEN(...) SECTION()
	#define THEN(...) SECTION()
	#define REQUIRE(...) do { __VA_ARGS__; } while (false)
	#define Approx(x) x
#endif


#endif