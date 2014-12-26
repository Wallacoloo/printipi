//Simple wrapper file to conditionally include the real CATCH for if we're doing a build with tests enabled.
//in the future, replace CATCH macros with dummies if tests are to be disabled.
//#warning "including catch.hpp"
//#define CATCH_CONFIG_PREFIX_ALL
#ifndef PRINTIPI_CATCH_HPP
#define PRINTIPI_CATCH_HPP

#include "compileflags.h"
#if DO_TESTS
	#include "catch/catch.hpp"
	/*#include <mutex>
class ThreadSafeJunitReporter : public Catch::JunitReporter {
public:
    ThreadSafeJunitReporter(Catch::ReporterConfig const& _config) :
        Catch::JunitReporter(_config) { }

    static std::string getDescription() {
        return "Reports test results in an XML format that looks like Ant's junitreport target.\n"
            "\tThis reporter can be used in a multi-threaded environment";
    }

    inline virtual bool assertionEnded(Catch::AssertionStats const& assertionStats) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        return Catch::JunitReporter::assertionEnded(assertionStats);
    }

private:
    std::mutex m_mutex;
};

INTERNAL_CATCH_REGISTER_REPORTER("junit-thread-safe", ThreadSafeJunitReporter);*/

#else
//Taken from catch.hpp to generate unique variable/function names:
#define INTERNAL_CATCH_UNIQUE_NAME_LINE2( name, line ) name##line
#define INTERNAL_CATCH_UNIQUE_NAME_LINE( name, line ) INTERNAL_CATCH_UNIQUE_NAME_LINE2( name, line )
#define INTERNAL_CATCH_UNIQUE_NAME( name ) INTERNAL_CATCH_UNIQUE_NAME_LINE( name, __LINE__ )
//reimplement public catch functions as nop's:
#define TEST_CASE(...) static inline void INTERNAL_CATCH_UNIQUE_NAME(__DEAD_TEST_CODE)()
#define SECTION(...) if (false)
#define REQUIRE(...) do {} while (false);
#define Approx(...) 0
#endif


#endif