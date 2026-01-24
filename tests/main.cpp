#include <iostream>
#include <stdexcept>

// Test declarations
void test_instructions();
void test_stitch_graph();
void test_parser();

int main() {
    int failures = 0;

    try {
        std::cout << "Running instruction tests...\n";
        test_instructions();
        std::cout << "Instruction tests passed.\n";
    } catch (const std::exception& e) {
        std::cerr << "Instruction tests failed: " << e.what() << "\n";
        ++failures;
    }

    try {
        std::cout << "Running stitch graph tests...\n";
        test_stitch_graph();
        std::cout << "Stitch graph tests passed.\n";
    } catch (const std::exception& e) {
        std::cerr << "Stitch graph tests failed: " << e.what() << "\n";
        ++failures;
    }

    try {
        std::cout << "Running parser tests...\n";
        test_parser();
        std::cout << "Parser tests passed.\n";
    } catch (const std::exception& e) {
        std::cerr << "Parser tests failed: " << e.what() << "\n";
        ++failures;
    }

    if (failures == 0) {
        std::cout << "\nAll tests passed!\n";
        return 0;
    } else {
        std::cerr << "\n" << failures << " test group(s) failed.\n";
        return 1;
    }
}
