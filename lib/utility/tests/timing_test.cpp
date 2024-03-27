// UFO
#include <ufo/utility/timing.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

// STL
#include <thread>

TEST_CASE("Timing")
{
	using namespace std::chrono_literals;

	ufo::Timing t("Test");

	std::thread t1{[&t]() {
		ufo::Timing& a = t.start("Thread 1");
		std::this_thread::sleep_for(5ms);
		std::thread t2([&t = a]() {
			std::this_thread::sleep_for(5ms);
			t.start("Thread 1.2");
			std::this_thread::sleep_for(5ms);
			t.stop();
		});
		t.start("Wow");
		std::this_thread::sleep_for(10ms);
		t.stopAll();

		t2.join();
	}};

	std::thread t2{[&t]() {
		t.start("Thread 2");
		std::this_thread::sleep_for(5ms);
		t.stop();
	}};

	t1.join();
	t2.join();

	t.printNanoseconds(true, true, true, 1, 10, 2);

// ufo::Timing t("Test");

// t.start("First");
// t.start("Thing 1");
// t.stop();
// t.start("Thing 1");
// t.start("Wow");
// t.stop();
// t.stop();
// t.start("Thing 2");
// t.stop();
// t.stop();
// t.start("Second");
// t.stop();

// t.printNanoseconds(true, true, true, 1, 10, 2);
}