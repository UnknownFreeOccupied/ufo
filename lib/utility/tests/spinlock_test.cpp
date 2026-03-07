// UFO
#include <ufo/utility/spinlock.hpp>

// STL
#include <atomic>
#include <chrono>
#include <random>
#include <thread>
#include <vector>

// Catch2
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[Spinlock] basic lock/unlock")
{
	ufo::Spinlock lock;
	lock.lock();
	lock.unlock();
	REQUIRE(lock.try_lock());
	lock.unlock();
}

TEST_CASE("[Spinlock] try_lock behavior")
{
	ufo::Spinlock lock;
	REQUIRE(lock.try_lock());
	REQUIRE_FALSE(lock.try_lock());
	lock.unlock();
	REQUIRE(lock.try_lock());
	lock.unlock();
}

TEST_CASE("[Spinlock] lock blocks until unlock")
{
	ufo::Spinlock     lock;
	std::atomic<bool> locked{false};
	lock.lock();
	std::thread t([&] {
		lock.lock();
		locked = true;
		lock.unlock();
	});
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	REQUIRE_FALSE(locked);
	lock.unlock();
	t.join();
	REQUIRE(locked);
}

TEST_CASE("[Spinlock] unlock notifies waiting thread")
{
	ufo::Spinlock     lock;
	std::atomic<bool> done{false};
	lock.lock();
	std::thread t([&] {
		lock.lock();
		done = true;
		lock.unlock();
	});
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	REQUIRE_FALSE(done);
	lock.unlock();
	t.join();
	REQUIRE(done);
}

TEST_CASE("[Spinlock] multiple threads increment counter")
{
	ufo::Spinlock            lock;
	constexpr int            num_threads = 8;
	constexpr int            increments  = 10000;
	int                      counter     = 0;
	std::vector<std::thread> threads;
	for (int i = 0; i < num_threads; ++i) {
		threads.emplace_back([&] {
			for (int j = 0; j < increments; ++j) {
				lock.lock();
				++counter;
				lock.unlock();
			}
		});
	}
	for (auto& t : threads) t.join();
	REQUIRE(counter == num_threads * increments);
}

TEST_CASE("[Spinlock] stress test with random delays")
{
	ufo::Spinlock                   lock;
	constexpr int                   num_threads = 4;
	constexpr int                   increments  = 1000;
	int                             counter     = 0;
	std::vector<std::thread>        threads;
	std::random_device              rd;
	std::mt19937                    gen(rd());
	std::uniform_int_distribution<> dist(0, 5);
	for (int i = 0; i < num_threads; ++i) {
		threads.emplace_back([&] {
			for (int j = 0; j < increments; ++j) {
				std::this_thread::sleep_for(std::chrono::milliseconds(dist(gen)));
				lock.lock();
				++counter;
				lock.unlock();
			}
		});
	}
	for (auto& t : threads) t.join();
	REQUIRE(counter == num_threads * increments);
}

TEST_CASE("[Spinlock] edge cases: repeated lock/unlock")
{
	ufo::Spinlock lock;
	for (int i = 0; i < 100; ++i) {
		lock.lock();
		lock.unlock();
		REQUIRE(lock.try_lock());
		lock.unlock();
	}
}

TEST_CASE("[Spinlock] edge cases: try_lock after unlock")
{
	ufo::Spinlock lock;
	lock.lock();
	lock.unlock();
	REQUIRE(lock.try_lock());
	lock.unlock();
}
