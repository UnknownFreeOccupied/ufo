/**
 * @author Daniel Duberg (danielduberg@gmail.com)
 * @see https://github.com/UnknownFreeOccupied/ufo
 * @version 1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) 2020-2026, Daniel Duberg
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2020-2026, Daniel Duberg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_UTILITY_SPINLOCK_HPP
#define UFO_UTILITY_SPINLOCK_HPP

// STL
#include <atomic>

namespace ufo
{
/**
 * @class Spinlock
 * @brief A simple spinlock implementation using std::atomic_flag.
 *
 * @details
 * Spinlock is a lightweight mutual exclusion primitive for short critical sections.
 * It repeatedly checks a flag until it can acquire the lock, avoiding thread suspension.
 *
 * Uses C++20 atomic wait/notify for efficient spinning.
 *
 * Example usage:
 * @code{.cpp}
 *   ufo::Spinlock lock;
 *   lock.lock();
 *   // critical section
 *   lock.unlock();
 * @endcode
 */
class Spinlock
{
 public:
	/**
	 * @brief Acquires the lock. Spins until the lock is available.
	 *
	 * @details
	 * Uses atomic wait/notify for efficient spinning.
	 */
	void lock() noexcept;

	/**
	 * @brief Attempts to acquire the lock without blocking.
	 * @return true if lock was acquired, false otherwise.
	 */
	[[nodiscard]] bool try_lock() noexcept;

	/**
	 * @brief Releases the lock and notifies one waiting thread.
	 */
	void unlock() noexcept;

 private:
	/**
	 * @brief Atomic flag used for lock state.
	 */
	std::atomic_flag flag_ = ATOMIC_FLAG_INIT;
};
}  // namespace ufo

#endif  // UFO_UTILITY_SPINLOCK_HPP