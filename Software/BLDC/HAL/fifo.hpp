/**
 * \file
 * Generic FIFO module.
 * Implements a fixed length static FIFO. Data can be queued from one
 * end and dequeued (or just peeked) from the other
 */
#pragma once
#include <cstdint>
#include <array>

#include "critical.hpp"

template<typename T, uint16_t length>
class Fifo {
public:
	constexpr Fifo()
	: writePos(0)
	, level(0)
	, data()
	{};

	/**
	 * \brief Adds an element to the FIFO
	 *
	 * \param t the element to add
	 * \return true on success, false on failure
	 */
	bool enqueue(T t) {
		if(level < length) {
			CriticalSection crit;
			data[writePos] = t;
			writePos++;
			if(writePos >= length) {
				writePos -= length;
			}
			level++;
			return true;
		} else {
			return false;
		}
	}

	/**
	 * \brief Reads an element to the FIFO
	 *
	 * \param t Will contain the element if one is available
	 * \param justPeek If true the element will not be removed
	 * \return true on success, false on failure
	 */
	bool dequeue(T &t, bool justPeek = false) {
		if(level > 0) {
			CriticalSection crit;
			int16_t readPos = writePos - level;
			if(readPos < 0) {
				readPos += length;
			}
			t = data[readPos];
			if (!justPeek)
				level--;
			return true;
		} else {
			return false;
		}
	}

	/**
	 * \brief Reads the amount of data in the FIFO
	 */
	uint16_t getLevel() {
		return level;
	}

	/**
	 * \brief Gets the amount of free space in the FIFO
	 */
	uint16_t getSpace() {
		return length - level;
	}

	/**
	 * \brief Removes all data from the FIFO without reading it
	 */
	void clear() {
		level = 0;
		writePos = 0;
	}
private:
	uint16_t writePos;
	volatile uint16_t level;
	std::array<T, length> data;
};
