#pragma once

#include <cstddef>

namespace fault_injection
{

/**
 * Fixed-capacity array for PX4/NuttX builds that do not expose <array>.
 *
 * The container owns all storage inline, performs no allocation, and keeps the
 * subset of std::array's interface used by the FAULT fault infrastructure.
 */
template<typename T, std::size_t Capacity>
class FixedArray final
{
public:
	using value_type = T;
	using size_type = std::size_t;
	using iterator = T *;
	using const_iterator = const T *;

	FixedArray() = default;

	/*
	 * Preserve direct-list initialization such as:
	 *     FixedArray<Base *, 3>{&first, &second, &third}
	 * Missing trailing elements remain value-initialized, matching aggregate
	 * initialization of std::array.
	 */
	template<typename First, typename... Rest>
	FixedArray(First first, Rest... rest) :
		_items{static_cast<T>(first), static_cast<T>(rest)...}
	{
		static_assert(1U + sizeof...(Rest) <= Capacity,
			      "Too many initializers for FixedArray");
	}

	T &operator[](size_type index)
	{
		return _items[index];
	}

	const T &operator[](size_type index) const
	{
		return _items[index];
	}

	T &at(size_type index)
	{
		return _items[index];
	}

	const T &at(size_type index) const
	{
		return _items[index];
	}

	T &front()
	{
		return _items[0];
	}

	const T &front() const
	{
		return _items[0];
	}

	T &back()
	{
		return _items[Capacity - 1U];
	}

	const T &back() const
	{
		return _items[Capacity - 1U];
	}

	T *data()
	{
		return _items;
	}

	const T *data() const
	{
		return _items;
	}

	iterator begin()
	{
		return _items;
	}

	const_iterator begin() const
	{
		return _items;
	}

	const_iterator cbegin() const
	{
		return _items;
	}

	iterator end()
	{
		return _items + Capacity;
	}

	const_iterator end() const
	{
		return _items + Capacity;
	}

	const_iterator cend() const
	{
		return _items + Capacity;
	}

	constexpr size_type size() const
	{
		return Capacity;
	}

	constexpr bool empty() const
	{
		return Capacity == 0U;
	}

	void fill(const T &value)
	{
		for (size_type index = 0; index < Capacity; ++index) {
			_items[index] = value;
		}
	}

private:
	/* C++ forbids a zero-length native array; retain one unused slot for N=0. */
	static constexpr size_type StorageCapacity = (Capacity > 0U) ? Capacity : 1U;
	T _items[StorageCapacity]{};
};

} // namespace fault_injection
