#pragma once

#include <cstdint>
#include <functional>

namespace Runtime {

class Handle {
public:
	Handle() = default;
	explicit Handle(uint64_t value)
		: m_value(value) {}

	bool IsValid() const { return m_value != 0; }
	uint64_t Value() const { return m_value; }

	bool operator==(const Handle& rhs) const { return m_value == rhs.m_value; }
	bool operator!=(const Handle& rhs) const { return !(*this == rhs); }
	bool operator<(const Handle& rhs) const { return m_value < rhs.m_value; }

private:
	uint64_t m_value = 0;
};

} // namespace Runtime

template<>
struct std::hash<Runtime::Handle> {
	std::size_t operator()(const Runtime::Handle& h) const noexcept {
		return std::hash<uint64_t>{}(h.Value());
	}
};
