#pragma once

#include <cstdint>
#include <functional>

namespace Runtime {

class Handle {
public:
	constexpr Handle() noexcept = default;
	constexpr explicit Handle(uint64_t value) noexcept
		: m_value(value) {}

	[[nodiscard]] constexpr bool IsValid() const noexcept { return m_value != 0; }
	[[nodiscard]] constexpr uint64_t Value() const noexcept { return m_value; }

	[[nodiscard]] constexpr bool operator==(const Handle& rhs) const noexcept = default;
	[[nodiscard]] constexpr bool operator!=(const Handle& rhs) const noexcept = default;
	[[nodiscard]] constexpr bool operator<(const Handle& rhs) const noexcept { return m_value < rhs.m_value; }

private:
	uint64_t m_value = 0;
};

} // namespace Runtime

template<>
struct std::hash<Runtime::Handle> {
	[[nodiscard]] std::size_t operator()(const Runtime::Handle& h) const noexcept {
		return std::hash<uint64_t>{}(h.Value());
	}
};
