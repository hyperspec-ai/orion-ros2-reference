
#pragma once

#include <endian.h>
#include <stdint.h>

namespace EndianUtils
{
// templated conversion of network byte order to host byte order
template <typename T> T letoh(T);
// templated conversion of host byte order to network byte order
template <typename T> T betoh(T);
// templated conversion of host byte order to little endian
template <typename T> T htole(T);
// templated conversion of host byte order to big endian
template <typename T> T htobe(T);

// Template specialization for supported types
#define SPECIALIZE(type, func_name, conv)  \
    template <> type func_name(type value) \
    {                                      \
        return (conv);                     \
    }

// letoh
SPECIALIZE(uint8_t, letoh, value)
SPECIALIZE(uint16_t, letoh, le16toh(value))
SPECIALIZE(uint32_t, letoh, le32toh(value))
SPECIALIZE(uint64_t, letoh, le64toh(value))
SPECIALIZE(int8_t, letoh, value)
SPECIALIZE(int16_t, letoh, le16toh(value))
SPECIALIZE(int32_t, letoh, le32toh(value))
SPECIALIZE(int64_t, letoh, le64toh(value))

// betoh
SPECIALIZE(uint8_t, betoh, value)
SPECIALIZE(uint16_t, betoh, be16toh(value))
SPECIALIZE(uint32_t, betoh, be32toh(value))
SPECIALIZE(uint64_t, betoh, be64toh(value))
SPECIALIZE(int8_t, betoh, value)
SPECIALIZE(int16_t, betoh, be16toh(value))
SPECIALIZE(int32_t, betoh, be32toh(value))
SPECIALIZE(int64_t, betoh, be64toh(value))

// htole
SPECIALIZE(uint8_t, htole, value)
SPECIALIZE(uint16_t, htole, htole16(value))
SPECIALIZE(uint32_t, htole, htole32(value))
SPECIALIZE(uint64_t, htole, htole64(value))
SPECIALIZE(int8_t, htole, value)
SPECIALIZE(int16_t, htole, htole16(value))
SPECIALIZE(int32_t, htole, htole32(value))
SPECIALIZE(int64_t, htole, htole64(value))

// htobe
SPECIALIZE(uint8_t, htobe, value)
SPECIALIZE(uint16_t, htobe, htobe16(value))
SPECIALIZE(uint32_t, htobe, htobe32(value))
SPECIALIZE(uint64_t, htobe, htobe64(value))
SPECIALIZE(int8_t, htobe, value)
SPECIALIZE(int16_t, htobe, htobe16(value))
SPECIALIZE(int32_t, htobe, htobe32(value))
SPECIALIZE(int64_t, htobe, htobe64(value))

#undef SPECIALIZE

}  // namespace EndianUtils
