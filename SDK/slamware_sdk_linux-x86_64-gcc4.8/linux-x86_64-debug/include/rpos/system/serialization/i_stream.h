#pragma once
#include <stdint.h>
#include <vector>
#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace system { namespace serialization {

    class RPOS_CORE_API IStream
    {
    public:
        virtual size_t read(void *buffer, size_t count) = 0;
        virtual size_t write(const void *buffer, size_t count) = 0;
        virtual bool empty();

        IStream();

        virtual ~IStream();

        size_t readBuffer(uint8_t *buffer, size_t count);
        void writeBuffer(const uint8_t *buffer, size_t count);
    };

    class RPOS_CORE_API ISerializable
    {
    public:
        virtual bool readFromStream(IStream &in) = 0;
        virtual bool writeToStream(IStream &out) const = 0;
    };

#define DECLARE_ISTREAM_READ_WRITE( T ) \
		RPOS_CORE_API IStream& operator<<(IStream&out, const T &a); \
		RPOS_CORE_API IStream& operator>>(IStream&in, T &a);

    DECLARE_ISTREAM_READ_WRITE(bool)
    DECLARE_ISTREAM_READ_WRITE(uint8_t)
    DECLARE_ISTREAM_READ_WRITE(int8_t)
    DECLARE_ISTREAM_READ_WRITE(uint16_t)
    DECLARE_ISTREAM_READ_WRITE(int16_t)
    DECLARE_ISTREAM_READ_WRITE(uint32_t)
    DECLARE_ISTREAM_READ_WRITE(int32_t)
    DECLARE_ISTREAM_READ_WRITE(uint64_t)
    DECLARE_ISTREAM_READ_WRITE(int64_t)
    DECLARE_ISTREAM_READ_WRITE(float)
    DECLARE_ISTREAM_READ_WRITE(double)
    DECLARE_ISTREAM_READ_WRITE(std::vector<uint8_t>)

} } }