/*
* memory_read_stream.h
* MemoryReadStream use a buffer in memory to read from
*
* Created by Tony Huang (tony@slamtec.com) at 2017-4-13
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_stream.h"

#include <vector>
#include <cstdint>
#include <string>
#include <stdio.h>

namespace rpos { namespace system { namespace io {

    class RPOS_CORE_API MemoryReadStream : public IStream {
    public:
        MemoryReadStream(const void* buffer, size_t size);
        MemoryReadStream(const std::vector<std::uint8_t>& buffer);
        MemoryReadStream(std::vector<std::uint8_t>&& buffer);
        virtual ~MemoryReadStream();

    public:
        virtual bool isOpen();
        virtual bool canRead();
        virtual bool canWrite();
        virtual bool canSeek();

    public:
        virtual void close();

    public:
        virtual bool endOfStream();

    public:
        virtual int read(void* buffer, size_t size);
        virtual int write(const void* buffer, size_t size);
        virtual size_t tell();
        virtual void seek(SeekType type, int offset);

    public:
        size_t size() const;
        const std::uint8_t* buffer() const;

    private:
        std::vector<std::uint8_t> buffer_;
        size_t offset_;
    };

} } }
