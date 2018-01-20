#pragma once
/*
* i_stream.h
* IStream is the abstract interface of a byte stream
*
* Created by Tony Huang (tony@slamtec.com) at 2016-6-13
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <rpos/core/rpos_core_config.h>

namespace rpos { namespace system { namespace io {

    enum SeekType {
        SeekTypeSet,
        SeekTypeEnd,
        SeekTypeOffset
    };

	class RPOS_CORE_API IStream {
	public:
		IStream();
		virtual ~IStream();

	public:
		virtual bool isOpen() = 0;
		virtual bool canRead() = 0;
		virtual bool canWrite() = 0;
        virtual bool canSeek() = 0;

	public:
		virtual void close() = 0;

	public:
		virtual bool endOfStream() = 0;

	public:
		virtual int read(void* buffer, size_t size) = 0;
		virtual int write(const void* buffer, size_t size) = 0;
        virtual size_t tell() = 0;
        virtual void seek(SeekType type, int offset) = 0;
	};

} } }
